"""hand_controller.py — 16-DOF LEAP Hand 控制核心 (LeapNode & HandController).

遵循 TuinaDex 黄金不变量:
1. SI Radian 弧度单位契约;
2. 灵巧手舵机工作电流上限硬锁定为 150mA;
3. 优雅兼容 Mock 模式 (无硬件环境自测);
4. 姿态数据与 config/poses.json 动态绑定.
"""
from __future__ import annotations

import logging
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from tuinadex_hand.driver.dynamixel_client import DynamixelClient
from .kinematics_limits import angle_safety_clip
from .pose_manager import (
    DEFAULT_POSES,
    get_default_config_path,
    is_valid_pose as _is_valid_pose,
    load_poses,
    save_poses,
    unwrap_to_limits,
)

logger = logging.getLogger(__name__)

# 关节方向向量: +1 = 用户正输入 -> 电机加, -1 = 用户正输入 -> 电机减
# 弯向手心为正 (黄金不变量契约: q > 0)
JOINT_DIR = np.array([
    -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0,
     1.0, -1.0, -1.0, -1.0
], dtype=np.float64)

FINGER_NAMES = ["食指(Index)", "中指(Middle)", "无名指(Ring)", "拇指(Thumb)"]
JOINT_NAMES = ["MCP侧摆", "MCP前后", "PIP", "DIP"]        # 食指/中指/无名指
THUMB_JOINT_NAMES = ["MCP前后", "MCP侧摆", "PIP", "DIP"]  # 拇指: ID 12=mcp, 13=side

FINGER_START = {
    "食指": 0, "index": 0,
    "中指": 4, "middle": 4,
    "无名指": 8, "ring": 8,
    "拇指": 12, "thumb": 12
}


class MockDynamixelClient:
    """Mock Dynamixel 客户端，供单元测试与离线演示."""

    def __init__(self, motors: Sequence[int], port: str, baudrate: int):
        self.motors = list(motors)
        self.port = port
        self.baudrate = baudrate
        self.torque_enabled = {m: True for m in self.motors}
        self.current_pos = np.array([
            3.1155, 4.6204, 3.2076, 1.5785,
            3.2413, 3.0618, 3.1170, 4.5927,
            3.1186, 3.0756, 3.1799, 4.6004,
            3.1186, 1.5555, 4.6234, 4.7247
        ], dtype=np.float64)
        self.current_vel = np.zeros(len(self.motors), dtype=np.float64)
        self.current_cur = np.ones(len(self.motors), dtype=np.float64) * 20.0  # 20mA

    def connect(self):
        logger.info(f"[MockDynamixelClient] Connected to virtual port {self.port}")

    def disconnect(self):
        logger.info("[MockDynamixelClient] Disconnected")

    def sync_write(self, motors, values, addr, length):
        pass

    def set_torque_enabled(self, motors, enabled):
        for m in motors:
            self.torque_enabled[m] = enabled

    def write_desired_pos(self, motors, pos):
        for idx, m in enumerate(motors):
            if m < len(self.current_pos):
                self.current_pos[m] = float(pos[idx])

    def read_pos(self):
        return self.current_pos.copy()

    def read_vel(self):
        return self.current_vel.copy()

    def read_cur(self):
        return self.current_cur.copy()

    def read_pos_vel(self):
        return self.current_pos.copy(), self.current_vel.copy()

    def read_pos_vel_cur(self):
        return self.current_pos.copy(), self.current_vel.copy(), self.current_cur.copy()


class LeapNode:
    """LEAP Hand 16-DOF 舵机核心控制节点."""

    def __init__(
        self,
        port: Optional[str] = None,
        calib_mode: bool = False,
        mock: bool = False,
        kP: int = 300,
        kI: int = 0,
        kD: int = 100,
        curr_lim: int = 150,
    ):
        """初始化并连接 LEAP Hand.

        calib_mode=True: 标定模式，全开位数据无效时仍允许连接；
        mock=True: 离线仿真/Mock模式，无需物理串口.
        """
        self.poses, self.open_pose_valid = load_poses()
        self.open_pose = self.poses.get("全开/平伸", DEFAULT_POSES["全开/平伸"]).copy()

        # 安全门: 校准数据无效时拒绝驱动电机 (校准模式除外)
        if not self.open_pose_valid and not calib_mode:
            logger.warning("[⚠️ 告警] LEAP Hand 全开位校准数据无效, 拒绝驱动电机。")
            raise RuntimeError("LEAP Hand 全开位校准数据无效，请先运行 leap-calibrate 重新校准。")

        self.kP = kP
        self.kI = kI
        self.kD = kD
        # 严禁越过 150mA 安全红线
        self.curr_lim = min(curr_lim, 150)
        self.motors = list(range(16))
        self.mock = mock

        if self.mock:
            self.dxl_client = MockDynamixelClient(self.motors, port or "mock_port", 4000000)
            self.dxl_client.connect()
            print("[INFO] [Mock Mode] LEAP Hand virtual client initialized.")
        else:
            ports_to_try = [port] if port is not None else ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "COM13"]
            connected = False
            last_err = None
            for p in ports_to_try:
                try:
                    self.dxl_client = DynamixelClient(self.motors, p, 4000000)
                    self.dxl_client.connect()
                    print(f"[INFO] Connected to LEAP Hand on: {p}")
                    connected = True
                    break
                except Exception as e:
                    last_err = e

            if not connected:
                raise OSError(f"Could not connect to LEAP Hand on {ports_to_try}. Error: {last_err}")

        # 硬件参数初始化 (Return Delay = 0, 位置-电流模式)
        self.dxl_client.sync_write(self.motors, np.zeros(len(self.motors)), 9, 1)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)

        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)

        # 写入实测全开位
        self.curr_pos = self.open_pose.copy()
        self.prev_pos = self.open_pose.copy()
        if not calib_mode or self.open_pose_valid:
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        print(f"[INFO] LEAP Hand initialized (kP={self.kP}, kD={self.kD}, curr_lim={self.curr_lim}mA)")

    def set_leap(self, pose: Sequence[float]):
        """直接用 16 舵机硬件角度控制."""
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose, dtype=np.float64)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def set_pose(self, name: str) -> bool:
        """用录好的姿势名称控制."""
        if name not in self.poses:
            print(f"[错误] 未知姿势: {name}，可选: {list(self.poses.keys())}")
            return False
        self.set_leap(self.poses[name])
        return True

    def set_open(self):
        """控制手回到全开位."""
        self.set_leap(self.open_pose)

    def set_torque(self, enabled: bool):
        """使能或释放全部 16 个舵机扭矩."""
        self.dxl_client.set_torque_enabled(self.motors, enabled)

    def set_joint(self, motor_id: int, relative_angle: float):
        """设置单个关节, relative_angle 以全开位为基准 (rad, 弯曲为正)."""
        if 0 <= motor_id <= 15:
            self.curr_pos[motor_id] = self.open_pose[motor_id] + JOINT_DIR[motor_id] * relative_angle
            self.curr_pos = angle_safety_clip(self.curr_pos)
            self.set_leap(self.curr_pos)
        else:
            print("[错误] 电机 ID 必须在 0-15 范围内")

    def set_finger(self, finger_start_id: int, relative_angles: Sequence[float]):
        """设置一根手指的 4 个关节 (rad, 弯曲为正)."""
        for i, angle in enumerate(relative_angles):
            mid = finger_start_id + i
            self.curr_pos[mid] = self.open_pose[mid] + JOINT_DIR[mid] * angle
        self.curr_pos = angle_safety_clip(self.curr_pos)
        self.set_leap(self.curr_pos)

    def read_pos(self) -> np.ndarray:
        return self.dxl_client.read_pos()

    def read_vel(self) -> np.ndarray:
        return self.dxl_client.read_vel()

    def read_cur(self) -> np.ndarray:
        return self.dxl_client.read_cur()

    def pos_vel(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.dxl_client.read_pos_vel()

    def pos_vel_eff_srv(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.dxl_client.read_pos_vel_cur()

    def disconnect(self):
        self.dxl_client.disconnect()
        print("[INFO] Disconnected from LEAP Hand.")


class HandController:
    """面向交互式终端的高层手部控制器."""

    def __init__(self, port: Optional[str] = None, mock: bool = False):
        self.leap = LeapNode(port=port, mock=mock)
        self.open_pose = self.leap.open_pose.copy()
        self.target = self.open_pose.copy()

    def set_joint_relative(self, finger: str, joint: str, angle: float) -> bool:
        """以实测全开位为基准, 设置关节相对角度 (正值 = 弯曲)."""
        start_id = FINGER_START.get(finger.lower())
        if start_id is None:
            print(f"[错误] 未知手指: {finger}")
            return False

        # 拇指物理顺序与其他指不同: ID 12=mcp, 13=side, 14=pip, 15=dip
        if finger.lower() in ("thumb", "拇指"):
            joint_map = {"mcp": 0, "side": 1, "pip": 2, "dip": 3}
        else:
            joint_map = {"side": 0, "mcp": 1, "pip": 2, "dip": 3}

        if joint.lower() == "all":
            ids = list(range(start_id, start_id + 4))
        elif joint.lower() in joint_map:
            ids = [start_id + joint_map[joint.lower()]]
        else:
            print(f"[错误] 未知关节: {joint}，请用 side/mcp/pip/dip/all")
            return False

        for mid in ids:
            self.target[mid] = self.open_pose[mid] + JOINT_DIR[mid] * angle
        self.leap.set_leap(self.target)
        return True

    def print_state(self):
        pos, vel, cur = self.leap.pos_vel_eff_srv()
        print(f"\n{'─'*72}")
        print(f"  {'手指':<10} {'ID':>3}  {'关节':<8} {'LEAP实际':>10} {'LEAP全开':>10} {'相对位移':>10}")
        print(f"{'─'*72}")
        for fi, fname in enumerate(FINGER_NAMES):
            names = THUMB_JOINT_NAMES if fname.startswith("拇指") else JOINT_NAMES
            for ji, jname in enumerate(names):
                mid = fi * 4 + ji
                actual = pos[mid]
                base = self.open_pose[mid]
                offset = actual - base
                print(f"  {fname:<10} {mid:>3}  {jname:<8} {actual:>10.4f} {base:>10.4f} {offset:>+10.4f}")
        print(f"{'─'*72}")
        print(f"  电流: {cur.min():.0f}~{cur.max():.0f} mA")
        print()

    def disconnect(self):
        self.leap.set_open()
        time.sleep(0.2)
        self.leap.disconnect()

