"""leap_driver.py — 16-DOF LEAP Hand 硬件抽象与数学转换驱动层.

遵循 Invariants 契约:
1. ROS2 关节空间 q: 单位 rad，以 OPEN_POSE 为 0 rad 基准零位，屈曲弯曲为正 (q > 0)；
2. 硬件舵机绝对弧度 theta_dxl: 由 Dynamixel 协议读取与控制；
3. 严格限位裁剪与看门狗平滑回全开位 (Relax-to-OPEN) 保护；
4. 内置 Mock 模式支持高吞吐离线测试。
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import numpy as np

# Canonical 16-DOF Joint 命名
CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]

# 舵机驱动方向符号映射表 (针对 LEAP Hand 右手 16 舵机)
# 关节空间弯曲为正，对应舵机实际运动方向
DEFAULT_JOINT_DIR = np.array([
    -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0,
     1.0, -1.0, -1.0, -1.0
], dtype=np.float64)

# 默认实测全开平伸位姿 (rad)
DEFAULT_OPEN_POSE = np.array([
    3.1155, 4.6204, 3.2076, 1.5785,
    3.2413, 3.0618, 3.1170, 4.5927,
    3.1186, 3.0756, 3.1799, 4.6004,
    3.1186, 1.5555, 4.6234, 4.7247,
], dtype=np.float64)

# 默认舵机物理极限 (绝对弧度 rad，来自真机标定全局边界)
DEFAULT_DXL_LIMITS_MIN = np.array([
    2.50, 2.00, 1.00, -0.30,
    2.50, 1.00, 1.00,  2.50,
    2.50, 1.00, 1.00,  2.50,
    2.50, -0.30, 2.50,  2.50
], dtype=np.float64)

DEFAULT_DXL_LIMITS_MAX = np.array([
    3.80, 5.20, 3.80, 2.00,
    3.80, 3.80, 3.80, 5.20,
    3.80, 3.80, 3.80, 5.20,
    4.50, 2.50, 5.20, 5.80
], dtype=np.float64)


@dataclass(frozen=True)
class CanonicalJoints:
    names: Sequence[str] = tuple(CANONICAL_JOINT_NAMES)
    count: int = 16


class LeapHandDriver:
    """LEAP Hand 16-DOF 核心驱动类."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 4000000,
        kP: int = 300,
        kI: int = 0,
        kD: int = 100,
        curr_lim: int = 150,
        open_pose: Optional[np.ndarray] = None,
        joint_dir: Optional[np.ndarray] = None,
        mock_mode: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.curr_lim = curr_lim
        self.mock_mode = mock_mode

        self.open_pose = open_pose.copy() if open_pose is not None else DEFAULT_OPEN_POSE.copy()
        self.joint_dir = joint_dir.copy() if joint_dir is not None else DEFAULT_JOINT_DIR.copy()
        self.motor_limits_min = DEFAULT_DXL_LIMITS_MIN.copy()
        self.motor_limits_max = DEFAULT_DXL_LIMITS_MAX.copy()

        self.motor_ids = list(range(16))
        self.dxl_client = None
        self._is_connected = False

        # 内部状态缓存
        self._curr_joint_pos = np.zeros(16, dtype=np.float64)  # 机器人关节空间 (rad)
        self._curr_joint_vel = np.zeros(16, dtype=np.float64)  # (rad/s)
        self._curr_joint_eff = np.zeros(16, dtype=np.float64)  # (mA)

        # 看门狗平滑回退状态机
        self._loss_t0: Optional[float] = None
        self._relax_from_pos: Optional[np.ndarray] = None

    # ─── 坐标系双向转换 (Invariants 数学契约) ──────────────────────────

    def robot_joint_to_hardware_dxl(self, q_joint: np.ndarray) -> np.ndarray:
        """机器人关节空间 (0 为全开，弯曲为正) -> 舵机绝对弧度 (已裁剪限位)."""
        q = np.asarray(q_joint, dtype=np.float64)
        if q.shape != (16,):
            raise ValueError(f"Expected 16-element joint array, got {q.shape}")
        raw_dxl = self.open_pose + self.joint_dir * q
        clipped_dxl = np.clip(raw_dxl, self.motor_limits_min, self.motor_limits_max)
        return clipped_dxl

    def hardware_dxl_to_robot_joint(self, theta_dxl: np.ndarray) -> np.ndarray:
        """舵机绝对弧度 -> 机器人关节空间 (0 为全开，弯曲为正)."""
        theta = np.asarray(theta_dxl, dtype=np.float64)
        if theta.shape != (16,):
            raise ValueError(f"Expected 16-element dxl array, got {theta.shape}")
        q_joint = (theta - self.open_pose) * self.joint_dir
        return q_joint

    # ─── 硬件生命周期管理 ───────────────────────────────────────────

    def connect(self) -> bool:
        """连接硬件并配置工作模式为位置-电流模式 (Operating Mode = 5)."""
        if self._is_connected:
            return True

        if self.mock_mode:
            logging.info("[LeapHandDriver] Running in MOCK mode.")
            self._is_connected = True
            self._curr_joint_pos = np.zeros(16, dtype=np.float64)
            return True

        try:
            from .dynamixel_client import DynamixelClient

            self.dxl_client = DynamixelClient(self.motor_ids, self.port, self.baudrate)
            self.dxl_client.connect()

            # Return Delay Time = 0 (最快总线响应)
            self.dxl_client.sync_write(self.motor_ids, np.zeros(16), 9, 1)
            # 位置-电流模式 (Operating Mode = 5)
            self.dxl_client.sync_write(self.motor_ids, np.ones(16) * 5, 11, 1)
            # 使能力矩
            self.dxl_client.set_torque_enabled(self.motor_ids, True)

            # 配置 PID 与侧摆差异化阻尼
            self.dxl_client.sync_write(self.motor_ids, np.ones(16) * self.kP, 84, 2)
            self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kP * 0.75), 84, 2)
            self.dxl_client.sync_write(self.motor_ids, np.ones(16) * self.kI, 82, 2)
            self.dxl_client.sync_write(self.motor_ids, np.ones(16) * self.kD, 80, 2)
            self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kD * 0.75), 80, 2)
            # 写入限流 (单位 1mA)
            self.dxl_client.sync_write(self.motor_ids, np.ones(16) * self.curr_lim, 102, 2)

            # 初始到位全开位 (0 rad)
            self.set_open()
            self._is_connected = True
            logging.info(f"[LeapHandDriver] Connected on {self.port} at {self.baudrate} bps.")
            return True
        except Exception as e:
            logging.error(f"[LeapHandDriver] Failed to connect: {e}")
            self._is_connected = False
            self.dxl_client = None
            return False

    def disconnect(self) -> None:
        """安全平稳断开硬件."""
        if not self._is_connected:
            return
        if not self.mock_mode and self.dxl_client is not None:
            try:
                self.set_open()
                self.dxl_client.disconnect()
            except Exception as e:
                logging.warning(f"[LeapHandDriver] Disconnect exception: {e}")
            finally:
                self.dxl_client = None
        self._is_connected = False
        logging.info("[LeapHandDriver] Disconnected.")

    def is_connected(self) -> bool:
        return self._is_connected

    # ─── 读写核心接口 ─────────────────────────────────────────────

    def read_states(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """读取物理实测状态 -> (position_rad, velocity_rad_s, effort_mA)."""
        if not self._is_connected:
            raise RuntimeError("Cannot read states: hand is not connected.")

        if self.mock_mode:
            return self._curr_joint_pos.copy(), self._curr_joint_vel.copy(), self._curr_joint_eff.copy()

        # 硬件高效 SyncRead 位置、速度、电流
        dxl_pos, dxl_vel, dxl_cur = self.dxl_client.read_pos_vel_cur()
        self._curr_joint_pos = self.hardware_dxl_to_robot_joint(dxl_pos)
        self._curr_joint_vel = dxl_vel * self.joint_dir  # 速度对应方向
        self._curr_joint_eff = dxl_cur.copy()
        return self._curr_joint_pos.copy(), self._curr_joint_vel.copy(), self._curr_joint_eff.copy()

    def write_command(self, q_target: np.ndarray) -> None:
        """输入 16 维目标绝对关节弧度 (0 为全开) -> 下发至舵机."""
        if not self._is_connected:
            raise RuntimeError("Cannot write command: hand is not connected.")

        dxl_target = self.robot_joint_to_hardware_dxl(q_target)

        if self.mock_mode:
            # 仿真一阶惯性滞后响应
            self._curr_joint_pos = q_target.copy()
            return

        self.dxl_client.write_desired_pos(self.motor_ids, dxl_target)

    def set_open(self) -> None:
        """复位回全开平伸姿势 (0 rad)."""
        self.write_command(np.zeros(16, dtype=np.float64))

    # ─── 安全看门狗平滑阻尼回退 (Relax-to-OPEN) ─────────────────────

    def relax_to_open_step(self, now: float, hold_time: float = 0.30, ramp_time: float = 0.60) -> np.ndarray:
        """手部信号丢失时，平滑阻尼过渡回全开位 (0 rad).
        
        Args:
            now: 当前时间戳 (秒)
            hold_time: 初始保持当前姿态的时间 (秒)
            ramp_time: 线性过渡回 0 rad 的时间 (秒)
        Returns:
            当前插值并下发的目标关节角 (rad)
        """
        if self._loss_t0 is None:
            self._loss_t0 = now
            self._relax_from_pos = self._curr_joint_pos.copy()

        elapsed = now - self._loss_t0
        if elapsed < hold_time:
            target = self._relax_from_pos.copy()
        else:
            t = min(1.0, (elapsed - hold_time) / max(0.01, ramp_time))
            target = self._relax_from_pos * (1.0 - t)  # 逐渐收敛至 0 rad

        self.write_command(target)
        return target

    def reset_loss_state(self) -> None:
        """重置看门狗丢失计时器."""
        self._loss_t0 = None
        self._relax_from_pos = None
