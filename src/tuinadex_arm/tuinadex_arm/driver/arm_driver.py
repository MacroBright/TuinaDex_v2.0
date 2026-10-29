"""arm_driver.py — 6-DOF reBot B601-RS 机械臂硬件抽象与控制驱动层.

遵循 TuinaDex Invariants 契约:
1. ROS2 关节空间 q: 单位 rad，以规范 HOME 位姿为 0 rad 基准；
2. 规范 6-DOF 关节命名 (CANONICAL_ARM_JOINT_NAMES: joint1~joint6)；
3. 支持 MIT 力位混合/阻抗模式与速度平滑限制；
4. 内置 300ms Watchdog 平滑回零 (Relax-to-HOME) 保护；
5. 内置 Mock 模式支持离线与 CI 自动化测试。
"""
from __future__ import annotations

import logging
import math
import time
from typing import List, Optional, Sequence, Tuple
import numpy as np

from tuinadex_core.names import CANONICAL_ARM_JOINT_NAMES
from tuinadex_core.command_processing import ARM_SPEC_LIMITS

logger = logging.getLogger("tuinadex_arm.arm_driver")


class ArmDriver:
    """reBot B601-RS 6-DOF 核心驱动类."""

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 1000000,
        host_id: int = 0xFD,
        control_rate_hz: float = 50.0,
        default_speed_deg_s: float = 30.0,
        mock_mode: bool = False,
    ) -> None:
        self.channel = channel
        self.bitrate = bitrate
        self.host_id = host_id
        self.control_rate_hz = control_rate_hz
        self.default_speed_deg_s = default_speed_deg_s
        self.mock_mode = mock_mode

        self.is_connected = False
        self.is_enabled = False

        # 内部状态变量 (6-DOF rad)
        self._current_pos = np.zeros(6, dtype=np.float64)
        self._current_vel = np.zeros(6, dtype=np.float64)
        self._current_eff = np.zeros(6, dtype=np.float64)
        self._target_pos = np.zeros(6, dtype=np.float64)

        # 看门狗平滑回退轨迹状态
        self._loss_start_time: Optional[float] = None
        self._loss_snapshot_pos: Optional[np.ndarray] = None

        # 真实硬件控制器实例 (按需延迟引入，避免 mock 模式依赖 CAN 库)
        self._real_controller = None

    def connect(self) -> bool:
        """建立连接并完成使能."""
        if self.mock_mode:
            self.is_connected = True
            self.is_enabled = True
            self._current_pos = np.zeros(6, dtype=np.float64)
            self._target_pos = np.zeros(6, dtype=np.float64)
            logger.info("ArmDriver 运行于 Mock 模式，已虚拟连接。")
            return True

        try:
            from rebot.config import ControllerConfig, MotorConfig, TemperatureThresholds, ReturnZeroConfig
            from rebot.controller import ReBotRSMITController

            # 配置 6 个电机 (J1-J3: rs-06, J4-J6: rs-00)
            motors = (
                MotorConfig(motor_id=1, model="rs-06", kp=35.0, kd=2.0),
                MotorConfig(motor_id=2, model="rs-06", kp=65.0, kd=5.0),
                MotorConfig(motor_id=3, model="rs-06", kp=65.0, kd=5.0),
                MotorConfig(motor_id=4, model="rs-00", kp=30.0, kd=1.0),
                MotorConfig(motor_id=5, model="rs-00", kp=35.0, kd=3.0),
                MotorConfig(motor_id=6, model="rs-00", kp=30.0, kd=1.0),
            )
            temps = TemperatureThresholds(alarm_c=110.0, return_zero_c=125.0, disconnect_c=140.0)
            ret_zero = ReturnZeroConfig(max_speed_deg_s=20.0, thermal_max_speed_deg_s=30.0)

            cfg = ControllerConfig(
                channel=self.channel,
                host_id=self.host_id,
                control_hz=200.0,
                telemetry_hz=2.0,
                default_speed_deg_s=self.default_speed_deg_s,
                motors=motors,
                temperatures=temps,
                return_zero=ret_zero,
            )
            self._real_controller = ReBotRSMITController(cfg)
            self._real_controller.start()
            self.is_connected = True
            self.is_enabled = True
            logger.info(f"成功连接 reBot 机械臂 (channel={self.channel})。")
            return True
        except Exception as e:
            logger.error(f"连接机械臂失败: {e}")
            self.is_connected = False
            self.is_enabled = False
            return False

    def disconnect(self) -> None:
        """安全断开连接."""
        if self.mock_mode:
            self.is_connected = False
            self.is_enabled = False
            logger.info("ArmDriver Mock 已断开。")
            return

        if self._real_controller is not None:
            try:
                self._real_controller.stop(return_to_zero=True, wait=True)
            except Exception as e:
                logger.warning(f"断开机械臂异常: {e}")
            finally:
                self._real_controller = None
                self.is_connected = False
                self.is_enabled = False

    def write_command(self, target_q: Sequence[float]) -> None:
        """下发目标关节角 (rad)."""
        target = np.asarray(target_q, dtype=np.float64)
        if target.shape != (6,):
            raise ValueError(f"Arm command dim mismatch: expected 6, got {target.shape}")

        # 硬件限位保护裁剪
        clamped = np.clip(target, ARM_SPEC_LIMITS[:, 0], ARM_SPEC_LIMITS[:, 1])
        self._target_pos = clamped

        if self.mock_mode:
            # Mock 模式下执行 1 阶低通滤波响应 (dt=0.02s)
            alpha = 0.3
            self._current_pos = (1 - alpha) * self._current_pos + alpha * self._target_pos
            self._current_vel = (self._target_pos - self._current_pos) * 10.0
            return

        if self._real_controller is not None:
            # 弧度转为角度下发给底层控制器
            target_deg = [math.degrees(rad) for rad in clamped]
            self._real_controller.set_joint_angles(target_deg)

    def read_states(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """读取机械臂当前关节状态 (pos, vel, effort)，单位 rad, rad/s, Nm."""
        if self.mock_mode:
            return self._current_pos.copy(), self._current_vel.copy(), self._current_eff.copy()

        if self._real_controller is not None:
            angles_deg = self._real_controller.get_command_angles()
            self._current_pos = np.array([math.radians(deg) for deg in angles_deg], dtype=np.float64)
            return self._current_pos.copy(), self._current_vel.copy(), self._current_eff.copy()

        return self._current_pos.copy(), self._current_vel.copy(), self._current_eff.copy()

    def reset_loss_state(self) -> None:
        """清空失联回退轨迹."""
        self._loss_start_time = None
        self._loss_snapshot_pos = None

    def relax_to_open_step(self, now: float, hold_time: float = 0.30, ramp_time: float = 0.60) -> None:
        """看门狗超时阻尼回退 (Hold-then-Ramp to 0 rad HOME)."""
        if self._loss_start_time is None:
            self._loss_start_time = now
            self._loss_snapshot_pos = self._current_pos.copy()

        elapsed = now - self._loss_start_time
        if elapsed < hold_time:
            # 保持阶段
            self.write_command(self._loss_snapshot_pos)
        elif elapsed < (hold_time + ramp_time):
            # 渐变回退阶段: smoothstep 线性过渡至 0 rad HOME
            alpha = (elapsed - hold_time) / ramp_time
            # smoothstep 插值
            s = alpha * alpha * (3.0 - 2.0 * alpha)
            target = (1.0 - s) * self._loss_snapshot_pos
            self.write_command(target)
        else:
            # 完成回退，保持 0 rad
            self.write_command(np.zeros(6, dtype=np.float64))

    def set_home(self) -> None:
        """回到零位 HOME."""
        self.write_command(np.zeros(6, dtype=np.float64))

