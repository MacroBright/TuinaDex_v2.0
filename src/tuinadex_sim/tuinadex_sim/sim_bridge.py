"""sim_bridge.py — MuJoCo 物理仿真桥接器 (纯 Python / 动力学步进与状态提取).

遵循 Invariants 契约:
1. 关节空间 q: 16 维灵巧手 + 6 维机械臂 (支持 22-DOF 复合仿真或纯 16-DOF 灵巧手模式)；
2. 物理步长: 默认 0.002s (500 Hz)；
3. Canonical Joint 映射: 严格遵循 CANONICAL_JOINT_NAMES 与 CANONICAL_ARM_JOINT_NAMES 顺序索引；
4. 纯逻辑架构: 不依赖 rclpy，便于在 pytest 中快速单测与离线验证。
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional, Sequence
import numpy as np

from tuinadex_core.names import CANONICAL_JOINT_NAMES, CANONICAL_ARM_JOINT_NAMES

try:
    import mujoco
except ImportError:
    mujoco = None

# 默认推拿准备姿态 (机械臂 6 关节角度: 掌面平行朝下悬停，Palm Z=0.981m)
DEFAULT_READY_ARM_Q = np.array([0.0, 1.400, 1.230, -1.400, 0.0, 0.0], dtype=np.float64)


@dataclass
class SimState:
    time: float
    positions: np.ndarray    # 16 维 (rad)
    velocities: np.ndarray   # 16 维 (rad/s)
    efforts: np.ndarray      # 16 维 (actuator force / torque)


@dataclass
class ArmSimState:
    time: float
    positions: np.ndarray    # 6 维 (rad)
    velocities: np.ndarray   # 6 维 (rad/s)
    efforts: np.ndarray      # 6 维 (actuator force / torque)


class MujocoSimBridge:
    """MuJoCo 物理动力学桥接器 (支持 22-DOF 臂手一体或 16-DOF 独立手仿真)."""

    def __init__(self, xml_path: str, timestep: Optional[float] = None):
        if mujoco is None:
            raise RuntimeError("mujoco package is not installed.")
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"MuJoCo XML model file not found: {xml_path}")

        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        if timestep is not None:
            t = float(timestep)
            if t <= 0:
                raise ValueError(f"timestep must be > 0, got {timestep}")
            # ROS2 参数注入的物理步长覆盖 XML 缺省值 (须在 MjData 创建前设置)。
            self.model.opt.timestep = t
        self.data = mujoco.MjData(self.model)

        # 建立 16-DOF Canonical Hand Joint 映射
        self.joint_ids: List[int] = []
        self.qpos_indices: List[int] = []
        self.qvel_indices: List[int] = []
        self.actuator_ids: List[int] = []

        for name in CANONICAL_JOINT_NAMES:
            j_obj = self.model.joint(name)
            j_id = j_obj.id
            self.joint_ids.append(j_id)
            self.qpos_indices.append(int(self.model.jnt_qposadr[j_id]))
            self.qvel_indices.append(int(self.model.jnt_dofadr[j_id]))

            act_name = f"act_{name}"
            try:
                act_obj = self.model.actuator(act_name)
                self.actuator_ids.append(act_obj.id)
            except KeyError:
                # 若执行器命名不带 act_ 前缀，尝试直接匹配
                act_obj = self.model.actuator(name)
                self.actuator_ids.append(act_obj.id)

        # 检查是否包含 6-DOF 机械臂关节
        self.has_arm = False
        self.arm_joint_ids: List[int] = []
        self.arm_qpos_indices: List[int] = []
        self.arm_qvel_indices: List[int] = []
        self.arm_actuator_ids: List[int] = []

        model_joints = {self.model.joint(i).name for i in range(self.model.njnt)}
        if all(name in model_joints for name in CANONICAL_ARM_JOINT_NAMES):
            self.has_arm = True
            for name in CANONICAL_ARM_JOINT_NAMES:
                j_obj = self.model.joint(name)
                j_id = j_obj.id
                self.arm_joint_ids.append(j_id)
                self.arm_qpos_indices.append(int(self.model.jnt_qposadr[j_id]))
                self.arm_qvel_indices.append(int(self.model.jnt_dofadr[j_id]))

                act_name = f"act_{name}"
                try:
                    act_obj = self.model.actuator(act_name)
                    self.arm_actuator_ids.append(act_obj.id)
                except KeyError:
                    act_obj = self.model.actuator(name)
                    self.arm_actuator_ids.append(act_obj.id)

            # 初始化机械臂到预备推拿姿态
            for i, q_idx in enumerate(self.arm_qpos_indices):
                self.data.qpos[q_idx] = float(DEFAULT_READY_ARM_Q[i])
            for i, act_id in enumerate(self.arm_actuator_ids):
                self.data.ctrl[act_id] = float(DEFAULT_READY_ARM_Q[i])

        # 初始步进一次，确保系统状态一致
        mujoco.mj_forward(self.model, self.data)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def set_joint_targets(self, targets: Sequence[float]) -> None:
        """设置 16 维 Canonical 灵巧手关节的目标控制弧度."""
        if len(targets) != 16:
            raise ValueError(f"Expected 16 joint targets, got {len(targets)}")
        for i, act_id in enumerate(self.actuator_ids):
            self.data.ctrl[act_id] = float(targets[i])

    def set_arm_targets(self, targets: Sequence[float]) -> None:
        """设置 6 维 Canonical 机械臂关节的目标控制弧度."""
        if not self.has_arm:
            return
        if len(targets) != 6:
            raise ValueError(f"Expected 6 arm joint targets, got {len(targets)}")
        for i, act_id in enumerate(self.arm_actuator_ids):
            self.data.ctrl[act_id] = float(targets[i])

    def step(self, num_steps: int = 1) -> None:
        """前向物理步进 num_steps 次 (每次 timestep 秒)."""
        for _ in range(num_steps):
            mujoco.mj_step(self.model, self.data)

    def get_state(self) -> SimState:
        """读取当前 16 维 Canonical 灵巧手关节状态 (角度、速度、力矩)."""
        pos = np.array([self.data.qpos[idx] for idx in self.qpos_indices], dtype=np.float64)
        vel = np.array([self.data.qvel[idx] for idx in self.qvel_indices], dtype=np.float64)
        eff = np.array([self.data.actuator_force[idx] for idx in self.actuator_ids], dtype=np.float64)
        return SimState(
            time=float(self.data.time),
            positions=pos,
            velocities=vel,
            efforts=eff,
        )

    def get_arm_state(self) -> Optional[ArmSimState]:
        """读取当前 6 维 Canonical 机械臂关节状态 (角度、速度、力矩)."""
        if not self.has_arm:
            return None
        pos = np.array([self.data.qpos[idx] for idx in self.arm_qpos_indices], dtype=np.float64)
        vel = np.array([self.data.qvel[idx] for idx in self.arm_qvel_indices], dtype=np.float64)
        eff = np.array([self.data.actuator_force[idx] for idx in self.arm_actuator_ids], dtype=np.float64)
        return ArmSimState(
            time=float(self.data.time),
            positions=pos,
            velocities=vel,
            efforts=eff,
        )

    def reset(self) -> None:
        """重置仿真回到基准姿态 (灵巧手全开 0 位, 机械臂复位至 Ready 姿态)."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0.0
        if self.has_arm:
            for i, q_idx in enumerate(self.arm_qpos_indices):
                self.data.qpos[q_idx] = float(DEFAULT_READY_ARM_Q[i])
            for i, act_id in enumerate(self.arm_actuator_ids):
                self.data.ctrl[act_id] = float(DEFAULT_READY_ARM_Q[i])
        mujoco.mj_forward(self.model, self.data)
