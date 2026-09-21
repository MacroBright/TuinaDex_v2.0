"""sim_bridge.py — MuJoCo 物理仿真桥接器 (纯 Python / 动力学步进与状态提取).

遵循 Invariants 契约:
1. 关节空间 q: 16 维 SI 弧度 (rad)，0 为全开平伸基准姿势；
2. 物理步长: 默认 0.002s (500 Hz)；
3. Canonical Joint 映射: 严格遵循 CANONICAL_JOINT_NAMES 顺序索引；
4. 纯逻辑架构: 不依赖 rclpy，便于在 pytest 中快速单测与离线验证。
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None

# 与 Spec §1 保持一致的 Canonical 16-DOF Joint 命名
CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]


@dataclass
class SimState:
    time: float
    positions: np.ndarray    # 16 维 (rad)
    velocities: np.ndarray   # 16 维 (rad/s)
    efforts: np.ndarray      # 16 维 (actuator force / torque)


class MujocoSimBridge:
    """MuJoCo 物理动力学桥接器."""

    def __init__(self, xml_path: str):
        if mujoco is None:
            raise RuntimeError("mujoco package is not installed.")
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"MuJoCo XML model file not found: {xml_path}")

        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # 建立 Canonical Joint 映射
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

        # 初始步进一次，确保系统状态一致
        mujoco.mj_forward(self.model, self.data)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def set_joint_targets(self, targets: Sequence[float]) -> None:
        """设置 16 维 Canonical 关节的目标控制弧度."""
        if len(targets) != 16:
            raise ValueError(f"Expected 16 joint targets, got {len(targets)}")
        for i, act_id in enumerate(self.actuator_ids):
            self.data.ctrl[act_id] = float(targets[i])

    def step(self, num_steps: int = 1) -> None:
        """前向物理步进 num_steps 次 (每次 timestep 秒)."""
        for _ in range(num_steps):
            mujoco.mj_step(self.model, self.data)

    def get_state(self) -> SimState:
        """读取当前 16 维 Canonical 关节状态 (角度、速度、力)."""
        pos = np.array([self.data.qpos[idx] for idx in self.qpos_indices], dtype=np.float64)
        vel = np.array([self.data.qvel[idx] for idx in self.qvel_indices], dtype=np.float64)
        eff = np.array([self.data.actuator_force[idx] for idx in self.actuator_ids], dtype=np.float64)
        return SimState(
            time=float(self.data.time),
            positions=pos,
            velocities=vel,
            efforts=eff,
        )

    def reset(self) -> None:
        """重置仿真回到全开 0 位."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
