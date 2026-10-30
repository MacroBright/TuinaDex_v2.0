"""config.py — 机器人环境配置与自适应维度规范."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from tuinadex_core.names import CANONICAL_JOINT_NAMES, CANONICAL_ARM_JOINT_NAMES

VALID_MODES = ("hand_only", "arm_hand")


@dataclass(frozen=True)
class RobotConfig:
    """机器人与环境自适应参数配置.
    
    Attributes:
        mode: "hand_only" (16D) 或 "arm_hand" (22D = 6 臂 + 16 手).
        camera_names: 需要采集/渲染的摄像头视角列表 (默认 ("overhead", "wrist", "side")).
        fps: 采集控制与离线对齐目标频率 (Hz, 默认 50.0).
        physics_dt: 物理仿真步长 (默认 0.002s = 500Hz).
    """
    mode: str = "hand_only"
    camera_names: tuple[str, ...] = ("overhead", "wrist", "side")
    fps: float = 50.0
    physics_dt: float = 0.002

    def __init__(
        self,
        mode: str = "hand_only",
        camera_names: Sequence[str] = ("overhead", "wrist", "side"),
        fps: float = 50.0,
        physics_dt: float = 0.002,
    ):
        if mode not in VALID_MODES:
            raise ValueError(f"Invalid mode '{mode}'. Expected one of {VALID_MODES}")
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "camera_names", tuple(camera_names))
        object.__setattr__(self, "fps", float(fps))
        object.__setattr__(self, "physics_dt", float(physics_dt))

    @property
    def joint_names(self) -> tuple[str, ...]:
        """按规范顺序返回当前配置的关节名称列表."""
        if self.mode == "hand_only":
            return tuple(CANONICAL_JOINT_NAMES)
        elif self.mode == "arm_hand":
            return tuple(CANONICAL_ARM_JOINT_NAMES) + tuple(CANONICAL_JOINT_NAMES)
        raise ValueError(f"Unknown mode: {self.mode}")

    @property
    def action_dim(self) -> int:
        """动作空间维度."""
        return len(self.joint_names)

    @property
    def state_dim(self) -> int:
        """状态空间维度 (当前与动作空间对齐)."""
        return len(self.joint_names)
