"""validator.py — 示教数据集质量与物理安全验证器."""
from __future__ import annotations

from typing import List, Tuple
import numpy as np


# 规范逐关节限位 (rad, 相对全开位 0) — 与 LEAP_HAND_MVP_INTERFACE_SPEC §1 一致
SPEC_JOINT_LIMITS = (
    (-0.35, 0.35), (0.0, 2.00), (0.0, 1.85), (0.0, 1.80),   # index
    (-0.35, 0.35), (0.0, 2.00), (0.0, 1.85), (0.0, 1.80),   # middle
    (-0.35, 0.35), (0.0, 2.00), (0.0, 1.85), (0.0, 1.80),   # ring
    (-0.30, 1.80), (0.0, 1.60), (0.0, 1.80), (0.0, 1.80),   # thumb
)


class DatasetValidator:
    """检查导出的示教帧是否存在 NaN、越界或异常跳变."""

    def __init__(
        self,
        joint_limits=None,
        max_single_step_delta: float = 0.50,
    ):
        self.joint_limits = tuple(joint_limits) if joint_limits is not None else SPEC_JOINT_LIMITS
        self._lo = np.array([lo for lo, _ in self.joint_limits])
        self._hi = np.array([hi for _, hi in self.joint_limits])
        self.max_delta = max_single_step_delta

    def validate_episode(
        self,
        timestamps: np.ndarray,
        states: np.ndarray,
        actions: np.ndarray,
    ) -> Tuple[bool, List[str]]:
        """检验单个 Episode 数据的完整性与物理合理性."""
        errors = []

        # 1. 检查 NaN / Inf
        if not np.all(np.isfinite(states)):
            errors.append("State 包含 NaN 或 Inf 异常值")
        if not np.all(np.isfinite(actions)):
            errors.append("Action 包含 NaN 或 Inf 异常值")

        # 2. 检查时间戳严格单调递增
        dt = np.diff(timestamps)
        if np.any(dt <= 0):
            errors.append("时间戳存在非单调递增或回退")

        # 3. 检查物理关节限位有界性
        if np.any(states < self._lo) or np.any(states > self._hi):
            errors.append("State 存在超出规范逐关节限位的数值")
        if np.any(actions < self._lo) or np.any(actions > self._hi):
            errors.append("Action 存在超出规范逐关节限位的数值")

        # 4. 检查单步剧烈跳变
        state_delta = np.abs(np.diff(states, axis=0))
        if np.any(state_delta > self.max_delta):
            errors.append(f"State 存在单步瞬时跳变超过 {self.max_delta} rad 的飞车异常")

        is_valid = len(errors) == 0
        return is_valid, errors
