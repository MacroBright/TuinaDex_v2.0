"""validator.py — 示教数据集质量与物理安全验证器."""
from __future__ import annotations

from typing import List, Tuple
import numpy as np


class DatasetValidator:
    """检查导出的示教帧是否存在 NaN、越界或异常跳变."""

    def __init__(
        self,
        joint_limits_min: float = -0.50,
        joint_limits_max: float = 2.50,
        max_single_step_delta: float = 0.50,
    ):
        self.limits_min = joint_limits_min
        self.limits_max = joint_limits_max
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
        if np.any(states < self.limits_min) or np.any(states > self.limits_max):
            errors.append(f"State 存在超出限位区间 [{self.limits_min}, {self.limits_max}] 的数值")
        if np.any(actions < self.limits_min) or np.any(actions > self.limits_max):
            errors.append(f"Action 存在超出限位区间 [{self.limits_min}, {self.limits_max}] 的数值")

        # 4. 检查单步剧烈跳变
        state_delta = np.abs(np.diff(states, axis=0))
        if np.any(state_delta > self.max_delta):
            errors.append(f"State 存在单步瞬时跳变超过 {self.max_delta} rad 的飞车异常")

        is_valid = len(errors) == 0
        return is_valid, errors
