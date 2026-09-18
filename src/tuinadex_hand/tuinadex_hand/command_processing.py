"""command_processing.py — /hand/joint_command 报文校验与 Canonical 重排 (纯逻辑).

拒绝条件 (安全优先, 宁拒勿猜):
  - 维度 != 16
  - 含 NaN / Inf (脏值直通硬件 = 越界风险)
  - names 数量 != 16 或含未知关节名 (静默猜测对齐 = 错位风险)
"""
from __future__ import annotations
from typing import Optional, Sequence, Tuple
import numpy as np

from .driver.leap_driver import CANONICAL_JOINT_NAMES


def align_command(
    positions: Sequence[float],
    names: Optional[Sequence[str]] = None,
) -> Tuple[Optional[np.ndarray], str]:
    """校验并重排为 16 维 Canonical 目标弧度."""
    pos = np.asarray(list(positions), dtype=np.float64)
    if pos.shape != (16,):
        return None, f"dim_mismatch: expected 16, got {pos.shape[0]}"

    if not np.all(np.isfinite(pos)):
        bad = np.nonzero(~np.isfinite(pos))[0].tolist()
        return None, f"non_finite at indices {bad}"

    q = pos
    if names:
        if len(names) != 16:
            return None, f"name_dim_mismatch: expected 16 names, got {len(names)}"
        if list(names) != CANONICAL_JOINT_NAMES:
            name_to_idx = {n: i for i, n in enumerate(CANONICAL_JOINT_NAMES)}
            unknown = [n for n in names if n not in name_to_idx]
            if unknown:
                return None, f"unknown_joint_names: {unknown}"
            ordered = np.zeros(16, dtype=np.float64)
            for src, n in enumerate(names):
                ordered[name_to_idx[n]] = pos[src]
            q = ordered
    return q, ""
