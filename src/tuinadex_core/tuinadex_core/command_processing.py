"""command_processing.py — /hand/joint_command 报文校验与 Canonical 重排 (纯逻辑).

拒绝条件 (安全优先, 宁拒勿猜):
  - 维度 != 16
  - 含 NaN / Inf (脏值直通硬件 = 越界风险)
  - names 数量 != 16 或含未知关节名 (静默猜测对齐 = 错位风险)
安全护栏: 合法但越界(相对 Spec §1 逐关节限位)的指令被夹到边界, 绝不把越界弧度写进电机路径。
"""
from __future__ import annotations
from typing import Optional, Sequence, Tuple
import numpy as np

from .names import CANONICAL_JOINT_NAMES


# 规范逐关节关节空间限位 (rad, 相对全开位 0) — 与 LEAP_HAND_MVP_INTERFACE_SPEC §1 一致。
# 唯一事实源为 Spec §1; 此常量是电机路径的安全护栏 (clamp 门),
# 与离线 dataset.validator.SPEC_JOINT_LIMITS 同值且必须保持同步。
JOINT_SPEC_LIMITS = np.array([
    [-0.35, 0.35], [0.0, 2.00], [0.0, 1.85], [0.0, 1.80],   # index
    [-0.35, 0.35], [0.0, 2.00], [0.0, 1.85], [0.0, 1.80],   # middle
    [-0.35, 0.35], [0.0, 2.00], [0.0, 1.85], [0.0, 1.80],   # ring
    [-0.30, 1.80], [0.0, 1.60], [0.0, 1.80], [0.0, 1.80],   # thumb
], dtype=np.float64)


def align_command(
    positions: Sequence[float],
    names: Optional[Sequence[str]] = None,
) -> Tuple[Optional[np.ndarray], str]:
    """校验、重排为 16 维 Canonical 目标弧度, 并夹取到逐关节规范限位."""
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
    # 安全护栏: 越界指令被夹到 Spec §1 逐关节限位边沿 (绝不把越界弧度传给电机)。
    return np.clip(q, JOINT_SPEC_LIMITS[:, 0], JOINT_SPEC_LIMITS[:, 1]), ""
