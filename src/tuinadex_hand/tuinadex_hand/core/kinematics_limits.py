"""kinematics_limits.py — LEAP Hand 运动学角度限位、安全截断与仿真/真机角度转换."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Optional, Tuple
import numpy as np

from .pose_manager import get_default_config_path

# 实测真机舵机绝对弧度全局安全极限 (兜底边界)
DEFAULT_DXL_LIMITS_MIN = np.array([
    2.3517, 3.3196, 2.3133, -0.5706,
    2.5772, 1.7795, 0.7609,  2.6063,
    2.5787, 1.4021, 0.7149,  2.4775,
    2.5986, 2.8610, 1.2073,  2.2826
], dtype=np.float64)

DEFAULT_DXL_LIMITS_MAX = np.array([
    3.6232, 5.4287, 5.2400,  2.1107,
    3.8211, 3.7321, 3.7827,  5.2461,
    4.1202, 3.7413, 3.7873,  5.1832,
    4.8274, 5.0958, 4.1263,  6.2095
], dtype=np.float64)


def LEAPsim_limits(type: str = "regular") -> Tuple[np.ndarray, np.ndarray]:
    """返回 LEAP 仿真关节范围 (min, max)."""
    sim_min = np.array([
        -1.047, -0.314, -0.506, -0.366,
        -1.047, -0.314, -0.506, -0.366,
        -1.047, -0.314, -0.506, -0.366,
        -0.349, -0.470, -1.200, -1.340,
    ], dtype=np.float64)
    sim_max = np.array([
         1.047,  2.230,  1.885,  2.042,
         1.047,  2.230,  1.885,  2.042,
         1.047,  2.230,  1.885,  2.042,
         2.094,  2.443,  1.900,  1.880,
    ], dtype=np.float64)
    return sim_min, sim_max


def LEAPsim_to_LEAPhand(joints: np.ndarray) -> np.ndarray:
    """仿真角度转硬件实际绝对角度 (理论零位平移)."""
    return np.asarray(joints, dtype=np.float64) + np.pi


def LEAPhand_to_LEAPsim(joints: np.ndarray) -> np.ndarray:
    """硬件实际绝对角度转仿真角度 (理论零位平移)."""
    return np.asarray(joints, dtype=np.float64) - np.pi


def angle_safety_clip(joints: np.ndarray, limits_file: Optional[Path] = None) -> np.ndarray:
    """对 16 关节舵机绝对弧度执行安全截断，依据实测 motor_limits 物理边界防止硬件损坏."""
    target = np.asarray(joints, dtype=np.float64)
    path = limits_file if limits_file is not None else get_default_config_path("motor_limits.json")
    if path.exists():
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            min_lim = np.array(data["min"], dtype=np.float64)
            max_lim = np.array(data["max"], dtype=np.float64)
            return np.clip(target, min_lim, max_lim)
        except Exception:
            pass
    return np.clip(target, DEFAULT_DXL_LIMITS_MIN, DEFAULT_DXL_LIMITS_MAX)
