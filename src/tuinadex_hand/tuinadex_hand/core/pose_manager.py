"""Pose management, validation, persistence and encoder unwrapping for LEAP Hand.

遵循 TuinaDex 黄金不变量:
1. SI Radian 弧度单位契约;
2. 保持 OPEN_POSE 作为基准零位姿态管理;
3. 智能多级配置文件查找 (工作区源码优先 -> ament_index share -> 回退默认).
"""
import json
import os
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

# ─── 默认实测姿势 (LEAP 角度, 单位 rad) ──────────────────────────
DEFAULT_POSES = {
    "全开/平伸": np.array([3.1155, 4.6204, 3.2076, 1.5785,
                          3.2413, 3.0618, 3.1170, 4.5927,
                          3.1186, 3.0756, 3.1799, 4.6004,
                          3.1186, 1.5555, 4.6234, 4.7247], dtype=np.float64),

    "半握": np.array([3.1094, 4.5590, 1.8132, -0.0614,
                     1.7579, 3.0434, 3.0588,  3.1140,
                     3.1155, 3.0726, 1.8362,  2.8685,
                     3.1462, 1.5110, 4.5421,  2.7811], dtype=np.float64),

    "全握拳": np.array([3.0910, 4.0574, 1.3775, -0.1319,
                       1.4711, 2.4636, 3.0940,  2.8624,
                       3.1155, 2.3869, 1.4603,  2.8808,
                       3.5343, 1.4972, 3.2122,  2.7826], dtype=np.float64),

    "食指指": np.array([3.1201, 4.7968, 3.1845, 1.5463,
                       1.4711, 2.3761, 3.0756, 2.8655,
                       3.1462, 2.3853, 1.4619, 2.8808,
                       3.5159, 1.4174, 3.1983, 2.7826], dtype=np.float64),

    "比耶": np.array([2.8624, 4.8198, 3.1845, 1.5463,
                     2.9974, 3.2628, 3.3441, 4.8167,
                     3.1447, 2.0080, 1.5125, 2.8532,
                     3.5205, 1.4281, 3.2137, 2.7826], dtype=np.float64),

    "OK手势": np.array([3.1416, 3.4975, 1.9282, -0.0383,
                       2.9836, 3.2290, 3.0634,  4.9640,
                       3.0971, 2.9943, 3.4284,  4.6111,
                       2.8103, 1.1704, 3.3119,  2.7811], dtype=np.float64),

    "竖拇指": np.array([3.1401, 3.5113, 1.7135, -0.1120,
                       1.3990, 2.0862, 3.0250,  3.4775,
                       3.1431, 1.9957, 1.6674,  3.2306,
                       3.0542, -0.0138, 4.7277, 5.4487], dtype=np.float64),

    "揉法": np.array([3.1140, 4.7017, 2.8363, -0.0337,
                     3.0971, 3.1769, 1.2609,  3.1401,
                     3.1692, 3.2413, 1.2149,  3.1953,
                     3.1416, 4.4992, 1.7073,  4.2721], dtype=np.float64),
}

OPEN_POSE_DEFAULT = DEFAULT_POSES["全开/平伸"]


def get_default_config_path(filename: str = "poses.json") -> Path:
    """按优先级寻找配置文件的绝对路径.

    优先级:
    1. 环境变量 TUINADEX_HAND_CONFIG_DIR
    2. 源码工作区: <repo>/src/tuinadex_hand/config/<filename>
    3. ROS2 share 目录 (通过 ament_index_python)
    4. 包内相对路径
    """
    env_dir = os.environ.get("TUINADEX_HAND_CONFIG_DIR")
    if env_dir:
        p = Path(env_dir) / filename
        if p.exists():
            return p

    # 源码工作区优先，确保 calibrate 工具直接写入代码仓库
    source_candidate = Path(__file__).resolve().parents[2] / "config" / filename
    if source_candidate.exists():
        return source_candidate

    # 尝试 ROS2 ament_index_python
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("tuinadex_hand")) / "config"
        share_candidate = share_dir / filename
        if share_candidate.exists():
            return share_candidate
    except Exception:
        pass

    # 回退到源码候选路径（即使当前不存在也可以用于后续创建写入）
    return source_candidate


def is_valid_pose(pose: np.ndarray) -> bool:
    """判断一组 LEAP 角度是否为真实电机读数 (排除读取失败回退的全零/常数/越界)."""
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (16,):
        return False
    if not np.all(np.isfinite(pose)):
        return False
    # 读取失败回退特征: 全零 或 16 个值完全相同
    if np.all(np.abs(pose) < 1e-6) or np.ptp(pose) < 1e-6:
        return False
    # 真实手可动作范围: 依据实测 motor_limits 全局边界
    if pose.min() < -2.5 or pose.max() > 8.5:
        return False
    return True


def unwrap_to_limits(pos: np.ndarray, limits_path: Optional[Path] = None) -> np.ndarray:
    """把读取到的 16 关节值对齐到实测限位圈 (消除编码器跨 0 点回绕).

    锚点 = 每关节 motor_limits 中点: 读数若与锚点相差 > π, 平移 ±2π 归位.
    """
    if limits_path is None:
        limits_path = get_default_config_path("motor_limits.json")

    if limits_path.exists():
        try:
            with open(limits_path, "r", encoding="utf-8") as f:
                lim = json.load(f)
            mid = (np.array(lim["min"], dtype=float) + np.array(lim["max"], dtype=float)) / 2.0
        except Exception:
            mid = np.array(pos, dtype=float)
    else:
        mid = np.array(pos, dtype=float)

    out = np.array(pos, dtype=float).copy()
    for i in range(16):
        diff = out[i] - mid[i]
        while diff > np.pi:
            out[i] -= 2.0 * np.pi
            diff -= 2.0 * np.pi
        while diff < -np.pi:
            out[i] += 2.0 * np.pi
            diff += 2.0 * np.pi
    return out


def load_poses(config_file: Optional[Path] = None) -> Tuple[Dict[str, np.ndarray], bool]:
    """加载姿势字典 (优先文件, 回退默认值). 返回 (poses_dict, is_valid)."""
    poses = {k: v.copy() for k, v in DEFAULT_POSES.items()}
    if config_file is None:
        config_file = get_default_config_path("poses.json")

    valid = True
    if config_file.exists():
        try:
            with open(config_file, "r", encoding="utf-8") as f:
                loaded = json.load(f)
            for k, v in loaded.items():
                poses[k] = np.array(v, dtype=np.float64)
            open_pose = poses.get("全开/平伸", OPEN_POSE_DEFAULT)
            if not is_valid_pose(open_pose):
                valid = False
        except Exception:
            valid = False
    return poses, valid


def save_poses(poses: Dict[str, np.ndarray], config_file: Optional[Path] = None) -> Path:
    """保存姿势字典到 JSON 文件."""
    if config_file is None:
        config_file = get_default_config_path("poses.json")
    config_file.parent.mkdir(parents=True, exist_ok=True)
    serializable = {
        k: [round(float(x), 4) for x in v] if isinstance(v, (np.ndarray, list)) else v
        for k, v in poses.items()
    }
    with open(config_file, "w", encoding="utf-8") as f:
        json.dump(serializable, f, indent=2, ensure_ascii=False)
    return config_file

