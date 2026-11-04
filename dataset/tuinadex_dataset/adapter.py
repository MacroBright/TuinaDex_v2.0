"""adapter.py — rosbag2 → 50Hz 对齐 → 规范校验 → LeRobotDataset 离线转换."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .synchronizer import TimeSynchronizer
from .validator import DatasetValidator

# 与 Spec §1 / leap_driver 一致的 16 关节名 (纯离线模块, 独立声明不 import ROS 包)
CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]


def convert_bag_to_episode(bag_dir, fps: float = 50.0):
    """读取 state/action 双话题 -> 网格对齐 -> 规范校验."""
    # Lazy import keeps save_to_lerobot usable in the LeRobot-only conda env.
    from .bag_reader import read_joint_state_bag

    s_ts, s_pos = read_joint_state_bag(bag_dir, "/hand/joint_states")
    a_ts, a_pos = read_joint_state_bag(bag_dir, "/hand/joint_command")
    grid, states, actions = TimeSynchronizer(target_fps=fps).align_series(
        s_ts, s_pos, a_ts, a_pos)
    ok, errors = DatasetValidator().validate_episode(grid, states, actions)
    return grid, states, actions, ok, errors


def save_to_lerobot(grid, states, actions, repo_id: str, fps: float = 50.0) -> str:
    """写入 LeRobotDataset (0.4.4 add_frame/save_episode API), 返回数据集根路径."""
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = {
        "observation.state": {
            "dtype": "float32", "shape": (16,),
            "names": {"joints": list(CANONICAL_JOINT_NAMES)}},
        "action": {
            "dtype": "float32", "shape": (16,),
            "names": {"joints": list(CANONICAL_JOINT_NAMES)}},
    }
    output_root = Path(repo_id).resolve()
    info_path = output_root / "meta" / "info.json"
    if output_root.exists() and not info_path.exists():
        import shutil
        shutil.rmtree(output_root)

    dataset = LeRobotDataset.create(
        repo_id=output_root.name,
        fps=int(fps),
        features=features,
        root=output_root,
        use_videos=False,
    )
    for i in range(len(grid)):
        dataset.add_frame(
            frame={
                "observation.state": torch.from_numpy(states[i].astype(np.float32)),
                "action": torch.from_numpy(actions[i].astype(np.float32)),
                "task": "teleoperation",
            })
    dataset.save_episode()
    return str(output_root)
