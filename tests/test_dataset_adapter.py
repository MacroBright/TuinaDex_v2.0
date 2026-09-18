"""test_dataset_adapter.py — 合成 rosbag2 → 对齐 → 校验 → LeRobotDataset 全链 (需 ROS2+lerobot)."""
import pytest
rclpy = pytest.importorskip("rclpy")

import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "dataset") not in sys.path:
    sys.path.insert(0, str(ROOT / "dataset"))

from tuinadex_dataset.adapter import convert_bag_to_episode, save_to_lerobot


@pytest.fixture
def synthetic_bag(tmp_path):
    """用 rosbag2_py 写一段 1 秒合成 bag: state 50Hz 正弦, command 40Hz 方波."""
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState

    bag_dir = str(tmp_path / "synthetic")
    w = SequentialWriter()
    w.open(StorageOptions(uri=bag_dir, storage_id="sqlite3"),
           ConverterOptions("cdr", "cdr"))
    for topic in ("/hand/joint_states", "/hand/joint_command"):
        w.create_topic(TopicMetadata(name=topic, type="sensor_msgs/msg/JointState",
                                     serialization_format="cdr"))

    t0 = 1000  # 固定整秒起点
    for k in range(50):   # state: 50Hz
        m = JointState()
        m.header.stamp.sec = t0 + k // 50
        m.header.stamp.nanosec = (k % 50) * 20_000_000
        m.position = [float(np.sin(0.1 * k))] * 16
        w.write("/hand/joint_states", serialize_message(m),
                m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
    for k in range(40):   # action: 40Hz
        m = JointState()
        m.header.stamp.sec = t0 + k // 40
        m.header.stamp.nanosec = (k % 40) * 25_000_000
        m.position = [float(k % 2)] * 16
        w.write("/hand/joint_command", serialize_message(m),
                m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
    return bag_dir


def test_read_joint_state_bag(synthetic_bag):
    from tuinadex_dataset.bag_reader import read_joint_state_bag
    ts, pos = read_joint_state_bag(synthetic_bag, "/hand/joint_states")
    assert ts.shape == (50,) and pos.shape == (50, 16)
    assert np.all(np.diff(ts) > 0)     # header.stamp 严格单调
    assert abs(ts[0] - 1000.0) < 1e-6


def test_convert_and_validate(synthetic_bag):
    grid, states, actions, ok, errors = convert_bag_to_episode(synthetic_bag)
    assert states.shape == (len(grid), 16) and actions.shape == (len(grid), 16)
    dt = np.diff(grid)
    np.testing.assert_allclose(dt, 0.02, atol=1e-5)
    assert ok is True, errors   # 合成数据在规范限位内


def test_save_to_lerobot(synthetic_bag, tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    grid, states, actions, ok, _ = convert_bag_to_episode(synthetic_bag)
    assert ok
    out = save_to_lerobot(grid, states, actions,
                          repo_id=str(tmp_path / "tuina_synthetic"), fps=50)
    assert Path(out).exists()
    ds = LeRobotDataset(Path(out).name, root=out)
    assert ds.meta.total_frames == len(grid)
    frame = ds[0]
    assert frame["observation.state"].shape == (16,)
    assert frame["action"].shape == (16,)
