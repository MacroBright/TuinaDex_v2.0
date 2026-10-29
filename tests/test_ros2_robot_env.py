"""test_ros2_robot_env.py — ROS2RobotEnv 话题桥接与多模态同步单元测试."""
import pytest
import numpy as np
import time

rclpy = pytest.importorskip("rclpy")
from sensor_msgs.msg import JointState, Image

from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs.ros2_robot_env import ROS2RobotEnv


def _make_mock_joint_state(names, positions, t_sec=100.0):
    msg = JointState()
    msg.header.stamp.sec = int(t_sec)
    msg.header.stamp.nanosec = int((t_sec - int(t_sec)) * 1e9)
    msg.name = list(names)
    msg.position = [float(p) for p in positions]
    return msg


def _make_mock_image(h=120, w=160, channels=3, encoding="rgb8", t_sec=100.0):
    msg = Image()
    msg.header.stamp.sec = int(t_sec)
    msg.header.stamp.nanosec = int((t_sec - int(t_sec)) * 1e9)
    msg.height = h
    msg.width = w
    msg.encoding = encoding
    msg.step = w * channels
    # 填充渐变测试图像
    arr = np.arange(h * w * channels, dtype=np.uint8).reshape((h, w, channels))
    msg.data = arr.tobytes()
    return msg, arr


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_ros2_robot_env_hand_only_sync(rclpy_init):
    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    env = ROS2RobotEnv(cfg, node_name="test_env_hand")

    # 1. 模拟收到手部 16 维状态
    hand_names = cfg.joint_names
    hand_positions = np.linspace(0.1, 1.6, 16)
    env._on_hand_state(_make_mock_joint_state(hand_names, hand_positions, t_sec=101.5))

    # 2. 模拟收到 head 相机图像
    img_msg, raw_arr = _make_mock_image(h=120, w=160, encoding="rgb8", t_sec=101.5)
    env._on_image("head", img_msg)

    # 3. 提取观测
    obs = env.get_observation()
    assert obs["state"].shape == (16,)
    np.testing.assert_allclose(obs["state"], hand_positions, atol=1e-5)
    assert "head" in obs["images"]
    assert obs["images"]["head"].shape == (120, 160, 3)
    np.testing.assert_array_equal(obs["images"]["head"], raw_arr)
    assert abs(obs["timestamp"] - 101.5) < 1e-4

    # 4. 执行 step (发布指令)
    action = np.zeros(16, dtype=np.float32)
    next_obs, info = env.step(action)
    assert next_obs["state"].shape == (16,)

    env.close()


def test_ros2_robot_env_arm_hand_sync(rclpy_init):
    cfg = RobotConfig(mode="arm_hand", camera_names=["head", "wrist"])
    env = ROS2RobotEnv(cfg, node_name="test_env_arm_hand")

    arm_names = cfg.joint_names[:6]
    hand_names = cfg.joint_names[6:]

    arm_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    hand_positions = [0.05] * 16

    env._on_arm_state(_make_mock_joint_state(arm_names, arm_positions, t_sec=200.0))
    env._on_hand_state(_make_mock_joint_state(hand_names, hand_positions, t_sec=200.0))

    img_head, _ = _make_mock_image(h=60, w=80, encoding="rgb8", t_sec=200.0)
    img_wrist, _ = _make_mock_image(h=60, w=80, encoding="bgr8", t_sec=200.0)

    env._on_image("head", img_head)
    env._on_image("wrist", img_wrist)

    obs = env.get_observation()
    assert obs["state"].shape == (22,)
    np.testing.assert_allclose(obs["state"][:6], arm_positions)
    np.testing.assert_allclose(obs["state"][6:], hand_positions)
    assert "head" in obs["images"] and "wrist" in obs["images"]

    # 验证动作发布与步进
    act22 = np.zeros(22, dtype=np.float32)
    next_obs, _ = env.step(act22)
    assert next_obs["state"].shape == (22,)

    env.close()
