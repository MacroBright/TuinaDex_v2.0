"""test_leap_hand_node_integration.py — mock 模式下节点级闭环验证 (需 ROS2)."""
import sys
from pathlib import Path
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

WORKSPACE_SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_hand"
if str(WORKSPACE_SRC) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_SRC))

from sensor_msgs.msg import JointState
from tuinadex_hand.leap_hand_node import LeapHandNode


@pytest.fixture
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_mock_loop_and_watchdog(rclpy_ctx):
    """发布指令后 joint_states 跟随; 停发后看门狗平滑回 0 rad."""
    overrides = dict(mock_mode=True, watchdog_timeout_sec=0.10,
                     watchdog_hold_sec=0.10, watchdog_ramp_sec=0.20)
    node = LeapHandNode(parameter_overrides=overrides)

    received = []
    node.create_subscription(
        JointState, "/hand/joint_states",
        lambda m: received.append(np.asarray(m.position)), 10)

    # 1) 正常跟随: 持续发 0.8 rad 指令
    pub = node.create_publisher(JointState, "/hand/joint_command", 10)
    import time as _time
    cmd = JointState(); cmd.position = [0.8] * 16
    t_end = _time.monotonic() + 0.4
    while _time.monotonic() < t_end:
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.02)
    assert received and np.allclose(received[-1], 0.8, atol=1e-6)

    # 2) 停发指令 -> 看门狗 (0.1+0.1+0.2s) 平滑回 0
    t_end = _time.monotonic() + 1.0
    while _time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)
    assert np.allclose(received[-1], 0.0, atol=1e-3)
    node.destroy_node()
