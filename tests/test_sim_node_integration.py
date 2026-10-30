"""test_sim_node_integration.py — ROS2 loop tests for LeapHandSimNode."""
import sys
import time
from pathlib import Path

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

ROOT = Path(__file__).resolve().parents[1]
for package in ("tuinadex_core", "tuinadex_sim"):
    path = str(ROOT / "src" / package)
    if path not in sys.path:
        sys.path.insert(0, path)

from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from tuinadex_core.names import CANONICAL_JOINT_NAMES
from tuinadex_sim.leap_hand_sim_node import LeapHandSimNode


class FakeBridge:
    def __init__(self):
        self._positions = np.zeros(16, dtype=np.float64)
        self._target = np.zeros(16, dtype=np.float64)
        self.steps = 0
        self.time = 0.0

    def set_joint_targets(self, targets):
        self._target = np.asarray(targets, dtype=np.float64).copy()

    def step(self, num_steps):
        self.steps += num_steps
        self.time += num_steps * 0.002
        self._positions = self._target.copy()

    def get_state(self):
        from tuinadex_sim.sim_bridge import SimState

        return SimState(
            time=self.time,
            positions=self._positions.copy(),
            velocities=np.zeros(16, dtype=np.float64),
            efforts=np.zeros(16, dtype=np.float64),
        )


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def in_limit_command(flexion=0.8):
    return [0.2 if index in (0, 4, 8) else flexion for index in range(16)]


def spin_until(executor, predicate, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.005)
        if predicate():
            return True
    return False


def test_command_follows_relaxes_and_publishes_at_50hz(ros_context):
    node = LeapHandSimNode(
        parameter_overrides={
            "publish_rate_hz": 50.0,
            "watchdog_timeout_sec": 0.30,
            "watchdog_hold_sec": 0.30,
            "watchdog_ramp_sec": 0.60,
            "physics_substeps": 10,
        },
        bridge=FakeBridge(),
    )
    helper = rclpy.create_node("sim_node_test_helper")
    received = []
    helper.create_subscription(
        JointState,
        "/hand/joint_states",
        lambda msg: received.append(msg),
        10,
    )
    publisher = helper.create_publisher(JointState, "/hand/joint_command", 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)

    try:
        command = JointState()
        command.header.stamp = helper.get_clock().now().to_msg()
        command.name = list(CANONICAL_JOINT_NAMES)
        command.position = in_limit_command()

        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            command.header.stamp = helper.get_clock().now().to_msg()
            publisher.publish(command)
            executor.spin_once(timeout_sec=0.005)

        assert received
        last = np.asarray(received[-1].position)
        np.testing.assert_allclose(last, in_limit_command(), atol=1e-6)

        stamps = np.asarray([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for msg in received[-20:]
        ])
        intervals = np.diff(stamps)
        assert intervals.size >= 5
        assert float(np.median(intervals)) == pytest.approx(0.02, abs=0.005)

        spin_until(
            executor,
            lambda: received and np.allclose(received[-1].position, 0.0, atol=1e-6),
            timeout_sec=1.8,
        )
        np.testing.assert_allclose(received[-1].position, 0.0, atol=1e-6)
    finally:
        executor.remove_node(node)
        executor.remove_node(helper)
        node.destroy_node()
        helper.destroy_node()


def test_invalid_command_does_not_poison_state(ros_context):
    node = LeapHandSimNode(
        parameter_overrides={"publish_rate_hz": 50.0, "physics_substeps": 10},
        bridge=FakeBridge(),
    )
    helper = rclpy.create_node("sim_node_invalid_test_helper")
    received = []
    helper.create_subscription(
        JointState,
        "/hand/joint_states",
        lambda msg: received.append(np.asarray(msg.position)),
        10,
    )
    publisher = helper.create_publisher(JointState, "/hand/joint_command", 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)

    try:
        invalid = JointState()
        invalid.name = list(CANONICAL_JOINT_NAMES)
        invalid.position = [0.8] * 15
        publisher.publish(invalid)
        spin_until(executor, lambda: len(received) > 5, timeout_sec=0.3)

        assert received
        assert np.all(np.isfinite(received[-1]))
        np.testing.assert_allclose(received[-1], 0.0, atol=1e-12)
    finally:
        executor.remove_node(node)
        executor.remove_node(helper)
        node.destroy_node()
        helper.destroy_node()


def test_gui_parameter_defaults_false_and_handles_fake_bridge(ros_context):
    node = LeapHandSimNode(
        parameter_overrides={"gui": False},
        bridge=FakeBridge(),
    )
    try:
        assert node.gui_enabled is False
        assert node.viewer is None
    finally:
        node.destroy_node()


def test_arm_dual_publish_competition_prevention(ros_context):
    """验证机械臂双发话题竞争防护: 默认 publish_arm_states=False 时不与 ArmNode 冲突."""
    class FakeArmBridge(FakeBridge):
        def __init__(self):
            super().__init__()
            self.has_arm = True
            self.arm_targets = np.zeros(6, dtype=np.float64)

        def set_arm_targets(self, targets):
            self.arm_targets = np.asarray(targets, dtype=np.float64).copy()

    # 1. 默认配置: publish_arm_states=False，防止与 ArmNode 双发冲突
    default_node = LeapHandSimNode(
        parameter_overrides={"gui": False, "publish_arm_states": False},
        bridge=FakeArmBridge(),
    )
    try:
        assert default_node.arm_state_pub is None, "默认不得创建 arm_state_pub，防止话题双发"
        assert default_node.arm_sync_sub is not None, "应创建 arm_sync_sub 用于同步姿态"
    finally:
        default_node.destroy_node()

    # 2. 独立运行配置: publish_arm_states=True，允许接管机械臂发布
    standalone_node = LeapHandSimNode(
        parameter_overrides={"gui": False, "publish_arm_states": True},
        bridge=FakeArmBridge(),
    )
    try:
        assert standalone_node.arm_state_pub is not None, "独立模式下应创建 arm_state_pub"
        assert standalone_node.arm_cmd_sub is not None, "独立模式下应创建 arm_cmd_sub"
    finally:
        standalone_node.destroy_node()

