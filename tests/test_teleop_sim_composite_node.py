"""test_teleop_sim_composite_node.py — Integration test for TeleopSimCompositeNode.

Verifies:
1. Node initializes properly with mock camera, mock IMU, and headless simulation.
2. Node publishes target commands on /arm/joint_command (6-DOF) and /hand/joint_command (16-DOF).
3. In sim mode, node publishes simulation feedback on /arm/joint_states and /hand/joint_states.
4. Message payloads strictly conform to TuinaDex canonical names and valid radian arrays.
"""
from __future__ import annotations

import time
import numpy as np
import pytest

import rclpy
from sensor_msgs.msg import JointState

from tuinadex_core.names import CANONICAL_ARM_JOINT_NAMES, CANONICAL_JOINT_NAMES
from tuinadex_teleop.teleop_sim_composite_node import TeleopSimCompositeNode


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_teleop_sim_composite_node_mock_lifecycle(rclpy_init):
    """Test full loopback execution of TeleopSimCompositeNode in mock mode."""
    from rclpy.parameter import Parameter
    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("mock_imu", Parameter.Type.BOOL, True),
        Parameter("auto_engage", Parameter.Type.BOOL, True),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 50.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)

    arm_cmd_received = []
    hand_cmd_received = []
    arm_state_received = []
    hand_state_received = []

    sub_node = rclpy.create_node("teleop_sim_test_sub")

    def _arm_cmd_cb(msg: JointState):
        arm_cmd_received.append(msg)

    def _hand_cmd_cb(msg: JointState):
        hand_cmd_received.append(msg)

    def _arm_state_cb(msg: JointState):
        arm_state_received.append(msg)

    def _hand_state_cb(msg: JointState):
        hand_state_received.append(msg)

    sub_node.create_subscription(JointState, "/arm/joint_command", _arm_cmd_cb, 10)
    sub_node.create_subscription(JointState, "/hand/joint_command", _hand_cmd_cb, 10)
    sub_node.create_subscription(JointState, "/arm/joint_states", _arm_state_cb, 10)
    sub_node.create_subscription(JointState, "/hand/joint_states", _hand_state_cb, 10)

    try:
        # Step node for 15 cycles
        for _ in range(15):
            node._loop_step()
            rclpy.spin_once(sub_node, timeout_sec=0.01)
            time.sleep(0.01)

        # Verify command publication
        assert len(arm_cmd_received) > 0, "No /arm/joint_command received"
        assert len(hand_cmd_received) > 0, "No /hand/joint_command received"

        latest_arm_cmd = arm_cmd_received[-1]
        assert list(latest_arm_cmd.name) == list(CANONICAL_ARM_JOINT_NAMES)
        assert len(latest_arm_cmd.position) == 6
        assert np.all(np.isfinite(latest_arm_cmd.position))

        latest_hand_cmd = hand_cmd_received[-1]
        assert list(latest_hand_cmd.name) == list(CANONICAL_JOINT_NAMES)
        assert len(latest_hand_cmd.position) == 16
        assert np.all(np.isfinite(latest_hand_cmd.position))

        # Verify simulation state publication
        if node.use_sim:
            assert len(arm_state_received) > 0, "No /arm/joint_states received"
            assert len(hand_state_received) > 0, "No /hand/joint_states received"

            latest_arm_state = arm_state_received[-1]
            assert list(latest_arm_state.name) == list(CANONICAL_ARM_JOINT_NAMES)
            assert len(latest_arm_state.position) == 6
            assert np.all(np.isfinite(latest_arm_state.position))

            latest_hand_state = hand_state_received[-1]
            assert list(latest_hand_state.name) == list(CANONICAL_JOINT_NAMES)
            assert len(latest_hand_state.position) == 16
            assert np.all(np.isfinite(latest_hand_state.position))

    finally:
        node.destroy_node()
        sub_node.destroy_node()
