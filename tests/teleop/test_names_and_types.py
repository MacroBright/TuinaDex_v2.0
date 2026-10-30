"""Unit tests for canonical naming, data contracts, and shape validation."""
import numpy as np
import pytest

from tuinadex_teleop.core.names import (
    CANONICAL_HAND_JOINT_NAMES,
    CANONICAL_ARM_JOINT_NAMES,
    CANONICAL_COMPOSITE_JOINT_NAMES,
    ARM_NUM_DOFS,
    HAND_NUM_DOFS,
    COMPOSITE_NUM_DOFS,
)
from tuinadex_teleop.core.types import (
    ClutchState,
    TeleopMode,
    TeleopArmCommand,
    TeleopHandCommand,
    TeleopCommand,
    RobotObservation,
    ArUcoCalibrationData,
)


def test_canonical_joint_dimensions():
    assert len(CANONICAL_ARM_JOINT_NAMES) == 6
    assert len(CANONICAL_HAND_JOINT_NAMES) == 16
    assert len(CANONICAL_COMPOSITE_JOINT_NAMES) == 22
    assert ARM_NUM_DOFS == 6
    assert HAND_NUM_DOFS == 16
    assert COMPOSITE_NUM_DOFS == 22

    # Verify canonical hand order starts with index and ends with thumb
    assert CANONICAL_HAND_JOINT_NAMES[0] == "index_yaw"
    assert CANONICAL_HAND_JOINT_NAMES[-1] == "thumb_ip"

    # Verify canonical arm order starts with joint1 and ends with joint6
    assert CANONICAL_ARM_JOINT_NAMES[0] == "joint1"
    assert CANONICAL_ARM_JOINT_NAMES[-1] == "joint6"


def test_teleop_arm_command_validation():
    pose = np.eye(4)
    q = np.zeros(6)
    cmd = TeleopArmCommand(timestamp=1.0, target_pose=pose, target_q=q)
    assert cmd.clutch_state == ClutchState.DISENGAGED

    # Invalid pose shape
    with pytest.raises(ValueError, match="target_pose must be"):
        TeleopArmCommand(timestamp=1.0, target_pose=np.zeros((3, 3)), target_q=q)

    # Invalid joint shape
    with pytest.raises(ValueError, match="target_q must be"):
        TeleopArmCommand(timestamp=1.0, target_pose=pose, target_q=np.zeros(7))


def test_teleop_hand_command_validation():
    q = np.zeros(16)
    cmd = TeleopHandCommand(timestamp=1.0, target_q=q, confidence=0.95)
    assert cmd.confidence == 0.95
    assert cmd.is_clutched is True

    # Invalid joint shape
    with pytest.raises(ValueError, match="target_q must be"):
        TeleopHandCommand(timestamp=1.0, target_q=np.zeros(15))


def test_unified_teleop_command_composition():
    arm_cmd = TeleopArmCommand(
        timestamp=1.0,
        target_pose=np.eye(4),
        target_q=np.ones(6) * 0.1,
    )
    hand_cmd = TeleopHandCommand(
        timestamp=1.0,
        target_q=np.ones(16) * 0.2,
    )
    unified = TeleopCommand(
        timestamp=1.0,
        arm_command=arm_cmd,
        hand_command=hand_cmd,
        clutch_state=ClutchState.ENGAGED,
    )
    assert unified.composite_q.shape == (22,)
    np.testing.assert_allclose(unified.composite_q[:6], 0.1)
    np.testing.assert_allclose(unified.composite_q[6:], 0.2)


def test_robot_observation_contracts():
    obs = RobotObservation(
        timestamp=1.0,
        arm_actual_q=np.zeros(6),
        arm_actual_dq=np.zeros(6),
        arm_effort=np.zeros(6),
        hand_actual_q=np.zeros(16),
        hand_current=np.zeros(16),
        is_connected=True,
    )
    assert obs.composite_state.shape == (22,)
    assert obs.is_connected is True

    with pytest.raises(ValueError, match="arm_actual_q must be"):
        RobotObservation(
            timestamp=1.0,
            arm_actual_q=np.zeros(5),
            arm_actual_dq=np.zeros(6),
            arm_effort=np.zeros(6),
            hand_actual_q=np.zeros(16),
        )


def test_aruco_calibration_point_transform():
    calib = ArUcoCalibrationData(tag_id=0, is_valid=True)
    # Set t_table_to_cam as translation by [0, 0, -0.5]
    t = np.eye(4)
    t[2, 3] = -0.5
    calib.t_table_to_cam = t

    point_cam = np.array([0.1, 0.2, 0.8])
    point_table = calib.transform_point_to_table(point_cam)
    np.testing.assert_allclose(point_table, [0.1, 0.2, 0.3])

