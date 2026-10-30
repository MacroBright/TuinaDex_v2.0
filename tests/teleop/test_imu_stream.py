"""test_imu_stream.py — Comprehensive unit tests for IMU stream and wrist attitude tracker."""
from __future__ import annotations

import math
import numpy as np
import pytest

from tuinadex_teleop.core.clutch import ClutchController
from tuinadex_teleop.core.types import ClutchState
from tuinadex_teleop.kinematics.arm_ik_solver import (
    ArmIKSolver,
    ArmOrientationMode,
    R_DOWNWARD_DEFAULT,
)
from tuinadex_teleop.perception.imu_stream import (
    ImuReading,
    ImuStream,
    MockImuStream,
    WristImuTracker,
    parse_axis_mapping,
    quat_to_rotation_matrix,
    rotation_matrix_to_euler_deg,
    rotation_matrix_to_quat,
)


def test_quat_and_rotation_matrix_roundtrip():
    """Verify quaternion <-> rotation matrix conversion accuracy and orthonormality."""
    # Identity test
    q_id = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    R_id = quat_to_rotation_matrix(q_id)
    np.testing.assert_allclose(R_id, np.eye(3), atol=1e-7)
    q_rec = rotation_matrix_to_quat(R_id)
    np.testing.assert_allclose(q_rec, q_id, atol=1e-7)

    # 45 deg rotation around X
    theta = math.radians(45.0)
    q_x = np.array([math.cos(theta / 2.0), math.sin(theta / 2.0), 0.0, 0.0], dtype=np.float64)
    R_x = quat_to_rotation_matrix(q_x)
    assert abs(np.linalg.det(R_x) - 1.0) < 1e-6
    q_x_rec = rotation_matrix_to_quat(R_x)
    np.testing.assert_allclose(q_x_rec, q_x, atol=1e-6)

    # 30 deg pitch around Y
    theta_y = math.radians(30.0)
    q_y = np.array([math.cos(theta_y / 2.0), 0.0, math.sin(theta_y / 2.0), 0.0], dtype=np.float64)
    R_y = quat_to_rotation_matrix(q_y)
    assert abs(np.linalg.det(R_y) - 1.0) < 1e-6
    q_y_rec = rotation_matrix_to_quat(R_y)
    np.testing.assert_allclose(q_y_rec, q_y, atol=1e-6)


def test_axis_mapping_parser():
    """Verify axis permutation string parsing into orthonormal matrices."""
    # Identity
    R = parse_axis_mapping("+x,+y,+z")
    np.testing.assert_allclose(R, np.eye(3))

    # 90 deg Yaw (-y, +x, +z)
    R_yaw = parse_axis_mapping("-y,+x,+z")
    expected_yaw = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
    np.testing.assert_allclose(R_yaw, expected_yaw)

    # Inverted Z (+x, +y, -z)
    R_inv_z = parse_axis_mapping("+x,+y,-z")
    expected_inv_z = np.diag([1.0, 1.0, -1.0])
    np.testing.assert_allclose(R_inv_z, expected_inv_z)

    with pytest.raises(ValueError):
        parse_axis_mapping("+x,+y")
    with pytest.raises(ValueError):
        parse_axis_mapping("+x,+y,+w")


def test_mock_imu_stream_lifecycle():
    """Verify MockImuStream initialization, continuous readings, and synthetic data."""
    mock_imu = MockImuStream(publish_rate_hz=50.0)
    assert not mock_imu.is_connected
    assert mock_imu.start()
    assert mock_imu.is_connected

    reading1 = mock_imu.read()
    assert reading1.is_valid
    assert reading1.update_count == 1
    assert reading1.rotation_matrix.shape == (3, 3)
    assert abs(np.linalg.det(reading1.rotation_matrix) - 1.0) < 1e-6

    reading2 = mock_imu.read()
    assert reading2.update_count == 2
    assert reading2.timestamp >= reading1.timestamp

    # Manual Euler injection
    mock_imu.set_synthetic_euler(roll_deg=15.0, pitch_deg=-20.0, yaw_deg=35.0)
    reading_custom = mock_imu.read()
    assert abs(reading_custom.euler_deg[0] - 15.0) < 1e-3
    assert abs(reading_custom.euler_deg[1] - (-20.0)) < 1e-3
    assert abs(reading_custom.euler_deg[2] - 35.0) < 1e-3

    mock_imu.stop()
    assert not mock_imu.is_connected


def test_wrist_imu_tracker_zero_referencing():
    """Verify that zeroing establishing neutral orientation produces identity delta."""
    tracker = WristImuTracker(axis_mapping="+x,+y,+z", neutral_robot_rot=R_DOWNWARD_DEFAULT)
    assert not tracker.is_calibrated

    mock_imu = MockImuStream()
    mock_imu.start()
    mock_imu.set_synthetic_euler(roll_deg=12.0, pitch_deg=8.0, yaw_deg=-15.0)
    reading = mock_imu.read()

    # Zero reference
    tracker.zero(reading)
    assert tracker.is_calibrated

    # Immediately after zeroing, target orientation must exactly match neutral robot posture
    R_tgt = tracker.compute_target_rotation(reading)
    np.testing.assert_allclose(R_tgt, R_DOWNWARD_DEFAULT, atol=1e-5)
    np.testing.assert_allclose(tracker.delta_euler_deg, (0.0, 0.0, 0.0), atol=1e-3)


def test_wrist_imu_tracker_relative_rotation():
    """Verify tracking of relative pitch rotation after zeroing."""
    tracker = WristImuTracker(axis_mapping="+x,+y,+z", neutral_robot_rot=R_DOWNWARD_DEFAULT)
    mock_imu = MockImuStream()
    mock_imu.start()

    # Zero at 0 deg
    mock_imu.set_synthetic_euler(0.0, 0.0, 0.0)
    reading_zero = mock_imu.read()
    tracker.zero(reading_zero)

    # Human wrist pitches up by +20 deg
    mock_imu.set_synthetic_euler(0.0, 20.0, 0.0)
    reading_pitched = mock_imu.read()
    R_tgt = tracker.compute_target_rotation(reading_pitched)

    # Verify orthonormality
    assert abs(np.linalg.det(R_tgt) - 1.0) < 1e-6

    # Verify delta pitch is detected
    d_roll, d_pitch, d_yaw = tracker.delta_euler_deg
    assert abs(d_pitch - 20.0) < 1.0


def test_clutch_spatial_orientation_mapping():
    """Verify ClutchController orientation latching, delta mapping, and disengage holding."""
    clutch = ClutchController(axis_mapping="+x,+y,+z")
    R_neutral = R_DOWNWARD_DEFAULT.copy()

    # Initial human orientation
    R_human_init = np.eye(3, dtype=np.float64)

    # Engage clutch with position and orientation
    clutch.engage(
        current_human_pos=np.zeros(3),
        current_robot_pos=np.array([0.2, 0.0, 0.2]),
        current_human_rot=R_human_init,
        current_robot_rot=R_neutral,
    )
    assert clutch.state == ClutchState.ENGAGED

    # Without moving, target orientation equals anchor
    R_tgt0 = clutch.compute_target_orientation(R_human_init)
    np.testing.assert_allclose(R_tgt0, R_neutral, atol=1e-6)

    # Rotate human by +15 deg yaw
    c, s = math.cos(math.radians(15.0)), math.sin(math.radians(15.0))
    R_delta_yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
    R_human_moved = R_delta_yaw @ R_human_init

    R_tgt_moved = clutch.compute_target_orientation(R_human_moved)
    expected_R = R_delta_yaw @ R_neutral
    np.testing.assert_allclose(R_tgt_moved, expected_R, atol=1e-5)

    # Disengage clutch: target remains held
    clutch.disengage()
    assert clutch.state == ClutchState.DISENGAGED
    R_further = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64) @ R_human_moved
    R_held = clutch.compute_target_orientation(R_further)
    np.testing.assert_allclose(R_held, R_tgt_moved, atol=1e-6)


def test_arm_ik_track_imu_mode():
    """Verify 6-DOF ArmIKSolver solves arm motion in TRACK_IMU mode."""
    solver = ArmIKSolver()
    home_p, home_R = solver.forward_kinematics(solver.home_q)

    # Rotate target by 15 deg pitch
    theta = math.radians(15.0)
    c, s = math.cos(theta), math.sin(theta)
    R_pitch = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
    R_target = R_pitch @ home_R

    # Run 10 steps of IK convergence
    cmd = None
    for i in range(10):
        cmd = solver.solve(
            target_pos=home_p,
            target_rot=R_target,
            orientation_mode=ArmOrientationMode.TRACK_IMU,
            dt=0.033,
            clutch_state=ClutchState.ENGAGED,
            timestamp=10.0 + i * 0.033,
        )

    pos_act, rot_act = solver.forward_kinematics(cmd.target_q)
    assert np.linalg.norm(home_p - pos_act) < 0.015

    # Check angular error between commanded R_target and solved rot_act
    R_err = R_target @ rot_act.T
    angle_err = math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0)))
    assert angle_err < 0.05, f"Orientation tracking error too high: {math.degrees(angle_err):.2f} deg"


def test_imu_stream_invalid_port_fallback():
    """Verify ImuStream safely handles non-existent serial port without unhandled exception."""
    stream = ImuStream(port="/dev/nonexistent_device_xyz123", auto_detect=False)
    assert not stream.start()
    assert not stream.is_connected
    reading = stream.read()
    assert not reading.is_valid
    stream.stop()


def test_rotation_matrix_to_rotvec():
    """Verify rotation_matrix_to_rotvec produces accurate angle and axis."""
    from tuinadex_teleop.perception.imu_stream import rotation_matrix_to_rotvec

    # Identity produces zero vector
    np.testing.assert_allclose(rotation_matrix_to_rotvec(np.eye(3)), np.zeros(3), atol=1e-7)

    # 45 deg around X
    theta = math.radians(45.0)
    c, s = math.cos(theta), math.sin(theta)
    Rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
    v = rotation_matrix_to_rotvec(Rx)
    np.testing.assert_allclose(v, [theta, 0.0, 0.0], atol=1e-5)


def test_solve_imu_alignment_gestures():
    """Verify solve_imu_alignment_from_gestures correctly identifies mounting permutations."""
    from tuinadex_teleop.perception.imu_stream import solve_imu_alignment_from_gestures, parse_axis_mapping

    R_ref = np.eye(3, dtype=np.float64)

    # Case A: Standard dorsal mounting (IMU Y=along fingers, X=across knuckles, Z=up) -> expect '+x,+y,+z'
    # Pitch UP: rotate around +X by 35 deg
    c, s = math.cos(math.radians(35.0)), math.sin(math.radians(35.0))
    R_pitch_a = np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
    # Roll RIGHT: rotate around +Y by 35 deg
    R_roll_a = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)

    mapping_a = solve_imu_alignment_from_gestures(R_ref, R_pitch_a, R_roll_a)
    assert mapping_a == "+x,+y,+z"
    R_a = parse_axis_mapping(mapping_a)
    assert abs(np.linalg.det(R_a) - 1.0) < 1e-6

    # Case B: IMU X points opposite (Pitch UP rotates around -X) -> expect '-x,+y,-z'
    R_pitch_b = np.array([[1, 0, 0], [0, c, s], [0, -s, c]], dtype=np.float64)
    R_roll_b = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)

    mapping_b = solve_imu_alignment_from_gestures(R_ref, R_pitch_b, R_roll_b)
    assert mapping_b == "-x,+y,-z"
    R_b = parse_axis_mapping(mapping_b)
    assert abs(np.linalg.det(R_b) - 1.0) < 1e-6

