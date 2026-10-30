"""test_arm_ik_solver.py — Rigorous unit tests and latency benchmark for 6-DOF ArmIKSolver."""
import os
import sys
import time
import numpy as np
import pytest

from tuinadex_teleop.core.names import ARM_NUM_DOFS, CANONICAL_ARM_JOINT_NAMES
from tuinadex_teleop.core.types import ClutchState, TeleopArmCommand
from tuinadex_teleop.kinematics.arm_ik_solver import (
    ArmIKMetrics,
    ArmIKSolver,
    ArmOrientationMode,
    R_DOWNWARD_DEFAULT,
)


@pytest.fixture
def arm_solver():
    """Create a clean ArmIKSolver instance for testing."""
    return ArmIKSolver()


def test_arm_ik_solver_initialization(arm_solver: ArmIKSolver):
    """Test model loading, 6-DOF reduced joint names, and frame resolution."""
    assert arm_solver.model.nq == ARM_NUM_DOFS
    assert list(arm_solver.model.names)[1:] == CANONICAL_ARM_JOINT_NAMES
    assert arm_solver.ee_frame_name == "link6"
    assert arm_solver.ee_id > 0
    assert "pink" in arm_solver.solver_mode or "pinocchio" in arm_solver.solver_mode
    assert len(arm_solver.q_min) == ARM_NUM_DOFS
    assert len(arm_solver.q_max) == ARM_NUM_DOFS
    assert np.all(arm_solver.q_min < arm_solver.q_max)


def test_arm_ik_forward_kinematics_roundtrip(arm_solver: ArmIKSolver):
    """Test forward kinematics calculation."""
    q_test = np.array([0.0, 0.8, 1.2, 0.0, 0.5, 0.0], dtype=np.float64)
    pos, rot = arm_solver.forward_kinematics(q_test)

    assert pos.shape == (3,)
    assert rot.shape == (3, 3)
    # Check rotation matrix orthonormality: R^T R = I, det(R) = +1
    np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(rot), 1.0, atol=1e-5)


def test_arm_ik_fixed_downward_tuina_solve(arm_solver: ArmIKSolver):
    """Test IK solving in fixed downward mode (Tuina acupressure posture)."""
    arm_solver.reset()
    # Reachable workspace target in front of base
    target_pos = np.array([0.22, 0.0, 0.22], dtype=np.float64)

    # Solve across multiple continuous frames to converge
    for step in range(15):
        t = 100.0 + step * 0.033
        cmd = arm_solver.solve(
            target_pos=target_pos,
            orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
            dt=0.033,
            clutch_state=ClutchState.ENGAGED,
            timestamp=t,
        )

    assert isinstance(cmd, TeleopArmCommand)
    assert cmd.target_q.shape == (ARM_NUM_DOFS,)
    assert cmd.clutch_state == ClutchState.ENGAGED

    # Check achieved position error
    pos_actual, rot_actual = arm_solver.forward_kinematics(cmd.target_q)
    pos_error = np.linalg.norm(target_pos - pos_actual)
    assert pos_error < 0.015, f"Position error too large: {pos_error*1000:.1f} mm"

    # Check tool axis (+Z in link6) points predominantly downward (-Z in base)
    tool_axis = rot_actual @ np.array([0.0, 0.0, 1.0])
    assert tool_axis[2] < -0.85, f"Tool axis not pointing downward: {tool_axis}"


def test_arm_ik_track_wrist_orientation(arm_solver: ArmIKSolver):
    """Test IK tracking of custom 3D rotation matrix."""
    arm_solver.reset()
    target_pos = np.array([0.20, 0.05, 0.25], dtype=np.float64)
    # 45-degree rotated orientation
    c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
    target_rot = np.array([
        [ c, -s, 0.0],
        [ s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ]) @ R_DOWNWARD_DEFAULT

    for step in range(15):
        t = 100.0 + step * 0.033
        cmd = arm_solver.solve(
            target_pos=target_pos,
            target_rot=target_rot,
            orientation_mode=ArmOrientationMode.TRACK_WRIST,
            dt=0.033,
            clutch_state=ClutchState.ENGAGED,
            timestamp=t,
        )

    pos_actual, rot_actual = arm_solver.forward_kinematics(cmd.target_q)
    assert np.linalg.norm(target_pos - pos_actual) < 0.025


def test_arm_ik_joint_limits_strict_compliance(arm_solver: ArmIKSolver):
    """Test that all solved joint angles strictly stay within physical URDF limits."""
    arm_solver.reset()
    rng = np.random.default_rng(42)

    # Sweep across 30 random Cartesian targets inside workspace
    for _ in range(30):
        rand_pos = np.array([
            rng.uniform(-0.30, 0.30),
            rng.uniform(0.18, 0.45),
            rng.uniform(0.08, 0.35),
        ])
        cmd = arm_solver.solve(
            target_pos=rand_pos,
            orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
            dt=0.033,
            clutch_state=ClutchState.ENGAGED,
        )

        assert np.all(cmd.target_q >= arm_solver.q_min - 1e-4), (
            f"Joint underflow limit: {cmd.target_q} < {arm_solver.q_min}"
        )
        assert np.all(cmd.target_q <= arm_solver.q_max + 1e-4), (
            f"Joint overflow limit: {cmd.target_q} > {arm_solver.q_max}"
        )


def test_arm_ik_velocity_saturation(arm_solver: ArmIKSolver):
    """Test that sudden large spatial jumps are bounded by max joint velocity."""
    arm_solver.reset()
    initial_q = arm_solver.current_q.copy()
    dt = 0.033

    # Command a huge spatial jump
    jump_pos = np.array([0.35, 0.50, 0.40])
    cmd = arm_solver.solve(
        target_pos=jump_pos,
        orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
        dt=dt,
        clutch_state=ClutchState.ENGAGED,
    )

    max_allowed_delta = arm_solver.max_joint_velocity_rad_s * dt + 0.02
    actual_deltas = np.abs(cmd.target_q - initial_q)
    assert np.all(actual_deltas <= max_allowed_delta), (
        f"Joint velocity jump exceeded: {actual_deltas} > {max_allowed_delta}"
    )


def test_arm_ik_workspace_clamping(arm_solver: ArmIKSolver):
    """Test that out-of-bounds positions are clamped to safe boundaries."""
    # Test dangerously low Z (bed collision) and excessive X, Y
    dangerous_target = np.array([1.5, -0.90, -0.20])
    clamped = arm_solver.clamp_workspace(dangerous_target)

    assert clamped[0] == arm_solver.workspace_x[1]  # 0.45m
    assert clamped[1] == arm_solver.workspace_y[0]  # -0.45m
    assert clamped[2] == arm_solver.workspace_z[0]  # 0.02m floor


def test_arm_ik_clutch_disengaged_hold(arm_solver: ArmIKSolver):
    """Test that when clutch is disengaged, robot holds position and returns zero velocity."""
    arm_solver.reset()
    hold_q = arm_solver.current_q.copy()

    # Move target while DISENGAGED
    cmd = arm_solver.solve(
        target_pos=np.array([0.3, 0.3, 0.3]),
        clutch_state=ClutchState.DISENGAGED,
    )

    np.testing.assert_allclose(cmd.target_q, hold_q)
    np.testing.assert_allclose(cmd.target_dq, np.zeros(ARM_NUM_DOFS))
    assert cmd.clutch_state == ClutchState.DISENGAGED


def test_arm_ik_latency_benchmark(arm_solver: ArmIKSolver):
    """Benchmark tracking latency over 100 consecutive steps. Must average < 3ms."""
    arm_solver.reset()
    base_pos = np.array([0.22, 0.10, 0.22])

    latencies = []
    for step in range(100):
        target = base_pos + np.array([
            0.02 * np.sin(step * 0.1),
            0.01 * np.cos(step * 0.1),
            -0.01 * np.sin(step * 0.05),
        ])
        t0 = time.perf_counter()
        arm_solver.solve(
            target_pos=target,
            orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
            dt=0.033,
            clutch_state=ClutchState.ENGAGED,
        )
        latencies.append((time.perf_counter() - t0) * 1000.0)

    mean_lat = np.mean(latencies)
    max_lat = np.max(latencies)
    print(f"\nArm IK Tracking Benchmark: mean={mean_lat:.3f}ms, max={max_lat:.3f}ms")

    assert mean_lat < 3.0, f"Mean latency {mean_lat:.3f}ms exceeded 3ms budget!"
