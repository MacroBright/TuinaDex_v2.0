"""test_mujoco_sim_adapter.py — Verification of composite robot MuJoCo digital twin adapter."""
from __future__ import annotations

import time
import numpy as np
import pytest

from tuinadex_teleop.adapters import MujocoSimAdapter, DEFAULT_READY_ARM_Q
from tuinadex_teleop.core.names import (
    ARM_NUM_DOFS,
    CANONICAL_ARM_JOINT_NAMES,
    CANONICAL_COMPOSITE_JOINT_NAMES,
    CANONICAL_HAND_JOINT_NAMES,
    COMPOSITE_NUM_DOFS,
    HAND_NUM_DOFS,
)
from tuinadex_teleop.core.types import (
    ClutchState,
    RobotObservation,
    TeleopArmCommand,
    TeleopCommand,
    TeleopHandCommand,
)


@pytest.fixture
def sim_adapter():
    """Fixture providing connected headless MujocoSimAdapter."""
    adapter = MujocoSimAdapter(headless=True)
    assert adapter.connect()
    yield adapter
    adapter.disconnect()


def test_mujoco_adapter_lifecycle():
    """Verify clean connect/disconnect lifecycle and state reporting."""
    adapter = MujocoSimAdapter(headless=True)
    assert not adapter.is_connected
    assert adapter.connect()
    assert adapter.is_connected
    # Redundant connect should be idempotent
    assert adapter.connect()
    adapter.disconnect()
    assert not adapter.is_connected


def test_mujoco_model_dofs_and_names(sim_adapter):
    """Verify 22-DOF composite robot (Arm 6 + Hand 16) matches canonical naming."""
    assert sim_adapter.model.nq == COMPOSITE_NUM_DOFS
    assert sim_adapter.model.nu == COMPOSITE_NUM_DOFS
    assert len(sim_adapter._arm_qpos_indices) == ARM_NUM_DOFS
    assert len(sim_adapter._hand_qpos_indices) == HAND_NUM_DOFS

    # Actuator limits check
    assert sim_adapter._arm_ctrl_limits.shape == (6, 2)
    assert sim_adapter._hand_ctrl_limits.shape == (16, 2)


def test_mujoco_ready_pose_stability(sim_adapter):
    """Verify initial ready posture remains dynamically stable under gravity."""
    obs_init = sim_adapter.read_observation()
    np.testing.assert_allclose(obs_init.arm_actual_q, DEFAULT_READY_ARM_Q, atol=1e-3)
    np.testing.assert_allclose(obs_init.hand_actual_q, 0.0, atol=1e-3)

    # Step physics for 200 steps (0.4s) under gravity
    sim_adapter.step(200)

    obs_after = sim_adapter.read_observation()
    assert not np.isnan(obs_after.arm_actual_q).any()
    assert not np.isnan(obs_after.hand_actual_q).any()
    # High stiffness holding arm near ready pose
    np.testing.assert_allclose(obs_after.arm_actual_q, DEFAULT_READY_ARM_Q, atol=0.08)


def test_mujoco_arm_joint_tracking(sim_adapter):
    """Verify 6-DOF arm joints track step reference commands."""
    obs = sim_adapter.read_observation()
    # Lift arm slightly and rotate base joint 1 away from obstacles in free air
    target_arm = obs.arm_actual_q.copy()
    target_arm[0] = -0.25  # Yaw base joint 1
    target_arm[1] = 1.10   # Shoulder pitch joint 2
    target_arm[3] = 0.20   # Wrist pitch joint 4

    cmd = TeleopCommand(
        timestamp=time.time(),
        arm_command=TeleopArmCommand(
            timestamp=time.time(),
            target_pose=np.eye(4),
            target_q=target_arm,
        ),
        hand_command=TeleopHandCommand(
            timestamp=time.time(),
            target_q=np.zeros(HAND_NUM_DOFS),
        ),
        clutch_state=ClutchState.ENGAGED,
    )
    sim_adapter.send_command(cmd)

    # Step 300 steps (0.6s) to allow complete mechanical settling
    sim_adapter.step(300)
    obs_final = sim_adapter.read_observation()

    np.testing.assert_allclose(obs_final.arm_actual_q[0], -0.25, atol=0.05)
    np.testing.assert_allclose(obs_final.arm_actual_q[1], 1.10, atol=0.05)
    np.testing.assert_allclose(obs_final.arm_actual_q[3], 0.20, atol=0.08)


def test_mujoco_hand_finger_tracking(sim_adapter):
    """Verify 16-DOF LEAP hand fingers flex in response to commands."""
    target_hand = np.zeros(HAND_NUM_DOFS)
    target_hand[1] = 1.0   # index_mcp
    target_hand[5] = 1.0   # middle_mcp
    target_hand[9] = 1.0   # ring_mcp
    target_hand[14] = 1.0  # thumb_mcp

    # Lift arm into free air away from dummy contact for unobstructed finger curl test
    arm_free_air = np.array([0.0, 0.60, 0.80, 0.0, 0.0, 0.0])

    cmd = TeleopCommand(
        timestamp=time.time(),
        arm_command=TeleopArmCommand(
            timestamp=time.time(),
            target_pose=np.eye(4),
            target_q=arm_free_air,
        ),
        hand_command=TeleopHandCommand(
            timestamp=time.time(),
            target_q=target_hand,
        ),
        clutch_state=ClutchState.ENGAGED,
    )
    sim_adapter.send_command(cmd)

    # Step 300 steps (0.6s) to allow complete mechanical settling
    sim_adapter.step(300)
    obs_final = sim_adapter.read_observation()

    assert obs_final.hand_actual_q[1] > 0.80, "index_mcp tracking failed"
    assert obs_final.hand_actual_q[5] > 0.80, "middle_mcp tracking failed"
    assert obs_final.hand_actual_q[9] > 0.80, "ring_mcp tracking failed"
    assert obs_final.hand_actual_q[14] > 0.80, "thumb_mcp tracking failed"


def test_mujoco_joint_limit_clamping(sim_adapter):
    """Verify adapter clamps command exceeding actuator limits without error."""
    insane_arm_q = np.full(ARM_NUM_DOFS, 100.0)
    insane_hand_q = np.full(HAND_NUM_DOFS, -50.0)

    cmd = TeleopCommand(
        timestamp=time.time(),
        arm_command=TeleopArmCommand(
            timestamp=time.time(),
            target_pose=np.eye(4),
            target_q=insane_arm_q,
        ),
        hand_command=TeleopHandCommand(
            timestamp=time.time(),
            target_q=insane_hand_q,
        ),
        clutch_state=ClutchState.ENGAGED,
    )
    # Should not raise exception, will clip internally
    sim_adapter.send_command(cmd)
    sim_adapter.step(50)

    obs = sim_adapter.read_observation()
    assert not np.isnan(obs.arm_actual_q).any()
    assert not np.isnan(obs.hand_actual_q).any()


def test_mujoco_observation_contract(sim_adapter):
    """Verify returned RobotObservation strictly satisfies type and shape contracts."""
    obs = sim_adapter.read_observation()
    assert isinstance(obs, RobotObservation)
    assert obs.arm_actual_q.shape == (ARM_NUM_DOFS,)
    assert obs.arm_actual_dq.shape == (ARM_NUM_DOFS,)
    assert obs.arm_effort.shape == (ARM_NUM_DOFS,)
    assert obs.hand_actual_q.shape == (HAND_NUM_DOFS,)
    assert obs.hand_current.shape == (HAND_NUM_DOFS,)
    assert obs.composite_state.shape == (COMPOSITE_NUM_DOFS,)
    assert obs.is_connected is True


def test_mujoco_contact_forces_telemetry(sim_adapter):
    """Verify contact force query dictionary structure."""
    forces = sim_adapter.get_contact_forces()
    assert isinstance(forces, dict)
    assert "index" in forces
    assert "middle" in forces
    assert "ring" in forces
    assert "thumb" in forces
    assert "total" in forces
    assert forces["total"] >= 0.0


def test_mujoco_step_latency_benchmark(sim_adapter):
    """Verify physical step time supports >= 500Hz simulation rate (< 2ms per step)."""
    n_benchmark_steps = 100
    t0 = time.perf_counter()
    sim_adapter.step(n_benchmark_steps)
    dt = time.perf_counter() - t0
    step_ms = (dt / n_benchmark_steps) * 1000.0

    print(f"\n[BENCHMARK] MuJoCo Physics Step: {step_ms:.3f} ms / step ({1000.0/step_ms:.0f} Hz)")
    assert step_ms < 2.0, f"Step too slow: {step_ms:.3f} ms (expected < 2.0 ms)"
