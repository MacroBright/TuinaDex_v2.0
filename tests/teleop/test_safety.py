"""Unit tests for TuinaDex Multi-Tier Teleoperation Safety Guardrails."""
import math
import numpy as np
import pytest

from tuinadex_teleop.core.safety import (
    MinimumJerkTrajectory,
    SafetyConfig,
    SafetyLevel,
    SafetyReport,
    SafetySupervisor,
    SafetyViolation,
    TwoStageRetractController,
)
from tuinadex_teleop.core.types import ClutchState


class MockArmIKSolver:
    """Mock IK solver providing manipulability and joint limit queries for testing."""

    def __init__(self, min_sv: float = 0.15, is_near_limit: bool = False):
        self.min_sv = min_sv
        self.is_near_limit = is_near_limit

    def compute_manipulability(self, q=None):
        return (0.012, self.min_sv, 12.0)

    def check_joint_limits(self, q=None, margin_rad=0.05):
        return (self.is_near_limit, 0.02 if self.is_near_limit else 0.50, 0)


def test_minimum_jerk_trajectory_boundary_conditions():
    """Verify that MinimumJerkTrajectory satisfies zero initial and final velocity."""
    traj = MinimumJerkTrajectory(
        start_val=np.array([0.0, 10.0]),
        goal_val=np.array([1.0, 20.0]),
        duration_s=2.0,
        start_time=10.0,
    )

    # At start (t = 10.0)
    p0, v0 = traj.evaluate(10.0)
    np.testing.assert_allclose(p0, [0.0, 10.0], atol=1e-5)
    np.testing.assert_allclose(v0, [0.0, 0.0], atol=1e-5)

    # At midpoint (t = 11.0)
    p_mid, v_mid = traj.evaluate(11.0)
    np.testing.assert_allclose(p_mid, [0.5, 15.0], atol=1e-5)
    assert np.all(v_mid > 0.0)

    # At end (t = 12.0)
    p1, v1 = traj.evaluate(12.0)
    np.testing.assert_allclose(p1, [1.0, 20.0], atol=1e-5)
    np.testing.assert_allclose(v1, [0.0, 0.0], atol=1e-5)
    assert traj.is_finished(12.0)


def test_hand_loss_timeout_and_transient_hold():
    """Verify that hand loss <= 0.35s causes HOLD, and > 0.35s causes two-stage RETRACT."""
    config = SafetyConfig(hand_loss_timeout_s=0.35, lift_off_height_m=0.06)
    supervisor = SafetySupervisor(config)

    t0 = 100.0
    cur_arm_q = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    cur_arm_pos = np.array([0.2, 0.1, 0.05])
    cur_arm_rot = np.eye(3)
    cur_hand_q = np.ones(16) * 0.5
    home_arm_q = np.zeros(6)
    forces = {"total": 0.0}

    # Frame 1: Hand is tracking normally
    rep1 = supervisor.evaluate(
        t_now=t0,
        wrist_tracking_valid=True,
        wrist_pos=np.array([0.0, 0.0, 0.0]),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces=forces,
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep1.level == SafetyLevel.NORMAL

    # Frame 2: Transient occlusion at t0 + 0.15s (<= 0.35s)
    rep2 = supervisor.evaluate(
        t_now=t0 + 0.15,
        wrist_tracking_valid=False,
        wrist_pos=None,
        wrist_rot=None,
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces=forces,
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep2.level == SafetyLevel.HOLD

    # Frame 3: Persistent loss at t0 + 0.55s (0.40s elapsed > 0.35s) -> Must trigger RETRACT
    rep3 = supervisor.evaluate(
        t_now=t0 + 0.55,
        wrist_tracking_valid=False,
        wrist_pos=None,
        wrist_rot=None,
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces=forces,
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep3.level == SafetyLevel.RETRACTING
    assert rep3.violation == SafetyViolation.HAND_LOSS_TIMEOUT
    assert supervisor.retract_controller is not None

    # Verify Stage 1 of retractor: lifts +Z and opens hand
    retract_pos, _, retract_arm_q, retract_hand_q, finished = supervisor.retract_controller.step(
        t0 + 0.55 + 0.30, cur_arm_q
    )
    assert retract_arm_q is None  # Stage 1 operates in Cartesian space
    assert retract_pos is not None
    assert retract_pos[2] > cur_arm_pos[2]  # Lifted upwards
    assert np.all(retract_hand_q < cur_hand_q)  # Hand opening


def test_inter_frame_pose_jump_detection():
    """Verify that teleportation / abrupt frame mutations trigger immediate retract."""
    config = SafetyConfig(max_pos_jump_m=0.08, max_rot_jump_deg=35.0, max_hand_q_jump_rad=0.80)
    supervisor = SafetySupervisor(config)

    t0 = 200.0
    cur_arm_q = np.zeros(6)
    cur_arm_pos = np.array([0.2, 0.1, 0.1])
    cur_arm_rot = np.eye(3)
    cur_hand_q = np.zeros(16)
    home_arm_q = np.zeros(6)
    forces = {"total": 0.0}

    # Initial frame
    supervisor.evaluate(
        t_now=t0,
        wrist_tracking_valid=True,
        wrist_pos=np.array([0.10, 0.10, 0.10]),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces=forces,
        clutch_state=ClutchState.ENGAGED,
    )

    # Frame with 0.15m Cartesian jump (> 0.08m)
    rep_jump = supervisor.evaluate(
        t_now=t0 + 0.033,
        wrist_tracking_valid=True,
        wrist_pos=np.array([0.25, 0.10, 0.10]),  # 15cm jump
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 0.033,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces=forces,
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep_jump.level == SafetyLevel.RETRACTING
    assert rep_jump.violation == SafetyViolation.POS_JUMP


def test_contact_force_overload_thresholds():
    """Verify contact force limits: total > 70N, finger > 40N, surge rate > 100N/s."""
    config = SafetyConfig(
        max_total_force_n=70.0,
        max_finger_force_n=40.0,
        max_force_rate_n_s=100.0,
        min_force_rate_trigger_n=10.0,
    )
    supervisor = SafetySupervisor(config)

    t0 = 300.0
    cur_arm_q = np.zeros(6)
    cur_arm_pos = np.array([0.2, 0.1, 0.03])
    cur_arm_rot = np.eye(3)
    cur_hand_q = np.zeros(16)
    home_arm_q = np.zeros(6)

    # Initial safe contact
    supervisor.evaluate(
        t_now=t0,
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 20.0, "index": 5.0, "middle": 5.0, "ring": 5.0, "thumb": 5.0},
        clutch_state=ClutchState.ENGAGED,
    )

    # Test 1: Total force overload (> 70N)
    rep_tot = supervisor.evaluate(
        t_now=t0 + 0.033,
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 0.033,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 75.0, "index": 20.0, "middle": 20.0, "ring": 20.0, "thumb": 15.0},
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep_tot.level == SafetyLevel.RETRACTING
    assert rep_tot.violation == SafetyViolation.FORCE_TOTAL_OVERLOAD

    # Reset supervisor and test single finger overload (> 40N)
    supervisor.reset()
    supervisor.evaluate(
        t_now=t0 + 1.0,
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 1.0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 45.0, "index": 42.0, "middle": 1.0, "ring": 1.0, "thumb": 1.0},
        clutch_state=ClutchState.ENGAGED,
    )
    assert supervisor.level == SafetyLevel.RETRACTING
    assert supervisor.last_violation == SafetyViolation.FORCE_FINGER_OVERLOAD

    # Reset supervisor and test force surge rate (> 100N/s with total force > 10N)
    supervisor.reset()
    supervisor.evaluate(
        t_now=t0 + 2.0,
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 2.0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 5.0, "index": 2.0, "middle": 1.0, "ring": 1.0, "thumb": 1.0},
        clutch_state=ClutchState.ENGAGED,
    )
    rep_surge = supervisor.evaluate(
        t_now=t0 + 2.05,  # dt = 0.05s
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 2.05,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 20.0, "index": 5.0, "middle": 5.0, "ring": 5.0, "thumb": 5.0},  # rate = 300N/s > 100N/s
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep_surge.level == SafetyLevel.RETRACTING
    assert rep_surge.violation == SafetyViolation.FORCE_RATE_OVERLOAD


def test_singularity_and_joint_limit_freeze():
    """Verify that singularity or joint limit approaches freeze robot in place (SAFE_HOLD)."""
    config = SafetyConfig(min_singular_value=0.025, joint_limit_margin_rad=0.05)
    supervisor = SafetySupervisor(config)

    t0 = 400.0
    cur_arm_q = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    cur_arm_pos = np.array([0.2, 0.1, 0.1])
    cur_arm_rot = np.eye(3)
    cur_hand_q = np.zeros(16)
    home_arm_q = np.zeros(6)

    # Solver reporting dangerously low singular value (min_sv = 0.010 < 0.025)
    mock_solver_singular = MockArmIKSolver(min_sv=0.010, is_near_limit=False)

    rep_sing = supervisor.evaluate(
        t_now=t0,
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 0.0},
        clutch_state=ClutchState.ENGAGED,
        arm_ik_solver=mock_solver_singular,
    )
    assert rep_sing.level == SafetyLevel.FROZEN
    assert rep_sing.violation == SafetyViolation.SINGULARITY_APPROACH
    np.testing.assert_allclose(supervisor.frozen_arm_q, cur_arm_q)

    # Test recovery via disengagement
    rep_clear = supervisor.evaluate(
        t_now=t0 + 0.1,
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 0.1,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 0.0},
        clutch_state=ClutchState.DISENGAGED,  # Operator unlatched clutch
        arm_ik_solver=mock_solver_singular,
    )
    assert rep_clear.level == SafetyLevel.NORMAL


def test_disengaged_standby_immunity_to_false_safety_retract():
    """Verify that when clutch is DISENGAGED (robot idle/standby at Home),
    hand loss and force settling spikes do NOT trigger false retract loops.
    """
    config = SafetyConfig(hand_loss_timeout_s=0.35, max_total_force_n=70.0, max_force_rate_n_s=100.0)
    supervisor = SafetySupervisor(config)

    t0 = 500.0
    cur_arm_q = np.array([0.0, 0.85, 0.95, -0.80, 0.0, 0.0])
    cur_arm_pos = np.array([0.18, 0.0, 0.34])
    cur_arm_rot = np.eye(3)
    cur_hand_q = np.zeros(16)
    home_arm_q = cur_arm_q.copy()

    # Case 1: Hand not visible at startup while DISENGAGED -> MUST remain NORMAL
    rep1 = supervisor.evaluate(
        t_now=t0,
        wrist_tracking_valid=False,
        wrist_pos=None,
        wrist_rot=None,
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 0.0},
        clutch_state=ClutchState.DISENGAGED,
    )
    assert rep1.level == SafetyLevel.NORMAL
    assert rep1.violation == SafetyViolation.NONE

    # Persistent absence after 1.0s while DISENGAGED -> MUST STILL remain NORMAL (No infinite retract!)
    rep2 = supervisor.evaluate(
        t_now=t0 + 1.0,
        wrist_tracking_valid=False,
        wrist_pos=None,
        wrist_rot=None,
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 0.0},
        clutch_state=ClutchState.DISENGAGED,
    )
    assert rep2.level == SafetyLevel.NORMAL

    # Case 2: Transient physics settling impulse while DISENGAGED -> MUST NOT trigger retract
    rep3 = supervisor.evaluate(
        t_now=t0 + 1.033,
        wrist_tracking_valid=True,
        wrist_pos=np.array([0.1, 0.1, 0.1]),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 1.033,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 25.0, "middle": 25.0},
        clutch_state=ClutchState.DISENGAGED,
    )
    assert rep3.level == SafetyLevel.NORMAL
    assert supervisor.retract_controller is None


def test_step_retract_lifecycle_and_auto_completion():
    """Verify that step_retract smoothly steps and auto-resets supervisor when finished."""
    config = SafetyConfig(hand_loss_timeout_s=0.35, lift_off_duration_s=0.2, retract_duration_s=0.3)
    supervisor = SafetySupervisor(config)

    t0 = 600.0
    cur_arm_q = np.array([0.0, 0.5, 1.0, 0.0, 0.5, 0.0])
    cur_arm_pos = np.array([0.2, 0.1, 0.05])
    cur_arm_rot = np.eye(3)
    cur_hand_q = np.ones(16) * 0.5
    home_arm_q = np.zeros(6)

    # Trigger retract via hand loss while ENGAGED
    supervisor.evaluate(
        t_now=t0,
        wrist_tracking_valid=False,
        wrist_pos=None,
        wrist_rot=None,
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 0.0},
        clutch_state=ClutchState.ENGAGED,
    )
    # After 0.4s (> 0.35s timeout)
    rep = supervisor.evaluate(
        t_now=t0 + 0.40,
        wrist_tracking_valid=False,
        wrist_pos=None,
        wrist_rot=None,
        wrist_timestamp=t0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 0.0},
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep.level == SafetyLevel.RETRACTING

    # Step at mid-retract (t0 + 0.55s) -> Not finished
    pos, rot, arm_q, hand_q, finished = supervisor.step_retract(t0 + 0.55, cur_arm_q)
    assert not finished
    assert supervisor.level == SafetyLevel.RETRACTING

    # Step past total duration (0.2s + 0.3s = 0.5s after trigger at t0+0.40 => t >= t0+0.90)
    pos, rot, arm_q, hand_q, finished = supervisor.step_retract(t0 + 0.95, cur_arm_q)
    assert finished
    assert supervisor.level == SafetyLevel.NORMAL
    assert supervisor.retract_controller is None


def test_safety_bypass_mode_unrestricted_operation():
    """Verify that when safety is in BYPASS mode (or toggled off),
    no safety gates trigger retract or freeze, matching baseline unrestricted behavior.
    """
    supervisor = SafetySupervisor(enabled=False)
    assert supervisor.level == SafetyLevel.BYPASS

    t0 = 700.0
    cur_arm_q = np.zeros(6)
    cur_arm_pos = np.array([0.2, 0.1, 0.05])
    cur_arm_rot = np.eye(3)
    cur_hand_q = np.zeros(16)
    home_arm_q = np.zeros(6)

    # Extreme overload forces, jump, lost hand -> Must return BYPASS and NEVER retract
    rep = supervisor.evaluate(
        t_now=t0,
        wrist_tracking_valid=False,
        wrist_pos=None,
        wrist_rot=None,
        wrist_timestamp=t0 - 5.0,  # Huge latency
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 200.0, "middle": 150.0},  # Huge force
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep.level == SafetyLevel.BYPASS
    assert supervisor.retract_controller is None

    # Test toggling enabled back on
    supervisor.toggle_enabled()
    assert supervisor.enabled is True
    assert supervisor.level == SafetyLevel.NORMAL

    # Now with high force (> 120N limit), it should trigger retract
    rep_on = supervisor.evaluate(
        t_now=t0 + 1.0,
        wrist_tracking_valid=True,
        wrist_pos=np.zeros(3),
        wrist_rot=np.eye(3),
        wrist_timestamp=t0 + 1.0,
        current_arm_q=cur_arm_q,
        current_arm_pos=cur_arm_pos,
        current_arm_rot=cur_arm_rot,
        current_hand_q=cur_hand_q,
        home_arm_q=home_arm_q,
        contact_forces={"total": 150.0},
        clutch_state=ClutchState.ENGAGED,
    )
    assert rep_on.level == SafetyLevel.RETRACTING


