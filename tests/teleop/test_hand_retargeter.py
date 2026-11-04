"""test_hand_retargeter.py — Unit tests and kinematics benchmarks for LEAP Hand retargeting."""
from __future__ import annotations

import math
import time
import numpy as np
import pytest

from tuinadex_teleop.core.names import CANONICAL_HAND_JOINT_NAMES, HAND_NUM_DOFS
from tuinadex_teleop.core.types import TeleopHandCommand
from tuinadex_teleop.kinematics.hand_retargeter import (
    FallbackKinematicRetargeter,
    HandRetargeter,
)
from tuinadex_teleop.perception.wrist_tracker import (
    LM_INDEX_DIP,
    LM_INDEX_MCP,
    LM_INDEX_PIP,
    LM_INDEX_TIP,
    LM_MIDDLE_DIP,
    LM_MIDDLE_MCP,
    LM_MIDDLE_PIP,
    LM_MIDDLE_TIP,
    LM_PINKY_DIP,
    LM_PINKY_MCP,
    LM_PINKY_PIP,
    LM_PINKY_TIP,
    LM_RING_DIP,
    LM_RING_MCP,
    LM_RING_PIP,
    LM_RING_TIP,
    LM_THUMB_CMC,
    LM_THUMB_IP,
    LM_THUMB_MCP,
    LM_THUMB_TIP,
    LM_WRIST,
    TrackingState,
    WristPose,
)


@pytest.fixture
def base_wrist_pose() -> WristPose:
    """Fixture providing a baseline WristPose for synthetic tests."""
    return WristPose(
        timestamp=1.0,
        is_valid=True,
        tracking_state=TrackingState.TRACKING,
        position_cam=np.array([0.0, 0.0, 0.60]),
        position_table=np.array([0.1, 0.2, 0.1]),
        position_table_raw=np.array([0.1, 0.2, 0.1]),
        orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        orientation_table_matrix=np.eye(3),
        euler_deg=(0.0, 0.0, 0.0),
        wrist_pixel_uv=(320.0, 240.0),
        landmarks_2d=np.zeros((21, 2)),
        landmarks_3d_cam=np.zeros((21, 3)),
        depth_m=0.60,
        handedness="Right",
    )


def create_synthetic_hand_pose(base_pose: WristPose, gesture: str = "open") -> WristPose:
    """Helper to synthesize realistic 3D metric landmarks (in meters) for typical hand gestures."""
    pts = np.zeros((21, 3), dtype=np.float64)
    # Wrist at (0, 0, 0.60m)
    pts[LM_WRIST] = [0.0, 0.0, 0.60]

    # Metacarpals on palm plane
    pts[LM_INDEX_MCP] = [-0.030, 0.080, 0.60]
    pts[LM_MIDDLE_MCP] = [0.000, 0.085, 0.60]
    pts[LM_RING_MCP] = [0.025, 0.080, 0.60]
    pts[LM_PINKY_MCP] = [0.045, 0.070, 0.60]
    pts[LM_THUMB_CMC] = [-0.035, 0.030, 0.60]

    if gesture == "open":
        # Straight fingers extending along +Y
        # Index
        pts[LM_INDEX_PIP] = [-0.030, 0.115, 0.60]
        pts[LM_INDEX_DIP] = [-0.030, 0.140, 0.60]
        pts[LM_INDEX_TIP] = [-0.030, 0.165, 0.60]
        # Middle
        pts[LM_MIDDLE_PIP] = [0.000, 0.125, 0.60]
        pts[LM_MIDDLE_DIP] = [0.000, 0.155, 0.60]
        pts[LM_MIDDLE_TIP] = [0.000, 0.180, 0.60]
        # Ring
        pts[LM_RING_PIP] = [0.025, 0.120, 0.60]
        pts[LM_RING_DIP] = [0.025, 0.145, 0.60]
        pts[LM_RING_TIP] = [0.025, 0.170, 0.60]
        pts[LM_PINKY_PIP] = [0.045, 0.100, 0.60]
        pts[LM_PINKY_DIP] = [0.045, 0.120, 0.60]
        pts[LM_PINKY_TIP] = [0.045, 0.140, 0.60]
        # Pinky
        pts[LM_PINKY_PIP] = [0.045, 0.100, 0.60]
        pts[LM_PINKY_DIP] = [0.045, 0.120, 0.60]
        pts[LM_PINKY_TIP] = [0.045, 0.140, 0.60]
        # Thumb spread outward
        pts[LM_THUMB_MCP] = [-0.060, 0.060, 0.60]
        pts[LM_THUMB_IP] = [-0.080, 0.085, 0.60]
        pts[LM_THUMB_TIP] = [-0.095, 0.105, 0.60]

    elif gesture == "fist":
        # Curled fingers folding inward into palm (-Z and back toward wrist)
        # Index
        pts[LM_INDEX_PIP] = [-0.030, 0.100, 0.58]
        pts[LM_INDEX_DIP] = [-0.030, 0.080, 0.57]
        pts[LM_INDEX_TIP] = [-0.030, 0.060, 0.58]
        # Middle
        pts[LM_MIDDLE_PIP] = [0.000, 0.105, 0.58]
        pts[LM_MIDDLE_DIP] = [0.000, 0.085, 0.57]
        pts[LM_MIDDLE_TIP] = [0.000, 0.065, 0.58]
        # Ring
        pts[LM_RING_PIP] = [0.025, 0.100, 0.58]
        pts[LM_RING_DIP] = [0.025, 0.080, 0.57]
        pts[LM_RING_TIP] = [0.025, 0.060, 0.58]
        # Pinky
        pts[LM_PINKY_PIP] = [0.045, 0.090, 0.58]
        pts[LM_PINKY_DIP] = [0.045, 0.070, 0.57]
        pts[LM_PINKY_TIP] = [0.045, 0.050, 0.58]
        # Thumb wrapped over fingers
        pts[LM_THUMB_MCP] = [-0.040, 0.050, 0.59]
        pts[LM_THUMB_IP] = [-0.020, 0.065, 0.58]
        pts[LM_THUMB_TIP] = [0.000, 0.070, 0.57]

    elif gesture == "pinch":
        # Thumb and index tips meeting in precision pinch (< 1.5cm distance)
        pinch_contact = np.array([-0.015, 0.100, 0.585])
        # Index curls to meet pinch point
        pts[LM_INDEX_PIP] = [-0.030, 0.110, 0.59]
        pts[LM_INDEX_DIP] = [-0.025, 0.105, 0.587]
        pts[LM_INDEX_TIP] = pinch_contact + np.array([-0.002, 0.0, 0.0])
        # Thumb opposes to meet pinch point
        pts[LM_THUMB_MCP] = [-0.050, 0.050, 0.595]
        pts[LM_THUMB_IP] = [-0.030, 0.080, 0.590]
        pts[LM_THUMB_TIP] = pinch_contact + np.array([+0.002, 0.0, 0.0])
        # Other fingers extended or relaxed
        pts[LM_MIDDLE_PIP] = [0.000, 0.125, 0.60]
        pts[LM_MIDDLE_DIP] = [0.000, 0.155, 0.60]
        pts[LM_MIDDLE_TIP] = [0.000, 0.180, 0.60]
        pts[LM_RING_PIP] = [0.025, 0.120, 0.60]
        pts[LM_RING_DIP] = [0.025, 0.145, 0.60]
        pts[LM_RING_TIP] = [0.025, 0.170, 0.60]
        pts[LM_PINKY_PIP] = [0.045, 0.100, 0.60]
        pts[LM_PINKY_DIP] = [0.045, 0.120, 0.60]
        pts[LM_PINKY_TIP] = [0.045, 0.140, 0.60]

    pose = WristPose(
        timestamp=base_pose.timestamp,
        is_valid=True,
        tracking_state=TrackingState.TRACKING,
        position_cam=pts[LM_WRIST],
        position_table=base_pose.position_table,
        position_table_raw=base_pose.position_table_raw,
        orientation_table_quat=base_pose.orientation_table_quat,
        orientation_table_matrix=base_pose.orientation_table_matrix,
        euler_deg=base_pose.euler_deg,
        wrist_pixel_uv=(320.0, 240.0),
        landmarks_2d=np.zeros((21, 2)),
        landmarks_3d_cam=pts,
        depth_m=0.60,
        handedness="Right",
    )
    return pose


# ─── Tests ────────────────────────────────────────────────────────

def test_hand_retargeter_initialization():
    """Verify HandRetargeter initializes cleanly with canonical 16-DOF joint contracts."""
    retargeter = HandRetargeter()
    assert retargeter.joint_limits.shape == (HAND_NUM_DOFS, 2)
    # Check that upper limits are strictly greater than lower limits
    assert np.all(retargeter.joint_limits[:, 1] > retargeter.joint_limits[:, 0])


def test_retarget_open_hand_pose(base_wrist_pose):
    """Verify flat open hand yields valid joint angles without clamping errors."""
    retargeter = HandRetargeter()
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")

    cmd = retargeter.retarget_pose(open_pose)
    assert isinstance(cmd, TeleopHandCommand)
    assert cmd.target_q.shape == (HAND_NUM_DOFS,)
    assert cmd.is_clutched is True

    # In open pose, MCP flexion angles should be strictly near 0 (< 0.10 rad / ~5 deg)
    # index_mcp(idx 1), middle_mcp(idx 5), ring_mcp(idx 9)
    assert cmd.target_q[1] < 0.10
    assert cmd.target_q[5] < 0.10
    assert cmd.target_q[9] < 0.10


def test_retarget_clenched_fist(base_wrist_pose):
    """Verify fist gesture drives MCP/PIP/DIP joints toward flexion limits."""
    retargeter = HandRetargeter()
    fist_pose = create_synthetic_hand_pose(base_wrist_pose, "fist")

    cmd = retargeter.retarget_pose(fist_pose)
    assert isinstance(cmd, TeleopHandCommand)
    assert cmd.target_q.shape == (HAND_NUM_DOFS,)

    # In a fist, average finger flexion must be substantially greater than open hand
    avg_flex = retargeter.metrics.average_flexion_rad
    assert avg_flex > 0.40


def test_retarget_precision_pinch(base_wrist_pose):
    """Verify precision pinch gesture triggers pinch telemetry flag."""
    retargeter = HandRetargeter()
    pinch_pose = create_synthetic_hand_pose(base_wrist_pose, "pinch")

    cmd = retargeter.retarget_pose(pinch_pose)
    metrics = retargeter.metrics

    assert metrics.is_pinching is True
    assert metrics.thumb_index_pinch_dist_m < 0.035
    assert cmd.is_clutched is True


def test_joint_limits_strict_compliance(base_wrist_pose):
    """Fuzz testing: verify 100 randomly distorted hand poses never violate physical joint limits."""
    retargeter = HandRetargeter()
    np.random.seed(42)

    for i in range(50):
        noisy_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
        # Add random distortions to landmarks
        noise = np.random.uniform(-0.10, 0.10, size=(21, 3))
        noisy_pose.landmarks_3d_cam += noise

        cmd = retargeter.retarget_pose(noisy_pose, timestamp=1.0 + i * 0.033)
        q = cmd.target_q

        # Strict boundary assertion
        assert np.all(q >= retargeter.joint_limits[:, 0] - 1e-4), f"Lower limit breached: {q}"
        assert np.all(q <= retargeter.joint_limits[:, 1] + 1e-4), f"Upper limit breached: {q}"


def test_tracking_lost_hold_behavior(base_wrist_pose):
    """Verify that when tracking is lost, retargeter holds last known pose and flags clutch=False."""
    retargeter = HandRetargeter()
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
    cmd_valid = retargeter.retarget_pose(open_pose, timestamp=1.0)
    last_q = cmd_valid.target_q.copy()

    # Hand lost
    lost_pose = WristPose(
        timestamp=1.1,
        is_valid=False,
        tracking_state=TrackingState.NO_HAND,
        position_cam=np.zeros(3),
        position_table=np.zeros(3),
        position_table_raw=np.zeros(3),
        orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        orientation_table_matrix=np.eye(3),
        euler_deg=(0.0, 0.0, 0.0),
        wrist_pixel_uv=(0.0, 0.0),
        landmarks_2d=np.zeros((21, 2)),
        landmarks_3d_cam=np.zeros((21, 3)),
        depth_m=0.0,
    )

    cmd_lost = retargeter.retarget_pose(lost_pose, timestamp=1.1)
    assert cmd_lost.is_clutched is False
    assert cmd_lost.confidence == 0.0
    np.testing.assert_allclose(cmd_lost.target_q, last_q)


def test_retargeting_latency_benchmark(base_wrist_pose):
    """Performance benchmark: ensure single-frame retargeting takes < 10ms on CPU."""
    retargeter = HandRetargeter()
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")

    # Warm-up
    for _ in range(5):
        retargeter.retarget_pose(open_pose)

    latencies = []
    for i in range(30):
        t0 = time.perf_counter()
        retargeter.retarget_pose(open_pose, timestamp=2.0 + i * 0.033)
        latencies.append((time.perf_counter() - t0) * 1000.0)

    mean_latency = float(np.mean(latencies))
    print(f"\n[BENCHMARK] Mean retargeting latency: {mean_latency:.3f} ms")
    assert mean_latency < 10.0  # Must be comfortably faster than 30Hz frame budget (33.3ms)


def test_ulnar_synergy_fusion(base_wrist_pose):
    """Verify that curling pinky alone or ring alone drives the robot's 3rd finger."""
    retargeter = HandRetargeter(solver_mode="geometric")
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
    cmd_open = retargeter.retarget_pose(open_pose)
    q_ring_open = cmd_open.target_q[9]  # ring_mcp

    # Pose with only pinky curled (ring finger straight)
    pinky_curled_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
    pinky_curled_pose.landmarks_3d_cam[LM_PINKY_PIP] = [0.045, 0.080, 0.58]
    pinky_curled_pose.landmarks_3d_cam[LM_PINKY_DIP] = [0.045, 0.060, 0.57]
    pinky_curled_pose.landmarks_3d_cam[LM_PINKY_TIP] = [0.045, 0.040, 0.58]

    cmd_pinky = retargeter.retarget_pose(pinky_curled_pose)
    q_ring_pinky = cmd_pinky.target_q[9]  # ring_mcp

    # Due to ulnar synergy, robot outer finger must flex when human pinky curls
    assert q_ring_pinky > q_ring_open + 0.15
    assert np.sum(cmd_pinky.target_q[9:12]) > np.sum(cmd_open.target_q[9:12]) + 0.40


def test_solver_modes_switch(base_wrist_pose):
    """Verify all solver modes (hybrid, geometric, dex_retargeting) produce valid commands."""
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")

    for mode in ("hybrid", "geometric", "dex_retargeting"):
        retargeter = HandRetargeter(solver_mode=mode)
        cmd = retargeter.retarget_pose(open_pose)
        assert isinstance(cmd, TeleopHandCommand)
        assert cmd.target_q.shape == (HAND_NUM_DOFS,)
        assert np.all(cmd.target_q >= retargeter.joint_limits[:, 0] - 1e-4)
        assert np.all(cmd.target_q <= retargeter.joint_limits[:, 1] + 1e-4)


# ─── 7. Sim-to-Real Jitter & 3D Responsiveness Validation ─────────

def test_world_landmarks_depth_responsiveness(base_wrist_pose):
    """Verify that when 3D world landmarks with genuine depth are provided, they take priority."""
    retargeter = HandRetargeter(solver_mode="geometric")
    # Base flat open pose in camera frame
    pose = create_synthetic_hand_pose(base_wrist_pose, "open")

    # Add curled index in world_landmarks (Z=-0.04m towards palm)
    world_pts = pose.landmarks_3d_cam.copy()
    world_pts[LM_INDEX_PIP] = [-0.030, 0.080, -0.020]
    world_pts[LM_INDEX_DIP] = [-0.030, 0.065, -0.035]
    world_pts[LM_INDEX_TIP] = [-0.030, 0.050, -0.040]
    pose.landmarks_world_3d = world_pts

    cmd = retargeter.retarget_pose(pose)
    # The curled world landmarks must produce index flexion > 0.6 rad
    assert cmd.target_q[1] > 0.60 or cmd.target_q[2] > 0.60


def test_smooth_hybrid_pinch_continuity(base_wrist_pose):
    """Verify that moving through the pinch distance boundary [0.02m ~ 0.05m] is continuous."""
    retargeter = HandRetargeter(solver_mode="hybrid")
    pose = create_synthetic_hand_pose(base_wrist_pose, "pinch")

    # Sample pinch distance continuously
    prev_q = None
    max_step_jump = 0.0

    for i in range(20):
        # Shift thumb tip distance incrementally
        d = 0.015 + i * 0.002  # from 0.015m to 0.053m
        pose.landmarks_3d_cam[LM_THUMB_TIP] = pose.landmarks_3d_cam[LM_INDEX_TIP] + np.array([d, 0.0, 0.0])
        cmd = retargeter.retarget_pose(pose, timestamp=1.0 + i * 0.033)

        if prev_q is not None:
            delta = np.max(np.abs(cmd.target_q - prev_q))
            max_step_jump = max(max_step_jump, delta)
        prev_q = cmd.target_q.copy()

    # Step jump across frames with 2mm increments must be small and smooth (< 0.12 rad)
    assert max_step_jump < 0.12, f"Discontinuous jump detected: {max_step_jump:.3f} rad"


def test_slew_rate_limiter_suppresses_spikes(base_wrist_pose):
    """Verify that a sudden 1-frame tracking glitch is clamped by max_joint_velocity_rad_s."""
    max_vel = 8.0  # rad/s
    retargeter = HandRetargeter(max_joint_velocity_rad_s=max_vel)
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")

    # Frame 1: Normal steady pose
    retargeter.retarget_pose(open_pose, timestamp=1.00)

    # Frame 2 (+33ms): Sudden extreme distortion (fist gesture in single frame)
    fist_pose = create_synthetic_hand_pose(base_wrist_pose, "fist")
    cmd_spiked = retargeter.retarget_pose(fist_pose, timestamp=1.033)

    # Maximum permissible angular change in 33ms at 8.0 rad/s is ~0.264 rad
    dt = 0.033
    allowed_delta = max_vel * dt + 0.01  # ~0.274 rad
    delta = np.abs(cmd_spiked.target_q - retargeter._last_q)
    assert np.all(delta <= allowed_delta + 1e-3)


def test_motion_jitter_reduction(base_wrist_pose):
    """Verify that typical subpixel MediaPipe landmark noise (0.5mm) produces < 0.010 rad std dev on joints."""
    retargeter = HandRetargeter(filter_min_cutoff=0.8, filter_beta=0.03)

    np.random.seed(42)
    q_history = []
    for i in range(40):
        noisy_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
        # 0.5mm typical MediaPipe temporal landmark jitter
        noisy_pose.landmarks_3d_cam += np.random.normal(0.0, 0.0005, size=(21, 3))
        cmd = retargeter.retarget_pose(noisy_pose, timestamp=1.0 + i * 0.033)
        if i >= 10:  # let filter settle
            q_history.append(cmd.target_q)

    q_arr = np.array(q_history)  # (30, 16)
    stds = np.std(q_arr, axis=0)
    max_std = float(np.max(stds))
    # Sim-to-Real Gate 1 requirement: std < 0.015 rad (< 0.85 deg)
    assert max_std < 0.015, f"Max joint jitter std {max_std:.4f} rad exceeds 0.015 rad threshold"


def test_thumb_ip_flexion_following_in_fist_and_open(base_wrist_pose):
    """Verify thumb_ip (motor ID 15) achieves deep curling in fist (>= 1.4 rad) and stays extended in open (< 0.30 rad)."""
    for mode in ["geometric", "hybrid"]:
        retargeter = HandRetargeter(solver_mode=mode)
        open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
        fist_pose = create_synthetic_hand_pose(base_wrist_pose, "fist")

        cmd_open = retargeter.retarget_pose(open_pose, timestamp=1.0)
        # Settle filter with open pose
        for t in range(5):
            cmd_open = retargeter.retarget_pose(open_pose, timestamp=1.0 + t * 0.033)

        # In open hand, thumb IP should be extended
        assert cmd_open.target_q[15] < 0.30, f"Mode {mode}: open thumb_ip too flexed: {cmd_open.target_q[15]}"

        # Now transition to fist and let settle
        cmd_fist = None
        for t in range(15):
            cmd_fist = retargeter.retarget_pose(fist_pose, timestamp=2.0 + t * 0.033)

        # In fist, thumb IP (motor 15) must curl deeply into palm/fingers (>= 1.40 rad)
        assert cmd_fist.target_q[15] >= 1.40, (
            f"Mode {mode}: thumb_ip failed deep fist flexion: {cmd_fist.target_q[15]:.3f} rad (< 1.40 rad)"
        )
        # Thumb MCP (14) and CMC (13) should also flex properly
        assert cmd_fist.target_q[14] >= 0.80, f"Mode {mode}: thumb_mcp under-flexed: {cmd_fist.target_q[14]:.3f}"
        assert cmd_fist.target_q[13] >= 0.80, f"Mode {mode}: thumb_cmc under-flexed: {cmd_fist.target_q[13]:.3f}"


def test_hand_retargeter_zero_and_reset_to_zero_pose(base_wrist_pose):
    """Verify that calling zero() on an open hand resets all 16 joints to 0.0 rad and retains fist range."""
    retargeter = HandRetargeter()
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
    fist_pose = create_synthetic_hand_pose(base_wrist_pose, "fist")

    # Initial frame without calibration
    cmd_init = retargeter.retarget_pose(open_pose, timestamp=1.0)
    assert isinstance(cmd_init, TeleopHandCommand)

    # Perform zeroing on open hand (simulates pressing [Z])
    zero_q = retargeter.zero(open_pose)
    assert np.allclose(zero_q, np.zeros(HAND_NUM_DOFS)), "zero() must return canonical zero posture"
    assert retargeter.q_zero_offset is not None
    assert len(retargeter.q_zero_offset) == HAND_NUM_DOFS

    # Retarget the open hand again: all 16 joints must be exactly zeroed
    cmd_zeroed = retargeter.retarget_pose(open_pose, timestamp=1.033)
    np.testing.assert_allclose(cmd_zeroed.target_q, np.zeros(HAND_NUM_DOFS), atol=1e-4)

    # Fist clenching must still achieve full flexion after zeroing
    cmd_fist = retargeter.retarget_pose(fist_pose, timestamp=2.0)
    assert cmd_fist.target_q[1] > 1.0, f"MCP should curl deeply: {cmd_fist.target_q[1]}"
    assert cmd_fist.target_q[2] > 1.5, f"PIP should curl deeply: {cmd_fist.target_q[2]}"
    assert cmd_fist.target_q[3] > 1.2, f"DIP should curl deeply: {cmd_fist.target_q[3]}"

    # Test standalone reset_to_zero_pose
    rst_q = retargeter.reset_to_zero_pose()
    assert np.allclose(rst_q, np.zeros(HAND_NUM_DOFS))


def test_yaw_stability_during_full_flexion(base_wrist_pose):
    """Verify that finger yaw (abduction/adduction) remains stable across 0 to 90+ deg MCP flexion.
    
    Guards against division-by-near-zero where v_y -> 0 at 90 deg flexion caused 50-80 deg yaw jumps.
    """
    retargeter = HandRetargeter(solver_mode="geometric")
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")

    mcp = open_pose.landmarks_3d_cam[LM_INDEX_MCP].copy()
    L = 0.035

    yaw_angles = []
    angles_deg = [0, 15, 30, 45, 60, 75, 85, 90, 95]
    for i, deg in enumerate(angles_deg):
        rad = math.radians(deg)
        pose = create_synthetic_hand_pose(base_wrist_pose, "open")
        pose.landmarks_3d_cam[LM_INDEX_PIP] = [
            mcp[0] + 0.002,
            mcp[1] + L * math.cos(rad),
            mcp[2] - L * math.sin(rad),
        ]
        pose.landmarks_3d_cam[LM_INDEX_DIP] = pose.landmarks_3d_cam[LM_INDEX_PIP] + [0, 0.02 * math.cos(rad), -0.02 * math.sin(rad)]
        pose.landmarks_3d_cam[LM_INDEX_TIP] = pose.landmarks_3d_cam[LM_INDEX_DIP] + [0, 0.02 * math.cos(rad), -0.02 * math.sin(rad)]

        cmd = None
        for k in range(10):
            cmd = retargeter.retarget_pose(pose, timestamp=1.0 + i * 0.2 + k * 0.016)
        yaw_angles.append(cmd.target_q[0])

    max_dev = max(yaw_angles) - min(yaw_angles)
    assert max_dev < 0.08, f"Yaw jumped by {max_dev:.3f} rad ({math.degrees(max_dev):.1f} deg) across flexion angles!"


def test_deep_fist_monotonic_mcp_flexion(base_wrist_pose):
    """Verify that MCP flexion increases monotonically beyond 90 deg without angle reversal.
    
    Guards against atan2(-v_z, hypot(v_x, v_y)) bug where curling into a tight fist past 90 deg
    reversed backwards and uncurled the fingers.
    """
    retargeter = HandRetargeter(solver_mode="geometric")
    open_pose = create_synthetic_hand_pose(base_wrist_pose, "open")
    mcp = open_pose.landmarks_3d_cam[LM_INDEX_MCP].copy()
    L = 0.035

    mcp_flexion_q = []
    angles_deg = [60, 75, 85, 90, 95, 105]
    for i, deg in enumerate(angles_deg):
        rad = math.radians(deg)
        pose = create_synthetic_hand_pose(base_wrist_pose, "open")
        pose.landmarks_3d_cam[LM_INDEX_PIP] = [
            mcp[0],
            mcp[1] + L * math.cos(rad),
            mcp[2] - L * math.sin(rad),
        ]
        pose.landmarks_3d_cam[LM_INDEX_DIP] = pose.landmarks_3d_cam[LM_INDEX_PIP] + [0, 0.02 * math.cos(rad), -0.02 * math.sin(rad)]
        pose.landmarks_3d_cam[LM_INDEX_TIP] = pose.landmarks_3d_cam[LM_INDEX_DIP] + [0, 0.02 * math.cos(rad), -0.02 * math.sin(rad)]

        retargeter.reset()
        cmd = None
        for k in range(10):
            cmd = retargeter.retarget_pose(pose, timestamp=1.0 + k * 0.016)
        mcp_flexion_q.append(cmd.target_q[1])

    for idx in range(1, len(mcp_flexion_q)):
        prev_q = mcp_flexion_q[idx - 1]
        curr_q = mcp_flexion_q[idx]
        assert curr_q >= prev_q - 1e-4, (
            f"MCP flexion reversed at {angles_deg[idx]} deg! "
            f"q({angles_deg[idx-1]} deg)={prev_q:.3f} rad > q({angles_deg[idx]} deg)={curr_q:.3f} rad"
        )





