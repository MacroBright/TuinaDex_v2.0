"""test_wrist_tracker.py — Comprehensive unit tests for 3D wrist tracking, depth filtering, and temporal hold."""
from __future__ import annotations

import math
import numpy as np
import pytest

from tuinadex_teleop.core.types import ArUcoCalibrationData
from tuinadex_teleop.perception.realsense_stream import CameraIntrinsics
from tuinadex_teleop.perception.wrist_tracker import (
    LM_WRIST,
    LM_INDEX_MCP,
    LM_MIDDLE_MCP,
    LM_PINKY_MCP,
    TrackingState,
    WristPose,
    WristTracker,
    compute_palm_basis,
    draw_wrist_hud,
    median_depth_at,
    rotation_matrix_to_euler_deg,
)


@pytest.fixture
def standard_intrinsics() -> CameraIntrinsics:
    """Fixture providing typical RealSense D455 intrinsics."""
    return CameraIntrinsics(
        fx=381.27,
        fy=380.95,
        cx=312.91,
        cy=238.57,
        width=640,
        height=480,
    )


@pytest.fixture
def calibrated_table() -> ArUcoCalibrationData:
    """Fixture providing a mock table calibration frame shifted by 0.5m in Z and 0.1m in X."""
    T_table_to_cam = np.eye(4, dtype=np.float64)
    # Camera is 0.7m above table looking down: table origin is at camera [0, 0, 0.7]
    # So P_table = P_cam - [0.1, 0.2, 0.7]
    T_table_to_cam[0, 3] = -0.10
    T_table_to_cam[1, 3] = -0.20
    T_table_to_cam[2, 3] = -0.70

    T_cam_to_table = np.linalg.inv(T_table_to_cam)

    return ArUcoCalibrationData(
        t_cam_to_table=T_cam_to_table,
        t_table_to_cam=T_table_to_cam,
        tag_id=0,
        is_valid=True,
    )


# ─── 1. Depth Patch Median Filter Tests ───────────────────────────

def test_median_depth_filter_uniform():
    """Verify uniform depth patch extracts exact depth."""
    depth = np.ones((100, 100), dtype=np.float32) * 0.85
    val = median_depth_at(depth, u=50.0, v=50.0, patch_radius=3)
    assert math.isclose(val, 0.85, rel_tol=1e-4)


def test_median_depth_filter_outlier_rejection():
    """Verify noisy pixels (0, negative, >2.5m, nan) are rejected in favor of true median."""
    depth = np.zeros((50, 50), dtype=np.float32)
    # Center pixel is 0.0 (near-field drop)
    # Surrounding pixels contain true depths and high/low outliers
    depth[23:28, 23:28] = np.array([
        [0.0,  0.80,  0.82,  0.00,  3.50],
        [0.79, 0.81,  0.80,  0.83,  0.00],
        [0.00, 0.80,  0.00,  0.81,  np.nan],
        [0.82, 0.79,  0.81,  0.80,  0.00],
        [5.00, 0.00,  0.80,  0.82,  0.00],
    ], dtype=np.float32)

    val = median_depth_at(depth, u=25.0, v=25.0, patch_radius=2, min_depth_m=0.15, max_depth_m=2.5)
    assert math.isfinite(val)
    # Median of [0.79, 0.79, 0.80, 0.80, 0.80, 0.80, 0.80, 0.81, 0.81, 0.81, 0.82, 0.82, 0.82, 0.83] is 0.805
    assert 0.79 <= val <= 0.82


def test_median_depth_filter_expansion():
    """Verify patch expands when small patch is empty or all zeros."""
    depth = np.zeros((100, 100), dtype=np.float32)
    # Small 7x7 patch around (50, 50) is all 0.0 (hole)
    # At radius 6 (13x13 window), valid depths exist
    depth[44, 44] = 0.65
    depth[44, 56] = 0.67
    depth[56, 44] = 0.66

    val = median_depth_at(depth, u=50.0, v=50.0, patch_radius=3, max_radius=10)
    assert math.isfinite(val)
    assert 0.64 <= val <= 0.68


def test_median_depth_filter_all_invalid():
    """Verify all zeros or NaN returns float('nan')."""
    depth = np.zeros((50, 50), dtype=np.float32)
    val = median_depth_at(depth, u=25.0, v=25.0)
    assert math.isnan(val)

    # Out of bounds
    val_oob = median_depth_at(depth, u=-10.0, v=25.0)
    assert math.isnan(val_oob)


# ─── 2. Palm Basis & Coordinate Geometry Tests ─────────────────────

def test_palm_basis_orthonormality():
    """Verify computed palm basis is an orthonormal right-handed frame (det(R) == +1.0)."""
    pts = np.zeros((21, 3), dtype=np.float64)
    # Hand facing camera at 0.6m distance
    # Wrist at origin of hand: [0.0, 0.0, 0.60]
    # Middle MCP: forward along +Y by 0.09m -> [0.0, 0.09, 0.60]
    # Index MCP: left (-X) by 0.035m -> [-0.035, 0.08, 0.60]
    # Pinky MCP: right (+X) by 0.035m -> [+0.035, 0.07, 0.60]
    pts[LM_WRIST] = [0.0, 0.0, 0.60]
    pts[LM_MIDDLE_MCP] = [0.0, 0.09, 0.60]
    pts[LM_INDEX_MCP] = [-0.035, 0.08, 0.60]
    pts[LM_PINKY_MCP] = [0.035, 0.07, 0.60]

    x_palm, y_palm, z_palm, R = compute_palm_basis(pts, handedness="Right")

    # Unit norms
    assert math.isclose(np.linalg.norm(x_palm), 1.0, abs_tol=1e-5)
    assert math.isclose(np.linalg.norm(y_palm), 1.0, abs_tol=1e-5)
    assert math.isclose(np.linalg.norm(z_palm), 1.0, abs_tol=1e-5)

    # Orthogonality
    assert math.isclose(float(np.dot(x_palm, y_palm)), 0.0, abs_tol=1e-5)
    assert math.isclose(float(np.dot(y_palm, z_palm)), 0.0, abs_tol=1e-5)
    assert math.isclose(float(np.dot(z_palm, x_palm)), 0.0, abs_tol=1e-5)

    # Proper rotation (right-handed determinant = +1.0)
    det = np.linalg.det(R)
    assert math.isclose(det, 1.0, abs_tol=1e-5)


def test_rotation_matrix_to_euler():
    """Verify Euler angles extract correct pitch and yaw."""
    # Identity rotation: forward is along +Y, up is +Z
    R_ident = np.eye(3, dtype=np.float64)
    r, p, y = rotation_matrix_to_euler_deg(R_ident)
    assert math.isclose(p, 0.0, abs_tol=1e-4)
    assert math.isclose(y, 90.0, abs_tol=1e-4)  # Y axis is at 90 deg from X axis


# ─── 3. WristPose Dataclass Tests ─────────────────────────────────

def test_wrist_pose_dataclass():
    """Verify WristPose contracts and SE(3) matrix representation."""
    pose = WristPose(
        timestamp=10.0,
        is_valid=True,
        tracking_state=TrackingState.TRACKING,
        position_cam=np.array([0.1, 0.2, 0.6]),
        position_table=np.array([0.0, 0.0, 0.1]),
        position_table_raw=np.array([0.0, 0.0, 0.1]),
        orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        orientation_table_matrix=np.eye(3),
        euler_deg=(0.0, 0.0, 0.0),
        wrist_pixel_uv=(320.0, 240.0),
        landmarks_2d=np.zeros((21, 2)),
        landmarks_3d_cam=np.zeros((21, 3)),
        depth_m=0.6,
    )
    assert pose.is_valid is True
    assert pose.tracking_state == TrackingState.TRACKING

    T = pose.as_4x4_table_pose()
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[:3, 3], [0.0, 0.0, 0.1])
    np.testing.assert_allclose(T[:3, :3], np.eye(3))


# ─── 4. Temporal Hold & Anti-Depth-Hole State Machine Tests ─────────

def test_wrist_tracker_temporal_hold_cycle(standard_intrinsics, calibrated_table):
    """Test full cycle: TRACKING -> TEMPORAL_HOLD -> TIMEOUT LOST -> RECOVERY."""
    tracker = WristTracker(max_hold_time_s=0.30)

    # Synthetic hand detector returning fixed landmark positions
    def mock_detector(img):
        lm = np.zeros((21, 3), dtype=np.float32)
        lm[LM_WRIST] = [0.50, 0.50, 0.0]
        lm[LM_MIDDLE_MCP] = [0.50, 0.40, 0.0]
        lm[LM_INDEX_MCP] = [0.45, 0.42, 0.0]
        lm[LM_PINKY_MCP] = [0.55, 0.43, 0.0]
        return {"landmarks": lm, "handedness": "Right"}

    tracker.set_custom_detector(mock_detector)

    dummy_color = np.zeros((480, 640, 3), dtype=np.uint8)
    valid_depth = np.ones((480, 640), dtype=np.float32) * 0.60
    hole_depth = np.zeros((480, 640), dtype=np.float32)  # All zeros (complete depth hole)

    # 1. Frame 1: Valid depth -> TRACKING
    p1 = tracker.process_frame(dummy_color, valid_depth, standard_intrinsics, calibrated_table, timestamp=1.00)
    assert p1.tracking_state == TrackingState.TRACKING
    assert p1.is_valid is True
    assert math.isclose(p1.depth_m, 0.60, abs_tol=1e-3)
    pos1_table = p1.position_table.copy()

    # 2. Frame 2 (+0.10s): Depth drops to zero (depth hole) -> TEMPORAL_HOLD
    p2 = tracker.process_frame(dummy_color, hole_depth, standard_intrinsics, calibrated_table, timestamp=1.10)
    assert p2.tracking_state == TrackingState.TEMPORAL_HOLD
    assert p2.is_valid is True
    assert math.isclose(p2.hold_duration, 0.10, abs_tol=1e-3)
    # Position should be held smoothly without sudden jumps
    np.testing.assert_allclose(p2.position_table, pos1_table, atol=0.02)

    # 3. Frame 3 (+0.25s): Depth hole persists -> still TEMPORAL_HOLD (< 0.3s)
    p3 = tracker.process_frame(dummy_color, hole_depth, standard_intrinsics, calibrated_table, timestamp=1.25)
    assert p3.tracking_state == TrackingState.TEMPORAL_HOLD
    assert p3.is_valid is True
    assert math.isclose(p3.hold_duration, 0.25, abs_tol=1e-3)

    # 4. Frame 4 (+0.40s): Depth hole persists beyond 0.30s -> transitions to LOST
    p4 = tracker.process_frame(dummy_color, hole_depth, standard_intrinsics, calibrated_table, timestamp=1.40)
    assert p4.tracking_state == TrackingState.LOST
    assert p4.is_valid is False

    # 5. Frame 5 (+0.45s): Depth recovers -> returns immediately to TRACKING
    p5 = tracker.process_frame(dummy_color, valid_depth, standard_intrinsics, calibrated_table, timestamp=1.45)
    assert p5.tracking_state == TrackingState.TRACKING
    assert p5.is_valid is True
    assert p5.hold_duration == 0.0


# ─── 5. Coordinate Transformation Verification ─────────────────────

def test_camera_to_table_transformation(standard_intrinsics, calibrated_table):
    """Verify camera frame wrist point converts accurately to calibrated table frame."""
    tracker = WristTracker()

    # Place hand at camera optical center: u=cx, v=cy, depth=0.6m
    cx = standard_intrinsics.cx
    cy = standard_intrinsics.cy

    def center_hand_detector(img):
        lm = np.zeros((21, 3), dtype=np.float32)
        lm[LM_WRIST] = [cx / 640.0, cy / 480.0, 0.0]
        lm[LM_MIDDLE_MCP] = [cx / 640.0, (cy - 40) / 480.0, 0.0]
        lm[LM_INDEX_MCP] = [(cx - 20) / 640.0, (cy - 35) / 480.0, 0.0]
        lm[LM_PINKY_MCP] = [(cx + 20) / 640.0, (cy - 35) / 480.0, 0.0]
        return {"landmarks": lm, "handedness": "Right"}

    tracker.set_custom_detector(center_hand_detector)

    dummy_color = np.zeros((480, 640, 3), dtype=np.uint8)
    depth_06 = np.ones((480, 640), dtype=np.float32) * 0.60

    pose = tracker.process_frame(dummy_color, depth_06, standard_intrinsics, calibrated_table, timestamp=1.0)
    assert pose.is_valid is True

    # In camera frame: at (cx, cy) and Z=0.6m -> X_cam=0, Y_cam=0, Z_cam=0.6m
    np.testing.assert_allclose(pose.position_cam, [0.0, 0.0, 0.60], atol=1e-3)

    # In calibrated table frame: P_table = P_cam - [0.10, 0.20, 0.70] = [-0.10, -0.20, -0.10]
    expected_table_pos = np.array([-0.10, -0.20, -0.10])
    np.testing.assert_allclose(pose.position_table_raw, expected_table_pos, atol=1e-3)


# ─── 6. HUD Drawing Verification ──────────────────────────────────

def test_draw_wrist_hud_execution(standard_intrinsics):
    """Ensure HUD rendering completes with no exceptions across all tracking states."""
    dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)

    for state in (TrackingState.TRACKING, TrackingState.TEMPORAL_HOLD, TrackingState.NO_HAND, TrackingState.LOST):
        pose = WristPose(
            timestamp=1.0,
            is_valid=(state in (TrackingState.TRACKING, TrackingState.TEMPORAL_HOLD)),
            tracking_state=state,
            position_cam=np.array([0.05, 0.02, 0.65]),
            position_table=np.array([0.15, 0.25, 0.08]),
            position_table_raw=np.array([0.15, 0.25, 0.08]),
            orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            orientation_table_matrix=np.eye(3),
            euler_deg=(5.0, -10.0, 45.0),
            wrist_pixel_uv=(320.0, 240.0),
            landmarks_2d=np.ones((21, 2)) * 200.0,
            landmarks_3d_cam=np.ones((21, 3)) * 0.6,
            depth_m=0.65,
            hold_duration=0.15 if state == TrackingState.TEMPORAL_HOLD else 0.0,
        )

        out_img = draw_wrist_hud(dummy_img, pose, intrinsics=standard_intrinsics, draw_skeleton=True)
        assert out_img.shape == (480, 640, 3)
        assert out_img.dtype == np.uint8


def test_wrist_tracker_handedness_consensus_lock(standard_intrinsics):
    """Verify that temporal consensus lock filters out transient single-frame handedness inversions."""
    tracker = WristTracker(default_handedness="Right")
    dummy_color = np.zeros((480, 640, 3), dtype=np.uint8)
    dummy_depth = np.ones((480, 640), dtype=np.float32) * 0.60

    current_hand = "Right"

    def mock_det(img):
        lm = np.zeros((21, 3), dtype=np.float32)
        lm[LM_WRIST] = [0.5, 0.5, 0.0]
        lm[LM_MIDDLE_MCP] = [0.5, 0.4, 0.0]
        lm[LM_INDEX_MCP] = [0.45, 0.42, 0.0]
        lm[LM_PINKY_MCP] = [0.55, 0.42, 0.0]
        return {"landmarks": lm, "handedness": current_hand, "score": 0.95}

    tracker.set_custom_detector(mock_det)

    # Frame 1: Tracking Right hand
    pose = tracker.process_frame(dummy_color, dummy_depth, standard_intrinsics, timestamp=1.0)
    assert pose.handedness == "Right"

    # Frame 2: Transient glitch (MediaPipe misclassifies single frame as Left)
    current_hand = "Left"
    pose = tracker.process_frame(dummy_color, dummy_depth, standard_intrinsics, timestamp=1.016)
    assert pose.handedness == "Right", "Transient Left glitch must not flip locked Right handedness"

    # Feed 8 more frames of Left (total 9 frames) -> still locked as Right (< 10 threshold)
    for i in range(8):
        pose = tracker.process_frame(dummy_color, dummy_depth, standard_intrinsics, timestamp=1.033 + i * 0.016)
        assert pose.handedness == "Right"

    # 10th consecutive frame of Left -> consensus reached, switches to Left
    pose = tracker.process_frame(dummy_color, dummy_depth, standard_intrinsics, timestamp=1.20)
    assert pose.handedness == "Left", "10 consecutive high-confidence frames must switch handedness"


