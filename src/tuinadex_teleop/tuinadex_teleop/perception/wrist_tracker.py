"""wrist_tracker.py — 3D Wrist Tracking, Anti-Depth-Hole Filtering & Table Transformation.

Provides:
- MediaPipe Tasks HandLandmarker 21-keypoint extraction.
- Multi-scale patch median depth filtering (7x7 -> 15x15 -> 25x25) with outlier rejection.
- Temporal Hold mechanism for robust handling of depth holes / contact occlusions (<300ms).
- Transformation of wrist 3D position and palm orientation to the calibrated Table Frame.
- 1€ (One-Euro) adaptive filtering for jitter-free 3D position and quaternion SLERP smoothing.
- Real-time diagnostic HUD overlay and visualization.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import math
from pathlib import Path
import time
from typing import Callable, List, Optional, Tuple, Union

import cv2
import numpy as np

from ..core.filters import OneEuroFilter, slerp
from ..core.types import ArUcoCalibrationData
from .realsense_stream import CameraIntrinsics
from .table_calibrator import (
    rotation_matrix_to_quaternion,
    quaternion_to_rotation_matrix,
)

# MediaPipe Tasks API conditional import
try:
    from mediapipe import Image as MpImage, ImageFormat
    from mediapipe.tasks.python import BaseOptions
    from mediapipe.tasks.python.vision import (
        HandLandmarker,
        HandLandmarkerOptions,
        RunningMode,
    )
    _HAS_MEDIAPIPE = True
except ImportError:
    import os
    import sys
    _candidate = os.path.expanduser(
        "~/win_office/conda/envs/leap_hand/lib/python3.14/site-packages"
    )
    if os.path.isdir(_candidate) and _candidate not in sys.path:
        sys.path.append(_candidate)
    try:
        from mediapipe import Image as MpImage, ImageFormat
        from mediapipe.tasks.python import BaseOptions
        from mediapipe.tasks.python.vision import (
            HandLandmarker,
            HandLandmarkerOptions,
            RunningMode,
        )
        _HAS_MEDIAPIPE = True
    except ImportError:
        _HAS_MEDIAPIPE = False


# Canonical 21 MediaPipe hand landmark indices
LM_WRIST = 0
LM_THUMB_CMC = 1
LM_THUMB_MCP = 2
LM_THUMB_IP = 3
LM_THUMB_TIP = 4
LM_INDEX_MCP = 5
LM_INDEX_PIP = 6
LM_INDEX_DIP = 7
LM_INDEX_TIP = 8
LM_MIDDLE_MCP = 9
LM_MIDDLE_PIP = 10
LM_MIDDLE_DIP = 11
LM_MIDDLE_TIP = 12
LM_RING_MCP = 13
LM_RING_PIP = 14
LM_RING_DIP = 15
LM_RING_TIP = 16
LM_PINKY_MCP = 17
LM_PINKY_PIP = 18
LM_PINKY_DIP = 19
LM_PINKY_TIP = 20

# Standard hand skeleton connections for visualization
HAND_CONNECTIONS = [
    # Thumb
    (LM_WRIST, LM_THUMB_CMC), (LM_THUMB_CMC, LM_THUMB_MCP),
    (LM_THUMB_MCP, LM_THUMB_IP), (LM_THUMB_IP, LM_THUMB_TIP),
    # Index
    (LM_WRIST, LM_INDEX_MCP), (LM_INDEX_MCP, LM_INDEX_PIP),
    (LM_INDEX_PIP, LM_INDEX_DIP), (LM_INDEX_DIP, LM_INDEX_TIP),
    # Middle
    (LM_WRIST, LM_MIDDLE_MCP), (LM_MIDDLE_MCP, LM_MIDDLE_PIP),
    (LM_MIDDLE_PIP, LM_MIDDLE_DIP), (LM_MIDDLE_DIP, LM_MIDDLE_TIP),
    # Ring
    (LM_WRIST, LM_RING_MCP), (LM_RING_MCP, LM_RING_PIP),
    (LM_RING_PIP, LM_RING_DIP), (LM_RING_DIP, LM_RING_TIP),
    # Pinky
    (LM_WRIST, LM_PINKY_MCP), (LM_PINKY_MCP, LM_PINKY_PIP),
    (LM_PINKY_PIP, LM_PINKY_DIP), (LM_PINKY_DIP, LM_PINKY_TIP),
    # Palm arch
    (LM_INDEX_MCP, LM_MIDDLE_MCP), (LM_MIDDLE_MCP, LM_RING_MCP),
    (LM_RING_MCP, LM_PINKY_MCP),
]


class TrackingState(Enum):
    """Operational state of the 3D wrist tracking pipeline."""
    NO_HAND = auto()        # No human hand detected in frame
    TRACKING = auto()       # Active tracking with fresh direct depth measurements
    TEMPORAL_HOLD = auto()  # Hand detected but depth in hole/occlusion; holding last valid 3D pose
    LOST = auto()           # Tracking lost or temporal hold exceeded timeout limit


@dataclass
class WristPose:
    """Complete 3D spatial and kinematic state of tracked wrist."""
    timestamp: float
    is_valid: bool
    tracking_state: TrackingState
    position_cam: np.ndarray                   # [X, Y, Z] (3,) meters in camera frame
    position_table: np.ndarray                # [X, Y, Z] (3,) meters in table frame (filtered)
    position_table_raw: np.ndarray            # [X, Y, Z] (3,) meters in table frame (unfiltered)
    orientation_table_quat: np.ndarray        # [w, x, y, z] (4,) unit quaternion in table frame
    orientation_table_matrix: np.ndarray      # 3x3 rotation matrix in table frame
    euler_deg: Tuple[float, float, float]     # (roll, pitch, yaw) in degrees in table frame
    wrist_pixel_uv: Tuple[float, float]       # (u, v) pixel coordinates
    landmarks_2d: np.ndarray                  # (21, 2) pixel coordinates
    landmarks_3d_cam: np.ndarray              # (21, 3) 3D camera coordinates in meters
    depth_m: float                            # Estimated metric depth at wrist
    hold_duration: float = 0.0                # Elapsed seconds in TEMPORAL_HOLD state
    handedness: str = "Right"                 # "Right" or "Left"
    landmarks_world_3d: Optional[np.ndarray] = None  # (21, 3) metric 3D keypoints from MediaPipe

    def as_4x4_table_pose(self) -> np.ndarray:
        """Return 4x4 SE(3) transformation matrix in Table Frame."""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.orientation_table_matrix
        T[:3, 3] = self.position_table
        return T


def find_default_model_path() -> Optional[Path]:
    """Search candidate paths for MediaPipe hand_landmarker.task model file."""
    candidates = [
        Path(__file__).resolve().parent / "models" / "hand_landmarker.task",
        Path("/home/bright/win_office/ubantu_files/project/TuinaDex_v2.0/src/tuinadex_teleop/tuinadex_teleop/tracking/models/hand_landmarker.task"),
        Path("/home/bright/win_office/ubantu_files/project/TuinaDex/Arm-robot_VLA/src/arm_robot/vision/models/hand_landmarker.task"),
    ]
    for p in candidates:
        if p.exists():
            return p
    return None


def median_depth_at(
    depth_m: np.ndarray,
    u: float,
    v: float,
    patch_radius: int = 3,
    max_radius: int = 12,
    min_depth_m: float = 0.15,
    max_depth_m: float = 2.50,
) -> float:
    """Multi-scale patch median depth extraction in metric meters.

    Args:
        depth_m: (H, W) float32 numpy array with depth in meters.
        u: Horizontal pixel coordinate.
        v: Vertical pixel coordinate.
        patch_radius: Initial patch radius (e.g. 3 -> 7x7 patch).
        max_radius: Maximum expansion radius (e.g. 12 -> 25x25 patch).
        min_depth_m: Minimum valid depth threshold (filters out camera near-field noise).
        max_depth_m: Maximum valid depth threshold (filters out far background).

    Returns:
        Median depth in meters, or float("nan") if no valid pixels found.
    """
    if depth_m is None or not np.isfinite(u) or not np.isfinite(v):
        return float("nan")

    h, w = depth_m.shape[:2]
    int_u = int(round(u))
    int_v = int(round(v))

    if int_u < 0 or int_u >= w or int_v < 0 or int_v >= h:
        return float("nan")

    # Multi-scale patch search radii
    radii = [patch_radius, (patch_radius + max_radius) // 2, max_radius]
    for r in radii:
        u0 = max(0, int_u - r)
        u1 = min(w, int_u + r + 1)
        v0 = max(0, int_v - r)
        v1 = min(h, int_v + r + 1)

        patch = depth_m[v0:v1, u0:u1]
        valid_mask = (patch >= min_depth_m) & (patch <= max_depth_m) & np.isfinite(patch)
        valid_vals = patch[valid_mask]

        if valid_vals.size > 0:
            return float(np.median(valid_vals))

    return float("nan")


def compute_palm_basis(
    pts_cam: np.ndarray,
    handedness: str = "Right",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute orthonormal palm coordinate frame from 3D hand keypoints.

    Coordinate Frame Convention:
    - Y_palm: Longitudinal axis from Wrist (0) to Middle MCP (9), normalized.
    - Z_palm: Dorsal normal vector pointing outward from the back of the palm.
    - X_palm: Lateral axis across the palm, completing the right-handed system (X = Y x Z).

    Args:
        pts_cam: (21, 3) 3D coordinates in meters.
        handedness: "Right" or "Left".

    Returns:
        Tuple of (X_palm, Y_palm, Z_palm, R_cam_3x3).
    """
    p_wrist = pts_cam[LM_WRIST]
    p_middle_mcp = pts_cam[LM_MIDDLE_MCP]
    p_index_mcp = pts_cam[LM_INDEX_MCP]
    p_pinky_mcp = pts_cam[LM_PINKY_MCP]

    # Forward vector (wrist to middle MCP)
    f = p_middle_mcp - p_wrist
    f_norm = np.linalg.norm(f)
    if f_norm < 1e-6:
        y_palm = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    else:
        y_palm = f / f_norm

    # Lateral vector across palm
    if handedness.lower() == "left":
        across = p_index_mcp - p_pinky_mcp
    else:
        across = p_pinky_mcp - p_index_mcp

    across_norm = np.linalg.norm(across)
    if across_norm < 1e-6:
        across = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        across = across / across_norm

    # Dorsal normal vector Z = across x forward
    z_raw = np.cross(across, y_palm)
    z_norm = np.linalg.norm(z_raw)
    if z_norm < 1e-6:
        z_palm = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        z_palm = z_raw / z_norm

    # Gram-Schmidt re-orthogonalization: X = Y x Z
    x_palm = np.cross(y_palm, z_palm)
    x_norm = np.linalg.norm(x_palm)
    if x_norm < 1e-6:
        x_palm = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        x_palm = x_palm / x_norm

    R = np.column_stack([x_palm, y_palm, z_palm])
    return x_palm, y_palm, z_palm, R


def rotation_matrix_to_euler_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract intuitive Euler angles (Roll, Pitch, Yaw) in degrees from 3x3 rotation matrix.

    In Table Frame:
    - Pitch: hand pointing up (+) or down toward table (-)
    - Yaw: hand azimuth heading on the table XY plane
    - Roll: hand axial rotation about longitudinal axis
    """
    # Y axis forward vector
    y_fwd = R[:, 1]
    pitch_rad = math.asin(max(-1.0, min(1.0, float(y_fwd[2]))))
    yaw_rad = math.atan2(float(y_fwd[1]), float(y_fwd[0]))

    # Roll about forward axis
    z_up = R[:, 2]
    roll_rad = math.atan2(float(z_up[0]), float(z_up[2]))

    return (
        math.degrees(roll_rad),
        math.degrees(pitch_rad),
        math.degrees(yaw_rad),
    )


class WristTracker:
    """Robust 3D Wrist Tracker with MediaPipe, depth patch filtering, and temporal hold."""

    def __init__(
        self,
        model_path: Optional[str] = None,
        max_hold_time_s: float = 0.30,
        filter_min_cutoff: float = 1.0,
        filter_beta: float = 0.05,
        quat_slerp_alpha: float = 0.40,
        min_depth_m: float = 0.15,
        max_depth_m: float = 2.50,
        depth_patch_radius: int = 3,
        default_handedness: str = "Right",
    ):
        self.max_hold_time_s = max_hold_time_s
        self.filter_min_cutoff = filter_min_cutoff
        self.filter_beta = filter_beta
        self.quat_slerp_alpha = quat_slerp_alpha
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.depth_patch_radius = depth_patch_radius
        self.default_handedness = default_handedness

        # Find model file
        resolved_model = Path(model_path) if model_path else find_default_model_path()
        self.model_path = str(resolved_model) if resolved_model else None

        # MediaPipe landmarker
        self._landmarker: Optional[HandLandmarker] = None
        self._last_video_ts_ms = 0
        if _HAS_MEDIAPIPE and self.model_path and Path(self.model_path).exists():
            options = HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=self.model_path),
                running_mode=RunningMode.VIDEO,
                num_hands=1,
                min_hand_detection_confidence=0.35,
                min_hand_presence_confidence=0.30,
                min_tracking_confidence=0.30,
            )
            self._landmarker = HandLandmarker.create_from_options(options)

        # Filters
        self._pos_filter = OneEuroFilter(
            n_dim=3,
            freq_hz=30.0,
            min_cutoff=self.filter_min_cutoff,
            beta=self.filter_beta,
        )
        self._filtered_quat: Optional[np.ndarray] = None

        # State tracking & temporal hold memory
        self._last_valid_pose: Optional[WristPose] = None
        self._last_valid_timestamp: float = 0.0
        self._hold_start_timestamp: Optional[float] = None

        # Custom / Mock detector override (for headless unit tests)
        self._custom_detector: Optional[Callable[[np.ndarray], Optional[dict]]] = None

    def set_custom_detector(
        self,
        detector: Optional[Callable[[np.ndarray], Optional[dict]]],
    ) -> None:
        """Inject a custom detection callback for testing and offline simulation."""
        self._custom_detector = detector

    def reset(self) -> None:
        """Reset temporal filters and state machine buffers."""
        self._pos_filter.reset()
        self._filtered_quat = None
        self._last_valid_pose = None
        self._last_valid_timestamp = 0.0
        self._hold_start_timestamp = None

    def close(self) -> None:
        """Release MediaPipe resources."""
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None

    def __enter__(self) -> "WristTracker":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def process_frame(
        self,
        color_bgr: np.ndarray,
        depth_m: Optional[np.ndarray],
        intrinsics: CameraIntrinsics,
        table_calib: Optional[ArUcoCalibrationData] = None,
        timestamp: Optional[float] = None,
    ) -> WristPose:
        """Process aligned color and depth frame to produce filtered 3D wrist pose."""
        now = time.monotonic() if timestamp is None else float(timestamp)
        h, w = color_bgr.shape[:2]

        # 1. Run Hand Detection
        detected_hand = self._detect_hand(color_bgr, now)

        # If no hand detected at all by visual detector
        if detected_hand is None:
            return self._handle_detection_lost(now, intrinsics, table_calib)

        # 2. Extract 2D and 3D landmarks
        landmarks_norm = detected_hand["landmarks"]  # (21, 3) normalized x, y, z
        handedness = detected_hand.get("handedness", self.default_handedness)

        landmarks_2d = np.zeros((21, 2), dtype=np.float32)
        landmarks_2d[:, 0] = np.clip(landmarks_norm[:, 0] * w, 0, w - 1)
        landmarks_2d[:, 1] = np.clip(landmarks_norm[:, 1] * h, 0, h - 1)

        wrist_uv = (float(landmarks_2d[LM_WRIST, 0]), float(landmarks_2d[LM_WRIST, 1]))

        # 3. Depth Sampling & Anti-Depth-Hole Logic
        wrist_depth = float("nan")
        if depth_m is not None:
            wrist_depth = median_depth_at(
                depth_m,
                wrist_uv[0],
                wrist_uv[1],
                patch_radius=self.depth_patch_radius,
                min_depth_m=self.min_depth_m,
                max_depth_m=self.max_depth_m,
            )

            # If wrist is directly in a hole, check adjacent palm MCP landmarks
            if not math.isfinite(wrist_depth):
                palm_mcp_indices = [LM_INDEX_MCP, LM_MIDDLE_MCP, LM_PINKY_MCP]
                mcp_depths = []
                for idx in palm_mcp_indices:
                    d = median_depth_at(
                        depth_m,
                        landmarks_2d[idx, 0],
                        landmarks_2d[idx, 1],
                        patch_radius=self.depth_patch_radius,
                        min_depth_m=self.min_depth_m,
                        max_depth_m=self.max_depth_m,
                    )
                    if math.isfinite(d):
                        mcp_depths.append(d)
                if mcp_depths:
                    wrist_depth = float(np.median(mcp_depths))

        # Check if depth is valid
        is_depth_valid = math.isfinite(wrist_depth) and (wrist_depth > 0.0)

        # 4. Handle Depth Hole / Temporal Hold State Machine
        if not is_depth_valid:
            if self._last_valid_pose is not None:
                hold_duration = max(0.0, now - self._last_valid_timestamp)
                if hold_duration <= self.max_hold_time_s:
                    # Valid temporal hold: reuse last known depth & position
                    held_depth = self._last_valid_pose.depth_m
                    held_pos_cam = intrinsics.deproject_pixel_to_point(wrist_uv[0], wrist_uv[1], held_depth)

                    # Update 3D points in camera frame for orientation calculation
                    landmarks_3d_cam = self._compute_3d_landmarks(landmarks_2d, held_depth, intrinsics, landmarks_norm)
                    _, _, _, R_cam = compute_palm_basis(landmarks_3d_cam, handedness)

                    # Transform to table frame
                    held_pos_table_raw = (
                        table_calib.transform_point_to_table(held_pos_cam)
                        if table_calib is not None
                        else held_pos_cam
                    )
                    pos_table_filtered = self._pos_filter.filter(held_pos_table_raw, timestamp=now)

                    # Orientation in table frame
                    R_table = (
                        table_calib.t_table_to_cam[:3, :3] @ R_cam
                        if table_calib is not None
                        else R_cam
                    )
                    q_table = rotation_matrix_to_quaternion(R_table)
                    if self._filtered_quat is not None:
                        self._filtered_quat = slerp(self._filtered_quat, q_table, t=self.quat_slerp_alpha)
                    else:
                        self._filtered_quat = q_table

                    euler_deg = rotation_matrix_to_euler_deg(R_table)

                    return WristPose(
                        timestamp=now,
                        is_valid=True,
                        tracking_state=TrackingState.TEMPORAL_HOLD,
                        position_cam=held_pos_cam,
                        position_table=pos_table_filtered,
                        position_table_raw=held_pos_table_raw,
                        orientation_table_quat=self._filtered_quat.copy(),
                        orientation_table_matrix=R_table,
                        euler_deg=euler_deg,
                        wrist_pixel_uv=wrist_uv,
                        landmarks_2d=landmarks_2d,
                        landmarks_3d_cam=landmarks_3d_cam,
                        depth_m=held_depth,
                        hold_duration=hold_duration,
                        handedness=handedness,
                        landmarks_world_3d=self._last_valid_pose.landmarks_world_3d if self._last_valid_pose else None,
                    )
                else:
                    # Hold timed out -> LOST
                    self._pos_filter.reset()
                    self._filtered_quat = None
                    return WristPose(
                        timestamp=now,
                        is_valid=False,
                        tracking_state=TrackingState.LOST,
                        position_cam=np.zeros(3, dtype=np.float64),
                        position_table=np.zeros(3, dtype=np.float64),
                        position_table_raw=np.zeros(3, dtype=np.float64),
                        orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                        orientation_table_matrix=np.eye(3, dtype=np.float64),
                        euler_deg=(0.0, 0.0, 0.0),
                        wrist_pixel_uv=wrist_uv,
                        landmarks_2d=landmarks_2d,
                        landmarks_3d_cam=np.zeros((21, 3), dtype=np.float64),
                        depth_m=0.0,
                        hold_duration=hold_duration,
                        handedness=handedness,
                    )
            else:
                # No prior valid depth ever recorded -> LOST
                return WristPose(
                    timestamp=now,
                    is_valid=False,
                    tracking_state=TrackingState.LOST,
                    position_cam=np.zeros(3, dtype=np.float64),
                    position_table=np.zeros(3, dtype=np.float64),
                    position_table_raw=np.zeros(3, dtype=np.float64),
                    orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                    orientation_table_matrix=np.eye(3, dtype=np.float64),
                    euler_deg=(0.0, 0.0, 0.0),
                    wrist_pixel_uv=wrist_uv,
                    landmarks_2d=landmarks_2d,
                    landmarks_3d_cam=np.zeros((21, 3), dtype=np.float64),
                    depth_m=0.0,
                    hold_duration=0.0,
                    handedness=handedness,
                )

        # 5. Fresh Valid Tracking State
        self._hold_start_timestamp = None  # Clear hold

        # 3D Back-projection
        pos_cam = intrinsics.deproject_pixel_to_point(wrist_uv[0], wrist_uv[1], wrist_depth)
        landmarks_3d_cam = self._compute_3d_landmarks(landmarks_2d, wrist_depth, intrinsics, landmarks_norm)

        # Palm basis & orientation in camera frame
        _, _, _, R_cam = compute_palm_basis(landmarks_3d_cam, handedness)

        # Transform to Table Frame
        if table_calib is not None and table_calib.is_valid:
            pos_table_raw = table_calib.transform_point_to_table(pos_cam)
            R_table = table_calib.t_table_to_cam[:3, :3] @ R_cam
        else:
            pos_table_raw = pos_cam.copy()
            R_table = R_cam.copy()

        # Filtering
        pos_table_filtered = self._pos_filter.filter(pos_table_raw, timestamp=now)
        q_table = rotation_matrix_to_quaternion(R_table)
        if self._filtered_quat is None:
            self._filtered_quat = q_table.copy()
        else:
            self._filtered_quat = slerp(self._filtered_quat, q_table, t=self.quat_slerp_alpha)

        euler_deg = rotation_matrix_to_euler_deg(R_table)

        pose = WristPose(
            timestamp=now,
            is_valid=True,
            tracking_state=TrackingState.TRACKING,
            position_cam=pos_cam,
            position_table=pos_table_filtered,
            position_table_raw=pos_table_raw,
            orientation_table_quat=self._filtered_quat.copy(),
            orientation_table_matrix=R_table,
            euler_deg=euler_deg,
            wrist_pixel_uv=wrist_uv,
            landmarks_2d=landmarks_2d,
            landmarks_3d_cam=landmarks_3d_cam,
            depth_m=wrist_depth,
            hold_duration=0.0,
            handedness=handedness,
            landmarks_world_3d=detected_hand.get("world_landmarks", None),
        )

        self._last_valid_pose = pose
        self._last_valid_timestamp = now
        return pose

    def _compute_3d_landmarks(
        self,
        landmarks_2d: np.ndarray,
        base_depth: float,
        intrinsics: CameraIntrinsics,
        landmarks_norm: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Back-project all 21 2D landmarks to 3D camera metric coordinates.
        
        When landmarks_norm (including relative z) is available, incorporates
        per-landmark depth offsets rather than assuming a completely flat hand.
        """
        pts_3d = np.zeros((21, 3), dtype=np.float64)
        z_offsets = np.zeros(21, dtype=np.float64)

        if landmarks_norm is not None and landmarks_norm.shape[1] >= 3:
            # Estimate metric hand scale from palm MCP width
            # Index MCP (5) to Pinky MCP (17) in normalized image coords
            p_idx_norm = landmarks_norm[LM_INDEX_MCP, :2]
            p_pky_norm = landmarks_norm[LM_PINKY_MCP, :2]
            norm_width = float(np.linalg.norm(p_idx_norm - p_pky_norm))
            # Average human palm breadth across knuckles is ~75mm (0.075m)
            # MediaPipe's z is roughly on the same scale as normalized image dimensions
            scale_m = 0.075 / max(0.04, min(0.35, norm_width))
            z_offsets = (landmarks_norm[:, 2] - landmarks_norm[LM_WRIST, 2]) * scale_m
            # Clamp individual landmark depth deviations to realistic hand depth (+/- 12cm)
            z_offsets = np.clip(z_offsets, -0.12, 0.12)

        for i in range(21):
            u, v = landmarks_2d[i]
            z_i = max(0.10, base_depth + z_offsets[i])
            pts_3d[i] = intrinsics.deproject_pixel_to_point(u, v, z_i)
        return pts_3d

    def _detect_hand(self, color_bgr: np.ndarray, now: float) -> Optional[dict]:
        """Run either custom mock detector or real MediaPipe HandLandmarker."""
        if self._custom_detector is not None:
            return self._custom_detector(color_bgr)

        if self._landmarker is None:
            return None

        # Convert to MediaPipe Image
        rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        mp_image = MpImage(image_format=ImageFormat.SRGB, data=rgb)

        ts_ms = int(now * 1000)
        if ts_ms <= self._last_video_ts_ms:
            ts_ms = self._last_video_ts_ms + 1
        self._last_video_ts_ms = ts_ms

        result = self._landmarker.detect_for_video(mp_image, ts_ms)
        if not result.hand_landmarks:
            return None

        landmarks_obj = result.hand_landmarks[0]
        pts_norm = np.zeros((21, 3), dtype=np.float32)
        for i, lm in enumerate(landmarks_obj):
            pts_norm[i] = [lm.x, lm.y, lm.z]

        pts_world = None
        if result.hand_world_landmarks and len(result.hand_world_landmarks) > 0:
            world_obj = result.hand_world_landmarks[0]
            pts_world = np.zeros((21, 3), dtype=np.float64)
            for i, lm in enumerate(world_obj):
                pts_world[i] = [lm.x, lm.y, lm.z]

        handedness = "Right"
        if result.handedness and len(result.handedness) > 0 and len(result.handedness[0]) > 0:
            handedness = result.handedness[0][0].category_name

        return {
            "landmarks": pts_norm,
            "world_landmarks": pts_world,
            "handedness": handedness,
        }

    def _handle_detection_lost(
        self,
        now: float,
        intrinsics: CameraIntrinsics,
        table_calib: Optional[ArUcoCalibrationData],
    ) -> WristPose:
        """Handle case where no hand is visually detected."""
        hold_duration = max(0.0, now - self._last_valid_timestamp) if self._last_valid_pose is not None else 0.0

        if self._last_valid_pose is not None and hold_duration <= self.max_hold_time_s:
            # Short drop temporal hold (< 300ms)
            return WristPose(
                timestamp=now,
                is_valid=True,
                tracking_state=TrackingState.TEMPORAL_HOLD,
                position_cam=self._last_valid_pose.position_cam.copy(),
                position_table=self._last_valid_pose.position_table.copy(),
                position_table_raw=self._last_valid_pose.position_table_raw.copy(),
                orientation_table_quat=self._last_valid_pose.orientation_table_quat.copy(),
                orientation_table_matrix=self._last_valid_pose.orientation_table_matrix.copy(),
                euler_deg=self._last_valid_pose.euler_deg,
                wrist_pixel_uv=self._last_valid_pose.wrist_pixel_uv,
                landmarks_2d=self._last_valid_pose.landmarks_2d.copy(),
                landmarks_3d_cam=self._last_valid_pose.landmarks_3d_cam.copy(),
                depth_m=self._last_valid_pose.depth_m,
                hold_duration=hold_duration,
                handedness=self._last_valid_pose.handedness,
            )

        # Fully lost
        self._pos_filter.reset()
        self._filtered_quat = None
        return WristPose(
            timestamp=now,
            is_valid=False,
            tracking_state=TrackingState.NO_HAND if hold_duration == 0.0 else TrackingState.LOST,
            position_cam=np.zeros(3, dtype=np.float64),
            position_table=np.zeros(3, dtype=np.float64),
            position_table_raw=np.zeros(3, dtype=np.float64),
            orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            orientation_table_matrix=np.eye(3, dtype=np.float64),
            euler_deg=(0.0, 0.0, 0.0),
            wrist_pixel_uv=(0.0, 0.0),
            landmarks_2d=np.zeros((21, 2), dtype=np.float32),
            landmarks_3d_cam=np.zeros((21, 3), dtype=np.float64),
            depth_m=0.0,
            hold_duration=hold_duration,
            handedness=self.default_handedness,
        )


def draw_wrist_hud(
    image: np.ndarray,
    wrist_pose: WristPose,
    intrinsics: Optional[CameraIntrinsics] = None,
    draw_skeleton: bool = True,
    patch_radius: int = 3,
) -> np.ndarray:
    """Render real-time HUD annotations, hand skeleton, and depth diagnostic overlay."""
    annotated = image.copy()
    h, w = annotated.shape[:2]

    # 1. Hand Skeleton overlay
    if draw_skeleton and wrist_pose.tracking_state in (TrackingState.TRACKING, TrackingState.TEMPORAL_HOLD):
        pts = wrist_pose.landmarks_2d.astype(int)

        # Draw connections
        for start_idx, end_idx in HAND_CONNECTIONS:
            p1 = (pts[start_idx, 0], pts[start_idx, 1])
            p2 = (pts[end_idx, 0], pts[end_idx, 1])
            cv2.line(annotated, p1, p2, (230, 230, 230), 2, cv2.LINE_AA)

        # Draw joints
        for i in range(21):
            p = (pts[i, 0], pts[i, 1])
            color = (0, 255, 0) if i == LM_WRIST else (255, 180, 0)
            radius = 5 if i in (LM_WRIST, LM_MIDDLE_MCP) else 3
            cv2.circle(annotated, p, radius, color, -1, cv2.LINE_AA)

        # Draw depth sampling patch square at wrist
        uw, vw = int(wrist_pose.wrist_pixel_uv[0]), int(wrist_pose.wrist_pixel_uv[1])
        r = patch_radius
        cv2.rectangle(annotated, (uw - r, vw - r), (uw + r, vw + r), (0, 255, 255), 1)

    # 2. 3D Coordinate Axis Triad at Wrist
    if (
        intrinsics is not None
        and wrist_pose.tracking_state in (TrackingState.TRACKING, TrackingState.TEMPORAL_HOLD)
        and wrist_pose.is_valid
    ):
        origin_3d = wrist_pose.position_cam
        axis_len = 0.05  # 50 mm
        # Palm coordinate vectors in camera space: columns of R_cam
        # We can extract R_cam from landmarks_3d_cam
        _, _, _, R_cam = compute_palm_basis(wrist_pose.landmarks_3d_cam, wrist_pose.handedness)
        axes_3d = np.array([
            origin_3d,
            origin_3d + R_cam[:, 0] * axis_len,  # X: Red
            origin_3d + R_cam[:, 1] * axis_len,  # Y: Green
            origin_3d + R_cam[:, 2] * axis_len,  # Z: Blue
        ], dtype=np.float32)

        proj_2d, _ = cv2.projectPoints(
            axes_3d,
            np.zeros(3, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
            intrinsics.matrix,
            np.zeros(5, dtype=np.float32),
        )
        pts_proj = proj_2d.reshape(-1, 2).astype(int)
        p_origin = tuple(pts_proj[0])
        cv2.arrowedLine(annotated, p_origin, tuple(pts_proj[1]), (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.2)
        cv2.arrowedLine(annotated, p_origin, tuple(pts_proj[2]), (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.2)
        cv2.arrowedLine(annotated, p_origin, tuple(pts_proj[3]), (255, 0, 0), 2, cv2.LINE_AA, tipLength=0.2)

    # 3. HUD Status Box (Top-Left)
    hud_x, hud_y = 12, 12
    box_w, box_h = 320, 125
    overlay = annotated.copy()
    cv2.rectangle(overlay, (hud_x, hud_y), (hud_x + box_w, hud_y + box_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.70, annotated, 0.30, 0, annotated)
    cv2.rectangle(annotated, (hud_x, hud_y), (hud_x + box_w, hud_y + box_h), (80, 80, 80), 1)

    # Status Badge
    state = wrist_pose.tracking_state
    if state == TrackingState.TRACKING:
        badge_text = "STATUS: TRACKING"
        badge_color = (0, 255, 0)
    elif state == TrackingState.TEMPORAL_HOLD:
        badge_text = f"STATUS: HOLD ({wrist_pose.hold_duration:.2f}s)"
        badge_color = (0, 165, 255)  # Orange
    elif state == TrackingState.NO_HAND:
        badge_text = "STATUS: NO HAND"
        badge_color = (0, 0, 255)
    else:
        badge_text = "STATUS: LOST"
        badge_color = (0, 0, 255)

    cv2.putText(annotated, badge_text, (hud_x + 10, hud_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, badge_color, 2)

    # Coordinates in Table Frame
    pos = wrist_pose.position_table
    coord_str = f"Table XYZ: [{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}] m"
    cv2.putText(annotated, coord_str, (hud_x + 10, hud_y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

    # Euler angles
    r, p, y = wrist_pose.euler_deg
    euler_str = f"RPY (deg): R:{r:+.1f} P:{p:+.1f} Y:{y:+.1f}"
    cv2.putText(annotated, euler_str, (hud_x + 10, hud_y + 70), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

    # Depth & Handedness
    depth_str = f"Depth: {wrist_pose.depth_m:.3f} m  Hand: {wrist_pose.handedness}"
    cv2.putText(annotated, depth_str, (hud_x + 10, hud_y + 92), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

    # Filter info
    filter_str = f"1e-Filter: active (cutoff=1.0Hz)"
    cv2.putText(annotated, filter_str, (hud_x + 10, hud_y + 114), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160, 160, 160), 1)

    return annotated
