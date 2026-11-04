"""hand_retargeter.py — Dexterous Hand 16-DOF Kinematic Retargeting.

Fuses:
1. 3D Hand Landmarks from RealSense D455 + MediaPipe (Scheme C - wrist_tracker.py).
2. Biomechanical relative bone-flexion kinematic angles with Ulnar Synergy Fusion (5-to-4 fingers).
3. Optional Pinocchio-based kinematic optimization from dex-retargeting (DexPilot + Vector).
4. 1€ adaptive filtering and canonical 16-DOF joint mapping for LEAP Hand.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Tuple, Union

import numpy as np

from ..core.filters import LandmarkOneEuroFilter, OneEuroFilter
from ..core.names import CANONICAL_HAND_JOINT_NAMES, HAND_NUM_DOFS
from ..core.types import TeleopHandCommand
from ..perception.wrist_tracker import (
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
    compute_palm_basis,
)

# Optional dynamic loader for dex_retargeting
_HAS_DEX_RETARGETING = False

# Defense-in-depth: Remove incompatible Python version packages (e.g., ROS Lyrical Python 3.14 on Ubuntu 24.04)
if sys.version_info[:2] == (3, 10):
    sys.path = [p for p in sys.path if not ("python3.1" in p and "python3.10" not in p)]

# Dynamically locate cmeel Pinocchio dynamic library path in active virtualenv/conda
_py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
_site_pkgs = os.path.join(sys.prefix, "lib", _py_ver, "site-packages")
_cmeel_lib = os.path.join(_site_pkgs, "cmeel.prefix", "lib")
_cmeel_pkg = os.path.join(_cmeel_lib, _py_ver, "site-packages")

if not os.path.isdir(_cmeel_lib):
    _fallback_prefix = os.path.expanduser("~/win_office/conda/envs/tuinadex-control")
    _site_pkgs = os.path.join(_fallback_prefix, "lib", _py_ver, "site-packages")
    _cmeel_lib = os.path.join(_site_pkgs, "cmeel.prefix", "lib")
    _cmeel_pkg = os.path.join(_cmeel_lib, _py_ver, "site-packages")

if os.path.isdir(_cmeel_lib) and _cmeel_lib not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = f"{_cmeel_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"
if os.path.isdir(_cmeel_pkg) and _cmeel_pkg not in sys.path:
    sys.path.insert(0, _cmeel_pkg)
if os.path.isdir(_site_pkgs) and _site_pkgs not in sys.path:
    sys.path.insert(0, _site_pkgs)

try:
    import dex_retargeting
    from dex_retargeting.retargeting_config import RetargetingConfig
    from dex_retargeting.optimizer import DexPilotOptimizer, VectorOptimizer
    _HAS_DEX_RETARGETING = True
except Exception as _err:
    _HAS_DEX_RETARGETING = False


# Transformation from human hand local basis (compute_palm_basis) to LEAP Hand root frame (palm_lower)
# Human local basis:
#   +X: Lateral axis across palm (thumb -> pinky)
#   +Y: Longitudinal axis along palm (wrist -> middle MCP)
#   +Z: Dorsal normal vector pointing outward from back of hand
# LEAP Hand palm_lower frame:
#   +X: Longitudinal axis along fingers (forward into scene)
#   -Y: Lateral axis across knuckles (index +Y, ring -Y)
#   +Z: Dorsal/palmar normal vector
R_ALIGN_HUMAN_TO_ROBOT = np.array([
    [ 0.0,  1.0,  0.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0],
], dtype=np.float64)


@dataclass
class HandRetargetingMetrics:
    """Diagnostic telemetry from the hand retargeter."""
    computation_time_ms: float = 0.0
    thumb_index_pinch_dist_m: float = 0.0
    thumb_middle_pinch_dist_m: float = 0.0
    is_pinching: bool = False
    average_flexion_rad: float = 0.0
    solver_mode: str = "hybrid"


def angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
    """Calculate unsigned angle (radians) between two 3D vectors."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_a = np.dot(v1, v2) / (n1 * n2)
    cos_a = np.clip(cos_a, -1.0, 1.0)
    return float(math.acos(cos_a))


class BiomechanicalKinematicRetargeter:
    """High-precision analytical bone-segment retargeter with Ulnar Synergy Fusion.
    
    Features:
    1. Direct bone-flexion angles for MCP, PIP, and DIP (0 ~ 90+ deg range).
    2. Ulnar Synergy Fusion: Combines human Ring (13-16) and Pinky (17-20) into LEAP Hand's 3rd finger.
    3. Opposition kinematics for thumb CMC, MCP, and IP.
    4. Deterministic, zero-lag (< 0.05ms computation).
    """

    def __init__(self, joint_limits: Optional[np.ndarray] = None):
        if joint_limits is not None:
            self.joint_limits = joint_limits
        else:
            self.joint_limits = np.array([
                [-1.047,  1.047],  # index_yaw
                [-0.314,  2.230],  # index_mcp
                [-0.506,  1.885],  # index_pip
                [-0.366,  2.042],  # index_dip
                [-1.047,  1.047],  # middle_yaw
                [-0.314,  2.230],  # middle_mcp
                [-0.506,  1.885],  # middle_pip
                [-0.366,  2.042],  # middle_dip
                [-1.047,  1.047],  # ring_yaw
                [-0.314,  2.230],  # ring_mcp
                [-0.506,  1.885],  # ring_pip
                [-0.366,  2.042],  # ring_dip
                [-1.047,  1.047],  # thumb_yaw
                [-0.314,  2.000],  # thumb_cmc
                [-0.506,  1.885],  # thumb_mcp
                [-0.366,  2.042],  # thumb_ip
            ], dtype=np.float64)

    def retarget(self, pts_local: np.ndarray) -> np.ndarray:
        """Compute 16-DOF joint angles from local hand keypoints."""
        q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)

        # -------------------------------------------------------------
        # 1. Index Finger (joints 0..3)
        # -------------------------------------------------------------
        v_prox_idx = pts_local[LM_INDEX_PIP] - pts_local[LM_INDEX_MCP]
        v_inter_idx = pts_local[LM_INDEX_DIP] - pts_local[LM_INDEX_PIP]
        v_dist_idx = pts_local[LM_INDEX_TIP] - pts_local[LM_INDEX_DIP]

        # Lateral spread (yaw): project against sagittal length hypot(v_y, v_z) to avoid division by zero during flexion
        r_sagittal_idx = math.hypot(v_prox_idx[1], v_prox_idx[2])
        q[0] = math.atan2(v_prox_idx[0], max(1e-4, r_sagittal_idx)) * 0.70
        # MCP pitch flexion: continuous atan2(-v_z, v_y) preventing reversal past 90 deg into fist
        ang_mcp_idx = math.atan2(-v_prox_idx[2], v_prox_idx[1])
        ang_pip_idx = angle_between_vectors(v_prox_idx, v_inter_idx)
        ang_dip_idx = angle_between_vectors(v_inter_idx, v_dist_idx)

        q[1] = ang_mcp_idx * 1.75  # index_mcp
        q[2] = ang_pip_idx * 1.60  # index_pip
        # FDP flexor tendon synergy: distal DIP curls with PIP flexion for natural tight fist closure
        q[3] = ang_dip_idx * 1.15 + max(0.0, ang_pip_idx - 0.20) * 0.45  # index_dip

        # -------------------------------------------------------------
        # 2. Middle Finger (joints 4..7)
        # -------------------------------------------------------------
        v_prox_mid = pts_local[LM_MIDDLE_PIP] - pts_local[LM_MIDDLE_MCP]
        v_inter_mid = pts_local[LM_MIDDLE_DIP] - pts_local[LM_MIDDLE_PIP]
        v_dist_mid = pts_local[LM_MIDDLE_TIP] - pts_local[LM_MIDDLE_DIP]

        r_sagittal_mid = math.hypot(v_prox_mid[1], v_prox_mid[2])
        q[4] = math.atan2(v_prox_mid[0], max(1e-4, r_sagittal_mid)) * 0.70
        ang_mcp_mid = math.atan2(-v_prox_mid[2], v_prox_mid[1])
        ang_pip_mid = angle_between_vectors(v_prox_mid, v_inter_mid)
        ang_dip_mid = angle_between_vectors(v_inter_mid, v_dist_mid)

        q[5] = ang_mcp_mid * 1.75  # middle_mcp
        q[6] = ang_pip_mid * 1.60  # middle_pip
        q[7] = ang_dip_mid * 1.15 + max(0.0, ang_pip_mid - 0.20) * 0.45  # middle_dip

        # -------------------------------------------------------------
        # 3. Ring Finger (joints 8..11) with Ulnar Synergy Fusion
        # Fuses Human Ring (13..16) and Human Pinky (17..20)
        # -------------------------------------------------------------
        v_prox_ring = pts_local[LM_RING_PIP] - pts_local[LM_RING_MCP]
        v_inter_ring = pts_local[LM_RING_DIP] - pts_local[LM_RING_PIP]
        v_dist_ring = pts_local[LM_RING_TIP] - pts_local[LM_RING_DIP]

        v_prox_pky = pts_local[LM_PINKY_PIP] - pts_local[LM_PINKY_MCP]
        v_inter_pky = pts_local[LM_PINKY_DIP] - pts_local[LM_PINKY_PIP]
        v_dist_pky = pts_local[LM_PINKY_TIP] - pts_local[LM_PINKY_DIP]

        # Angles for ring finger
        r_sagittal_ring = math.hypot(v_prox_ring[1], v_prox_ring[2])
        ring_yaw = math.atan2(v_prox_ring[0], max(1e-4, r_sagittal_ring)) * 0.70
        ang_mcp_ring = math.atan2(-v_prox_ring[2], v_prox_ring[1])
        ang_pip_ring = angle_between_vectors(v_prox_ring, v_inter_ring)
        ang_dip_ring = angle_between_vectors(v_inter_ring, v_dist_ring)
        ring_mcp = ang_mcp_ring * 1.75
        ring_pip = ang_pip_ring * 1.60
        ring_dip = ang_dip_ring * 1.15 + max(0.0, ang_pip_ring - 0.20) * 0.45

        # Angles for pinky finger
        r_sagittal_pky = math.hypot(v_prox_pky[1], v_prox_pky[2])
        pky_yaw = math.atan2(v_prox_pky[0], max(1e-4, r_sagittal_pky)) * 0.70
        ang_mcp_pky = math.atan2(-v_prox_pky[2], v_prox_pky[1])
        ang_pip_pky = angle_between_vectors(v_prox_pky, v_inter_pky)
        ang_dip_pky = angle_between_vectors(v_inter_pky, v_dist_pky)
        pky_mcp = ang_mcp_pky * 1.75
        pky_pip = ang_pip_pky * 1.60
        pky_dip = ang_dip_pky * 1.15 + max(0.0, ang_pip_pky - 0.20) * 0.45

        # Ulnar synergy fusion: Active when either ring or pinky is curled
        q[8] = 0.70 * ring_yaw + 0.30 * pky_yaw
        q[9] = max(ring_mcp, pky_mcp)
        q[10] = max(ring_pip, pky_pip)
        q[11] = max(ring_dip, pky_dip)

        # -------------------------------------------------------------
        # 4. Thumb (joints 12..15)
        # -------------------------------------------------------------
        v_cmc = pts_local[LM_THUMB_CMC] - pts_local[LM_WRIST]
        v_mcp = pts_local[LM_THUMB_MCP] - pts_local[LM_THUMB_CMC]
        v_ip = pts_local[LM_THUMB_IP] - pts_local[LM_THUMB_MCP]
        v_tip = pts_local[LM_THUMB_TIP] - pts_local[LM_THUMB_IP]

        # Opposition & fist curl synergy:
        # Distance from thumb tip to index/middle base indicates opposition and fist closure.
        d_oppose = float(np.linalg.norm(pts_local[LM_THUMB_TIP] - pts_local[LM_INDEX_MCP]))
        raw_oppose = float(np.clip((0.080 - d_oppose) / 0.045, 0.0, 1.0))
        # Smoothstep S(x) = 3x^2 - 2x^3 to eliminate abrupt step twitches
        oppose_boost = raw_oppose * raw_oppose * (3.0 - 2.0 * raw_oppose)
        ang_mcp = angle_between_vectors(v_mcp, v_ip)
        ang_ip = angle_between_vectors(v_ip, v_tip)

        r_sagittal_thumb = math.hypot(v_mcp[1], v_mcp[2])
        q[12] = (math.atan2(v_mcp[0], max(1e-4, r_sagittal_thumb)) * 0.85) + 0.45 * oppose_boost  # thumb_yaw
        q[13] = angle_between_vectors(v_cmc, v_mcp) * 1.20 + 0.35 * oppose_boost          # thumb_cmc
        q[14] = ang_mcp * 1.45 + 0.35 * oppose_boost                                     # thumb_mcp
        # Distal IP joint (id 15): human thumb has 2 flexion joints (MCP/IP) vs LEAP Hand's 3 joints (CMC/MCP/IP).
        # When curling into a fist or grasping, thumb_ip must curl deeply. Couple direct IP angle with proximal
        # MCP flexion (FPL muscle synergy) plus oppose_boost.
        q[15] = ang_ip * 1.65 + max(0.0, ang_mcp - 0.05) * 0.90 + 0.55 * oppose_boost   # thumb_ip

        # Clamp strictly inside URDF physical joint limits
        q = np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])
        return q


# Keep backward-compatible alias
FallbackKinematicRetargeter = BiomechanicalKinematicRetargeter

# Natural relaxed four-finger open zero offset (corresponds to post-[Z] calibration posture)
DEFAULT_OPEN_ZERO_OFFSET = np.array([
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    -0.2828, 0.4142, 0.4132, 0.5698,
], dtype=np.float64)


class HandRetargeter:
    """High-precision LEAP Hand retargeter fusing 3D Keypoints and Kinematic Optimization."""

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        config_path: Optional[str] = None,
        solver_mode: str = "hybrid",
        scaling_factor: float = 1.60,
        filter_freq_hz: float = 60.0,
        filter_min_cutoff: float = 0.8,
        filter_beta: float = 0.06,
        pinch_threshold_m: float = 0.035,
        max_joint_velocity_rad_s: float = 8.0,
    ):
        self.solver_mode = solver_mode.lower()
        self.scaling_factor = scaling_factor
        self.filter_freq_hz = filter_freq_hz
        self.pinch_threshold_m = pinch_threshold_m
        self.max_joint_velocity_rad_s = max_joint_velocity_rad_s
        self.q_zero_offset: Optional[np.ndarray] = None
        self._last_raw_q: Optional[np.ndarray] = None
        self.palm_reference_length_m: float = 0.085

        # Resolve paths
        project_root = Path(__file__).resolve().parents[2]
        resolved_urdf = (
            Path(urdf_path).resolve()
            if urdf_path
            else (project_root / "configs" / "urdf" / "leap_hand_right.urdf").resolve()
        )
        if not resolved_urdf.exists():
            candidate = Path(
                "/home/bright/win_office/ubantu_files/project/TuinaDex_v2.0/src/tuinadex_sim/urdf/leap_hand_right.urdf"
            ).resolve()
            if candidate.exists():
                resolved_urdf = candidate

        self.urdf_path = str(resolved_urdf)

        # Joint limits cache
        self.joint_limits = np.array([
            [-1.047,  1.047],  # index_yaw
            [-0.314,  2.230],  # index_mcp
            [-0.506,  1.885],  # index_pip
            [-0.366,  2.042],  # index_dip
            [-1.047,  1.047],  # middle_yaw
            [-0.314,  2.230],  # middle_mcp
            [-0.506,  1.885],  # middle_pip
            [-0.366,  2.042],  # middle_dip
            [-1.047,  1.047],  # ring_yaw
            [-0.314,  2.230],  # ring_mcp
            [-0.506,  1.885],  # ring_pip
            [-0.366,  2.042],  # ring_dip
            [-1.047,  1.047],  # thumb_yaw
            [-0.314,  2.000],  # thumb_cmc
            [-0.506,  1.885],  # thumb_mcp
            [-0.366,  2.042],  # thumb_ip
        ], dtype=np.float64)

        # 1€ Filter for 16 joint angles (adapted for 60Hz loop rate)
        self._filter = OneEuroFilter(
            n_dim=HAND_NUM_DOFS,
            freq_hz=filter_freq_hz,
            min_cutoff=filter_min_cutoff,
            beta=filter_beta,
            d_cutoff=1.2,
        )

        # Biomechanical retargeter with ulnar synergy
        self._bio_retargeter = BiomechanicalKinematicRetargeter(self.joint_limits)
        self._fallback_retargeter = self._bio_retargeter

        # Initialize DexPilot optimizer if available
        self._dexpilot_retargeter = None
        self._vector_retargeter = None
        self._has_dex_retargeting = False

        if _HAS_DEX_RETARGETING and Path(self.urdf_path).exists():
            self._init_dex_retargeting()

        # If user explicitly requested dex_retargeting but it failed to init, fall back to hybrid
        if self.solver_mode == "dex_retargeting" and not self._has_dex_retargeting:
            print("[WARN] DexPilot unavailable. Falling back to hybrid solver mode.")
            self.solver_mode = "hybrid"

        self._last_metrics = HandRetargetingMetrics(solver_mode=self.solver_mode)
        self._last_q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._last_valid_time: Optional[float] = None

    def _init_dex_retargeting(self) -> None:
        """Initialize Pinocchio-based DexPilot and Vector retargeters."""
        try:
            dexpilot_cfg = {
                "type": "DexPilot",
                "urdf_path": self.urdf_path,
                "wrist_link_name": "palm_lower",
                "finger_tip_link_names": ["thumb_fingertip", "fingertip", "fingertip_2", "fingertip_3"],
                "target_joint_names": CANONICAL_HAND_JOINT_NAMES,
                "scaling_factor": self.scaling_factor,
                "low_pass_alpha": 0.25,
            }
            cfg1 = RetargetingConfig.from_dict(dexpilot_cfg)
            self._dexpilot_retargeter = cfg1.build()

            self._dexpilot_origin_idx, self._dexpilot_task_idx = DexPilotOptimizer.generate_link_indices(4)
            # Map human joints for 4 fingers (wrist, thumb, index, middle, ring/pinky virtual)
            self._human_joint_map = np.array([LM_WRIST, LM_THUMB_TIP, LM_INDEX_TIP, LM_MIDDLE_TIP, LM_RING_TIP])

            self._has_dex_retargeting = True
        except Exception as e:
            print(f"[WARN] Failed to initialize Pinocchio dex-retargeting ({e}). Using robust biomechanical solver.")
            self._has_dex_retargeting = False

    def reset(self) -> None:
        """Reset temporal filter buffers."""
        self._filter.reset()
        self._last_q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._last_valid_time = None

    def reset_to_zero_pose(self) -> np.ndarray:
        """Immediately reset internal filters and return the canonical open zero posture."""
        self.reset()
        return np.zeros(HAND_NUM_DOFS, dtype=np.float64)

    def compute_raw_joint_angles(self, wrist_pose: WristPose) -> np.ndarray:
        """Compute uncalibrated raw joint angles from wrist_pose before zero-offset calibration."""
        if wrist_pose.landmarks_world_3d is not None and np.any(wrist_pose.landmarks_world_3d != 0):
            pts_source = wrist_pose.landmarks_world_3d.copy()
            p_wrist = pts_source[LM_WRIST]
            _, _, _, R_basis = compute_palm_basis(pts_source, wrist_pose.handedness)
            pts_local = (R_basis.T @ (pts_source - p_wrist).T).T
        else:
            pts_3d_cam = wrist_pose.landmarks_3d_cam
            p_wrist = pts_3d_cam[LM_WRIST]
            _, _, _, R_cam = compute_palm_basis(pts_3d_cam, wrist_pose.handedness)
            pts_local = (R_cam.T @ (pts_3d_cam - p_wrist).T).T

        palm_len = float(np.linalg.norm(pts_local[LM_MIDDLE_MCP] - pts_local[LM_WRIST]))
        if palm_len > 0.03:
            scale_factor = self.palm_reference_length_m / palm_len
            pts_local = pts_local * scale_factor

        return self._bio_retargeter.retarget(pts_local)

    def zero(self, wrist_pose: Optional[WristPose] = None) -> np.ndarray:
        """Latch current naturally open hand posture as zero reference, and reset hand joints to open zero pose.

        Args:
            wrist_pose: Optional current WristPose. If provided and valid, solves the uncalibrated
                        raw joint angles and sets them as q_zero_offset. If None, uses the last
                        computed raw uncalibrated joint angles.

        Returns:
            np.ndarray: Canonical zero joint angles (all zeros, shape (16,)).
        """
        raw_zero = None
        if wrist_pose is not None and wrist_pose.is_valid and wrist_pose.tracking_state == TrackingState.TRACKING:
            try:
                raw_zero = self.compute_raw_joint_angles(wrist_pose)
            except Exception:
                raw_zero = None

        if raw_zero is None and self._last_raw_q is not None:
            raw_zero = self._last_raw_q.copy()

        if raw_zero is not None:
            self.q_zero_offset = raw_zero.copy()

        # Flush filter history and reset velocity limiter so hand opens immediately to zero
        self._filter.reset()
        self._last_q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._last_valid_time = None

        return np.zeros(HAND_NUM_DOFS, dtype=np.float64)

    def set_calibration(
        self,
        q_zero_offset: Optional[np.ndarray | list[float]] = None,
        palm_reference_length_m: Optional[float] = None,
    ) -> None:
        """Apply pre-calibrated user hand zero offset and physical reference palm scale."""
        if q_zero_offset is not None:
            self.q_zero_offset = np.asarray(q_zero_offset, dtype=np.float64)
        if palm_reference_length_m is not None and palm_reference_length_m > 0.03:
            self.palm_reference_length_m = float(palm_reference_length_m)

    @property
    def metrics(self) -> HandRetargetingMetrics:
        """Return diagnostic metrics from the last retargeting step."""
        return self._last_metrics

    @property
    def last_metrics(self) -> HandRetargetingMetrics:
        """Alias for metrics."""
        return self._last_metrics

    def get_metrics(self) -> HandRetargetingMetrics:
        """Return diagnostic metrics from the last retargeting step."""
        return self._last_metrics

    def retarget_pose(
        self,
        wrist_pose: WristPose,
        timestamp: Optional[float] = None,
    ) -> TeleopHandCommand:
        """Retarget a full WristPose from wrist_tracker into 16-DOF LEAP Hand angles."""
        now = wrist_pose.timestamp if timestamp is None else float(timestamp)

        # If hand tracking is completely lost or invalid, hold last pose smoothly
        if not wrist_pose.is_valid or wrist_pose.tracking_state == TrackingState.NO_HAND:
            return TeleopHandCommand(
                timestamp=now,
                target_q=self._last_q.copy(),
                confidence=0.0,
                is_clutched=False,
            )

        t_start = time.perf_counter()

        # 1. Transform 21 landmarks into local hand coordinate frame
        # Check if world landmarks are valid and physiologically plausible (non-zero and non-degenerate palm length)
        use_world = False
        if wrist_pose.landmarks_world_3d is not None:
            w_pts = wrist_pose.landmarks_world_3d
            if w_pts.shape == (21, 3) and np.all(np.isfinite(w_pts)) and not np.all(w_pts == 0):
                w_span = float(np.linalg.norm(w_pts[LM_MIDDLE_MCP] - w_pts[LM_WRIST]))
                if 0.04 < w_span < 0.20:
                    use_world = True

        if use_world:
            pts_source = wrist_pose.landmarks_world_3d.copy()
            p_wrist = pts_source[LM_WRIST]
            _, _, _, R_basis = compute_palm_basis(pts_source, wrist_pose.handedness)
            pts_local = (R_basis.T @ (pts_source - p_wrist).T).T
        else:
            pts_3d_cam = wrist_pose.landmarks_3d_cam
            p_wrist = pts_3d_cam[LM_WRIST]
            _, _, _, R_cam = compute_palm_basis(pts_3d_cam, wrist_pose.handedness)
            pts_local = (R_cam.T @ (pts_3d_cam - p_wrist).T).T  # (21, 3)

        # Hand scale normalization based on adult reference palm length (~0.085m or calibrated length)
        palm_len = float(np.linalg.norm(pts_local[LM_MIDDLE_MCP] - pts_local[LM_WRIST]))
        if palm_len > 0.03:
            scale_factor = self.palm_reference_length_m / palm_len
            pts_local = pts_local * scale_factor

        # 2. Compute gesture telemetry (pinch distance, flexion)
        thumb_tip = pts_local[LM_THUMB_TIP]
        index_tip = pts_local[LM_INDEX_TIP]
        middle_tip = pts_local[LM_MIDDLE_TIP]
        pinch_dist = float(np.linalg.norm(thumb_tip - index_tip))
        pinch_mid_dist = float(np.linalg.norm(thumb_tip - middle_tip))
        is_pinching = pinch_dist < self.pinch_threshold_m

        # 3. Solve 16-DOF joint angles based on active mode
        raw_q = None

        if self.solver_mode == "geometric":
            raw_q = self._bio_retargeter.retarget(pts_local)

        elif self.solver_mode == "dex_retargeting" and self._has_dex_retargeting:
            try:
                # Transform to LEAP Hand root frame (palm_lower)
                pts_robot = (R_ALIGN_HUMAN_TO_ROBOT @ pts_local.T).T
                # Apply ulnar synergy to the 4th outer finger link
                p_virtual_outer = 0.65 * pts_robot[LM_RING_TIP] + 0.35 * pts_robot[LM_PINKY_TIP]
                pts_robot_augmented = pts_robot.copy()
                pts_robot_augmented[LM_RING_TIP] = p_virtual_outer

                task_pts = pts_robot_augmented[self._human_joint_map[self._dexpilot_task_idx]]
                origin_pts = pts_robot_augmented[self._human_joint_map[self._dexpilot_origin_idx]]
                dexpilot_vectors = task_pts - origin_pts

                raw_q = self._dexpilot_retargeter.retarget(dexpilot_vectors)
            except Exception:
                raw_q = self._bio_retargeter.retarget(pts_local)

        else:
            # "hybrid" mode (Default):
            # 1. Base joint angles from biomechanical bone-flexion (immediate, full-range curl, ulnar synergy)
            q_bio = self._bio_retargeter.retarget(pts_local)

            # 2. Continuous smooth pinch-contact optimization blend
            # Transitions continuously over distance window [0.025m, 0.045m] to eliminate step-function twitching.
            if self._has_dex_retargeting:
                d_min = 0.025
                d_max = 0.045
                if pinch_dist < d_max:
                    # Continuous blending weight from 0.0 (at d_max) to 0.5 (at d_min)
                    alpha = float(np.clip((d_max - pinch_dist) / (d_max - d_min), 0.0, 1.0))
                    w = 0.5 * alpha
                    try:
                        pts_robot = (R_ALIGN_HUMAN_TO_ROBOT @ pts_local.T).T
                        p_virtual_outer = 0.65 * pts_robot[LM_RING_TIP] + 0.35 * pts_robot[LM_PINKY_TIP]
                        pts_robot_augmented = pts_robot.copy()
                        pts_robot_augmented[LM_RING_TIP] = p_virtual_outer

                        task_pts = pts_robot_augmented[self._human_joint_map[self._dexpilot_task_idx]]
                        origin_pts = pts_robot_augmented[self._human_joint_map[self._dexpilot_origin_idx]]
                        dexpilot_vectors = task_pts - origin_pts

                        q_opt = self._dexpilot_retargeter.retarget(dexpilot_vectors)
                        raw_q = q_bio.copy()
                        raw_q[12:15] = (1.0 - w) * q_bio[12:15] + w * q_opt[12:15]  # Thumb yaw, cmc, mcp
                        # Ensure DexPilot optimization does not degrade distal curling of thumb_ip
                        raw_q[15] = max(q_bio[15], (1.0 - w) * q_bio[15] + w * q_opt[15])
                        raw_q[3] = (1.0 - w) * q_bio[3] + w * q_opt[3]              # Index DIP
                    except Exception:
                        raw_q = q_bio
                else:
                    raw_q = q_bio
            else:
                raw_q = q_bio

        self._last_raw_q = raw_q.copy()

        # Zero-offset bias cancellation with range preservation (LEAP Hand & AnyTeleop standard)
        if self.q_zero_offset is not None:
            # 1. Yaw spread joints: align to neutral zero
            yaw_indices = [0, 4, 8, 12]
            for idx in yaw_indices:
                raw_q[idx] = raw_q[idx] - float(self.q_zero_offset[idx])

            # 2. Active flexion joint indices: MCP/PIP/DIP for index, middle, ring, and thumb
            flex_indices = [1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15]
            for idx in flex_indices:
                q0 = float(self.q_zero_offset[idx])
                q_max = float(self.joint_limits[idx, 1])
                if q0 > 0.01:
                    # Piecewise linear rescaling: flat hand maps to 0 while full fist reaches q_max
                    effective_q0 = min(q0, 0.50 * q_max)
                    if raw_q[idx] <= effective_q0:
                        raw_q[idx] = 0.0
                    else:
                        raw_q[idx] = (raw_q[idx] - effective_q0) / (q_max - effective_q0) * q_max
                else:
                    raw_q[idx] = max(0.0, raw_q[idx] - q0)

        # 4. Joint limits compliance check
        raw_q = np.clip(raw_q, self.joint_limits[:, 0], self.joint_limits[:, 1])

        # 5. Temporal 1€ filter smoothing
        filtered_q = self._filter.filter(raw_q, timestamp=now)
        filtered_q = np.clip(filtered_q, self.joint_limits[:, 0], self.joint_limits[:, 1])

        # 6. Slew-rate velocity limit (protect hardware and suppress single-frame spikes)
        if self._last_valid_time is not None and (now > self._last_valid_time + 1e-4) and np.any(self._last_q != 0.0):
            dt_s = min(0.10, now - self._last_valid_time)
            max_delta = self.max_joint_velocity_rad_s * dt_s
            filtered_q = np.clip(
                filtered_q,
                self._last_q - max_delta,
                self._last_q + max_delta,
            )
        self._last_valid_time = now
        self._last_q = filtered_q.copy()
        dt_ms = (time.perf_counter() - t_start) * 1000.0

        # Update telemetry metrics
        self._last_metrics = HandRetargetingMetrics(
            computation_time_ms=dt_ms,
            thumb_index_pinch_dist_m=pinch_dist,
            thumb_middle_pinch_dist_m=pinch_mid_dist,
            is_pinching=is_pinching,
            average_flexion_rad=float(np.mean(filtered_q[[1, 2, 3, 5, 6, 7, 9, 10, 11]])),
            solver_mode=self.solver_mode,
        )

        return TeleopHandCommand(
            timestamp=now,
            target_q=filtered_q,
            confidence=1.0 if wrist_pose.tracking_state == TrackingState.TRACKING else 0.5,
            is_clutched=(wrist_pose.tracking_state in (TrackingState.TRACKING, TrackingState.TEMPORAL_HOLD)),
        )

