"""table_calibrator.py — Dual-marker (ID0 & ID1) diagonal layout ArUco table calibration.

Supports:
- 80mm ArUco markers (DICT_4X4_50) in diagonal layout.
- OpenCV 5.0 / 4.7+ ArucoDetector with SOLVEPNP_IPPE_SQUARE.
- Markley quaternion averaging over 30 stable frames.
- Relative transform recovery between ID0 and ID1.
- Complete table coordinate frame T_cam_to_table and (T_cam_to_table)^-1.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np

try:
    import yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False

try:
    from tuinadex_teleop.core.types import ArUcoCalibrationData
except ImportError:
    from tuina_teleop.core.types import ArUcoCalibrationData
from .realsense_stream import CameraIntrinsics


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to unit quaternion [w, x, y, z]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def average_quaternions(quaternions: List[np.ndarray]) -> np.ndarray:
    """Optimal quaternion averaging via Markley eigenvector algorithm."""
    if not quaternions:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if len(quaternions) == 1:
        return quaternions[0] / np.linalg.norm(quaternions[0])

    q_ref = quaternions[0]
    A = np.zeros((4, 4), dtype=np.float64)
    for q in quaternions:
        q_norm = q / np.linalg.norm(q)
        if np.dot(q_norm, q_ref) < 0.0:
            q_norm = -q_norm  # Sign consistency
        A += np.outer(q_norm, q_norm)

    eigenvalues, eigenvectors = np.linalg.eigh(A)
    max_idx = np.argmax(eigenvalues)
    avg_q = eigenvectors[:, max_idx]
    if avg_q[0] < 0.0:
        avg_q = -avg_q
    return avg_q / np.linalg.norm(avg_q)


@dataclass
class MarkerPoseObservation:
    """Observation of a single ArUco marker in camera frame."""
    marker_id: int
    t_cam_to_marker: np.ndarray  # 4x4
    corners_2d: np.ndarray       # (4, 2)
    center_2d: np.ndarray        # (2,)


class TableCalibrator:
    """Detects dual diagonal markers (ID0 & ID1) and computes calibrated Table Frame."""

    def __init__(
        self,
        marker_size_m: float = 0.080,
        dictionary_name: str = "DICT_4X4_50",
        primary_id: int = 0,
        secondary_id: int = 1,
        min_samples: int = 30,
    ):
        self.marker_size_m = marker_size_m
        self.primary_id = primary_id
        self.secondary_id = secondary_id
        self.min_samples = min_samples

        # 3D planar corner object points in marker local frame (center at origin, Z=0)
        half_s = marker_size_m / 2.0
        self._obj_points = np.array([
            [-half_s,  half_s, 0.0],
            [ half_s,  half_s, 0.0],
            [ half_s, -half_s, 0.0],
            [-half_s, -half_s, 0.0],
        ], dtype=np.float32)

        # Initialize OpenCV ArUco detector
        dict_id = getattr(cv2.aruco, dictionary_name, cv2.aruco.DICT_4X4_50)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Multi-frame sample buffer
        self._samples_id0: List[np.ndarray] = []
        self._samples_id1: List[np.ndarray] = []
        self._relative_id0_to_id1: List[np.ndarray] = []

    def detect_markers(
        self,
        color_image: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> Dict[int, MarkerPoseObservation]:
        """Detect ArUco markers and solve 6-DOF pose for each detected marker."""
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        results: Dict[int, MarkerPoseObservation] = {}
        if ids is None or len(ids) == 0:
            return results

        camera_matrix = intrinsics.matrix
        dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

        for i, marker_id in enumerate(ids.flatten()):
            c_2d = corners[i][0]  # shape (4, 2)
            # Solve PnP for planar square
            success, rvec, tvec = cv2.solvePnP(
                self._obj_points,
                c_2d,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue

            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()

            center = np.mean(c_2d, axis=0)
            results[int(marker_id)] = MarkerPoseObservation(
                marker_id=int(marker_id),
                t_cam_to_marker=T,
                corners_2d=c_2d,
                center_2d=center,
            )

        return results

    def add_frame_sample(self, observations: Dict[int, MarkerPoseObservation]) -> bool:
        """Add current frame observations to calibration buffer.
        
        Returns:
            True if primary marker (ID0) was successfully captured in this frame.
        """
        has_id0 = self.primary_id in observations
        has_id1 = self.secondary_id in observations

        if has_id0:
            self._samples_id0.append(observations[self.primary_id].t_cam_to_marker)
        if has_id1:
            self._samples_id1.append(observations[self.secondary_id].t_cam_to_marker)

        # If both are visible, compute relative transform ID0 -> ID1
        if has_id0 and has_id1:
            T_cam_0 = observations[self.primary_id].t_cam_to_marker
            T_cam_1 = observations[self.secondary_id].t_cam_to_marker
            T_0_1 = np.linalg.inv(T_cam_0) @ T_cam_1
            self._relative_id0_to_id1.append(T_0_1)

        return has_id0

    @property
    def sample_count(self) -> int:
        return len(self._samples_id0)

    @property
    def is_ready(self) -> bool:
        return self.sample_count >= self.min_samples

    def reset(self) -> None:
        self._samples_id0.clear()
        self._samples_id1.clear()
        self._relative_id0_to_id1.clear()

    def compute_calibration(self) -> ArUcoCalibrationData:
        """Compute robust average Table Frame from accumulated samples."""
        if not self._samples_id0:
            raise RuntimeError("No primary marker (ID0) samples captured.")

        # 1. Average ID0 pose
        t_id0_samples = [T[:3, 3] for T in self._samples_id0]
        q_id0_samples = [rotation_matrix_to_quaternion(T[:3, :3]) for T in self._samples_id0]

        avg_t0 = np.mean(t_id0_samples, axis=0)
        avg_q0 = average_quaternions(q_id0_samples)
        avg_R0 = quaternion_to_rotation_matrix(avg_q0)

        # Primary Table Frame: Origin at ID0, Z-axis pointing up (normal to table)
        T_cam_to_table = np.eye(4, dtype=np.float64)
        T_cam_to_table[:3, :3] = avg_R0
        T_cam_to_table[:3, 3] = avg_t0

        T_table_to_cam = np.linalg.inv(T_cam_to_table)

        calib = ArUcoCalibrationData(
            t_cam_to_table=T_cam_to_table,
            t_table_to_cam=T_table_to_cam,
            tag_id=self.primary_id,
            tag_size_m=self.marker_size_m,
            num_samples=len(self._samples_id0),
            is_valid=True,
        )
        return calib

    def get_baseline_distance(self) -> Optional[float]:
        """Get measured diagonal distance (meters) between ID0 and ID1."""
        if not self._relative_id0_to_id1:
            return None
        translations = [T[:3, 3] for T in self._relative_id0_to_id1]
        avg_trans = np.mean(translations, axis=0)
        return float(np.linalg.norm(avg_trans))

    def save_calibration(
        self,
        calib: ArUcoCalibrationData,
        filepath: str,
    ) -> None:
        """Save calibration result to a YAML or JSON file."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        baseline = self.get_baseline_distance()

        data = {
            "marker_size_m": float(self.marker_size_m),
            "primary_id": int(self.primary_id),
            "secondary_id": int(self.secondary_id),
            "num_samples": int(calib.num_samples),
            "baseline_distance_m": float(baseline) if baseline is not None else None,
            "t_cam_to_table": calib.t_cam_to_table.tolist(),
            "t_table_to_cam": calib.t_table_to_cam.tolist(),
        }
        if _HAS_YAML and (filepath.endswith(".yaml") or filepath.endswith(".yml")):
            with open(filepath, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, sort_keys=False)
        else:
            # Fallback to standard JSON or json-compatible format
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)

    @classmethod
    def load_calibration(cls, filepath: str) -> ArUcoCalibrationData:
        """Load calibration result from a YAML or JSON file."""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Calibration file not found: {filepath}")

        data = None
        if _HAS_YAML and (filepath.endswith(".yaml") or filepath.endswith(".yml")):
            with open(filepath, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        if data is None:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

        t_cam = np.array(data["t_cam_to_table"], dtype=np.float64)
        t_tab = np.array(data["t_table_to_cam"], dtype=np.float64)

        return ArUcoCalibrationData(
            t_cam_to_table=t_cam,
            t_table_to_cam=t_tab,
            tag_id=int(data.get("primary_id", 0)),
            tag_size_m=float(data.get("marker_size_m", 0.080)),
            num_samples=int(data.get("num_samples", 0)),
            is_valid=True,
        )

    def draw_axes(
        self,
        image: np.ndarray,
        observations: Dict[int, MarkerPoseObservation],
        intrinsics: CameraIntrinsics,
    ) -> np.ndarray:
        """Draw 3D coordinate axes and marker boundaries on image."""
        vis = image.copy()
        axis_length = self.marker_size_m * 0.8
        camera_matrix = intrinsics.matrix
        dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

        for marker_id, obs in observations.items():
            R = obs.t_cam_to_marker[:3, :3]
            t = obs.t_cam_to_marker[:3, 3]
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape((3, 1))

            cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, axis_length, 2)
            # Label marker
            cx, cy = int(obs.center_2d[0]), int(obs.center_2d[1])
            dist_str = f"ID{marker_id}: {t[2]:.2f}m"
            cv2.putText(vis, dist_str, (cx - 30, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return vis
