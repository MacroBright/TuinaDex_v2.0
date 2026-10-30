"""imu_stream.py — High-performance 6-axis IMU interface and wrist attitude tracker for TuinaDex teleoperation.

Features:
  1. ImuReading dataclass containing normalized quaternion (w, x, y, z), 3x3 rotation matrix,
     Euler angles (roll, pitch, yaw) in degrees and radians, linear acceleration, and angular velocity.
  2. ImuStream: Real serial communication with Yahboom/CH340 6-axis IMU via YbImuProtocol.
  3. MockImuStream: Fully deterministic simulated stream for offline tests and CI/CD.
  4. WristImuTracker: Relative spatial attitude mapping from wrist IMU to robot end-effector frame,
     supporting zero-referencing, axis alignment, and spatial clutching.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np

# Ensure rebot_imu protocol can be imported if present in repo
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_REBOT_IMU_DIR = _PROJECT_ROOT.parent / "imu" / "rebot_imu"
if _REBOT_IMU_DIR.exists() and str(_REBOT_IMU_DIR) not in sys.path:
    sys.path.insert(0, str(_REBOT_IMU_DIR))

try:
    from rebot_imu.protocol import YbImuProtocol, ImuDataSnapshot
    _HAS_REBOT_IMU_PROTOCOL = True
except ImportError:
    _HAS_REBOT_IMU_PROTOCOL = False


# Default downward rotation (maps +Z_tool to -Z_base, identical to ArmIKSolver.R_DOWNWARD_DEFAULT)
R_DOWNWARD_DEFAULT = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=np.float64)


def quat_to_rotation_matrix(q: Union[np.ndarray, Tuple[float, float, float, float]]) -> np.ndarray:
    """Convert unit quaternion (w, x, y, z) to 3x3 rotation matrix.
    
    Args:
        q: 4-element array-like [w, x, y, z]
    Returns:
        (3, 3) orthonormal rotation matrix in SO(3)
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-8:
        return np.eye(3, dtype=np.float64)
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    R = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)
    return R


def rotation_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to unit quaternion (w, x, y, z)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    q = np.array([w, x, y, z], dtype=np.float64)
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-8 else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def rotation_matrix_to_euler_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """Convert 3x3 rotation matrix to intrinsic Euler angles (roll, pitch, yaw) in degrees."""
    # Using ZYX (yaw, pitch, roll) convention
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    if math.cos(pitch) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def parse_axis_mapping(mapping_str: str) -> np.ndarray:
    """Parse axis mapping string into 3x3 permutation/sign matrix.
    
    Examples:
      '+x,+y,+z' -> Identity
      '+y,-x,+z' -> Hand dorsal mounting (Y along fingers, X across knuckles)
      '-y,+x,+z' -> 90 deg yaw
      '+x,-y,-z' -> mirror
    """
    tokens = [t.strip().lower() for t in mapping_str.split(",")]
    if len(tokens) != 3:
        raise ValueError(f"axis_mapping must contain 3 tokens, got '{mapping_str}'")
    R = np.zeros((3, 3), dtype=np.float64)
    axis_map = {"x": 0, "y": 1, "z": 2}
    for row, token in enumerate(tokens):
        sign = -1.0 if token.startswith("-") else 1.0
        char = token.lstrip("+-")
        if char not in axis_map:
            raise ValueError(f"Invalid axis '{char}' in token '{token}'")
        col = axis_map[char]
        R[row, col] = sign
    return R


def rotation_matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 orthonormal rotation matrix to rotation vector (axis * angle)."""
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = math.acos(tr)
    if abs(theta) < 1e-6:
        return np.zeros(3, dtype=np.float64)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]], dtype=np.float64)
    axis = v / (2.0 * math.sin(theta))
    return axis * theta


def solve_imu_alignment_from_gestures(
    R_ref: np.ndarray,
    R_pitch_up: np.ndarray,
    R_roll_right: np.ndarray,
    R_yaw: Optional[np.ndarray] = None,
) -> str:
    """Solve the 3-axis alignment mapping string from reference gestures.
    
    Bed-side coordinate convention (robot on left side of massage bed):
      - Robot Base Forward Axis (X): perpendicular to patient spine midline.
        Driven by Hand Pitch (围绕指关节横轴压抬/掌屈背伸, IMU X).
      - Robot Base Lateral Axis (Y): along patient body (head<- ->legs).
        Driven by Hand Roll (围绕中指/前臂纵轴旋前旋后, IMU Y).
      - Robot Base Vertical Axis (Z): perpendicular to bed surface (up/down).
        Driven by Hand Yaw (围绕手背法向量偏摆, IMU Z, solved with det(R) = +1).

    Gestures:
      - R_ref: Neutral baseline posture (hand flat on table, palm down, pointing forward).
      - R_pitch_up: Hand pitched upward (lift fingers up ~25-45 deg from table).
                    Mapped to Robot Base Forward Axis (X).
      - R_roll_right: Hand rolled to the right (rotate hand ~25-45 deg towards right).
                    Mapped to Robot Base Lateral Axis (Y).
      - R_yaw: (Optional) Hand yawed along desk surface (~20-40 deg).
                    Mapped to Robot Base Vertical Axis (Z).
                    
    Returns:
      Canonical 3-token mapping string, e.g. '+x,+y,+z', where:
      token 0: IMU axis driving Robot X (Forward axis pitch/tilt)
      token 1: IMU axis driving Robot Y (Lateral axis roll)
      token 2: IMU axis driving Robot Z (Vertical axis yaw, solved for right-handedness det=+1)
    """
    axis_names = ["x", "y", "z"]

    # 1. Delta for Hand Pitch (driving Robot Forward Axis X)
    delta_pitch = R_pitch_up @ R_ref.T
    rotvec_pitch = rotation_matrix_to_rotvec(delta_pitch)
    angle_pitch = float(np.linalg.norm(rotvec_pitch))
    if angle_pitch < math.radians(10.0):
        raise ValueError(
            f"Pitch gesture movement too small ({math.degrees(angle_pitch):.1f}° < 10°). "
            "Please lift your fingers clearly upward by at least 25° to 45°."
        )
    pitch_axis_idx = int(np.argmax(np.abs(rotvec_pitch)))
    pitch_sign = 1 if rotvec_pitch[pitch_axis_idx] > 0 else -1
    token_x = f"{'+' if pitch_sign > 0 else '-'}{axis_names[pitch_axis_idx]}"

    # 2. Delta for Hand Roll (driving Robot Lateral Axis Y)
    delta_roll = R_roll_right @ R_ref.T
    rotvec_roll = rotation_matrix_to_rotvec(delta_roll)
    angle_roll = float(np.linalg.norm(rotvec_roll))
    if angle_roll < math.radians(10.0):
        raise ValueError(
            f"Roll gesture movement too small ({math.degrees(angle_roll):.1f}° < 10°). "
            "Please tilt your hand clearly to the right by at least 25° to 45°."
        )
    roll_axis_idx = int(np.argmax(np.abs(rotvec_roll)))
    roll_sign = 1 if rotvec_roll[roll_axis_idx] > 0 else -1
    token_y = f"{'+' if roll_sign > 0 else '-'}{axis_names[roll_axis_idx]}"

    if roll_axis_idx == pitch_axis_idx:
        raise ValueError(
            f"Ambiguous calibration: Roll and Pitch both detected primarily on IMU axis '{axis_names[roll_axis_idx]}'. "
            "Please ensure clear distinct motions (pitching strictly up, rolling strictly right)."
        )

    # 3. Solve remaining axis for Yaw (Robot Z) to guarantee right-handedness det(R) = +1
    used_axes = {roll_axis_idx, pitch_axis_idx}
    yaw_axis_idx = list({0, 1, 2} - used_axes)[0]

    preferred_signs = [1, -1]
    if R_yaw is not None:
        delta_yaw = R_yaw @ R_ref.T
        rotvec_yaw = rotation_matrix_to_rotvec(delta_yaw)
        angle_yaw = float(np.linalg.norm(rotvec_yaw))
        if angle_yaw >= math.radians(8.0):
            meas_yaw_axis = int(np.argmax(np.abs(rotvec_yaw)))
            meas_yaw_sign = 1 if rotvec_yaw[meas_yaw_axis] > 0 else -1
            if meas_yaw_axis == yaw_axis_idx:
                preferred_signs = [meas_yaw_sign, -meas_yaw_sign]

    for sign_z in preferred_signs:
        token_z = f"{'+' if sign_z > 0 else '-'}{axis_names[yaw_axis_idx]}"
        candidate = f"{token_x},{token_y},{token_z}"
        R = parse_axis_mapping(candidate)
        if np.isclose(np.linalg.det(R), 1.0):
            return candidate

    raise RuntimeError("Failed to resolve right-handed orthonormal matrix for IMU alignment.")


@dataclass
class ImuReading:
    """Standardized snapshot of 6-axis/9-axis IMU readings."""
    timestamp: float = 0.0
    quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))  # (w, x, y, z)
    rotation_matrix: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=np.float64))             # 3x3
    euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)                                              # (roll, pitch, yaw) deg
    euler_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0)                                              # (roll, pitch, yaw) rad
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))                    # m/s^2
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))                     # rad/s
    update_count: int = 0
    is_valid: bool = False


class ImuStream:
    """Production serial interface to Yahboom CH340 6-axis IMU."""

    def __init__(
        self,
        port: str = "/dev/myimu",
        baudrate: int = 115200,
        timeout: float = 0.5,
        auto_detect: bool = True,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.auto_detect = auto_detect
        self._protocol: Optional[Any] = None
        self._is_connected = False
        self._last_reading = ImuReading()

    def _resolve_port(self) -> Optional[str]:
        """Check user requested port and standard candidate paths."""
        candidates = [self.port, "/dev/myimu", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"] if self.auto_detect else [self.port]
        seen = set()
        for p in candidates:
            if p and p not in seen:
                seen.add(p)
                if os.path.exists(p):
                    return p
        return None

    def start(self) -> bool:
        """Open serial port and start high-frequency background worker."""
        if not _HAS_REBOT_IMU_PROTOCOL:
            print("[WARN] YbImuProtocol not found in sys.path. ImuStream unavailable.")
            return False

        resolved = self._resolve_port()
        if not resolved:
            print(f"[WARN] No available IMU serial port found (tried {self.port}, /dev/myimu, /dev/ttyUSB*).")
            return False

        try:
            self._protocol = YbImuProtocol(port=resolved, baudrate=self.baudrate, timeout=self.timeout)
            if self._protocol.open():
                self._is_connected = True
                print(f"[INFO] ImuStream connected successfully on {resolved} @ {self.baudrate} baud.")
                return True
            else:
                self._protocol = None
                self._is_connected = False
                return False
        except Exception as e:
            print(f"[ERROR] Failed to start ImuStream: {e}")
            self._protocol = None
            self._is_connected = False
            return False

    def stop(self) -> None:
        """Close serial port safely."""
        if self._protocol is not None:
            try:
                self._protocol.close()
            except Exception:
                pass
            self._protocol = None
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        return self._is_connected and (self._protocol is not None) and self._protocol.is_open()

    def read(self) -> ImuReading:
        """Read the latest thread-safe snapshot from the IMU."""
        if not self.is_connected or self._protocol is None:
            return ImuReading(timestamp=time.time(), is_valid=False)

        try:
            snap = self._protocol.get_snapshot()
            quat_np = np.array(snap.quat, dtype=np.float64)
            # Ensure unit quaternion
            norm = np.linalg.norm(quat_np)
            if norm > 1e-6:
                quat_np /= norm
            else:
                quat_np = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

            R_mat = quat_to_rotation_matrix(quat_np)

            reading = ImuReading(
                timestamp=snap.timestamp if snap.timestamp > 0 else time.time(),
                quat=quat_np,
                rotation_matrix=R_mat,
                euler_deg=snap.euler_deg,
                euler_rad=snap.euler_rad,
                accel=np.array(snap.accel, dtype=np.float64),
                gyro=np.array(snap.gyro, dtype=np.float64),
                update_count=snap.update_count,
                is_valid=snap.update_count > 0,
            )
            self._last_reading = reading
            return reading
        except Exception as e:
            print(f"[WARN] Error reading IMU snapshot: {e}")
            return ImuReading(timestamp=time.time(), is_valid=False)


class MockImuStream:
    """Deterministic simulated IMU stream for automated testing and CI/CD."""

    def __init__(self, publish_rate_hz: float = 50.0) -> None:
        self.publish_rate_hz = publish_rate_hz
        self._running = False
        self._t0 = time.monotonic()
        self._update_count = 0
        self._manual_euler_deg: Optional[Tuple[float, float, float]] = None

    def start(self) -> bool:
        self._running = True
        self._t0 = time.monotonic()
        self._update_count = 0
        return True

    def stop(self) -> None:
        self._running = False

    @property
    def is_connected(self) -> bool:
        return self._running

    def set_synthetic_euler(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        """Directly inject synthetic orientation angles for test verification."""
        self._manual_euler_deg = (float(roll_deg), float(pitch_deg), float(yaw_deg))

    def clear_synthetic_euler(self) -> None:
        self._manual_euler_deg = None

    def read(self) -> ImuReading:
        """Generate synthetic IMU reading with realistic sinusoidal or manual posture."""
        now = time.monotonic()
        self._update_count += 1

        if self._manual_euler_deg is not None:
            r_deg, p_deg, y_deg = self._manual_euler_deg
        else:
            # Gentle sinusoidal motion (+-20 deg pitch, +-25 deg roll, +-10 deg yaw)
            t = now - self._t0
            r_deg = 20.0 * math.sin(t * 1.2)
            p_deg = 15.0 * math.cos(t * 0.9)
            y_deg = 10.0 * math.sin(t * 0.6)

        r_rad = math.radians(r_deg)
        p_rad = math.radians(p_deg)
        y_rad = math.radians(y_deg)

        # Build 3x3 rotation matrix from ZYX Euler angles
        cz, sz = math.cos(y_rad), math.sin(y_rad)
        cy, sy = math.cos(p_rad), math.sin(p_rad)
        cx, sx = math.cos(r_rad), math.sin(r_rad)

        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
        R_mat = Rz @ Ry @ Rx

        quat = rotation_matrix_to_quat(R_mat)

        return ImuReading(
            timestamp=now,
            quat=quat,
            rotation_matrix=R_mat,
            euler_deg=(r_deg, p_deg, y_deg),
            euler_rad=(r_rad, p_rad, y_rad),
            accel=np.array([0.0, 0.0, 9.80665], dtype=np.float64),
            gyro=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            update_count=self._update_count,
            is_valid=True,
        )


class WristImuTracker:
    """Manages wrist attitude tracking, zero-referencing, and spatial mapping to robot tool frame.
    
    Mathematical Formulation:
      1. Human wrist relative rotation since reference posture:
           Delta_R_imu = R_imu(t) * R_imu_ref^T
      2. Transform to robot base coordinate frame via mounting alignment:
           Delta_R_robot = R_align * Delta_R_imu * R_align^T
      3. Apply to robot tool target orientation:
           R_target = Delta_R_robot * R_neutral_robot
    """

    def __init__(
        self,
        axis_mapping: str = "+x,+y,+z",
        neutral_robot_rot: Optional[np.ndarray] = None,
        max_angle_delta_rad: float = 1.60,  # Safety clamp: max 91 degrees from neutral
    ) -> None:
        self.axis_mapping_str = axis_mapping
        self.R_align = parse_axis_mapping(axis_mapping)
        self.R_neutral = (
            np.array(neutral_robot_rot, dtype=np.float64)
            if neutral_robot_rot is not None
            else R_DOWNWARD_DEFAULT.copy()
        )
        self.max_angle_delta_rad = max_angle_delta_rad

        self._R_ref: Optional[np.ndarray] = None
        self._is_calibrated = False
        self._last_target_R = self.R_neutral.copy()
        self._last_delta_euler_deg = (0.0, 0.0, 0.0)

    @property
    def is_calibrated(self) -> bool:
        """Whether a zero-reference orientation has been established."""
        return self._is_calibrated and (self._R_ref is not None)

    def set_axis_mapping(self, mapping_str: str) -> None:
        """Set coordinate axis permutation string, e.g. '+x,+y,+z', '-y,+x,+z'."""
        self.R_align = parse_axis_mapping(mapping_str)
        self.axis_mapping_str = mapping_str

    def zero(self, reading: Optional[ImuReading] = None) -> None:
        """Establish current posture as the zero-reference neutral orientation."""
        if reading is not None and reading.is_valid:
            self._R_ref = reading.rotation_matrix.copy()
            self._is_calibrated = True
        else:
            self._R_ref = np.eye(3, dtype=np.float64)
            self._is_calibrated = True
        self._last_target_R = self.R_neutral.copy()
        self._last_delta_euler_deg = (0.0, 0.0, 0.0)

    def set_reference_quat(self, quat_wxyz: Union[np.ndarray, List[float], Tuple[float, float, float, float]]) -> None:
        """Set pre-calibrated reference quaternion from session calibration."""
        R = quat_to_rotation_matrix(quat_wxyz)
        self._R_ref = R.copy()
        self._is_calibrated = True
        self._last_target_R = self.R_neutral.copy()
        self._last_delta_euler_deg = (0.0, 0.0, 0.0)

    def reset(self) -> None:
        """Clear calibration and reset to neutral."""
        self._R_ref = None
        self._is_calibrated = False
        self._last_target_R = self.R_neutral.copy()
        self._last_delta_euler_deg = (0.0, 0.0, 0.0)

    def compute_target_rotation(
        self,
        reading: ImuReading,
        fallback_rot: Optional[np.ndarray] = None,
        anchor_robot_rot: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compute 3x3 target orientation matrix for 6-DOF robot arm end-effector.
        
        Args:
            reading: Current ImuReading snapshot.
            fallback_rot: Rotation to use if IMU reading is invalid.
            anchor_robot_rot: Optional reference robot rotation to anchor to (defaults to R_neutral).
            
        Returns:
            (3, 3) orthonormal target rotation matrix in robot base frame.
        """
        R_base_anchor = anchor_robot_rot if anchor_robot_rot is not None else self.R_neutral

        if not reading.is_valid:
            if fallback_rot is not None:
                return np.asarray(fallback_rot, dtype=np.float64)
            return self._last_target_R.copy()

        # Auto-calibrate reference on very first valid reading
        if not self._is_calibrated or self._R_ref is None:
            self.zero(reading)

        # 1. Delta rotation in IMU sensor body frame: Delta_R_imu = R_curr * R_ref^T
        R_curr = reading.rotation_matrix
        Delta_R_imu = R_curr @ self._R_ref.T

        # 2. Similarity transformation to robot base frame: Delta_R_robot = R_align * Delta_R_imu * R_align^T
        Delta_R_robot = self.R_align @ Delta_R_imu @ self.R_align.T

        # 3. Apply rotation delta to neutral robot tool orientation
        R_target = Delta_R_robot @ R_base_anchor

        # Ensure strict orthonormality via SVD projection
        U, _, Vt = np.linalg.svd(R_target)
        R_target_ortho = U @ Vt
        if np.linalg.det(R_target_ortho) < 0:
            U[:, -1] *= -1.0
            R_target_ortho = U @ Vt

        # Compute delta Euler angles for HUD telemetry
        self._last_delta_euler_deg = rotation_matrix_to_euler_deg(Delta_R_robot)
        self._last_target_R = R_target_ortho
        return R_target_ortho

    @property
    def delta_euler_deg(self) -> Tuple[float, float, float]:
        """Return relative (roll, pitch, yaw) angles in degrees from reference pose."""
        return self._last_delta_euler_deg
