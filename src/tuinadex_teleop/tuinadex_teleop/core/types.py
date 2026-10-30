"""Strongly-typed data contracts and value objects for TuinaDex teleoperation.

All physical coordinates are in SI units:
- Linear distances: meters (m)
- Angles and joint positions: radians (rad)
- Velocities: rad/s or m/s
- Torques / Efforts: N*m or N
- Timestamps: float seconds (monotonic time)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional
import numpy as np

from .names import ARM_NUM_DOFS, HAND_NUM_DOFS, COMPOSITE_NUM_DOFS


class ClutchState(Enum):
    """Clutch state machine state for incremental teleoperation."""
    DISENGAGED = auto()       # Robot stationary, human movements ignored
    ENGAGED = auto()          # Active teleoperation tracking
    LOCKED = auto()           # Position frozen at hold pose
    SAFE_RETRACTING = auto()  # Moving safely along 2-stage trajectory to ready pose
    SAFE_HOLD = auto()        # Frozen at current position due to limit or singularity
    ESTOP = auto()            # Emergency stop triggered


class TeleopMode(Enum):
    """Teleoperation operational mode."""
    STAGE1_FIXED_PITCH = auto()  # Phase 1: Position 3D + Fixed downward pitch R0
    STAGE2_IMU_FUSED = auto()    # Phase 2: Full 6-DOF wrist pose via IMU fusion


@dataclass
class ArUcoCalibrationData:
    """Desktop ArUco marker calibration transformation."""
    t_cam_to_table: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )
    t_table_to_cam: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )
    tag_id: int = 0
    tag_size_m: float = 0.10
    num_samples: int = 0
    is_valid: bool = False
    reprojection_error: float = 0.0

    def transform_point_to_table(self, point_cam: np.ndarray) -> np.ndarray:
        """Transform a 3D point from camera frame to table frame."""
        p_hom = np.array([point_cam[0], point_cam[1], point_cam[2], 1.0], dtype=np.float64)
        p_table = self.t_table_to_cam @ p_hom
        return p_table[:3]


@dataclass
class TeleopArmCommand:
    """6-DOF Robot Arm target command."""
    timestamp: float
    target_pose: np.ndarray       # 4x4 SE(3) target matrix in robot base frame
    target_q: np.ndarray          # (6,) target joint positions in radians
    target_dq: Optional[np.ndarray] = None  # (6,) target joint velocities in rad/s
    clutch_state: ClutchState = ClutchState.DISENGAGED
    kp_override: Optional[np.ndarray] = None  # Optional MIT Kp impedance
    kd_override: Optional[np.ndarray] = None  # Optional MIT Kd damping

    def __post_init__(self):
        if self.target_pose.shape != (4, 4):
            raise ValueError(f"target_pose must be (4, 4), got {self.target_pose.shape}")
        if self.target_q.shape != (ARM_NUM_DOFS,):
            raise ValueError(f"target_q must be ({ARM_NUM_DOFS},), got {self.target_q.shape}")


@dataclass
class TeleopHandCommand:
    """16-DOF Dexterous Hand target command."""
    timestamp: float
    target_q: np.ndarray          # (16,) target joint positions in radians (0.0 = OPEN_POSE)
    confidence: float = 1.0       # Vision tracking confidence [0.0, 1.0]
    is_clutched: bool = True      # False if hand tracking is paused/frozen

    def __post_init__(self):
        if self.target_q.shape != (HAND_NUM_DOFS,):
            raise ValueError(f"target_q must be ({HAND_NUM_DOFS},), got {self.target_q.shape}")


@dataclass
class TeleopCommand:
    """Unified 22-DOF composite command (Arm 6-DOF + Hand 16-DOF)."""
    timestamp: float
    arm_command: TeleopArmCommand
    hand_command: TeleopHandCommand
    clutch_state: ClutchState = ClutchState.DISENGAGED

    @property
    def composite_q(self) -> np.ndarray:
        """Concatenated 22-DOF target joint positions."""
        return np.concatenate([
            self.arm_command.target_q,
            self.hand_command.target_q
        ])


@dataclass
class RobotObservation:
    """Actual real-time feedback state from physical or simulated hardware."""
    timestamp: float
    arm_actual_q: np.ndarray       # (6,) actual arm joint positions (rad)
    arm_actual_dq: np.ndarray      # (6,) actual arm joint velocities (rad/s)
    arm_effort: np.ndarray         # (6,) actual motor torques (N*m)
    hand_actual_q: np.ndarray      # (16,) actual hand joint positions (rad)
    hand_current: Optional[np.ndarray] = None  # (16,) motor currents (mA)
    is_connected: bool = True

    def __post_init__(self):
        if self.arm_actual_q.shape != (ARM_NUM_DOFS,):
            raise ValueError(f"arm_actual_q must be ({ARM_NUM_DOFS},), got {self.arm_actual_q.shape}")
        if self.hand_actual_q.shape != (HAND_NUM_DOFS,):
            raise ValueError(f"hand_actual_q must be ({HAND_NUM_DOFS},), got {self.hand_actual_q.shape}")

    @property
    def composite_state(self) -> np.ndarray:
        """Concatenated 22-DOF actual joint positions."""
        return np.concatenate([self.arm_actual_q, self.hand_actual_q])

