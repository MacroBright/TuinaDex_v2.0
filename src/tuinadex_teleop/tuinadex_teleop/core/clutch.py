"""Clutch state machine and spatial incremental mapping for teleoperation."""
from __future__ import annotations

from typing import Optional, Tuple, Union
import numpy as np

from .types import ClutchState


class ClutchController:
    """Manages foot pedal / keyboard clutch states and delta-pose mapping.
    
    Formula:
        P_target(t) = P_robot_anchor + lambda_scale * R_align * (P_human(t) - P_human_anchor)
    """

    def __init__(
        self,
        scale: float = 1.0,
        workspace_x: Tuple[float, float] = (-0.45, 0.45),
        workspace_y: Tuple[float, float] = (-0.45, 0.70),
        workspace_z: Tuple[float, float] = (0.02, 0.50),
        axis_mapping: Optional[Union[str, np.ndarray]] = None,
    ):
        self.scale = scale
        self.workspace_x = workspace_x
        self.workspace_y = workspace_y
        self.workspace_z = workspace_z

        self.R_align_table_to_robot = np.eye(3, dtype=np.float64)
        self.axis_mapping_str = "+x,+y,+z"
        if axis_mapping is not None:
            if isinstance(axis_mapping, str):
                self.set_axis_mapping(axis_mapping)
            else:
                self.set_alignment_matrix(np.asarray(axis_mapping, dtype=np.float64))

        self.state = ClutchState.DISENGAGED
        self._human_anchor: Optional[np.ndarray] = None
        self._robot_anchor: Optional[np.ndarray] = None
        self._last_robot_target: Optional[np.ndarray] = None

        self._human_rot_anchor: Optional[np.ndarray] = None
        self._robot_rot_anchor: Optional[np.ndarray] = None
        self._last_robot_rot_target: Optional[np.ndarray] = None

    def set_alignment_matrix(self, R: np.ndarray) -> None:
        """Set explicit 3x3 rotation matrix from Table/Human frame to Robot Base frame."""
        if R.shape != (3, 3):
            raise ValueError(f"Alignment matrix must be (3, 3), got {R.shape}")
        self.R_align_table_to_robot = np.array(R, dtype=np.float64, copy=True)
        self.axis_mapping_str = "custom_matrix"

    def set_axis_mapping(self, mapping_str: str) -> None:
        """Parse axis mapping string, e.g. '+x,+y,+z', '-y,+x,+z', '+y,+x,+z'."""
        tokens = [t.strip().lower() for t in mapping_str.split(",")]
        if len(tokens) != 3:
            raise ValueError(f"axis_mapping must have 3 tokens (e.g. '+x,+y,+z'), got '{mapping_str}'")

        R = np.zeros((3, 3), dtype=np.float64)
        axis_indices = {"x": 0, "y": 1, "z": 2}
        for row, token in enumerate(tokens):
            sign = -1.0 if token.startswith("-") else 1.0
            axis_char = token.lstrip("+-")
            if axis_char not in axis_indices:
                raise ValueError(f"Invalid axis character '{axis_char}' in '{token}'")
            col = axis_indices[axis_char]
            R[row, col] = sign

        self.R_align_table_to_robot = R
        self.axis_mapping_str = mapping_str.lower()

    def engage(
        self,
        current_human_pos: np.ndarray,
        current_robot_pos: np.ndarray,
        current_human_rot: Optional[np.ndarray] = None,
        current_robot_rot: Optional[np.ndarray] = None,
    ) -> None:
        """Engage clutch: latch current human and robot positions/rotations as anchors."""
        if self.state == ClutchState.ESTOP:
            return  # Must explicitly reset from ESTOP

        self._human_anchor = np.array(current_human_pos, dtype=np.float64, copy=True)
        self._robot_anchor = np.array(current_robot_pos, dtype=np.float64, copy=True)
        self._last_robot_target = np.array(current_robot_pos, dtype=np.float64, copy=True)

        if current_human_rot is not None:
            self._human_rot_anchor = np.array(current_human_rot, dtype=np.float64, copy=True)
        if current_robot_rot is not None:
            self._robot_rot_anchor = np.array(current_robot_rot, dtype=np.float64, copy=True)
            self._last_robot_rot_target = np.array(current_robot_rot, dtype=np.float64, copy=True)

        self.state = ClutchState.ENGAGED

    def disengage(self) -> None:
        """Disengage clutch: freeze robot at last commanded target position and orientation."""
        if self.state == ClutchState.ESTOP:
            return
        self.state = ClutchState.DISENGAGED
        self._human_anchor = None
        self._human_rot_anchor = None

    def set_safe_hold(self) -> None:
        """Trigger SAFE_HOLD state: freeze robot at current position due to limit/singularity."""
        if self.state == ClutchState.ESTOP:
            return
        self.state = ClutchState.SAFE_HOLD
        self._human_anchor = None
        self._human_rot_anchor = None

    def set_safe_retracting(self) -> None:
        """Trigger SAFE_RETRACTING state: disengage teleop and smoothly retreat to ready pose."""
        if self.state == ClutchState.ESTOP:
            return
        self.state = ClutchState.SAFE_RETRACTING
        self._human_anchor = None
        self._human_rot_anchor = None

    def toggle(
        self,
        current_human_pos: Optional[np.ndarray] = None,
        current_robot_pos: Optional[np.ndarray] = None,
        current_human_rot: Optional[np.ndarray] = None,
        current_robot_rot: Optional[np.ndarray] = None,
    ) -> ClutchState:
        """Toggle between ENGAGED and DISENGAGED (also clears SAFE_HOLD / SAFE_RETRACTING)."""
        if self.state == ClutchState.ENGAGED:
            self.disengage()
        elif self.state in (ClutchState.DISENGAGED, ClutchState.LOCKED, ClutchState.SAFE_HOLD, ClutchState.SAFE_RETRACTING):
            if current_human_pos is not None and current_robot_pos is not None:
                self.engage(current_human_pos, current_robot_pos, current_human_rot, current_robot_rot)
            else:
                self.state = ClutchState.ENGAGED
        return self.state

    def estop(self) -> None:
        """Trigger emergency stop."""
        self.state = ClutchState.ESTOP
        self._human_anchor = None
        self._human_rot_anchor = None

    def reset_estop(self, safe_robot_pos: np.ndarray, safe_robot_rot: Optional[np.ndarray] = None) -> None:
        """Recover from emergency stop to DISENGAGED state."""
        self.state = ClutchState.DISENGAGED
        self._human_anchor = None
        self._human_rot_anchor = None
        self._robot_anchor = np.array(safe_robot_pos, dtype=np.float64, copy=True)
        self._last_robot_target = np.array(safe_robot_pos, dtype=np.float64, copy=True)
        if safe_robot_rot is not None:
            self._robot_rot_anchor = np.array(safe_robot_rot, dtype=np.float64, copy=True)
            self._last_robot_rot_target = np.array(safe_robot_rot, dtype=np.float64, copy=True)

    def reset(self, safe_robot_pos: Optional[np.ndarray] = None, safe_robot_rot: Optional[np.ndarray] = None) -> None:
        """Reset clutch anchors without tripping ESTOP."""
        self.state = ClutchState.DISENGAGED
        self._human_anchor = None
        self._human_rot_anchor = None
        if safe_robot_pos is not None:
            self._robot_anchor = np.array(safe_robot_pos, dtype=np.float64, copy=True)
            self._last_robot_target = np.array(safe_robot_pos, dtype=np.float64, copy=True)
        if safe_robot_rot is not None:
            self._robot_rot_anchor = np.array(safe_robot_rot, dtype=np.float64, copy=True)
            self._last_robot_rot_target = np.array(safe_robot_rot, dtype=np.float64, copy=True)

    def compute_target(
        self,
        current_human_pos: np.ndarray,
        fallback_robot_pos: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compute target robot position given current human tracking position."""
        if self.state != ClutchState.ENGAGED or self._human_anchor is None or self._robot_anchor is None:
            # Hold current or fallback position
            if self._last_robot_target is not None:
                return self._last_robot_target
            if fallback_robot_pos is not None:
                return np.asarray(fallback_robot_pos, dtype=np.float64)
            return np.zeros(3, dtype=np.float64)

        # Delta mapping in table frame
        delta_human = np.asarray(current_human_pos, dtype=np.float64) - self._human_anchor
        # Transform delta from table frame to robot base frame
        delta_robot = self.R_align_table_to_robot @ delta_human
        target = self._robot_anchor + self.scale * delta_robot

        # Workspace safety clamping
        target[0] = np.clip(target[0], self.workspace_x[0], self.workspace_x[1])
        target[1] = np.clip(target[1], self.workspace_y[0], self.workspace_y[1])
        target[2] = np.clip(target[2], self.workspace_z[0], self.workspace_z[1])

        self._last_robot_target = target
        return target

    def compute_target_orientation(
        self,
        current_human_rot: np.ndarray,
        fallback_robot_rot: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compute target robot orientation given current human wrist orientation in table frame.
        
        Formula:
            Delta_R_human = R_human * R_human_anchor^T
            Delta_R_robot = R_align * Delta_R_human * R_align^T
            R_target = Delta_R_robot * R_robot_anchor
        """
        if self.state != ClutchState.ENGAGED or self._human_rot_anchor is None or self._robot_rot_anchor is None:
            if self._last_robot_rot_target is not None:
                return self._last_robot_rot_target
            if fallback_robot_rot is not None:
                return np.asarray(fallback_robot_rot, dtype=np.float64)
            return np.eye(3, dtype=np.float64)

        # Delta rotation in human frame
        delta_R_human = np.asarray(current_human_rot, dtype=np.float64) @ self._human_rot_anchor.T
        # Transform delta rotation to robot frame
        delta_R_robot = self.R_align_table_to_robot @ delta_R_human @ self.R_align_table_to_robot.T
        target_R = delta_R_robot @ self._robot_rot_anchor

        # Orthonormalize via SVD projection
        U, _, Vt = np.linalg.svd(target_R)
        target_R_ortho = U @ Vt
        if np.linalg.det(target_R_ortho) < 0:
            U[:, -1] *= -1.0
            target_R_ortho = U @ Vt

        self._last_robot_rot_target = target_R_ortho
        return target_R_ortho


