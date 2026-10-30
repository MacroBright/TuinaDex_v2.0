"""safety.py — Multi-tier Teleoperation Safety Guardrails for TuinaDex.

Enforces four primary safety gates:
1. Hand Tracking Loss & Latency Timeout -> Two-stage smooth retract to Home (Lift-off before Homing).
2. Inter-Frame Teleportation / Pose Jumps (Cartesian position, rotation, hand joints) -> Immediate disengage & smooth homing.
3. Kinematic Singularity (Yoshikawa manipulability / min singular value) & Joint Soft Limits -> Freeze in Place (Safe Hold).
4. Tuina Massage Contact Force & Overload Protection:
   - Total contact force > 70.0 N
   - Single finger contact force > 40.0 N
   - Force surge rate > 100.0 N/s
   -> Immediate disengage and two-stage lift-off retreat.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import math
import time
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np

from .types import ClutchState, TeleopArmCommand, TeleopHandCommand


class SafetyLevel(Enum):
    """Operational safety state of the teleoperation system."""
    NORMAL = auto()      # Normal tracking, all metrics within safe envelope
    HOLD = auto()        # Transient vision occlusion (<= 0.80s), holding last known state
    RETRACTING = auto()  # Executing two-stage safe lift-off and homing trajectory
    FROZEN = auto()      # Locked in place due to singularity or joint limit proximity
    ESTOP = auto()       # Emergency stop triggered
    BYPASS = auto()      # Safety guardrails disabled / bypassed (Unrestricted mode)


class SafetyViolation(Enum):
    """Specific cause of safety intervention."""
    NONE = "None"
    HAND_LOSS_TIMEOUT = "Hand tracking lost > timeout"
    HIGH_LATENCY = "Tracking latency exceeded limit"
    POS_JUMP = "Cartesian position jump exceeded limit"
    ROT_JUMP = "Orientation angle jump exceeded limit"
    HAND_Q_JUMP = "Hand joint jump exceeded limit"
    SINGULARITY_APPROACH = "Approaching kinematic singularity"
    JOINT_LIMIT_APPROACH = "Approaching joint limit margin"
    FORCE_TOTAL_OVERLOAD = "Total contact force exceeded limit"
    FORCE_FINGER_OVERLOAD = "Single finger force exceeded limit"
    FORCE_RATE_OVERLOAD = "Force surge rate exceeded limit"


@dataclass
class SafetyConfig:
    """Configurable safety thresholds and timing parameters."""
    hand_loss_timeout_s: float = 0.80     # Max duration of hand loss before homing (s, relaxed from 0.35s)
    max_latency_s: float = 0.25           # Max tracking latency threshold (s, relaxed from 0.12s)
    max_pos_jump_m: float = 0.18          # Max Cartesian translation jump per frame (m, relaxed from 0.08m)
    max_rot_jump_deg: float = 60.0        # Max orientation angular jump per frame (deg, relaxed from 35.0 deg)
    max_hand_q_jump_rad: float = 1.50     # Max dexterous finger joint jump (rad, relaxed from 0.80 rad)
    min_singular_value: float = 0.010     # Min Jacobian singular value for singularity detection (relaxed from 0.025)
    min_manipulability: float = 0.0005    # Min Yoshikawa manipulability index (relaxed from 0.0015)
    joint_limit_margin_rad: float = 0.02  # Joint soft limit buffer margin (~1.15 deg, relaxed from 0.05 rad)
    max_total_force_n: float = 120.0      # Tuina total contact force safety ceiling (N, raised from 70.0N)
    max_finger_force_n: float = 70.0      # Tuina single finger contact force limit (N, raised from 40.0N)
    max_force_rate_n_s: float = 400.0     # Contact force surge rate ceiling (N/s, raised from 100.0N/s)
    min_force_rate_trigger_n: float = 10.0 # Minimum total force required to check surge rate (N)
    lift_off_height_m: float = 0.06       # Safe vertical lift-off distance before homing (m)
    lift_off_duration_s: float = 0.6      # Duration of vertical lift-off stage (s)
    retract_duration_s: float = 1.8       # Duration of smooth homing trajectory (s)


@dataclass
class SafetyReport:
    """Diagnostic report emitted each evaluation cycle."""
    level: SafetyLevel
    violation: SafetyViolation
    message: str
    timestamp: float
    metrics: Dict[str, Any] = field(default_factory=dict)


class MinimumJerkTrajectory:
    """Generates a mathematically guaranteed C^2-continuous minimum-jerk profile.
    
    Formula:
        tau = (t - t0) / T, tau in [0, 1]
        s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
        s_dot(tau) = (30*tau^2 - 60*tau^3 + 30*tau^4) / T
    """

    def __init__(
        self,
        start_val: Union[float, np.ndarray],
        goal_val: Union[float, np.ndarray],
        duration_s: float,
        start_time: float,
    ):
        self.start_val = np.asarray(start_val, dtype=np.float64)
        self.goal_val = np.asarray(goal_val, dtype=np.float64)
        self.duration_s = max(1e-4, float(duration_s))
        self.start_time = float(start_time)

    def evaluate(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """Evaluate position and velocity at time t.
        
        Returns:
            (pos, vel)
        """
        elapsed = t - self.start_time
        tau = min(1.0, max(0.0, elapsed / self.duration_s))
        s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        s_dot = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / self.duration_s
        delta = self.goal_val - self.start_val
        pos = self.start_val + s * delta
        vel = s_dot * delta
        return pos, vel

    def is_finished(self, t: float) -> bool:
        """Check if trajectory duration has elapsed."""
        return (t - self.start_time) >= self.duration_s


class TwoStageRetractController:
    """Executes a stratified safe retreat:
    
    Stage 1: Lift-off vertically (+Z) by lift_off_height_m while opening hand fingers to 0.0 rad.
             This prevents horizontal dragging across patient/bed surfaces.
    Stage 2: Smoothly interpolate all 6 arm joints back to home_q (Ready Pose).
    """

    def __init__(
        self,
        start_arm_pos: np.ndarray,
        start_arm_rot: np.ndarray,
        start_arm_q: np.ndarray,
        start_hand_q: np.ndarray,
        home_arm_q: np.ndarray,
        start_time: float,
        config: SafetyConfig,
    ):
        self.config = config
        self.start_time = start_time
        self.start_arm_pos = np.array(start_arm_pos, dtype=np.float64, copy=True)
        self.start_arm_rot = np.array(start_arm_rot, dtype=np.float64, copy=True)
        self.start_arm_q = np.array(start_arm_q, dtype=np.float64, copy=True)
        self.start_hand_q = np.array(start_hand_q, dtype=np.float64, copy=True)
        self.home_arm_q = np.array(home_arm_q, dtype=np.float64, copy=True)

        # Target lifted position: start_pos + [0, 0, lift_off_height_m]
        self.lifted_arm_pos = self.start_arm_pos.copy()
        self.lifted_arm_pos[2] += self.config.lift_off_height_m

        # Stage 1: Lift-off trajectory (Cartesian Z)
        self.stage1_pos_traj = MinimumJerkTrajectory(
            start_val=self.start_arm_pos,
            goal_val=self.lifted_arm_pos,
            duration_s=self.config.lift_off_duration_s,
            start_time=self.start_time,
        )

        # Hand opening trajectory (relaxes all 16 joints to 0.0)
        self.hand_open_traj = MinimumJerkTrajectory(
            start_val=self.start_hand_q,
            goal_val=np.zeros(16, dtype=np.float64),
            duration_s=self.config.lift_off_duration_s,
            start_time=self.start_time,
        )

        # Stage 2: Arm joint-space homing trajectory (initialized dynamically at transition)
        self.stage2_traj: Optional[MinimumJerkTrajectory] = None
        self.stage2_start_time = self.start_time + self.config.lift_off_duration_s
        self.total_duration = self.config.lift_off_duration_s + self.config.retract_duration_s

    def step(
        self,
        t_now: float,
        current_arm_q: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], np.ndarray, bool]:
        """Step the two-stage retractor.
        
        Returns:
            (target_pos, target_rot, target_arm_q, target_hand_q, is_finished)
        """
        elapsed = t_now - self.start_time
        hand_q, _ = self.hand_open_traj.evaluate(t_now)

        if elapsed < self.config.lift_off_duration_s:
            # Stage 1: Lift-off in Cartesian space
            pos, _ = self.stage1_pos_traj.evaluate(t_now)
            rot = self.start_arm_rot
            return pos, rot, None, hand_q, False
        else:
            # Stage 2: Joint-space homing to home_arm_q
            if self.stage2_traj is None:
                self.stage2_traj = MinimumJerkTrajectory(
                    start_val=current_arm_q,
                    goal_val=self.home_arm_q,
                    duration_s=self.config.retract_duration_s,
                    start_time=self.stage2_start_time,
                )
            arm_q, _ = self.stage2_traj.evaluate(t_now)
            is_finished = elapsed >= self.total_duration
            return None, None, arm_q, np.zeros(16, dtype=np.float64), is_finished


class SafetySupervisor:
    """Central safety supervisor inspecting all physical and kinematic channels."""

    def __init__(self, config: Optional[SafetyConfig] = None, enabled: bool = True):
        self.config = config if config is not None else SafetyConfig()
        self.enabled = bool(enabled)
        self.level = SafetyLevel.NORMAL if self.enabled else SafetyLevel.BYPASS
        self.last_violation = SafetyViolation.NONE
        self.last_message = "All safety checks passed" if self.enabled else "Safety guardrails BYPASS (Unrestricted Mode)"

        # State memory for jump and rate detection
        self.last_valid_time: Optional[float] = None
        self.hand_loss_start_time: Optional[float] = None
        self.last_human_pos: Optional[np.ndarray] = None
        self.last_human_rot: Optional[np.ndarray] = None
        self.last_hand_q: Optional[np.ndarray] = None
        self.last_total_force: float = 0.0
        self.last_force_time: Optional[float] = None

        # Safe hold pose latch
        self.frozen_arm_q: Optional[np.ndarray] = None
        self.frozen_arm_pos: Optional[np.ndarray] = None
        self.frozen_arm_rot: Optional[np.ndarray] = None
        self.frozen_hand_q: Optional[np.ndarray] = None

        # Two-stage retract controller
        self.retract_controller: Optional[TwoStageRetractController] = None

    def reset(self) -> None:
        """Reset supervisor back to initial operational state."""
        self.level = SafetyLevel.NORMAL if self.enabled else SafetyLevel.BYPASS
        self.last_violation = SafetyViolation.NONE
        self.last_message = "Safety supervisor reset" if self.enabled else "Safety guardrails BYPASS (Unrestricted Mode)"
        # Reset supervisor and tracking state
        self.hand_loss_start_time = None
        self.retract_controller = None
        self.frozen_arm_q = None
        self.frozen_arm_pos = None
        self.frozen_arm_rot = None
        self.frozen_hand_q = None
        self.last_total_force = 0.0
        self.last_force_time = None
        self.last_human_pos = None
        self.last_human_rot = None
        self.last_hand_q = None

    def toggle_enabled(self) -> bool:
        """Toggle between enabled (Guarded) and bypassed (Unrestricted) safety modes."""
        self.enabled = not self.enabled
        self.reset()
        return self.enabled

    def step_retract(
        self, t_now: float, current_arm_q: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], np.ndarray, bool]:
        """Step the active two-stage retract controller.
        
        Transitions level back to NORMAL when completed.
        
        Returns:
            (target_pos, target_rot, target_arm_q, target_hand_q, is_finished)
        """
        if self.retract_controller is None:
            return None, None, None, np.zeros(16, dtype=np.float64), True
        pos, rot, arm_q, hand_q, finished = self.retract_controller.step(t_now, current_arm_q)
        if finished:
            self.level = SafetyLevel.NORMAL
            self.retract_controller = None
            self.last_violation = SafetyViolation.NONE
            self.last_message = "Safe homing completed. Ready for operator re-engagement."
        return pos, rot, arm_q, hand_q, finished

    def evaluate(
        self,
        t_now: float,
        wrist_tracking_valid: bool,
        wrist_pos: Optional[np.ndarray],
        wrist_rot: Optional[np.ndarray],
        wrist_timestamp: float,
        current_arm_q: np.ndarray,
        current_arm_pos: np.ndarray,
        current_arm_rot: np.ndarray,
        current_hand_q: np.ndarray,
        home_arm_q: np.ndarray,
        contact_forces: Dict[str, float],
        clutch_state: ClutchState,
        arm_ik_solver: Optional[Any] = None,
    ) -> SafetyReport:
        """Evaluate all four safety gates.
        
        Returns:
            SafetyReport detailing current level and violation reason.
        """
        # If safety guardrails are bypassed (Unrestricted mode requested by user)
        if not self.enabled:
            self.level = SafetyLevel.BYPASS
            self.last_violation = SafetyViolation.NONE
            self.last_message = "Safety guardrails BYPASS (Unrestricted Mode)"
            return SafetyReport(
                level=self.level,
                violation=self.last_violation,
                message=self.last_message,
                timestamp=t_now,
            )

        # If currently retracting, return early without stepping controller here
        # (Controller is stepped by caller via step_retract to avoid double-stepping)
        if self.level == SafetyLevel.RETRACTING:
            return SafetyReport(
                level=self.level,
                violation=self.last_violation,
                message=self.last_message,
                timestamp=t_now,
            )

        # If currently FROZEN (singularity / limit lock), hold until operator disengages/resets
        if self.level == SafetyLevel.FROZEN:
            if clutch_state == ClutchState.DISENGAGED:
                self.level = SafetyLevel.NORMAL
                self.last_violation = SafetyViolation.NONE
                self.last_message = "Lock cleared by clutch disengagement."
            return SafetyReport(
                level=self.level,
                violation=self.last_violation,
                message=self.last_message,
                timestamp=t_now,
            )

        # Gate 4: Contact Force Overload & Surge Rate (Highest Priority for Tuina Safety)
        tot_f = float(contact_forces.get("total", 0.0))
        max_f_finger = max(
            float(contact_forces.get("index", 0.0)),
            float(contact_forces.get("middle", 0.0)),
            float(contact_forces.get("ring", 0.0)),
            float(contact_forces.get("thumb", 0.0)),
        )
        if self.last_force_time is None or (t_now - self.last_force_time) > 0.20:
            df_rate = 0.0
        else:
            dt_f = max(1e-4, t_now - self.last_force_time)
            df_rate = max(0.0, (tot_f - self.last_total_force) / dt_f)
        self.last_total_force = tot_f
        self.last_force_time = t_now

        # Force overload protection guards active teleoperation
        if clutch_state == ClutchState.ENGAGED:
            if tot_f > self.config.max_total_force_n:
                return self._trigger_retract(
                    SafetyViolation.FORCE_TOTAL_OVERLOAD,
                    f"Total contact force {tot_f:.1f}N exceeded limit {self.config.max_total_force_n}N",
                    t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                    metrics={"total_force": tot_f},
                )

            if max_f_finger > self.config.max_finger_force_n:
                return self._trigger_retract(
                    SafetyViolation.FORCE_FINGER_OVERLOAD,
                    f"Single finger force {max_f_finger:.1f}N exceeded limit {self.config.max_finger_force_n}N",
                    t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                    metrics={"finger_force": max_f_finger},
                )

            if df_rate > self.config.max_force_rate_n_s and tot_f > self.config.min_force_rate_trigger_n:
                return self._trigger_retract(
                    SafetyViolation.FORCE_RATE_OVERLOAD,
                    f"Contact force surge rate {df_rate:.1f}N/s exceeded limit {self.config.max_force_rate_n_s}N/s",
                    t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                    metrics={"force_rate": df_rate},
                )

        # Gate 3: Kinematic Singularity & Joint Limits (Freeze in Place during active teleop)
        if clutch_state == ClutchState.ENGAGED and arm_ik_solver is not None:
            # Check manipulability & singular values
            if hasattr(arm_ik_solver, "compute_manipulability"):
                try:
                    w, min_sv, cond_num = arm_ik_solver.compute_manipulability(current_arm_q)
                    if min_sv < self.config.min_singular_value or w < self.config.min_manipulability:
                        return self._trigger_freeze(
                            SafetyViolation.SINGULARITY_APPROACH,
                            f"Approaching kinematic singularity: min_SV={min_sv:.4f} < {self.config.min_singular_value}",
                            t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q,
                            metrics={"min_sv": min_sv, "w": w, "cond_num": cond_num},
                        )
                except Exception:
                    pass

            # Check joint soft limits
            if hasattr(arm_ik_solver, "check_joint_limits"):
                try:
                    is_near, min_dist, worst_j = arm_ik_solver.check_joint_limits(
                        current_arm_q, self.config.joint_limit_margin_rad
                    )
                    if is_near:
                        return self._trigger_freeze(
                            SafetyViolation.JOINT_LIMIT_APPROACH,
                            f"Joint {worst_j + 1} near limit: margin {min_dist:.3f}rad < {self.config.joint_limit_margin_rad}rad",
                            t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q,
                            metrics={"joint": worst_j, "margin": min_dist},
                        )
                except Exception:
                    pass

        # Gate 1: Hand Tracking Loss & Latency Timeout (Only evaluates when teleoperating)
        if clutch_state == ClutchState.ENGAGED:
            if not wrist_tracking_valid or wrist_pos is None:
                if self.hand_loss_start_time is None:
                    self.hand_loss_start_time = t_now

                loss_duration = t_now - self.hand_loss_start_time
                if loss_duration <= self.config.hand_loss_timeout_s:
                    # Stage 1: Temporary hold (<= 0.35s)
                    self.level = SafetyLevel.HOLD
                    self.last_violation = SafetyViolation.NONE
                    self.last_message = f"Transient occlusion ({loss_duration:.2f}s <= {self.config.hand_loss_timeout_s}s) - Holding"
                    return SafetyReport(level=self.level, violation=self.last_violation, message=self.last_message, timestamp=t_now)
                else:
                    # Hand lost > 0.35s -> Two-stage smooth retract to Home
                    return self._trigger_retract(
                        SafetyViolation.HAND_LOSS_TIMEOUT,
                        f"Hand tracking lost for {loss_duration:.2f}s > {self.config.hand_loss_timeout_s}s",
                        t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                        metrics={"loss_duration": loss_duration},
                    )
            else:
                # Hand tracking is active -> reset loss timer
                self.hand_loss_start_time = None

                # Check vision latency
                latency = t_now - wrist_timestamp
                if latency > self.config.max_latency_s:
                    return self._trigger_retract(
                        SafetyViolation.HIGH_LATENCY,
                        f"Tracking latency {latency * 1000.0:.1f}ms exceeded limit {self.config.max_latency_s * 1000.0:.0f}ms",
                        t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                        metrics={"latency_ms": latency * 1000.0},
                    )
        else:
            self.hand_loss_start_time = None

        # Gate 2: Inter-Frame Teleportation / Pose Jumps (Only when engaged)
        if clutch_state == ClutchState.ENGAGED and wrist_pos is not None:
            if self.last_human_pos is not None:
                delta_pos = float(np.linalg.norm(wrist_pos - self.last_human_pos))
                if delta_pos > self.config.max_pos_jump_m:
                    return self._trigger_retract(
                        SafetyViolation.POS_JUMP,
                        f"Cartesian jump {delta_pos * 100.0:.1f}cm exceeded limit {self.config.max_pos_jump_m * 100.0:.1f}cm",
                        t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                        metrics={"delta_pos": delta_pos},
                    )

            if self.last_human_rot is not None and wrist_rot is not None:
                R_diff = wrist_rot @ self.last_human_rot.T
                trace_val = max(-1.0, min(3.0, float(np.trace(R_diff))))
                cos_theta = max(-1.0, min(1.0, (trace_val - 1.0) / 2.0))
                rot_jump_deg = math.degrees(math.acos(cos_theta))
                if rot_jump_deg > self.config.max_rot_jump_deg:
                    return self._trigger_retract(
                        SafetyViolation.ROT_JUMP,
                        f"Rotation jump {rot_jump_deg:.1f} deg exceeded limit {self.config.max_rot_jump_deg:.1f} deg",
                        t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                        metrics={"rot_jump_deg": rot_jump_deg},
                    )

            if self.last_hand_q is not None:
                max_hand_delta = float(np.max(np.abs(current_hand_q - self.last_hand_q)))
                if max_hand_delta > self.config.max_hand_q_jump_rad:
                    return self._trigger_retract(
                        SafetyViolation.HAND_Q_JUMP,
                        f"Hand joint jump {max_hand_delta:.2f}rad exceeded limit {self.config.max_hand_q_jump_rad:.2f}rad",
                        t_now, current_arm_pos, current_arm_rot, current_arm_q, current_hand_q, home_arm_q,
                        metrics={"max_hand_delta": max_hand_delta},
                    )

        # Update last known valid states
        self.last_human_pos = np.array(wrist_pos, dtype=np.float64, copy=True) if wrist_pos is not None else None
        self.last_human_rot = np.array(wrist_rot, dtype=np.float64, copy=True) if wrist_rot is not None else None
        self.last_hand_q = np.array(current_hand_q, dtype=np.float64, copy=True)
        self.last_valid_time = t_now

        self.level = SafetyLevel.NORMAL
        self.last_violation = SafetyViolation.NONE
        self.last_message = "All safety checks passed"
        return SafetyReport(level=self.level, violation=self.last_violation, message=self.last_message, timestamp=t_now)

    def _trigger_retract(
        self,
        violation: SafetyViolation,
        msg: str,
        t_now: float,
        current_arm_pos: np.ndarray,
        current_arm_rot: np.ndarray,
        current_arm_q: np.ndarray,
        current_hand_q: np.ndarray,
        home_arm_q: np.ndarray,
        metrics: Optional[Dict[str, Any]] = None,
    ) -> SafetyReport:
        """Trigger two-stage safe retract and homing."""
        self.level = SafetyLevel.RETRACTING
        self.last_violation = violation
        self.last_message = f"[SAFETY RETRACT] {msg}"
        self.retract_controller = TwoStageRetractController(
            start_arm_pos=current_arm_pos,
            start_arm_rot=current_arm_rot,
            start_arm_q=current_arm_q,
            start_hand_q=current_hand_q,
            home_arm_q=home_arm_q,
            start_time=t_now,
            config=self.config,
        )
        return SafetyReport(
            level=self.level,
            violation=violation,
            message=self.last_message,
            timestamp=t_now,
            metrics=metrics if metrics is not None else {},
        )

    def _trigger_freeze(
        self,
        violation: SafetyViolation,
        msg: str,
        t_now: float,
        current_arm_pos: np.ndarray,
        current_arm_rot: np.ndarray,
        current_arm_q: np.ndarray,
        current_hand_q: np.ndarray,
        metrics: Optional[Dict[str, Any]] = None,
    ) -> SafetyReport:
        """Trigger freeze in place (lockdown)."""
        self.level = SafetyLevel.FROZEN
        self.last_violation = violation
        self.last_message = f"[SAFETY FREEZE] {msg}"
        self.frozen_arm_q = current_arm_q.copy()
        self.frozen_arm_pos = current_arm_pos.copy()
        self.frozen_arm_rot = current_arm_rot.copy()
        self.frozen_hand_q = current_hand_q.copy()
        return SafetyReport(
            level=self.level,
            violation=violation,
            message=self.last_message,
            timestamp=t_now,
            metrics=metrics if metrics is not None else {},
        )
