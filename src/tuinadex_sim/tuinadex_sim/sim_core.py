"""sim_core.py — deterministic simulation command/relax core (no rclpy, no MuJoCo import)."""
from __future__ import annotations

from typing import Optional, Sequence
import numpy as np

from tuinadex_core.command_processing import align_command
from tuinadex_core.watchdog import WatchdogAction, WatchdogStateMachine


class _RelaxTrajectory:
    """Hold then linearly ramp measured joint positions to OPEN (0 rad)."""

    def __init__(self, hold_sec: float, ramp_sec: float):
        self.hold_sec = hold_sec
        self.ramp_sec = ramp_sec
        self.reset()

    def reset(self) -> None:
        self._loss_t0: Optional[float] = None
        self._relax_from_pos: Optional[np.ndarray] = None

    def target(self, t_now: float, measured_positions: np.ndarray) -> np.ndarray:
        if self._loss_t0 is None:
            self._loss_t0 = t_now
            self._relax_from_pos = np.asarray(measured_positions, dtype=np.float64).copy()

        elapsed = t_now - self._loss_t0
        if elapsed < self.hold_sec:
            return self._relax_from_pos.copy()

        ramp_progress = min(
            1.0,
            (elapsed - self.hold_sec) / max(0.01, self.ramp_sec),
        )
        return self._relax_from_pos * (1.0 - ramp_progress)


class SimulationCore:
    """Drive a duck-typed MuJoCo bridge through the same command safety path as hardware."""

    def __init__(
        self,
        bridge,
        timeout_sec: float = 0.30,
        hold_sec: float = 0.30,
        ramp_sec: float = 0.60,
        physics_substeps: int = 10,
    ):
        self.bridge = bridge
        self.physics_substeps = int(physics_substeps)
        if self.physics_substeps <= 0:
            raise ValueError(f"physics_substeps must be > 0, got {physics_substeps}")
        self._watchdog = WatchdogStateMachine(timeout_sec=timeout_sec)
        self._relax = _RelaxTrajectory(hold_sec=hold_sec, ramp_sec=ramp_sec)
        self._target_command: Optional[np.ndarray] = None

    def apply_command(
        self,
        q_16: Sequence[float],
        t_now: float,
        names: Optional[Sequence[str]] = None,
    ) -> bool:
        q_target, reason = align_command(q_16, names)
        if q_target is None:
            return False

        self._target_command = q_target
        self._watchdog.on_command(t_now)
        self._relax.reset()
        return True

    def tick(self, t_now: float):
        action = self._watchdog.tick(t_now)
        if action is WatchdogAction.WRITE:
            target = self._target_command
        elif action is WatchdogAction.RELAX_STEP:
            target = self._relax.target(t_now, self.bridge.get_state().positions)
        else:
            target = self.bridge.get_state().positions

        self.bridge.set_joint_targets(target)
        self.bridge.step(self.physics_substeps)
        return self.bridge.get_state()
