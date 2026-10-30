"""Mock / simulation adapter for unit tests and offline closed-loop verification."""
from __future__ import annotations

import time
import numpy as np

try:
    from tuinadex_teleop.core.names import ARM_NUM_DOFS, HAND_NUM_DOFS
    from tuinadex_teleop.core.types import RobotObservation, TeleopCommand
except ImportError:
    from tuina_teleop.core.names import ARM_NUM_DOFS, HAND_NUM_DOFS
    from tuina_teleop.core.types import RobotObservation, TeleopCommand
from .base import BaseRobotAdapter


class MockSimAdapter(BaseRobotAdapter):
    """In-memory loopback adapter for tests without hardware or simulator."""

    def __init__(self):
        self._connected = False
        self._current_arm_q = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
        self._current_arm_dq = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
        self._current_arm_effort = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
        self._current_hand_q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._current_hand_current = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._last_cmd: TeleopCommand | None = None

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def send_command(self, cmd: TeleopCommand) -> None:
        if not self._connected:
            raise RuntimeError("Cannot send command: adapter is not connected.")
        self._last_cmd = cmd
        # Simple exponential follow model
        alpha = 0.5
        self._current_arm_q = (
            alpha * cmd.arm_command.target_q + (1.0 - alpha) * self._current_arm_q
        )
        self._current_hand_q = (
            alpha * cmd.hand_command.target_q + (1.0 - alpha) * self._current_hand_q
        )

    def read_observation(self) -> RobotObservation:
        return RobotObservation(
            timestamp=time.time(),
            arm_actual_q=self._current_arm_q.copy(),
            arm_actual_dq=self._current_arm_dq.copy(),
            arm_effort=self._current_arm_effort.copy(),
            hand_actual_q=self._current_hand_q.copy(),
            hand_current=self._current_hand_current.copy(),
            is_connected=self._connected,
        )

