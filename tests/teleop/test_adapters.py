"""Unit tests for MockSimAdapter loopback and observation contracts."""
import numpy as np
import pytest

from tuinadex_teleop.adapters.sim_adapter import MockSimAdapter
from tuinadex_teleop.core.types import TeleopArmCommand, TeleopHandCommand, TeleopCommand


def test_mock_sim_adapter_lifecycle():
    adapter = MockSimAdapter()
    assert not adapter.is_connected

    # Cannot send command before connect
    arm_cmd = TeleopArmCommand(timestamp=0.0, target_pose=np.eye(4), target_q=np.zeros(6))
    hand_cmd = TeleopHandCommand(timestamp=0.0, target_q=np.zeros(16))
    cmd = TeleopCommand(timestamp=0.0, arm_command=arm_cmd, hand_command=hand_cmd)

    with pytest.raises(RuntimeError, match="not connected"):
        adapter.send_command(cmd)

    # Connect
    assert adapter.connect() is True
    assert adapter.is_connected is True

    # Send command
    arm_cmd.target_q = np.ones(6) * 0.5
    hand_cmd.target_q = np.ones(16) * 0.8
    adapter.send_command(cmd)

    # Read observation
    obs = adapter.read_observation()
    assert obs.is_connected is True
    assert obs.arm_actual_q.shape == (6,)
    assert obs.hand_actual_q.shape == (16,)
    assert obs.composite_state.shape == (22,)

    # Values should follow target towards 0.5 and 0.8
    assert np.all(obs.arm_actual_q > 0.0)
    assert np.all(obs.hand_actual_q > 0.0)

    # Disconnect
    adapter.disconnect()
    assert not adapter.is_connected

