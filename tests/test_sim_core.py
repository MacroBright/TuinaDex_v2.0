"""test_sim_core.py — simulation core command, watchdog, and substep logic."""
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for package in ("tuinadex_core", "tuinadex_sim"):
    path = str(ROOT / "src" / package)
    if path not in sys.path:
        sys.path.insert(0, path)

from tuinadex_core.command_processing import JOINT_SPEC_LIMITS
from tuinadex_sim.sim_bridge import SimState
from tuinadex_sim.sim_core import SimulationCore


class FakeBridge:
    def __init__(self):
        self._positions = np.zeros(16, dtype=np.float64)
        self._target = np.zeros(16, dtype=np.float64)
        self.targets = []
        self.steps = 0
        self.time = 0.0

    def set_joint_targets(self, targets):
        self._target = np.asarray(targets, dtype=np.float64).copy()
        self.targets.append(self._target.copy())

    def step(self, num_steps):
        self.steps += num_steps
        self.time += num_steps * 0.002
        self._positions = self._target.copy()

    def get_state(self):
        return SimState(
            time=self.time,
            positions=self._positions.copy(),
            velocities=np.zeros(16, dtype=np.float64),
            efforts=np.zeros(16, dtype=np.float64),
        )


def in_limit_command(flexion=0.5):
    return [0.2 if index in (0, 4, 8) else flexion for index in range(16)]


def test_valid_command_is_follwed_in_ten_substeps():
    bridge = FakeBridge()
    core = SimulationCore(bridge)
    command = in_limit_command()

    assert core.apply_command(command, t_now=1.0) is True
    state = core.tick(t_now=1.0)

    assert bridge.steps == 10
    np.testing.assert_allclose(bridge.targets[-1], command)
    np.testing.assert_allclose(state.positions, command)


def test_out_of_spec_command_is_clamped_not_rejected():
    bridge = FakeBridge()
    core = SimulationCore(bridge)

    assert core.apply_command([2.5] * 16, t_now=1.0) is True
    core.tick(t_now=1.0)

    np.testing.assert_allclose(bridge.targets[-1], JOINT_SPEC_LIMITS[:, 1])


@pytest.mark.parametrize("invalid_command,names", [
    ([0.1] * 15, None),
    ([float("nan")] * 16, None),
    ([0.1] * 16, ["not_a_joint"] * 16),
])
def test_invalid_command_rejected_without_stepping(invalid_command, names):
    bridge = FakeBridge()
    bridge._positions[:] = 0.2
    core = SimulationCore(bridge)
    accepted = core.apply_command(invalid_command, t_now=1.0, names=names)

    assert accepted is False
    assert bridge.steps == 0
    np.testing.assert_allclose(bridge.get_state().positions, 0.2)


def test_commands_within_timeout_keep_following():
    bridge = FakeBridge()
    core = SimulationCore(bridge)
    command = in_limit_command()

    assert core.apply_command(command, t_now=1.0) is True
    for t_now in (1.0, 1.20, 1.30):
        state = core.tick(t_now=t_now)
        np.testing.assert_allclose(state.positions, command)

    assert bridge.steps == 30


def test_timeout_uses_hold_then_smooth_ramp_to_open():
    bridge = FakeBridge()
    core = SimulationCore(bridge)
    command = in_limit_command()
    core.apply_command(command, t_now=1.0)
    core.tick(t_now=1.0)

    hold_state = core.tick(t_now=1.31)     # RELAX begins; from-position held
    np.testing.assert_allclose(hold_state.positions, command)

    core.tick(t_now=1.60)                  # hold interval still active
    np.testing.assert_allclose(
        bridge.targets[-1], command
    )

    core.tick(t_now=1.71)                  # ramp progress: 0.1 / 0.6
    np.testing.assert_allclose(
        bridge.targets[-1], np.asarray(command) * (1.0 - 0.1 / 0.6)
    )

    core.tick(t_now=2.22)                  # ramp complete
    np.testing.assert_allclose(bridge.targets[-1], 0.0, atol=1e-12)


def test_new_command_resets_relax_trajectory():
    bridge = FakeBridge()
    core = SimulationCore(bridge)
    command = in_limit_command()
    core.apply_command(command, t_now=1.0)
    core.tick(t_now=1.0)
    core.tick(t_now=1.31)                  # enter RELAXING

    new_command = in_limit_command(flexion=0.3)
    assert core.apply_command(new_command, t_now=2.0) is True
    state = core.tick(t_now=2.0)

    np.testing.assert_allclose(state.positions, new_command)


def test_physics_substeps_are_injected_without_hard_coding():
    bridge = FakeBridge()
    core = SimulationCore(bridge, physics_substeps=5)
    core.apply_command(in_limit_command(), t_now=1.0)
    core.tick(t_now=1.0)

    assert bridge.steps == 5
