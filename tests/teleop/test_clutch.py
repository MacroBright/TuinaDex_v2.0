"""Unit tests for ClutchController state transitions and spatial mapping."""
import numpy as np
import pytest

from tuinadex_teleop.core.clutch import ClutchController
from tuinadex_teleop.core.types import ClutchState


def test_clutch_initial_state():
    clutch = ClutchController()
    assert clutch.state == ClutchState.DISENGAGED


def test_clutch_engage_and_disengage():
    clutch = ClutchController(scale=1.5)
    human_init = np.array([0.1, 0.2, 0.3])
    robot_init = np.array([0.0, 0.4, 0.2])

    clutch.engage(human_init, robot_init)
    assert clutch.state == ClutchState.ENGAGED

    # Move human by [+0.05, +0.10, -0.02]
    human_curr = human_init + np.array([0.05, 0.10, -0.02])
    target = clutch.compute_target(human_curr)

    expected = robot_init + 1.5 * np.array([0.05, 0.10, -0.02])
    np.testing.assert_allclose(target, expected)

    # Disengage: robot holds target
    clutch.disengage()
    assert clutch.state == ClutchState.DISENGAGED

    # Moving human while disengaged does not change target
    human_further = human_curr + np.array([1.0, 1.0, 1.0])
    target_held = clutch.compute_target(human_further)
    np.testing.assert_allclose(target_held, target)


def test_clutch_workspace_clamping():
    clutch = ClutchController(
        scale=1.0,
        workspace_x=(-0.2, 0.2),
        workspace_y=(0.1, 0.5),
        workspace_z=(0.0, 0.3),
    )
    clutch.engage(np.zeros(3), np.zeros(3))

    # Test exceeding upper X and lower Z bounds
    target = clutch.compute_target(np.array([1.0, 0.2, -0.5]))
    assert target[0] == 0.2   # clamped at x_max
    assert target[1] == 0.2   # within bounds
    assert target[2] == 0.0   # clamped at z_min


def test_clutch_estop_and_recovery():
    clutch = ClutchController()
    clutch.engage(np.zeros(3), np.zeros(3))
    assert clutch.state == ClutchState.ENGAGED

    clutch.estop()
    assert clutch.state == ClutchState.ESTOP

    # Cannot engage while in ESTOP
    clutch.engage(np.zeros(3), np.zeros(3))
    assert clutch.state == ClutchState.ESTOP

    # Reset with safe home pose
    safe_home = np.array([0.0, 0.3, 0.2])
    clutch.reset_estop(safe_home)
    assert clutch.state == ClutchState.DISENGAGED

    # Now can engage again
    clutch.engage(np.zeros(3), safe_home)
    assert clutch.state == ClutchState.ENGAGED


def test_clutch_axis_mapping():
    """Verify that axis mapping transforms delta vectors correctly."""
    clutch = ClutchController(scale=1.0, axis_mapping="-y,+x,+z")
    assert clutch.axis_mapping_str == "-y,+x,+z"

    human_init = np.array([0.0, 0.0, 0.0])
    robot_init = np.array([0.2, 0.0, 0.2])
    clutch.engage(human_init, robot_init)

    # Move human by [+0.10, +0.05, +0.02] in table frame
    # Mapping -y, +x, +z -> dx_rob = -dy_tab (-0.05), dy_rob = +dx_tab (+0.10), dz_rob = +dz_tab (+0.02)
    human_curr = np.array([0.10, 0.05, 0.02])
    target = clutch.compute_target(human_curr)

    expected = robot_init + np.array([-0.05, 0.10, 0.02])
    np.testing.assert_allclose(target, expected)


def test_clutch_inverted_xy_mapping():
    """Verify that '-x,-y,+z' inverts XY delta directions while preserving Z."""
    clutch = ClutchController(scale=1.0, axis_mapping="-x,-y,+z")
    assert clutch.axis_mapping_str == "-x,-y,+z"

    human_init = np.array([0.0, 0.0, 0.0])
    robot_init = np.array([0.3, 0.0, 0.2])
    clutch.engage(human_init, robot_init)

    # Human moves +X (+0.08), +Y (+0.05), +Z (+0.03)
    human_curr = np.array([0.08, 0.05, 0.03])
    target = clutch.compute_target(human_curr)

    # Robot moves -X (-0.08), -Y (-0.05), +Z (+0.03)
    expected = robot_init + np.array([-0.08, -0.05, +0.03])
    np.testing.assert_allclose(target, expected)


