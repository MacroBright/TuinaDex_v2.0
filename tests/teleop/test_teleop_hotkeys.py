"""W9: HUD 快捷键状态机与 I40 安全门禁测试 (零 ROS / 零 pinocchio 依赖)."""
from enum import Enum, auto

import pytest

from tuinadex_teleop.core.types import ClutchState
from tuinadex_teleop.perception.imu_stream import (
    ALL_IMU_AXIS_MAPPINGS,
    AXIS_MAPPING_CYCLE,
    HotkeyState,
    WristImuTracker,
    cycle_axis_mapping,
    cycle_imu_axis_mapping,
    cycle_orientation_mode,
    hotkey_allowed,
    swap_axis_mapping,
)


class _OrientationMode(Enum):
    """Stand-in for ArmOrientationMode (same member names, no pinocchio)."""

    FIXED_DOWNWARD = auto()
    TRACK_IMU = auto()
    TRACK_WRIST = auto()


def test_axis_mapping_cycles_through_all_presets():
    visited = []
    current = AXIS_MAPPING_CYCLE[0]
    for _ in range(len(AXIS_MAPPING_CYCLE)):
        visited.append(current)
        current = cycle_axis_mapping(current)
    assert visited == list(AXIS_MAPPING_CYCLE)
    assert current == AXIS_MAPPING_CYCLE[0], "axis cycle must wrap back to the first preset"


def test_axis_cycle_recovers_from_unknown_mapping():
    assert cycle_axis_mapping("custom_matrix") == AXIS_MAPPING_CYCLE[0]


def test_swap_axis_mapping_is_involutive():
    original = "+x,+y,+z"
    swapped = swap_axis_mapping(original)
    assert swapped == "+y,+x,+z"
    assert swap_axis_mapping(swapped) == original


def test_swap_axis_mapping_rejects_bad_mapping():
    with pytest.raises(ValueError, match="3 tokens"):
        swap_axis_mapping("+x,+y")


def test_tracker_swap_xy_axes_updates_alignment_and_roundtrips():
    tracker = WristImuTracker(axis_mapping="+x,+y,+z")
    first = tracker.swap_xy_axes()
    assert first == "+y,+x,+z"
    assert tracker.axis_mapping_str == first
    assert tracker.swap_xy_axes() == "+x,+y,+z"


def test_orientation_mode_cycles_and_wraps():
    assert (
        cycle_orientation_mode(_OrientationMode.FIXED_DOWNWARD)
        is _OrientationMode.TRACK_IMU
    )
    assert (
        cycle_orientation_mode(_OrientationMode.TRACK_IMU)
        is _OrientationMode.TRACK_WRIST
    )
    assert (
        cycle_orientation_mode(_OrientationMode.TRACK_WRIST)
        is _OrientationMode.FIXED_DOWNWARD
    )


def test_hotkey_guard_blocks_engaged_fast_motion():
    allowed, reason = hotkey_allowed(ClutchState.ENGAGED, 0.5)
    assert allowed is False
    assert "I40" in reason
    assert hotkey_allowed(ClutchState.ENGAGED, 0.01)[0] is True
    assert hotkey_allowed(ClutchState.DISENGAGED, 5.0)[0] is True


def test_hotkey_toast_expires_after_one_second():
    state = HotkeyState(
        axis_mapping="+x,+y,+z",
        orientation_mode=_OrientationMode.FIXED_DOWNWARD,
    )
    state.toast_now("PosAxis -> +y,+x,+z", now=10.0, seconds=1.0)
    assert state.active_toast(10.5) == "PosAxis -> +y,+x,+z"
    assert state.active_toast(11.1) == ""


def test_all_imu_axis_mappings_has_48_unique_valid_mappings():
    assert len(ALL_IMU_AXIS_MAPPINGS) == 48
    assert len(set(ALL_IMU_AXIS_MAPPINGS)) == 48


def test_cycle_imu_axis_mapping_forward_and_backward():
    first = ALL_IMU_AXIS_MAPPINGS[0]
    second = cycle_imu_axis_mapping(first, forward=True)
    assert second == ALL_IMU_AXIS_MAPPINGS[1]

    # Backward step
    back = cycle_imu_axis_mapping(second, forward=False)
    assert back == first

    # Wrap around backward from 0 to last
    last = cycle_imu_axis_mapping(first, forward=False)
    assert last == ALL_IMU_AXIS_MAPPINGS[-1]

    # Wrap around forward from last to 0
    wrap_first = cycle_imu_axis_mapping(last, forward=True)
    assert wrap_first == first


def test_wrist_imu_tracker_cycle_axis_mapping():
    tracker = WristImuTracker(axis_mapping="-y,+x,+z")
    mapping, idx, total = tracker.cycle_axis_mapping(forward=True)
    assert total == 48
    assert idx == 2
    assert mapping == ALL_IMU_AXIS_MAPPINGS[1]
    assert tracker.axis_mapping_str == mapping

    # Reverse cycle
    mapping_back, idx_back, _ = tracker.cycle_axis_mapping(forward=False)
    assert idx_back == 1
    assert mapping_back == ALL_IMU_AXIS_MAPPINGS[0]

