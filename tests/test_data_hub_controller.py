"""Hub controller tests for command queue, stale frames, and async saves."""
import time

import numpy as np

from tuinadex_teleop.collection.collector import CollectionState, TuinaCollector
from tuinadex_teleop.collection.data_channel import MockDataChannel
from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer
from tuinadex_teleop.collection.data_hub_controller import (
    DataHubController,
    HubAction,
)


def _make(fps=50.0, **kwargs):
    channel = MockDataChannel(shape=(48, 64, 3), dof=22)
    collector = TuinaCollector(env=None)
    ctrl = DataHubController(channel, collector, control_fps=fps, **kwargs)
    return channel, collector, ctrl


def test_commands_apply_on_tick_boundary():
    channel, collector, ctrl = _make()
    try:
        assert ctrl.status()["state"] == "IDLE"
        assert ctrl.submit(HubAction.START.value)["ok"] is True
        assert ctrl.status()["state"] == "IDLE"
        ctrl.tick()
        assert ctrl.status()["state"] == "RECORDING"
        for _ in range(5):
            ctrl.tick()
        assert ctrl.status()["frames_recorded"] == 5

        ctrl.submit(HubAction.STOP.value)
        ctrl.tick()
        assert ctrl.status()["state"] == "REVIEWING"
        ctrl.submit(HubAction.DISCARD.value)
        ctrl.tick()
        assert ctrl.status()["state"] == "IDLE"
        assert ctrl.status()["discarded"] == 1
    finally:
        ctrl.shutdown()


def test_async_save_does_not_block_tick():
    channel, collector, ctrl = _make()
    saved = []
    ctrl.on_save = lambda episode: (
        time.sleep(0.2),
        saved.append(len(episode)),
    )
    try:
        ctrl.submit(HubAction.START.value)
        ctrl.tick()
        for _ in range(3):
            ctrl.tick()
        ctrl.submit(HubAction.STOP.value)
        ctrl.tick()
        t0 = time.monotonic()
        ctrl.submit(HubAction.SAVE.value)
        ctrl.tick()
        assert time.monotonic() - t0 < 0.1
        assert ctrl.status()["state"] == "SAVING"
        deadline = time.monotonic() + 3.0
        while (
            ctrl.status()["state"] != "IDLE"
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert ctrl.status()["state"] == "IDLE"
        assert saved == [3]
    finally:
        ctrl.shutdown()


def test_stale_camera_frames_abort_episode_instead_of_duplicating():
    channel, collector, ctrl = _make()
    ctrl.max_staleness_s = -1.0
    try:
        ctrl.submit(HubAction.START.value)
        ctrl.tick()
        for _ in range(3):
            ctrl.tick()
        status = ctrl.status()
        assert status["state"] == "IDLE"
        assert status["frames_recorded"] == 0
        assert "stale_camera" in status["invalid_reason"]
    finally:
        ctrl.shutdown()


def test_submit_rejects_unknown_action():
    channel, collector, ctrl = _make()
    try:
        assert ctrl.submit("explode")["ok"] is False
    finally:
        ctrl.shutdown()


def test_continuity_allows_gap_within_staleness_window():
    channel, collector, ctrl = _make(max_staleness_s=0.06)
    saved = []
    ctrl.on_save = lambda episode: saved.append(len(episode))
    try:
        collector.buffer = EpisodeBuffer(episode_index=0)
        for timestamp in (0.0, 0.031, 0.062):
            collector.buffer.add_frame(
                state=np.zeros(22, dtype=np.float32),
                action=np.zeros(22, dtype=np.float32),
                images={},
                timestamp=timestamp,
            )
        collector.state = CollectionState.REVIEWING
        ctrl._finish_episode(save=True)

        deadline = time.monotonic() + 1.0
        while ctrl.status()["state"] != "IDLE" and time.monotonic() < deadline:
            time.sleep(0.01)
        assert saved == [3]
    finally:
        ctrl.shutdown()


def test_submit_home_action_invokes_on_home_callback():
    channel, collector, ctrl = _make()
    home_invoked = []
    ctrl.on_home = lambda: home_invoked.append(True)
    try:
        assert ctrl.submit("home")["ok"] is True
        ctrl.tick()
        assert home_invoked == [True]
    finally:
        ctrl.shutdown()


def test_submit_hand_hold_toggle_and_home_reset():
    channel, collector, ctrl = _make()
    states = []
    ctrl.on_hand_hold = lambda enabled: states.append(enabled)
    try:
        assert ctrl.status()["hand_hold"] is False

        assert ctrl.submit("hand_hold")["ok"] is True
        ctrl.tick()
        assert ctrl.status()["hand_hold"] is True
        assert states == [True]

        assert ctrl.submit("hand_hold")["ok"] is True
        ctrl.tick()
        assert ctrl.status()["hand_hold"] is False
        assert states == [True, False]

        # Explicit payload enable -> HOME must clear hand_hold
        assert ctrl.submit("hand_hold", {"enabled": True})["ok"] is True
        ctrl.tick()
        assert ctrl.status()["hand_hold"] is True
        assert ctrl.submit("home")["ok"] is True
        ctrl.tick()
        assert ctrl.status()["hand_hold"] is False
    finally:
        ctrl.shutdown()


def test_submit_axis_cycle_and_imu_axes_cycle_invokes_callbacks():
    channel, collector, ctrl = _make()
    axis_invoked = []
    imu_invoked = []
    ctrl.on_axis_cycle = lambda: axis_invoked.append(True)
    ctrl.on_imu_axes_cycle = lambda: imu_invoked.append(True)
    try:
        assert ctrl.submit("axis_cycle")["ok"] is True
        ctrl.tick()
        assert axis_invoked == [True]

        assert ctrl.submit("imu_axes_cycle")["ok"] is True
        ctrl.tick()
        assert imu_invoked == [True]
    finally:
        ctrl.shutdown()


def test_data_hub_controller_environment_tag():
    channel, collector, ctrl_sim = _make(env="sim")
    try:
        assert ctrl_sim.status()["env"] == "sim"
    finally:
        ctrl_sim.shutdown()

    channel2, collector2, ctrl_real = _make(env="real")
    try:
        assert ctrl_real.status()["env"] == "real"
    finally:
        ctrl_real.shutdown()


def test_submit_clutch_toggle_and_home_reset():
    channel, collector, ctrl = _make()
    states = []
    ctrl.on_clutch = lambda enabled: states.append(enabled)
    try:
        assert ctrl.status()["clutch_engaged"] is False

        assert ctrl.submit("clutch")["ok"] is True
        ctrl.tick()
        assert ctrl.status()["clutch_engaged"] is True
        assert states == [True]

        assert ctrl.submit("clutch")["ok"] is True
        ctrl.tick()
        assert ctrl.status()["clutch_engaged"] is False
        assert states == [True, False]

        # Explicit payload enable -> HOME must clear clutch
        assert ctrl.submit("clutch", {"enabled": True})["ok"] is True
        ctrl.tick()
        assert ctrl.status()["clutch_engaged"] is True
        assert ctrl.submit("home")["ok"] is True
        ctrl.tick()
        assert ctrl.status()["clutch_engaged"] is False

        # set_clutch_state sync from teleop node
        ctrl.set_clutch_state(True)
        assert ctrl.status()["clutch_engaged"] is True
        ctrl.set_clutch_state(False)
        assert ctrl.status()["clutch_engaged"] is False
    finally:
        ctrl.shutdown()


