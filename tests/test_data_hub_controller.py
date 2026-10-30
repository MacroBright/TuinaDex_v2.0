"""Hub controller tests for command queue, stale frames, and async saves."""
import time

from tuinadex_teleop.collection.collector import CollectionState, TuinaCollector
from tuinadex_teleop.collection.data_channel import MockDataChannel
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
