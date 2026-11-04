"""test_realsense_stream.py — 异步帧缓冲与采集线程单元测试 (W8)."""
from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from tuinadex_teleop.perception.realsense_stream import (
    AsyncFrameBuffer,
    MockRealSenseStream,
    run_capture_loop,
)


class _FakeFrame:
    def __init__(self, array):
        self._array = array

    def get_data(self):
        return self._array


class _FakeFrames:
    def __init__(self, color, depth):
        self._color = color
        self._depth = depth

    def get_color_frame(self):
        return _FakeFrame(self._color) if self._color is not None else None

    def get_depth_frame(self):
        return _FakeFrame(self._depth) if self._depth is not None else None


class _FakePipeline:
    """模拟 RealSense pipeline，可控产出与超时."""

    def __init__(self, total=5, fail_first=0):
        self.total = total
        self.fail_first = fail_first
        self.calls = 0

    def wait_for_frames(self, timeout_ms=1000):
        self.calls += 1
        if self.calls <= self.fail_first:
            raise RuntimeError("timeout")
        if self.calls > self.fail_first + self.total:
            raise RuntimeError("exhausted")
        color = np.full((4, 4, 3), self.calls % 256, dtype=np.uint8)
        depth = np.full((4, 4), 1000, dtype=np.uint16)
        return _FakeFrames(color, depth)


class _FakeAlign:
    def process(self, frames):
        return frames


def test_async_buffer_latest_wins_and_seq_monotonic():
    buf = AsyncFrameBuffer()
    assert buf.latest()[3] == 0

    for i in range(5):
        buf.publish(np.full((2, 2, 3), i, np.uint8), np.full((2, 2), 0.5, np.float32))
        time.sleep(0.001)

    color, depth, ts, seq = buf.latest()
    assert seq == 5
    assert int(color[0, 0, 0]) == 4          # 最新一帧胜出，无队列堆积
    assert depth[0, 0] == pytest.approx(0.5)
    assert ts > 0.0


def test_async_buffer_concurrent_read_write_is_consistent():
    buf = AsyncFrameBuffer()
    stop = threading.Event()

    def writer():
        while not stop.is_set():
            buf.publish(np.zeros((8, 8, 3), np.uint8), np.ones((8, 8), np.float32))

    t = threading.Thread(target=writer, daemon=True)
    t.start()
    try:
        for _ in range(200):
            color, depth, _ts, seq = buf.latest()
            if seq > 0:
                assert color.shape == (8, 8, 3)
                assert depth.shape == (8, 8)
    finally:
        stop.set()
        t.join(2.0)
    assert not t.is_alive()


def test_capture_loop_publishes_and_converts_depth_to_meters():
    buf = AsyncFrameBuffer()
    stop = threading.Event()
    pipeline = _FakePipeline(total=5)

    thread = threading.Thread(
        target=run_capture_loop,
        args=(pipeline, _FakeAlign(), 0.001, buf, stop),
        daemon=True,
    )
    thread.start()
    try:
        deadline = time.monotonic() + 2.0
        while buf.latest()[3] < 5 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert buf.latest()[3] >= 5, "capture loop did not publish frames"
        color, depth, _ts, _seq = buf.latest()
        assert color.dtype == np.uint8
        assert depth.dtype == np.float32
        assert depth[0, 0] == pytest.approx(1.0)   # 1000 * 0.001
    finally:
        stop.set()
        thread.join(2.0)
    assert not thread.is_alive()


def test_capture_loop_survives_timeouts_and_stops_promptly():
    buf = AsyncFrameBuffer()
    stop = threading.Event()
    pipeline = _FakePipeline(total=1, fail_first=2)

    thread = threading.Thread(
        target=run_capture_loop,
        args=(pipeline, None, 0.001, buf, stop),
        daemon=True,
    )
    thread.start()
    time.sleep(0.05)
    stop.set()
    thread.join(2.0)
    assert not thread.is_alive(), "capture loop must exit on stop event"


def test_capture_loop_skips_incomplete_frame_pairs():
    buf = AsyncFrameBuffer()
    stop = threading.Event()

    class _PartialPipeline:
        def wait_for_frames(self, timeout_ms=1000):
            time.sleep(0.01)
            return _FakeFrames(np.zeros((4, 4, 3), np.uint8), None)

    thread = threading.Thread(
        target=run_capture_loop,
        args=(_PartialPipeline(), None, 0.001, buf, stop),
        daemon=True,
    )
    thread.start()
    time.sleep(0.1)
    stop.set()
    thread.join(2.0)
    assert buf.latest()[3] == 0, "incomplete pairs must not be published"


def test_mock_stream_read_latest_is_non_blocking():
    stream = MockRealSenseStream(width=8, height=6, fps=30)
    stream.start()
    t0 = time.monotonic()
    color, depth, ts, seq = stream.read_latest()
    elapsed = time.monotonic() - t0
    assert elapsed < 0.05
    assert color.shape == (6, 8, 3)
    assert depth.shape == (6, 8)
    assert ts > 0.0
    # 兼容既有三元组接口
    color2, depth2, ts2 = stream.read_frames()
    assert color2.shape == (6, 8, 3)
    assert depth2.shape == (6, 8)
