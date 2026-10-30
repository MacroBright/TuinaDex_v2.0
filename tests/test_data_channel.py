"""Data-source channel contract tests."""
import time

import numpy as np
import pytest

from tuinadex_teleop.collection.data_channel import (
    BaseDataChannel,
    CameraPacket,
    MockDataChannel,
)


def test_mock_channel_contract():
    channel = MockDataChannel()
    try:
        packets = channel.get_camera_packets()
        assert set(packets.jpeg) == {"overhead", "wrist", "side"}
        assert all(
            isinstance(payload, bytes) and len(payload) > 100
            for payload in packets.jpeg.values()
        )
        assert packets.worst_age() < 1.0

        frames = channel.get_camera_frames()
        assert frames["overhead"].shape == (480, 640, 3)
        assert frames["overhead"].dtype == np.uint8

        q, effort, forces = channel.get_proprioception()
        assert q.shape == (22,)
        assert effort.shape == (22,)
        assert forces.shape == (4,)

        assert channel.get_action_label().shape == (22,)
    finally:
        channel.close()


def test_camera_packet_age_semantics():
    now = time.monotonic()
    packet = CameraPacket(
        jpeg={"overhead": b"x", "side": b"y"},
        timestamps={"overhead": now - 0.02, "side": now - 0.5},
    )
    ages = packet.ages(now)
    assert ages["overhead"] == pytest.approx(0.02, abs=1e-6)
    assert packet.worst_age(now) == pytest.approx(0.5, abs=1e-6)
    assert CameraPacket(jpeg={}, timestamps={}).worst_age(now) == float("inf")


def test_base_channel_is_abstract():
    with pytest.raises(TypeError):
        BaseDataChannel()


def test_sim_channel_reads_adapter_and_producer():
    """Verify the 22D/4-force/3-image contract with real rendering."""
    import threading
    from pathlib import Path

    from tuinadex_teleop.adapters.mujoco_sim_adapter import MujocoSimAdapter
    from tuinadex_teleop.cameras.sim_camera_rig import CameraProducerThread
    from tuinadex_teleop.collection.data_channel import SimDataChannel

    root = Path(__file__).resolve().parents[1]
    model_path = (
        root / "src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml"
    )
    assert model_path.is_file()
    adapter = MujocoSimAdapter(model_path=model_path, headless=True)
    assert adapter.connect()
    lock = threading.Lock()
    producer = CameraProducerThread(
        adapter.model, adapter.data, lock, target_fps=20.0
    )
    producer.start()
    try:
        assert producer.wait_ready(30.0)
        for _ in range(5):
            with lock:
                adapter.step(1)
            time.sleep(0.03)
        channel = SimDataChannel(
            adapter,
            producer,
            action_supplier=lambda: np.zeros(22, dtype=np.float32),
        )
        packets = channel.get_camera_packets()
        assert set(packets.jpeg) == {"overhead", "wrist", "side"}
        q, effort, forces = channel.get_proprioception()
        assert q.shape == (22,)
        assert effort.shape == (22,)
        assert forces.shape == (4,)
        assert np.all(np.isfinite(q))
        assert channel.get_action_label().shape == (22,)
        channel.close()
    finally:
        producer.stop()
        adapter.disconnect()
