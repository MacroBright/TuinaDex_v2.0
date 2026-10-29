"""test_episode_buffer.py — EpisodeBuffer 多模态试次内存缓存单元测试."""
import pytest
import numpy as np

from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer


def test_episode_buffer_lifecycle():
    buf = EpisodeBuffer()
    assert len(buf) == 0

    for i in range(10):
        state = np.full(16, i * 0.1, dtype=np.float32)
        action = np.full(16, i * 0.2, dtype=np.float32)
        head_img = np.full((120, 160, 3), i, dtype=np.uint8)
        wrist_img = np.full((120, 160, 3), i * 2, dtype=np.uint8)

        buf.add_frame(
            state=state,
            action=action,
            images={"head": head_img, "wrist": wrist_img},
            timestamp=float(i) * 0.02,
        )

    assert len(buf) == 10
    data = buf.to_dict()

    assert data["state"].shape == (10, 16)
    assert data["action"].shape == (10, 16)
    assert data["timestamp"].shape == (10,)
    assert "head" in data["images"] and "wrist" in data["images"]
    assert data["images"]["head"].shape == (10, 120, 160, 3)
    assert data["images"]["wrist"].shape == (10, 120, 160, 3)

    buf.clear()
    assert len(buf) == 0
    empty_data = buf.to_dict()
    assert empty_data["state"].shape[0] == 0


def test_episode_buffer_validation():
    buf = EpisodeBuffer()
    with pytest.raises(ValueError, match="state and action must have same dimension"):
        buf.add_frame(
            state=np.zeros(16, dtype=np.float32),
            action=np.zeros(10, dtype=np.float32),
            images={},
            timestamp=0.0,
        )
