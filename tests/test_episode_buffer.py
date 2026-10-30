"""test_episode_buffer.py — EpisodeBuffer 多模态试次内存缓存单元测试."""
import pytest
import numpy as np

from tuinadex_teleop.cameras.sim_camera_rig import encode_rgb_to_jpeg
from tuinadex_teleop.collection.episode_buffer import (
    BufferCapacityError,
    EpisodeBuffer,
    FrameData,
)


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


def _frame_kwargs(i: int, cam="overhead"):
    return dict(
        state=np.full(22, i * 0.01, dtype=np.float32),
        action=np.full(22, i * 0.02, dtype=np.float32),
        effort=np.full(22, i * 0.1, dtype=np.float32),
        contact_forces=np.array([i, i + 1, i + 2, i + 3], dtype=np.float32),
        images={cam: np.full((48, 64, 3), i, dtype=np.uint8)},
        timestamp=float(i) * 0.02,
        phase_label="press" if i < 3 else "release",
    )


def test_multi_modal_buffer_metadata_and_derived_done():
    buf = EpisodeBuffer(episode_index=7, language_instruction="按压大椎穴")
    for i in range(5):
        buf.add_frame(**_frame_kwargs(i))
    buf.seal_episode()

    data = buf.to_dict()
    assert data["state"].shape == (5, 22)
    assert data["effort"].shape == (5, 22)
    assert data["contact_forces"].shape == (5, 4)
    assert data["frame_index"].tolist() == [0, 1, 2, 3, 4]
    assert (data["episode_index"] == 7).all()
    assert data["next_done"].tolist() == [False, False, False, False, True]
    assert data["phase_label"].tolist() == [
        "press",
        "press",
        "press",
        "release",
        "release",
    ]
    assert data["language_instruction"] == "按压大椎穴"
    # 旧键向后兼容
    assert data["action"].shape == (5, 22)
    assert data["images"]["overhead"].shape == (5, 48, 64, 3)


def test_images_stored_as_jpeg_and_roundtrip_rgb():
    buf = EpisodeBuffer()
    red_rgb = np.zeros((16, 16, 3), dtype=np.uint8)
    red_rgb[..., 0] = 255
    buf.add_frame(
        state=np.zeros(22, np.float32),
        action=np.zeros(22, np.float32),
        images={"overhead": red_rgb},
        timestamp=0.0,
    )
    frame = buf.iter_frames().__next__()
    assert isinstance(frame.images_jpeg["overhead"], bytes)
    decoded = frame.decode_image("overhead")
    assert decoded.shape == (16, 16, 3)
    assert decoded[..., 0].mean() > 240 and decoded[..., 1].mean() < 20


def test_add_frame_accepts_preencoded_bytes_without_reencoding():
    payload = encode_rgb_to_jpeg(
        np.full((16, 16, 3), 7, dtype=np.uint8)
    )
    buf = EpisodeBuffer()
    buf.add_frame(
        state=np.zeros(22, np.float32),
        action=np.zeros(22, np.float32),
        images={"overhead": payload},
        timestamp=0.0,
    )
    assert buf.iter_frames().__next__().images_jpeg["overhead"] == payload


def test_buffer_capacity_guard():
    frame_limited = EpisodeBuffer(
        max_episode_frames=2, max_episode_bytes=1024 * 1024
    )
    frame_limited.add_frame(**_frame_kwargs(0))
    frame_limited.add_frame(**_frame_kwargs(1))
    with pytest.raises(BufferCapacityError, match="frame limit"):
        frame_limited.add_frame(**_frame_kwargs(2))

    byte_limited = EpisodeBuffer(max_episode_bytes=10)
    with pytest.raises(BufferCapacityError, match="byte limit"):
        byte_limited.add_frame(**_frame_kwargs(0))


def test_validate_continuity_reports_gaps():
    buf = EpisodeBuffer()
    for i in range(3):
        buf.add_frame(**_frame_kwargs(i))
    assert buf.validate_continuity(max_gap_s=0.1) == []
    buf._frames[2] = FrameData(
        **{**buf._frames[2].__dict__, "timestamp": 5.0}
    )
    assert any(
        "gap" in msg for msg in buf.validate_continuity(max_gap_s=0.1)
    )


def test_dimension_mismatch_raises():
    buf = EpisodeBuffer()
    with pytest.raises(ValueError, match="effort must match"):
        buf.add_frame(
            state=np.zeros(22, np.float32),
            action=np.zeros(22, np.float32),
            effort=np.zeros(16, np.float32),
            images={},
            timestamp=0.0,
        )
    with pytest.raises(ValueError, match="contact_forces must be"):
        buf.add_frame(
            state=np.zeros(22, np.float32),
            action=np.zeros(22, np.float32),
            contact_forces=np.zeros(3, np.float32),
            images={},
            timestamp=0.0,
        )
