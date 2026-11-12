"""Unit tests for physical UVC camera rig and extended RealDataChannel."""
from __future__ import annotations

import os
from pathlib import Path
import sys
import time

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
TELEOP_PKG = ROOT / "src" / "tuinadex_teleop"
CORE_PKG = ROOT / "src" / "tuinadex_core"
for pkg in (TELEOP_PKG, CORE_PKG):
    if str(pkg) not in sys.path:
        sys.path.insert(0, str(pkg))

from tuinadex_teleop.cameras import (
    RealCameraConfig,
    RealCameraProducerThread,
    decode_jpeg_to_rgb,
    load_real_camera_configs,
    parse_camera_map,
    resolve_default_camera_config_path,
)
from tuinadex_teleop.collection.data_channel import RealDataChannel


class _FakeCapture:
    """Thread-safe synthetic camera capture emitting solid BGR frames."""

    def __init__(self, bgr_color: tuple[int, int, int], width: int = 640, height: int = 480) -> None:
        self._frame = np.zeros((height, width, 3), dtype=np.uint8)
        self._frame[:, :] = bgr_color
        self._opened = True
        self.released = False

    def isOpened(self) -> bool:
        return self._opened

    def read(self) -> tuple[bool, np.ndarray]:
        time.sleep(0.005)
        return True, self._frame.copy()

    def release(self) -> None:
        self._opened = False
        self.released = True


def test_parse_camera_map_valid_and_invalid() -> None:
    assert parse_camera_map(None) == {}
    assert parse_camera_map("") == {}
    parsed = parse_camera_map("overhead=/dev/video8, side=/dev/video6")
    assert parsed == {
        "overhead": "/dev/video8",
        "side": "/dev/video6",
    }
    with pytest.raises(ValueError):
        parse_camera_map("overhead_missing_equals")
    with pytest.raises(ValueError):
        parse_camera_map("overhead=")


def test_load_real_camera_configs_yaml_and_overrides(tmp_path: Path) -> None:
    cfg_file = tmp_path / "cams.yaml"
    cfg_file.write_text(
        """
cameras:
  overhead:
    device: "/dev/v4l/by-path/pci-overhead"
    width: 640
    height: 480
    fps: 60.0
    fourcc: "MJPG"
    rotate_deg: 0
  side:
    device: "/dev/v4l/by-path/pci-side"
    width: 640
    height: 480
    fps: 60.0
    rotate_deg: 180
  wrist:
    device: ""
""".strip(),
        encoding="utf-8",
    )

    cfgs = load_real_camera_configs(
        ["overhead", "side"],
        config_path=cfg_file,
        overrides={"side": "/dev/video99"},
    )
    assert [c.name for c in cfgs] == ["overhead", "side"]
    assert cfgs[0].device == "/dev/v4l/by-path/pci-overhead"
    assert cfgs[1].device == "/dev/video99"
    assert cfgs[1].rotate_deg == 180

    with pytest.raises(ValueError, match="wrist"):
        load_real_camera_configs(["overhead", "wrist"], config_path=cfg_file)

    bad_rot = tmp_path / "bad_rot.yaml"
    bad_rot.write_text(
        "cameras:\n  overhead:\n    device: '/dev/video8'\n    rotate_deg: 45\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="rotate_deg"):
        load_real_camera_configs(["overhead"], config_path=bad_rot)


def test_load_real_camera_configs_rejects_duplicate_device(tmp_path: Path) -> None:
    real_dev = tmp_path / "video8"
    real_dev.write_text("fake", encoding="utf-8")
    symlink_dev = tmp_path / "by_path_cam"
    os.symlink(real_dev, symlink_dev)

    with pytest.raises(ValueError, match="Duplicate physical camera device"):
        load_real_camera_configs(
            ["overhead", "side"],
            config_path=None,
            overrides={"overhead": str(real_dev), "side": str(symlink_dev)},
        )


def test_repo_real_cameras_yaml_maps_overhead_and_side() -> None:
    cfg_path = resolve_default_camera_config_path()
    assert cfg_path is not None and cfg_path.is_file()
    cfgs = load_real_camera_configs(["overhead", "side"], config_path=cfg_path)
    assert len(cfgs) == 2
    assert cfgs[0].name == "overhead"
    assert cfgs[1].name == "side"
    assert cfgs[0].device != cfgs[1].device
    assert (cfgs[0].width, cfgs[0].height, cfgs[0].fourcc) == (640, 480, "MJPG")
    assert (cfgs[1].width, cfgs[1].height, cfgs[1].fourcc) == (640, 480, "MJPG")


def test_real_camera_producer_concurrent_frames_and_transforms() -> None:
    cfgs = [
        RealCameraConfig(name="overhead", device="/dev/fake_over", width=640, height=480),
        RealCameraConfig(
            name="side",
            device="/dev/fake_side",
            width=640,
            height=480,
            rotate_deg=90,
        ),
    ]
    captures: dict[str, _FakeCapture] = {}

    def _factory(cfg: RealCameraConfig) -> _FakeCapture:
        # overhead emits pure blue in BGR -> pure blue RGB [0, 0, 255]
        # side emits 480x640 green frame that becomes 640x480 after rotate_deg=90
        if cfg.name == "overhead":
            cap = _FakeCapture((240, 10, 10), width=640, height=480)
        else:
            cap = _FakeCapture((10, 240, 10), width=480, height=640)
        captures[cfg.name] = cap
        return cap

    producer = RealCameraProducerThread(cfgs, target_fps=50.0, capture_factory=_factory)
    producer.start()
    try:
        assert producer.wait_ready(2.0) is True
        latest = producer.latest()
        assert set(latest.keys()) == {"overhead", "side"}

        over_jpg, over_ts = latest["overhead"]
        side_jpg, side_ts = latest["side"]
        assert over_jpg.startswith(b"\xff\xd8")
        assert side_jpg.startswith(b"\xff\xd8")
        assert over_ts > 0 and side_ts > 0

        over_rgb = decode_jpeg_to_rgb(over_jpg)
        side_rgb = decode_jpeg_to_rgb(side_jpg)
        assert over_rgb.shape == (480, 640, 3)
        assert side_rgb.shape == (480, 640, 3)
        # BGR (240, 10, 10) -> RGB (~10, ~10, ~240)
        assert over_rgb[0, 0, 2] > 200 and over_rgb[0, 0, 0] < 40
        # BGR (10, 240, 10) -> RGB (~10, ~240, ~10)
        assert side_rgb[0, 0, 1] > 200 and side_rgb[0, 0, 2] < 40

        stats = producer.stats()
        assert stats["produced"] >= 2
        assert stats["achieved_fps"] > 0.0
        assert stats["last_error"] is None
        assert set(stats["cameras"].keys()) == {"overhead", "side"}
    finally:
        producer.stop(timeout=2.0)

    assert captures["overhead"].released is True
    assert captures["side"].released is True


def test_real_camera_producer_open_failure_fails_fast() -> None:
    cfgs = [RealCameraConfig(name="overhead", device="/dev/does_not_exist")]

    def _broken_factory(_cfg: RealCameraConfig):
        raise FileNotFoundError("no such physical camera")

    producer = RealCameraProducerThread(cfgs, capture_factory=_broken_factory)
    producer.start()
    t0 = time.monotonic()
    ok = producer.wait_ready(5.0)
    elapsed = time.monotonic() - t0
    producer.stop(timeout=1.0)

    assert ok is False
    assert elapsed < 1.0
    assert "no such physical camera" in (producer.stats()["last_error"] or "")


def test_real_data_channel_with_producer_and_state_supplier() -> None:
    with pytest.raises(ValueError):
        RealDataChannel(env=None, action_supplier=lambda: np.zeros(22, dtype=np.float32))

    cfgs = [
        RealCameraConfig(name="overhead", device="/dev/fake_over"),
        RealCameraConfig(name="side", device="/dev/fake_side"),
    ]
    producer = RealCameraProducerThread(
        cfgs,
        capture_factory=lambda cfg: _FakeCapture(
            (200, 20, 20) if cfg.name == "overhead" else (20, 200, 20)
        ),
    )
    producer.start()
    try:
        assert producer.wait_ready(2.0) is True
        q_obs = np.linspace(0.1, 2.2, 22, dtype=np.float32)
        eff_obs = np.ones(22, dtype=np.float32) * 0.5
        act = np.ones(22, dtype=np.float32) * 0.25

        channel = RealDataChannel(
            env=None,
            action_supplier=lambda: act,
            camera_names=("overhead", "side"),
            camera_producer=producer,
            state_supplier=lambda: {
                "state": q_obs,
                "effort": eff_obs,
            },
        )
        pkt = channel.get_camera_packets()
        assert set(pkt.jpeg.keys()) == {"overhead", "side"}
        assert set(pkt.timestamps.keys()) == {"overhead", "side"}

        q, effort, forces = channel.get_proprioception()
        assert np.allclose(q, q_obs)
        assert np.allclose(effort, eff_obs)
        assert forces.shape == (4,)
        assert np.allclose(forces, 0.0)
        assert np.allclose(channel.get_action_label(), act)
    finally:
        producer.stop(timeout=2.0)


def test_hub_index_html_avoids_hardcoded_wrist_img() -> None:
    html_path = TELEOP_PKG / "tuinadex_teleop" / "hub" / "static" / "index.html"
    html = html_path.read_text(encoding="utf-8")
    assert 'src="/video/wrist"' not in html
    assert 'id="primary-img"' in html
    assert 'id="secondary-strip"' in html
