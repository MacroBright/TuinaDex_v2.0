"""三机位规格、MJCF 相机与独立渲染线程测试 (需 EGL 离屏渲染)."""
import os
import threading
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco

from tuinadex_teleop.cameras.camera_specs import (
    CameraMountType,
    STANDARD_CAMERA_SPECS,
)
from tuinadex_teleop.cameras.sim_camera_rig import CameraProducerThread, SimCameraRig

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml"
SCENE_COPY = ROOT / "src/tuinadex_sim/worlds/tuina_scene.xml"


def _load_scene():
    assert SCENE.is_file(), f"场景缺失: {SCENE}"  # I35: 硬失败，不 skip
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_camera_specs_are_standard():
    assert set(STANDARD_CAMERA_SPECS) == {"overhead", "wrist", "side"}
    expected = {
        "overhead": ("overhead_cam", CameraMountType.EYE_TO_HAND, 60.0),
        "wrist": ("wrist_cam", CameraMountType.EYE_IN_HAND, 75.0),
        "side": ("side_cam", CameraMountType.EYE_TO_HAND, 45.0),
    }
    for name, (mjcf_name, mount, fovy) in expected.items():
        spec = STANDARD_CAMERA_SPECS[name]
        assert spec.mjcf_name == mjcf_name
        assert spec.mount_type is mount
        assert spec.fovy == fovy
        assert (spec.width, spec.height) == (640, 480)


def test_scene_contains_standard_cameras_with_matching_fovy():
    model, _ = _load_scene()
    for spec in STANDARD_CAMERA_SPECS.values():
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, spec.mjcf_name)
        assert cid >= 0, f"MJCF 缺少相机 {spec.mjcf_name}"
        assert model.cam_fovy[cid] == pytest.approx(spec.fovy, abs=1.0)


def test_scene_copies_are_byte_identical():
    assert SCENE.read_bytes() == SCENE_COPY.read_bytes(), "两份场景文件必须逐字节一致 (I34)"


def test_sim_camera_rig_renders_three_non_blank_views():
    model, data = _load_scene()
    rig = SimCameraRig(model, data)
    try:
        frames = rig.render_all()
        assert set(frames) == {"overhead", "wrist", "side"}
        for cam, img in frames.items():
            assert img.shape == (480, 640, 3), cam
            assert img.dtype == np.uint8
            assert img.mean() > 5.0, f"{cam} 渲染为空白"
        encoded = rig.encode_all()
        assert set(encoded) == {"overhead", "wrist", "side"}
        for cam, payload in encoded.items():
            assert isinstance(payload, bytes) and len(payload) > 1000, cam
    finally:
        rig.close()


def test_camera_producer_thread_runs_off_control_loop():
    model, data = _load_scene()
    lock = threading.Lock()
    producer = CameraProducerThread(model, data, lock, target_fps=25.0)
    producer.start()
    try:
        assert producer.wait_ready(timeout=30.0), "渲染线程未能建立 EGL 上下文"
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with lock:
                mujoco.mj_step(model, data)
            time.sleep(0.02)
        latest = producer.latest()
        assert set(latest) == {"overhead", "wrist", "side"}
        for cam, (payload, ts) in latest.items():
            assert len(payload) > 1000, cam
            assert ts <= time.monotonic()
        assert producer.stats()["produced"] >= 3
    finally:
        producer.stop()
    assert not producer.is_alive()
