"""MuJoCo offscreen three-camera rendering and capture thread."""
from __future__ import annotations

import logging
import threading
import time
from typing import Any

import cv2
import mujoco
import numpy as np

from .camera_specs import STANDARD_CAMERA_SPECS, CameraSpec

logger = logging.getLogger(__name__)


def encode_rgb_to_jpeg(image: np.ndarray, quality: int = 90) -> bytes:
    """Encode an RGB uint8 (H, W, 3) image as JPEG bytes."""
    arr = np.asarray(image)
    if arr.dtype != np.uint8 or arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"expect uint8 (H,W,3), got {arr.dtype} {arr.shape}")
    ok, buf = cv2.imencode(
        ".jpg",
        cv2.cvtColor(arr, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, int(quality)],
    )
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return buf.tobytes()


def decode_jpeg_to_rgb(payload: bytes) -> np.ndarray:
    """Decode JPEG bytes to an RGB uint8 (H, W, 3) image."""
    bgr = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("corrupted JPEG payload")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


class SimCameraRig:
    """Render all configured views with one EGL renderer.

    Construction and close() must occur on the same thread. Do not call this
    class directly from the robot control loop.
    """

    def __init__(
        self,
        model: Any,
        data: Any,
        specs: dict[str, CameraSpec] | None = None,
        jpeg_quality: int = 90,
    ):
        self.model = model
        self.data = data
        self.specs = dict(specs or STANDARD_CAMERA_SPECS)
        self.jpeg_quality = int(jpeg_quality)
        sizes = {(spec.width, spec.height) for spec in self.specs.values()}
        if len(sizes) != 1:
            raise ValueError(
                f"all camera specs must share one resolution, got {sizes}"
            )
        self.width, self.height = sizes.pop()
        self._renderer = mujoco.Renderer(
            model, height=self.height, width=self.width
        )
        self._closed = False

    def render_camera(self, name: str) -> np.ndarray:
        """Render one camera view."""
        if self._closed:
            raise RuntimeError("SimCameraRig already closed")
        spec = self.specs[name]
        self._renderer.update_scene(self.data, camera=spec.mjcf_name)
        return self._renderer.render()

    def render_all(self) -> dict[str, np.ndarray]:
        """Render every configured camera view."""
        return {name: self.render_camera(name) for name in self.specs}

    def encode_all(self) -> dict[str, bytes]:
        """Render and JPEG-encode every configured camera view."""
        return {
            name: encode_rgb_to_jpeg(
                self.render_camera(name), self.jpeg_quality
            )
            for name in self.specs
        }

    def close(self) -> None:
        """Release the renderer on its creating thread."""
        if not self._closed:
            self._closed = True
            self._renderer.close()


class CameraProducerThread(threading.Thread):
    """Capture camera frames off the control loop with private MjData."""

    def __init__(
        self,
        model: Any,
        data: Any,
        data_lock: threading.Lock,
        specs: dict[str, CameraSpec] | None = None,
        target_fps: float = 50.0,
        jpeg_quality: int = 90,
    ):
        super().__init__(name="camera-producer", daemon=True)
        if target_fps <= 0:
            raise ValueError("target_fps must be > 0")
        self.model = model
        self.data = data
        self._data_lock = data_lock
        self.specs = dict(specs or STANDARD_CAMERA_SPECS)
        self.target_fps = float(target_fps)
        self.jpeg_quality = int(jpeg_quality)
        self._render_data = mujoco.MjData(model)
        self._ready = threading.Event()
        self._stop_evt = threading.Event()
        self._cond = threading.Condition()
        self._latest: dict[str, tuple[bytes, float]] = {}
        self._stats = {
            "produced": 0,
            "errors": 0,
            "achieved_fps": 0.0,
            "last_error": None,
        }

    def wait_ready(self, timeout: float = 30.0) -> bool:
        """Wait until the EGL renderer is ready or startup fails."""
        return self._ready.wait(timeout)

    def latest(self) -> dict[str, tuple[bytes, float]]:
        """Return an atomic snapshot of the latest JPEG frame per camera."""
        with self._cond:
            return dict(self._latest)

    def stats(self) -> dict:
        """Return an atomic snapshot of capture statistics."""
        with self._cond:
            return dict(self._stats)

    def run(self) -> None:
        """Capture until stopped; all MuJoCo rendering stays in this thread."""
        rig = None
        try:
            rig = SimCameraRig(
                self.model,
                self._render_data,
                self.specs,
                self.jpeg_quality,
            )
            self._ready.set()
            period = 1.0 / self.target_fps
            t_start = time.monotonic()
            n = 0
            while not self._stop_evt.is_set():
                n += 1
                next_t = t_start + n * period
                with self._data_lock:
                    np.copyto(self._render_data.qpos, self.data.qpos)
                    np.copyto(self._render_data.qvel, self.data.qvel)
                mujoco.mj_forward(self.model, self._render_data)
                payload = rig.encode_all()
                now = time.monotonic()
                with self._cond:
                    self._latest = {
                        cam: (buf, now) for cam, buf in payload.items()
                    }
                    self._stats["produced"] += 1
                    elapsed = now - t_start
                    self._stats["achieved_fps"] = (
                        round(self._stats["produced"] / elapsed, 2)
                        if elapsed > 0
                        else 0.0
                    )
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0:
                    self._stop_evt.wait(sleep_s)
        except Exception as exc:  # noqa: BLE001 - expose thread failures
            with self._cond:
                self._stats["errors"] += 1
                self._stats["last_error"] = f"{type(exc).__name__}: {exc}"
            logger.exception("CameraProducerThread failed")
        finally:
            if rig is not None:
                rig.close()
            self._ready.set()

    def stop(self, timeout: float = 5.0) -> None:
        """Request shutdown and join the render thread."""
        self._stop_evt.set()
        with self._cond:
            self._cond.notify_all()
        self.join(timeout)
        if self.is_alive():
            logger.error(
                "CameraProducerThread did not stop within %.1fs", timeout
            )
