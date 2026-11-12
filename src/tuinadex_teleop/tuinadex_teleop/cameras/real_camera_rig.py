"""Physical USB/UVC multi-camera rig with off-loop producer threads."""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Sequence

import cv2
import numpy as np
import yaml


_VALID_ROTATIONS = {0, 90, 180, 270}
_ROTATE_CV_CODES = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


@dataclass(frozen=True)
class RealCameraConfig:
    """Configuration for a single physical camera stream."""

    name: str
    device: str
    width: int = 640
    height: int = 480
    fps: float = 60.0
    fourcc: str = "MJPG"
    rotate_deg: int = 0
    flip_h: bool = False
    flip_v: bool = False
    buffersize: int = 1


def encode_bgr_to_jpeg(frame_bgr: np.ndarray, quality: int = 90) -> bytes:
    """Encode an uint8 BGR frame into JPEG bytes using OpenCV."""
    if frame_bgr is None or frame_bgr.size == 0:
        raise ValueError("Cannot encode empty BGR frame to JPEG")
    q = max(1, min(100, int(quality)))
    ok, buf = cv2.imencode(
        ".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), q]
    )
    if not ok or buf is None:
        raise RuntimeError("cv2.imencode('.jpg', ...) failed")
    return buf.tobytes()


def _transform_bgr_frame(
    frame_bgr: np.ndarray, cfg: RealCameraConfig
) -> np.ndarray:
    """Apply optional resize, rotation, and flip transforms to a BGR frame."""
    out = frame_bgr
    if cfg.rotate_deg in _ROTATE_CV_CODES:
        out = cv2.rotate(out, _ROTATE_CV_CODES[cfg.rotate_deg])
    if cfg.flip_h and cfg.flip_v:
        out = cv2.flip(out, -1)
    elif cfg.flip_h:
        out = cv2.flip(out, 1)
    elif cfg.flip_v:
        out = cv2.flip(out, 0)
    h, w = out.shape[:2]
    if w != cfg.width or h != cfg.height:
        out = cv2.resize(
            out, (cfg.width, cfg.height), interpolation=cv2.INTER_AREA
        )
    return out


def resolve_default_camera_config_path() -> Path | None:
    """Locate configs/real_cameras.yaml in source tree or installed share."""
    source_candidate = (
        Path(__file__).resolve().parents[2] / "configs" / "real_cameras.yaml"
    )
    if source_candidate.is_file():
        return source_candidate
    try:
        from ament_index_python.packages import get_package_share_directory

        share_candidate = (
            Path(get_package_share_directory("tuinadex_teleop"))
            / "configs"
            / "real_cameras.yaml"
        )
        if share_candidate.is_file():
            return share_candidate
    except Exception:
        pass
    return None


def parse_camera_map(spec: str | Sequence[str] | None) -> dict[str, str]:
    """Parse camera mapping overrides such as 'overhead=/dev/video8,side=/dev/video6'."""
    if spec is None:
        return {}
    if isinstance(spec, str):
        raw_items = [item.strip() for item in spec.split(",") if item.strip()]
    else:
        raw_items = []
        for entry in spec:
            for item in str(entry).split(","):
                stripped = item.strip()
                if stripped:
                    raw_items.append(stripped)

    overrides: dict[str, str] = {}
    for item in raw_items:
        if "=" not in item:
            raise ValueError(
                f"Invalid camera map entry {item!r}; expected 'name=device_path'"
            )
        name, dev = item.split("=", 1)
        name = name.strip()
        dev = dev.strip()
        if not name or not dev:
            raise ValueError(
                f"Invalid camera map entry {item!r}; camera name and device must be non-empty"
            )
        overrides[name] = dev
    return overrides


def discover_uvc_devices() -> list[dict[str, str]]:
    """Discover V4L2 video capture devices and deterministic /dev/v4l/by-path symlinks."""
    by_path_map: dict[str, str] = {}
    by_path_dir = Path("/dev/v4l/by-path")
    if by_path_dir.is_dir():
        for entry in sorted(by_path_dir.iterdir()):
            resolved = os.path.realpath(entry)
            by_path_map.setdefault(resolved, str(entry))

    devices: list[dict[str, str]] = []
    sys_v4l = Path("/sys/class/video4linux")
    if sys_v4l.is_dir():
        for node_dir in sorted(sys_v4l.iterdir()):
            dev_node = f"/dev/{node_dir.name}"
            name_file = node_dir / "name"
            card_name = ""
            if name_file.is_file():
                card_name = name_file.read_text(encoding="utf-8", errors="ignore").strip()
            real = os.path.realpath(dev_node)
            devices.append(
                {
                    "node": dev_node,
                    "realpath": real,
                    "by_path": by_path_map.get(real, ""),
                    "card_name": card_name,
                }
            )
    return devices


def load_real_camera_configs(
    camera_names: Sequence[str],
    config_path: str | Path | None = None,
    overrides: dict[str, str] | None = None,
) -> list[RealCameraConfig]:
    """Load and validate physical camera configs without cross-camera aliasing."""
    if not camera_names:
        raise ValueError("camera_names must not be empty")

    overrides = dict(overrides or {})
    yaml_cameras: dict[str, dict[str, Any]] = {}

    resolved_cfg_path: Path | None = (
        Path(config_path) if config_path else resolve_default_camera_config_path()
    )
    if resolved_cfg_path is not None:
        if not resolved_cfg_path.is_file():
            raise FileNotFoundError(
                f"Camera config file not found: {resolved_cfg_path}"
            )
        raw = yaml.safe_load(resolved_cfg_path.read_text(encoding="utf-8")) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"Expected mapping in {resolved_cfg_path}")
        yaml_cameras = dict(raw.get("cameras") or {})

    configs: list[RealCameraConfig] = []
    seen_targets: dict[str, str] = {}

    for cam_name in camera_names:
        entry = dict(yaml_cameras.get(cam_name) or {})
        device = str(overrides.get(cam_name, entry.get("device", ""))).strip()
        if not device:
            raise ValueError(
                f"Camera '{cam_name}' has no physical device configured. "
                f"Configure it in configs/real_cameras.yaml or pass "
                f"--camera-map {cam_name}=/dev/v4l/by-path/..."
            )

        width = int(entry.get("width", 640))
        height = int(entry.get("height", 480))
        fps = float(entry.get("fps", 60.0))
        fourcc = str(entry.get("fourcc", "MJPG")).strip()
        rotate_deg = int(entry.get("rotate_deg", 0))
        flip_h = bool(entry.get("flip_h", False))
        flip_v = bool(entry.get("flip_v", False))
        buffersize = int(entry.get("buffersize", 1))

        if width <= 0 or height <= 0:
            raise ValueError(
                f"Camera '{cam_name}' dimensions must be positive, got {width}x{height}"
            )
        if fps <= 0:
            raise ValueError(
                f"Camera '{cam_name}' fps must be positive, got {fps}"
            )
        if rotate_deg not in _VALID_ROTATIONS:
            raise ValueError(
                f"Camera '{cam_name}' rotate_deg must be one of {sorted(_VALID_ROTATIONS)}, got {rotate_deg}"
            )

        canonical_target = (
            os.path.realpath(device) if os.path.exists(device) else device
        )
        prior_cam = seen_targets.get(canonical_target)
        if prior_cam is not None:
            raise ValueError(
                f"Duplicate physical camera device binding: both '{prior_cam}' "
                f"and '{cam_name}' resolve to {canonical_target}"
            )
        seen_targets[canonical_target] = cam_name

        configs.append(
            RealCameraConfig(
                name=cam_name,
                device=device,
                width=width,
                height=height,
                fps=fps,
                fourcc=fourcc,
                rotate_deg=rotate_deg,
                flip_h=flip_h,
                flip_v=flip_v,
                buffersize=buffersize,
            )
        )

    return configs


def _default_opencv_capture(cfg: RealCameraConfig) -> cv2.VideoCapture:
    """Open a physical V4L2 camera device with deterministic settings."""
    target: int | str = int(cfg.device) if cfg.device.isdigit() else cfg.device
    if isinstance(target, str) and not os.path.exists(target):
        raise FileNotFoundError(
            f"Physical camera device for '{cfg.name}' not found: {target}"
        )

    cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open V4L2 device {target} for camera '{cfg.name}'"
        )

    if cfg.fourcc and len(cfg.fourcc) == 4:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cfg.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(cfg.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(cfg.height))
    cap.set(cv2.CAP_PROP_FPS, float(cfg.fps))
    if cfg.buffersize > 0:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, float(cfg.buffersize))
    return cap


class RealCameraProducerThread:
    """Concurrent off-loop multi-camera reader matching CameraProducerThread API.

    Runs one dedicated reader thread per physical camera so blocking V4L2 reads
    execute in parallel and never reduce per-camera frame rate.
    """

    def __init__(
        self,
        configs: Sequence[RealCameraConfig],
        target_fps: float = 50.0,
        jpeg_quality: int = 90,
        capture_factory: Callable[[RealCameraConfig], Any] | None = None,
    ) -> None:
        if not configs:
            raise ValueError("configs must contain at least one RealCameraConfig")
        self._configs = list(configs)
        self._target_fps = float(target_fps)
        self._jpeg_quality = int(jpeg_quality)
        self._capture_factory = capture_factory or _default_opencv_capture

        self._stop = threading.Event()
        self._ready = threading.Event()
        self._failed = threading.Event()
        self._cond = threading.Condition()
        self._state_lock = threading.Lock()

        self._latest: dict[str, tuple[bytes, float]] = {}
        self._cam_ready: set[str] = set()
        self._threads: list[threading.Thread] = []
        self._started = False
        self._start_time = 0.0
        self._produced = 0
        self._last_error: str | None = None
        self._per_cam: dict[str, dict[str, Any]] = {
            cfg.name: {
                "device": cfg.device,
                "frames": 0,
                "failures": 0,
                "last_error": None,
                "start_time": 0.0,
            }
            for cfg in self._configs
        }

    def start(self) -> None:
        """Spawn one background capture thread per configured camera."""
        if self._started:
            return
        self._started = True
        self._start_time = time.monotonic()
        for cfg in self._configs:
            thread = threading.Thread(
                target=self._camera_loop,
                args=(cfg,),
                name=f"tuinadex-cam-{cfg.name}",
                daemon=True,
            )
            self._threads.append(thread)
            thread.start()

    def _mark_failed(self, cam_name: str, msg: str) -> None:
        with self._state_lock:
            self._last_error = msg
            self._per_cam[cam_name]["last_error"] = msg
        self._failed.set()
        with self._cond:
            self._cond.notify_all()

    def _camera_loop(self, cfg: RealCameraConfig) -> None:
        cap = None
        cam_name = cfg.name
        with self._state_lock:
            self._per_cam[cam_name]["start_time"] = time.monotonic()

        try:
            cap = self._capture_factory(cfg)
            if cap is None or (hasattr(cap, "isOpened") and not cap.isOpened()):
                raise RuntimeError(
                    f"Camera '{cam_name}' failed to open device {cfg.device}"
                )
        except Exception as exc:
            self._mark_failed(cam_name, str(exc))
            return

        consecutive_failures = 0
        min_period = (
            1.0 / self._target_fps
            if self._capture_factory is not _default_opencv_capture
            and self._target_fps > 0
            else 0.0
        )

        try:
            while not self._stop.is_set():
                t0 = time.monotonic()
                ok, frame_bgr = cap.read()
                stamp = time.monotonic()

                if not ok or frame_bgr is None or getattr(frame_bgr, "size", 0) == 0:
                    consecutive_failures += 1
                    with self._state_lock:
                        self._per_cam[cam_name]["failures"] += 1
                        if consecutive_failures >= 50:
                            err = (
                                f"Camera '{cam_name}' ({cfg.device}) returned "
                                f"{consecutive_failures} consecutive empty frames"
                            )
                            self._last_error = err
                            self._per_cam[cam_name]["last_error"] = err
                            self._failed.set()
                    if consecutive_failures >= 50:
                        with self._cond:
                            self._cond.notify_all()
                        break
                    self._stop.wait(0.01)
                    continue

                consecutive_failures = 0
                transformed = _transform_bgr_frame(frame_bgr, cfg)
                payload = encode_bgr_to_jpeg(transformed, self._jpeg_quality)

                with self._state_lock:
                    self._latest[cam_name] = (payload, stamp)
                    self._per_cam[cam_name]["frames"] += 1
                    self._produced += 1
                    self._cam_ready.add(cam_name)
                    all_ready = len(self._cam_ready) == len(self._configs)

                if all_ready and not self._ready.is_set():
                    self._ready.set()
                    with self._cond:
                        self._cond.notify_all()

                # Never busy-spin on near-instantaneous synthetic captures in tests,
                # while letting blocking V4L2 capture run unthrottled at native rate.
                elapsed = time.monotonic() - t0
                if min_period > 0.0 and elapsed < min_period:
                    self._stop.wait(min_period - elapsed)
                elif elapsed < 0.002:
                    self._stop.wait(0.002 - elapsed)
        except Exception as exc:
            self._mark_failed(cam_name, f"Camera '{cam_name}' loop error: {exc}")
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    def wait_ready(self, timeout: float = 10.0) -> bool:
        """Wait until every camera delivers at least one frame or any camera fails."""
        deadline = time.monotonic() + float(timeout)
        with self._cond:
            while not self._ready.is_set() and not self._failed.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(timeout=remaining)
        return self._ready.is_set() and not self._failed.is_set()

    def latest(self) -> dict[str, tuple[bytes, float]]:
        """Return shallow copy mapping camera name -> (jpeg_bytes, monotonic_ts)."""
        with self._state_lock:
            return dict(self._latest)

    def stats(self) -> dict[str, Any]:
        """Return overall and per-camera streaming statistics."""
        now = time.monotonic()
        with self._state_lock:
            elapsed = max(1e-6, now - self._start_time) if self._start_time else 1e-6
            per_cam_stats: dict[str, dict[str, Any]] = {}
            for name, info in self._per_cam.items():
                cam_elapsed = (
                    max(1e-6, now - info["start_time"])
                    if info["start_time"]
                    else elapsed
                )
                per_cam_stats[name] = {
                    "device": info["device"],
                    "frames": info["frames"],
                    "fps": info["frames"] / cam_elapsed if info["frames"] else 0.0,
                    "failures": info["failures"],
                    "last_error": info["last_error"],
                }
            num_cams = max(1, len(self._configs))
            return {
                "produced": self._produced,
                "achieved_fps": (self._produced / num_cams) / elapsed
                if self._produced
                else 0.0,
                "last_error": self._last_error,
                "cameras": per_cam_stats,
            }

    def stop(self, timeout: float = 5.0) -> None:
        """Signal all camera threads to stop and wait up to timeout."""
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        deadline = time.monotonic() + float(timeout)
        for thread in self._threads:
            remaining = max(0.05, deadline - time.monotonic())
            thread.join(timeout=remaining)
