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


def make_unbound_jpeg_frame(
    cam_name: str, width: int = 640, height: int = 480
) -> bytes:
    """Generate a clean dark placeholder frame with instructions."""
    img = np.full((height, width, 3), (35, 30, 25), dtype=np.uint8)
    title = f"[{cam_name.upper()}] CAMERA NOT BOUND"
    sub = "Select hardware device in Web Hub dropdown"
    cv2.putText(
        img,
        title,
        (max(20, int(width * 0.12)), int(height * 0.45)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 200, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        img,
        sub,
        (max(20, int(width * 0.08)), int(height * 0.55)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )
    return encode_bgr_to_jpeg(img, quality=80)


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
            index_file = node_dir / "index"
            card_name = ""
            if name_file.is_file():
                card_name = name_file.read_text(
                    encoding="utf-8", errors="ignore"
                ).strip()
            sysfs_index = ""
            if index_file.is_file():
                sysfs_index = index_file.read_text(
                    encoding="utf-8", errors="ignore"
                ).strip()

            # Filter out metadata nodes (index 1 on dual-interface UVC)
            if "metadata" in card_name.lower() or sysfs_index == "1":
                continue

            real = os.path.realpath(dev_node)
            by_path = by_path_map.get(real, "")
            label = f"{card_name} ({dev_node})" if card_name else dev_node
            devices.append(
                {
                    "node": dev_node,
                    "realpath": real,
                    "by_path": by_path,
                    "card_name": card_name,
                    "label": label,
                }
            )
    return devices


def save_real_camera_device(
    cam_name: str,
    device: str,
    config_path: str | Path | None = None,
) -> Path:
    """Persist camera hardware binding to real_cameras.yaml with port matching."""
    resolved_path = (
        Path(config_path) if config_path else resolve_default_camera_config_path()
    )
    if resolved_path is None:
        source_candidate = (
            Path(__file__).resolve().parents[2] / "configs" / "real_cameras.yaml"
        )
        resolved_path = source_candidate

    raw: dict[str, Any] = {}
    if resolved_path.is_file():
        raw = yaml.safe_load(resolved_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raw = {}
    cameras_map = dict(raw.get("cameras") or {})

    by_path_map: dict[str, str] = {}
    by_path_dir = Path("/dev/v4l/by-path")
    if by_path_dir.is_dir():
        for entry in sorted(by_path_dir.iterdir()):
            resolved = os.path.realpath(entry)
            if "index0" in entry.name:
                by_path_map[resolved] = str(entry)

    target_dev = device.strip()
    real_target = (
        os.path.realpath(target_dev)
        if target_dev and os.path.exists(target_dev)
        else target_dev
    )
    by_path_link = by_path_map.get(
        real_target, target_dev if "by-path" in target_dev else ""
    )

    # Clean prior camera that held this target to prevent duplicate binding
    for other_cam, other_cfg in cameras_map.items():
        if other_cam == cam_name or not isinstance(other_cfg, dict):
            continue
        cur_dev = str(other_cfg.get("device", "")).strip()
        if cur_dev and (
            cur_dev == target_dev
            or cur_dev == by_path_link
            or os.path.realpath(cur_dev) == real_target
        ):
            other_cfg["device"] = ""
            other_cfg["fallback_device"] = ""

    cam_cfg = dict(cameras_map.get(cam_name) or {})
    cam_cfg["device"] = by_path_link if by_path_link else target_dev
    cam_cfg["fallback_device"] = (
        real_target if real_target.startswith("/dev/video") else ""
    )
    cam_cfg.setdefault("width", 640)
    cam_cfg.setdefault("height", 480)
    cam_cfg.setdefault("fps", 60.0)
    cam_cfg.setdefault("fourcc", "MJPG")
    cam_cfg.setdefault("rotate_deg", 0)
    cam_cfg.setdefault("flip_h", False)
    cam_cfg.setdefault("flip_v", False)

    cameras_map[cam_name] = cam_cfg
    raw["cameras"] = cameras_map

    resolved_path.parent.mkdir(parents=True, exist_ok=True)
    resolved_path.write_text(yaml.dump(raw, sort_keys=False), encoding="utf-8")
    return resolved_path


def load_real_camera_configs(
    camera_names: Sequence[str],
    config_path: str | Path | None = None,
    overrides: dict[str, str] | None = None,
    tolerant: bool = False,
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
            if not tolerant:
                raise FileNotFoundError(
                    f"Camera config file not found: {resolved_cfg_path}"
                )
        else:
            raw = yaml.safe_load(resolved_cfg_path.read_text(encoding="utf-8")) or {}
            if not isinstance(raw, dict):
                raise ValueError(f"Expected mapping in {resolved_cfg_path}")
            yaml_cameras = dict(raw.get("cameras") or {})

    configs: list[RealCameraConfig] = []
    seen_targets: dict[str, str] = {}

    for cam_name in camera_names:
        entry = dict(yaml_cameras.get(cam_name) or {})
        device = str(overrides.get(cam_name, entry.get("device", ""))).strip()
        if not device and not tolerant:
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

        if device:
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
    if not cfg.device:
        raise ValueError(f"No device configured for camera '{cfg.name}'")
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
    """Concurrent off-loop multi-camera reader with dynamic device rebinding.

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
        self._configs_map: dict[str, RealCameraConfig] = {
            cfg.name: cfg for cfg in configs
        }
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
        self._cam_threads: dict[str, threading.Thread] = {}
        self._cam_stop_events: dict[str, threading.Event] = {}
        self._started = False
        self._start_time = 0.0
        self._produced = 0
        self._last_error: str | None = None
        self._per_cam: dict[str, dict[str, Any]] = {}

        now = time.monotonic()
        for cfg in configs:
            bound = bool(
                cfg.device
                and (
                    os.path.exists(cfg.device)
                    or self._capture_factory is not _default_opencv_capture
                )
            )
            self._per_cam[cfg.name] = {
                "device": cfg.device,
                "frames": 0,
                "failures": 0,
                "last_error": None if bound else "Not bound or device missing",
                "start_time": 0.0,
                "bound": bound,
            }
            # Guarantee every configured camera key exists immediately
            self._latest[cfg.name] = (
                make_unbound_jpeg_frame(cfg.name, cfg.width, cfg.height),
                now,
            )

    def start(self) -> None:
        """Spawn one background capture thread per configured camera."""
        if self._started:
            return
        self._started = True
        self._start_time = time.monotonic()
        for cfg in self._configs_map.values():
            self._start_camera_thread(cfg)

    def _start_camera_thread(self, cfg: RealCameraConfig) -> None:
        cam_name = cfg.name
        cam_stop = threading.Event()
        self._cam_stop_events[cam_name] = cam_stop
        thread = threading.Thread(
            target=self._camera_loop,
            args=(cfg, cam_stop),
            name=f"tuinadex-cam-{cam_name}",
            daemon=True,
        )
        self._cam_threads[cam_name] = thread
        thread.start()

    def bind_camera(self, cam_name: str, device: str) -> bool:
        """Dynamically rebind a camera to a new physical device without restarting."""
        if cam_name not in self._configs_map:
            return False

        old_stop = self._cam_stop_events.get(cam_name)
        if old_stop is not None:
            old_stop.set()
        old_thread = self._cam_threads.get(cam_name)
        if old_thread is not None and old_thread.is_alive():
            old_thread.join(timeout=1.0)

        old_cfg = self._configs_map[cam_name]
        clean_dev = device.strip()
        new_cfg = RealCameraConfig(
            name=cam_name,
            device=clean_dev,
            width=old_cfg.width,
            height=old_cfg.height,
            fps=old_cfg.fps,
            fourcc=old_cfg.fourcc,
            rotate_deg=old_cfg.rotate_deg,
            flip_h=old_cfg.flip_h,
            flip_v=old_cfg.flip_v,
            buffersize=old_cfg.buffersize,
        )
        self._configs_map[cam_name] = new_cfg

        with self._state_lock:
            self._cam_ready.discard(cam_name)
            self._per_cam[cam_name]["device"] = clean_dev
            self._per_cam[cam_name]["frames"] = 0
            self._per_cam[cam_name]["failures"] = 0
            self._per_cam[cam_name]["last_error"] = None
            self._failed.clear()
            if not clean_dev:
                self._per_cam[cam_name]["bound"] = False
                self._latest[cam_name] = (
                    make_unbound_jpeg_frame(cam_name, old_cfg.width, old_cfg.height),
                    time.monotonic(),
                )

        if not clean_dev:
            return True

        if self._started:
            self._start_camera_thread(new_cfg)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                with self._state_lock:
                    if cam_name in self._cam_ready:
                        return True
                time.sleep(0.05)
            with self._state_lock:
                return cam_name in self._cam_ready
        return True

    def get_bindings(self) -> dict[str, str]:
        """Return mapping of camera name -> current physical device."""
        with self._state_lock:
            return {
                name: str(info.get("device", ""))
                for name, info in self._per_cam.items()
            }

    def is_bound(self, cam_name: str) -> bool:
        """Return True if the specified camera is configured and actively bound."""
        with self._state_lock:
            info = self._per_cam.get(cam_name)
            return bool(info and info.get("bound", False))

    def is_all_bound(self) -> bool:
        """Return True only if all configured camera slots have active bound devices."""
        with self._state_lock:
            return all(
                info.get("bound", False) for info in self._per_cam.values()
            )

    def _mark_failed(self, cam_name: str, msg: str) -> None:
        with self._state_lock:
            self._last_error = msg
            self._per_cam[cam_name]["last_error"] = msg
            self._per_cam[cam_name]["bound"] = False
            self._failed.set()
        with self._cond:
            self._cond.notify_all()

    def _camera_loop(
        self, cfg: RealCameraConfig, cam_stop: threading.Event
    ) -> None:
        cam_name = cfg.name
        with self._state_lock:
            self._per_cam[cam_name]["start_time"] = time.monotonic()

        if not cfg.device:
            with self._state_lock:
                self._per_cam[cam_name]["bound"] = False
            while not self._stop.is_set() and not cam_stop.is_set():
                cam_stop.wait(0.2)
            return

        cap = None
        try:
            cap = self._capture_factory(cfg)
            if cap is None or (hasattr(cap, "isOpened") and not cap.isOpened()):
                raise RuntimeError(
                    f"Camera '{cam_name}' failed to open device {cfg.device}"
                )
        except Exception as exc:
            with self._state_lock:
                self._per_cam[cam_name]["last_error"] = str(exc)
                self._per_cam[cam_name]["bound"] = False
                self._latest[cam_name] = (
                    make_unbound_jpeg_frame(cam_name, cfg.width, cfg.height),
                    time.monotonic(),
                )
            self._mark_failed(cam_name, str(exc))
            while not self._stop.is_set() and not cam_stop.is_set():
                cam_stop.wait(0.5)
            return

        consecutive_failures = 0
        min_period = (
            1.0 / self._target_fps
            if self._capture_factory is not _default_opencv_capture
            and self._target_fps > 0
            else 0.0
        )

        try:
            while not self._stop.is_set() and not cam_stop.is_set():
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
                            self._per_cam[cam_name]["bound"] = False
                    if consecutive_failures >= 50:
                        with self._cond:
                            self._cond.notify_all()
                        break
                    cam_stop.wait(0.01)
                    continue

                consecutive_failures = 0
                transformed = _transform_bgr_frame(frame_bgr, cfg)
                payload = encode_bgr_to_jpeg(transformed, self._jpeg_quality)

                with self._state_lock:
                    self._latest[cam_name] = (payload, stamp)
                    self._per_cam[cam_name]["frames"] += 1
                    self._per_cam[cam_name]["bound"] = True
                    self._produced += 1
                    self._cam_ready.add(cam_name)
                    all_ready = len(self._cam_ready) == len(self._configs_map)

                if all_ready and not self._ready.is_set():
                    self._ready.set()
                    with self._cond:
                        self._cond.notify_all()

                elapsed = time.monotonic() - t0
                if min_period > 0.0 and elapsed < min_period:
                    cam_stop.wait(min_period - elapsed)
                elif elapsed < 0.002:
                    cam_stop.wait(0.002 - elapsed)
        except Exception as exc:
            self._mark_failed(cam_name, f"Camera '{cam_name}' loop error: {exc}")
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    def wait_ready(self, timeout: float = 10.0) -> bool:
        """Wait until every camera delivers at least one frame or timeout."""
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
                    "bound": info.get("bound", False),
                }
            num_cams = max(1, len(self._configs_map))
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
        for cam_stop in self._cam_stop_events.values():
            cam_stop.set()
        with self._cond:
            self._cond.notify_all()
        deadline = time.monotonic() + float(timeout)
        for thread in self._cam_threads.values():
            remaining = max(0.05, deadline - time.monotonic())
            thread.join(timeout=remaining)
