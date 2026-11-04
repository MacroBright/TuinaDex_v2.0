"""realsense_stream.py — RealSense D455 RGB-D streaming and camera intrinsics abstraction."""
from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time
from typing import Any, Optional, Tuple
import numpy as np

try:
    import pyrealsense2 as rs
    _HAS_PYREALSENSE2 = hasattr(rs, "pipeline")
except ImportError:
    import os
    import sys
    _candidate = os.path.expanduser(
        "~/win_office/conda/envs/leap_hand/lib/python3.14/site-packages"
    )
    if os.path.isdir(_candidate) and _candidate not in sys.path:
        sys.path.append(_candidate)
    try:
        import pyrealsense2 as rs
        _HAS_PYREALSENSE2 = hasattr(rs, "pipeline")
    except ImportError:
        _HAS_PYREALSENSE2 = False


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsic parameters."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    coeffs: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])

    @property
    def matrix(self) -> np.ndarray:
        """3x3 camera calibration matrix K."""
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def deproject_pixel_to_point(self, u: float, v: float, depth_m: float) -> np.ndarray:
        """Back-project 2D pixel (u, v) and depth into 3D camera coordinates (X, Y, Z)."""
        if depth_m <= 0.0:
            return np.zeros(3, dtype=np.float64)
        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        return np.array([x, y, depth_m], dtype=np.float64)


class AsyncFrameBuffer:
    """Thread-safe latest-frame slot: capture thread writes, control loop reads (I38)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: Tuple[Optional[np.ndarray], Optional[np.ndarray], float, int] = (
            None,
            None,
            0.0,
            0,
        )
        self._captured = 0
        self._dropped = 0

    def publish(
        self,
        color: np.ndarray,
        depth: np.ndarray,
        timestamp: Optional[float] = None,
    ) -> int:
        """Replace the latest frame (latest-wins, no queue growth)."""
        ts = time.monotonic() if timestamp is None else float(timestamp)
        with self._lock:
            seq = self._latest[3] + 1
            self._latest = (color, depth, ts, seq)
            self._captured += 1
            return seq

    def drop(self) -> None:
        with self._lock:
            self._dropped += 1

    def latest(
        self,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float, int]:
        """Snapshot of (color, depth_m, timestamp, seq); seq==0 means no frame yet."""
        with self._lock:
            return self._latest

    def stats(self) -> dict:
        with self._lock:
            return {
                "captured": self._captured,
                "dropped": self._dropped,
                "seq": self._latest[3],
            }


def run_capture_loop(
    pipeline: Any,
    align: Any,
    depth_scale: float,
    buffer: AsyncFrameBuffer,
    stop_event: threading.Event,
    timeout_ms: int = 1000,
) -> None:
    """Continuously pull frames off the driver; never blocks the control loop (I38)."""
    while not stop_event.is_set():
        try:
            frames = pipeline.wait_for_frames(timeout_ms=timeout_ms)
        except RuntimeError:
            buffer.drop()
            time.sleep(0.001)
            continue
        except Exception:
            buffer.drop()
            time.sleep(0.005)
            continue

        try:
            if align is not None:
                frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                buffer.drop()
                continue
            color = np.asanyarray(color_frame.get_data())
            depth = (
                np.asanyarray(depth_frame.get_data()).astype(np.float32)
                * float(depth_scale)
            )
            buffer.publish(color, depth, time.monotonic())
        except Exception:
            buffer.drop()
            time.sleep(0.005)


class RealSenseStream:
    """Hardware camera pipeline for Intel RealSense D455 with aligned color and depth."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: Optional[int] = None,
        align_depth: bool = True,
        device_id: Optional[str] = None,
    ):
        if not _HAS_PYREALSENSE2:
            raise RuntimeError("pyrealsense2 is not installed or available.")

        self.width = width
        self.height = height
        self.requested_fps = int(fps) if fps is not None else 0
        self.fps = int(fps) if fps is not None else self.detect_usb_fps()
        self.align_depth = align_depth
        self.device_id = device_id

        self._buffer = AsyncFrameBuffer()
        self._stop_evt = threading.Event()
        self._worker: Optional[threading.Thread] = None

        self._pipeline = rs.pipeline()
        self._config = rs.config()
        if device_id:
            self._config.enable_device(str(device_id))
        self._config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, self.fps)
        self._config.enable_stream(rs.stream.depth, width, height, rs.format.z16, self.fps)
        self._align = rs.align(rs.stream.color) if align_depth else None

        self._is_running = False
        self._depth_scale = 0.001  # Default fallback: 1 unit = 1mm
        self._intrinsics: Optional[CameraIntrinsics] = None

    @staticmethod
    def detect_usb_fps(default_fps: int = 30, usb3_fps: int = 60) -> int:
        """Pick 60 fps only on a USB 3.x link; USB 2.1 caps the D455 at 30 fps."""
        if not _HAS_PYREALSENSE2:
            return default_fps
        try:
            devices = rs.context().query_devices()
            if devices.size() == 0:
                return default_fps
            usb_type = str(devices[0].get_info(rs.camera_info.usb_type_descriptor))
        except Exception:
            return default_fps
        return usb3_fps if usb_type.startswith("3") else default_fps

    def _capture_loop(self) -> None:
        """Continuously pull frames off the driver; never blocks the control loop (I38)."""
        run_capture_loop(
            self._pipeline,
            self._align,
            self._depth_scale,
            self._buffer,
            self._stop_evt,
        )

    def start(self, timeout_ms: int = 5000) -> CameraIntrinsics:
        """Start streaming and extract factory calibration intrinsics."""
        if self._is_running:
            return self.get_intrinsics()

        try:
            profile = self._pipeline.start(self._config)
            self._is_running = True

            # Retrieve depth scale from sensor
            depth_sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = float(depth_sensor.get_depth_scale())

            # Retrieve stream intrinsics
            color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = color_stream.get_intrinsics()
            self._intrinsics = CameraIntrinsics(
                fx=float(intr.fx),
                fy=float(intr.fy),
                cx=float(intr.ppx),
                cy=float(intr.ppy),
                width=int(intr.width),
                height=int(intr.height),
                coeffs=list(intr.coeffs),
            )

            # Start the async producer; the control loop never blocks on frames (I38).
            self._stop_evt.clear()
            self._worker = threading.Thread(
                target=self._capture_loop, name="realsense-capture", daemon=True
            )
            self._worker.start()

            deadline = time.monotonic() + max(0.0, timeout_ms / 1000.0)
            while self._buffer.latest()[3] == 0 and time.monotonic() < deadline:
                time.sleep(0.005)

            return self._intrinsics
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        """Stop the capture thread and streaming pipeline."""
        self._stop_evt.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
            self._worker = None
        if self._is_running:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._is_running = False

    @property
    def is_running(self) -> bool:
        return self._is_running

    def get_intrinsics(self) -> CameraIntrinsics:
        if self._intrinsics is None:
            raise RuntimeError("Stream not started; intrinsics unavailable.")
        return self._intrinsics

    def get_depth_scale(self) -> float:
        return self._depth_scale

    def read_frames(self, timeout_ms: int = 2000) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float]:
        """Fetch latest synchronized, aligned (color_bgr, depth_m, timestamp).
        
        Returns:
            color_image: np.ndarray shape (H, W, 3), uint8 (BGR)
            depth_image_m: np.ndarray shape (H, W), float32 (meters)
            timestamp: float (monotonic time)
        """
        if not self._is_running:
            return None, None, time.time()

        color, depth, timestamp, seq = self._buffer.latest()
        if seq == 0:
            return None, None, time.monotonic()
        return color, depth, timestamp

    def latest_seq(self) -> int:
        """Monotonic sequence number of the newest captured frame."""
        return self._buffer.latest()[3]

    def stats(self) -> dict:
        """Capture-thread counters for HUD and diagnostics."""
        data = self._buffer.stats()
        data.update({"requested_fps": self.requested_fps, "fps": self.fps})
        return data


class MockRealSenseStream:
    """Mock synthetic stream for offline CI and automated unit testing."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        device_id: Optional[str] = None,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.device_id = device_id
        self._is_running = False
        self._seq = 0
        self._intrinsics = CameraIntrinsics(
            fx=381.27,
            fy=380.95,
            cx=312.91,
            cy=238.57,
            width=width,
            height=height,
        )
        self._depth_scale = 0.001

    def start(self, timeout_ms: int = 5000) -> CameraIntrinsics:
        self._is_running = True
        return self._intrinsics

    def stop(self) -> None:
        self._is_running = False

    @property
    def is_running(self) -> bool:
        return self._is_running

    def get_intrinsics(self) -> CameraIntrinsics:
        return self._intrinsics

    def get_depth_scale(self) -> float:
        return self._depth_scale

    def read_frames(self, timeout_ms: int = 2000) -> Tuple[np.ndarray, np.ndarray, float]:
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # Default mock plane at 0.65m distance
        depth_m = np.full((self.height, self.width), 0.65, dtype=np.float32)
        self._seq += 1
        return color, depth_m, time.monotonic()

    def latest_seq(self) -> int:
        """Sequence number of the most recently generated mock frame."""
        return self._seq

    def read_latest(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, float, int]:
        """Non-blocking latest-frame accessor mirroring RealSenseStream."""
        color, depth, timestamp = self.read_frames()
        return color, depth, timestamp, self._seq

