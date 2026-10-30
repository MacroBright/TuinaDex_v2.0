"""realsense_stream.py — RealSense D455 RGB-D streaming and camera intrinsics abstraction."""
from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Optional, Tuple
import numpy as np

try:
    import pyrealsense2 as rs
    _HAS_PYREALSENSE2 = True
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
        _HAS_PYREALSENSE2 = True
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


class RealSenseStream:
    """Hardware camera pipeline for Intel RealSense D455 with aligned color and depth."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        align_depth: bool = True,
        device_id: Optional[str] = None,
    ):
        if not _HAS_PYREALSENSE2:
            raise RuntimeError("pyrealsense2 is not installed or available.")

        self.width = width
        self.height = height
        self.fps = fps
        self.align_depth = align_depth
        self.device_id = device_id

        self._pipeline = rs.pipeline()
        self._config = rs.config()
        if device_id:
            self._config.enable_device(str(device_id))
        self._config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self._config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self._align = rs.align(rs.stream.color) if align_depth else None

        self._is_running = False
        self._depth_scale = 0.001  # Default fallback: 1 unit = 1mm
        self._intrinsics: Optional[CameraIntrinsics] = None

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

            # Warm-up auto-exposure (read first few frames safely, non-fatal on drops)
            for _ in range(5):
                try:
                    self._pipeline.wait_for_frames(timeout_ms=1000)
                except Exception:
                    pass

            return self._intrinsics
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        """Stop streaming pipeline."""
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

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=timeout_ms)
        except RuntimeError:
            # Temporary timeout or dropped frame (common on USB 2.0 buses)
            return None, None, time.time()

        ts = time.time()
        if self._align is not None:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None, ts

        color_image = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_image_m = depth_raw.astype(np.float32) * self._depth_scale

        return color_image, depth_image_m, ts


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
        return color, depth_m, time.time()

