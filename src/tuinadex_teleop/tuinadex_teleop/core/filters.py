"""High-performance temporal filtering utilities: One-Euro Filter & Quaternion SLERP."""
from __future__ import annotations

import math
from typing import Optional, Union
import numpy as np


class LowPassFilter:
    """First-order low-pass filter."""

    def __init__(self, alpha: float = 1.0):
        self._alpha = alpha
        self._y: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._y = None

    def filter(self, x: np.ndarray, alpha: Optional[float] = None) -> np.ndarray:
        if alpha is not None:
            self._alpha = alpha
        if self._y is None:
            self._y = np.array(x, dtype=np.float64, copy=True)
        else:
            self._y = self._alpha * x + (1.0 - self._alpha) * self._y
        return self._y

    @property
    def last_value(self) -> Optional[np.ndarray]:
        return self._y


class OneEuroFilter:
    """1€ Filter for adaptive jitter reduction and lag compensation.
    
    Casiez et al., CHI 2012: '1 € Filter: A Simple Speed-based Low-pass Filter for
    Noisy Input in Human-Computer Interfaces'.
    """

    def __init__(
        self,
        n_dim: int = 3,
        freq_hz: float = 60.0,
        min_cutoff: float = 0.8,
        beta: float = 0.06,
        d_cutoff: float = 1.2,
        deadband: float = 0.0,
    ):
        self.n_dim = n_dim
        self.freq_hz = freq_hz
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.deadband = deadband

        self._x_filter = LowPassFilter()
        self._dx_filter = LowPassFilter()
        self._last_time: Optional[float] = None

    def reset(self) -> None:
        self._x_filter.reset()
        self._dx_filter.reset()
        self._last_time = None

    def _calc_alpha(self, rate_hz: float, cutoff: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        te = 1.0 / rate_hz
        return 1.0 / (1.0 + tau / te)

    def filter(self, x: np.ndarray, timestamp: Optional[float] = None) -> np.ndarray:
        """Filter input array x with optional timestamp."""
        x = np.asarray(x, dtype=np.float64)
        if self._last_time is None or timestamp is None:
            rate_hz = self.freq_hz
        else:
            dt = timestamp - self._last_time
            rate_hz = np.clip(1.0 / dt, 1.0, 240.0) if dt > 1e-5 else self.freq_hz
        self._last_time = timestamp

        # Estimate derivative dx
        prev_x = self._x_filter.last_value
        if prev_x is None:
            dx = np.zeros_like(x)
        else:
            dx = (x - prev_x) * rate_hz

        alpha_d = self._calc_alpha(rate_hz, self.d_cutoff)
        edx = self._dx_filter.filter(dx, alpha=alpha_d)

        # Dynamic cutoff frequency based on speed
        cutoff = self.min_cutoff + self.beta * np.abs(edx)
        alpha = self._calc_alpha(rate_hz, cutoff)
        filtered = self._x_filter.filter(x, alpha=alpha)

        # Deadband threshold for micro-jitter suppression during static rests
        if prev_x is not None and self.deadband > 0.0:
            delta = np.abs(filtered - prev_x)
            filtered = np.where(delta < self.deadband, prev_x, filtered)
            self._x_filter._y = filtered.copy()

        return filtered


class LandmarkOneEuroFilter:
    """Specialized 1€ Filter for (N, 3) 3D Hand Landmarks with outlier velocity clamping."""

    def __init__(
        self,
        num_landmarks: int = 21,
        freq_hz: float = 60.0,
        min_cutoff: float = 1.2,
        beta: float = 0.03,
        d_cutoff: float = 1.2,
        max_jump_m: float = 0.12,
    ):
        self.num_landmarks = num_landmarks
        self.max_jump_m = max_jump_m
        self._filter = OneEuroFilter(
            n_dim=num_landmarks * 3,
            freq_hz=freq_hz,
            min_cutoff=min_cutoff,
            beta=beta,
            d_cutoff=d_cutoff,
        )
        self._last_pts: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._filter.reset()
        self._last_pts = None

    def filter(self, pts: np.ndarray, timestamp: Optional[float] = None) -> np.ndarray:
        pts = np.asarray(pts, dtype=np.float64)
        orig_shape = pts.shape
        flat = pts.reshape(-1)

        # Outlier rejection: if a single point jumps > max_jump_m, clamp to previous point + max_jump
        if self._last_pts is not None and self.max_jump_m > 0.0:
            diff = pts - self._last_pts
            dist = np.linalg.norm(diff, axis=-1, keepdims=True)
            scale = np.where(dist > self.max_jump_m, self.max_jump_m / np.maximum(dist, 1e-6), 1.0)
            pts_clamped = self._last_pts + diff * scale
            flat = pts_clamped.reshape(-1)

        filtered_flat = self._filter.filter(flat, timestamp=timestamp)
        filtered_pts = filtered_flat.reshape(orig_shape)
        self._last_pts = filtered_pts.copy()
        return filtered_pts


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions [w, x, y, z]."""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)

    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        res = q0 + t * (q1 - q0)
        return res / np.linalg.norm(res)

    theta_0 = math.acos(dot)
    theta = theta_0 * t

    sin_theta_0 = math.sin(theta_0)
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    return (s0 * q0) + (s1 * q1)

