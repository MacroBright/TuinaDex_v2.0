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
        freq_hz: float = 30.0,
        min_cutoff: float = 1.0,
        beta: float = 0.05,
        d_cutoff: float = 1.0,
    ):
        self.n_dim = n_dim
        self.freq_hz = freq_hz
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff

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
            rate_hz = 1.0 / dt if dt > 1e-5 else self.freq_hz
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
        return self._x_filter.filter(x, alpha=alpha)


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

