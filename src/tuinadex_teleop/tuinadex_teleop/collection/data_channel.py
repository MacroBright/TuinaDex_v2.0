"""Unified simulation/real data-source contracts."""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable

import numpy as np

from ..cameras.sim_camera_rig import decode_jpeg_to_rgb, encode_rgb_to_jpeg

DEFAULT_CAMERAS = ("overhead", "wrist", "side")


@dataclass(frozen=True)
class CameraPacket:
    """One camera snapshot with local monotonic observation times."""

    jpeg: dict[str, bytes]
    timestamps: dict[str, float]

    def ages(self, now: float | None = None) -> dict[str, float]:
        """Return per-camera packet age in seconds."""
        current = time.monotonic() if now is None else now
        return {
            cam: current - timestamp
            for cam, timestamp in self.timestamps.items()
        }

    def worst_age(self, now: float | None = None) -> float:
        """Return the oldest camera packet age, or infinity when empty."""
        ages = self.ages(now)
        return max(ages.values()) if ages else float("inf")


class BaseDataChannel(ABC):
    """Data-source boundary shared by simulation and real hardware."""

    camera_names: tuple[str, ...] = DEFAULT_CAMERAS

    @abstractmethod
    def get_camera_packets(self) -> CameraPacket:
        """Return latest encoded images and their capture timestamps."""

    @abstractmethod
    def get_proprioception(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (q[22] rad, effort[22], contact_forces[4] N)."""

    @abstractmethod
    def get_action_label(self) -> np.ndarray:
        """Return the 22D absolute target-position label in radians."""

    def get_camera_frames(self) -> dict[str, np.ndarray]:
        """Decode camera bytes for preview and debugging only."""
        return {
            cam: decode_jpeg_to_rgb(payload)
            for cam, payload in self.get_camera_packets().jpeg.items()
        }

    def close(self) -> None:
        """Release channel-owned resources."""
        return None


class SimDataChannel(BaseDataChannel):
    """Camera packets from a producer thread and state from MuJoCo."""

    def __init__(
        self,
        adapter,
        producer,
        action_supplier: Callable[[], np.ndarray],
        camera_names: tuple[str, ...] = DEFAULT_CAMERAS,
    ):
        self._adapter = adapter
        self._producer = producer
        self._action_supplier = action_supplier
        self.camera_names = tuple(camera_names)

    def get_camera_packets(self) -> CameraPacket:
        latest = self._producer.latest()
        selected = {
            cam: packet
            for cam, packet in latest.items()
            if cam in self.camera_names
        }
        return CameraPacket(
            jpeg={
                cam: payload
                for cam, (payload, _timestamp) in selected.items()
            },
            timestamps={
                cam: timestamp
                for cam, (_payload, timestamp) in selected.items()
            },
        )

    def get_proprioception(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        observation = self._adapter.read_observation()
        forces = self._adapter.get_contact_forces()
        q = np.asarray(observation.composite_state, dtype=np.float32)
        effort = np.concatenate(
            [
                np.asarray(observation.arm_effort, dtype=np.float32),
                np.asarray(observation.hand_current, dtype=np.float32),
            ]
        )
        contact_forces = np.array(
            [
                forces.get("index", 0.0),
                forces.get("middle", 0.0),
                forces.get("ring", 0.0),
                forces.get("thumb", 0.0),
            ],
            dtype=np.float32,
        )
        return q, effort, contact_forces

    def get_action_label(self) -> np.ndarray:
        return np.asarray(self._action_supplier(), dtype=np.float32).ravel()


class RealDataChannel(BaseDataChannel):
    """ROS2 observations with JPEG caching for unchanged camera arrays.

    The LEAP Hand has no fingertip FSR, so contact forces remain zero rather
    than being estimated.
    """

    def __init__(
        self,
        env,
        action_supplier: Callable[[], np.ndarray],
        camera_names: tuple[str, ...] = DEFAULT_CAMERAS,
        jpeg_quality: int = 90,
    ):
        self._env = env
        self._action_supplier = action_supplier
        self._jpeg_quality = int(jpeg_quality)
        self.camera_names = tuple(camera_names)
        self._cache: dict[str, tuple[np.ndarray, bytes]] = {}
        self._stamps: dict[str, float] = {}

    def get_camera_packets(self) -> CameraPacket:
        observation = self._env.get_observation()
        images = observation.get("images", {})
        now = time.monotonic()
        jpeg: dict[str, bytes] = {}
        timestamps: dict[str, float] = {}
        for cam in self.camera_names:
            image = images.get(cam)
            if image is None:
                continue
            cached = self._cache.get(cam)
            if cached is not None and cached[0] is image:
                jpeg[cam] = cached[1]
                timestamps[cam] = self._stamps[cam]
            else:
                payload = encode_rgb_to_jpeg(image, self._jpeg_quality)
                self._cache[cam] = (image, payload)
                self._stamps[cam] = now
                jpeg[cam] = payload
                timestamps[cam] = now
        return CameraPacket(jpeg=jpeg, timestamps=timestamps)

    def get_proprioception(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        observation = self._env.get_observation()
        q = np.asarray(observation["state"], dtype=np.float32)
        effort = np.asarray(
            observation.get("effort", np.zeros_like(q)), dtype=np.float32
        )
        forces = np.asarray(
            observation.get("contact_forces", np.zeros(4, dtype=np.float32)),
            dtype=np.float32,
        )
        return q, effort, forces

    def get_action_label(self) -> np.ndarray:
        return np.asarray(self._action_supplier(), dtype=np.float32).ravel()

    def close(self) -> None:
        self._cache.clear()
        self._stamps.clear()


class MockDataChannel(BaseDataChannel):
    """Deterministic synthetic channel for Hub API/frontend tests only."""

    def __init__(
        self,
        camera_names: tuple[str, ...] = DEFAULT_CAMERAS,
        shape: tuple[int, int, int] = (480, 640, 3),
        dof: int = 22,
    ):
        self.camera_names = tuple(camera_names)
        self._shape = shape
        self._dof = int(dof)
        self._frames = {
            cam: encode_rgb_to_jpeg(
                np.full(shape, 40 + 60 * i, dtype=np.uint8)
            )
            for i, cam in enumerate(self.camera_names)
        }
        self._t0 = time.monotonic()

    def get_camera_packets(self) -> CameraPacket:
        now = time.monotonic()
        return CameraPacket(
            jpeg=dict(self._frames),
            timestamps={cam: now for cam in self._frames},
        )

    def get_proprioception(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        phase = (time.monotonic() - self._t0) * 2.0
        q = (np.sin(phase) * 0.2).astype(np.float32) * np.ones(
            self._dof, dtype=np.float32
        )
        effort = (np.cos(phase) * 0.05).astype(np.float32) * np.ones(
            self._dof, dtype=np.float32
        )
        forces = np.full(
            4, max(0.0, np.sin(phase)) * 2.0, dtype=np.float32
        )
        return q, effort, forces

    def get_action_label(self) -> np.ndarray:
        return self.get_proprioception()[0].copy()
