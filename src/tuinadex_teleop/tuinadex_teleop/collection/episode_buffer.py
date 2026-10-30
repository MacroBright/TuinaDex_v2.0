"""Multimodal episode memory buffer with JPEG image storage."""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Iterator, Sequence

import numpy as np

from ..cameras.sim_camera_rig import decode_jpeg_to_rgb, encode_rgb_to_jpeg

DEFAULT_MAX_EPISODE_BYTES = 512 * 1024 * 1024
DEFAULT_MAX_EPISODE_FRAMES = 3000


class BufferCapacityError(RuntimeError):
    """Raised when a single episode exceeds its memory limits."""


@dataclass
class FrameData:
    """One multimodal sample in an episode."""

    state: np.ndarray
    action: np.ndarray
    effort: np.ndarray
    contact_forces: np.ndarray
    images_jpeg: dict[str, bytes]
    image_timestamps: dict[str, float]
    timestamp: float
    frame_index: int
    episode_index: int
    next_done: bool
    phase_label: str
    language_instruction: str

    def decode_image(self, cam: str) -> np.ndarray:
        """Decode one camera's JPEG payload to RGB uint8."""
        return decode_jpeg_to_rgb(self.images_jpeg[cam])


class EpisodeBuffer:
    """In-memory episode buffer backed by compressed JPEG images.

    ``add_frame`` accepts both RGB ndarrays and already-encoded JPEG bytes.
    """

    def __init__(
        self,
        episode_index: int = 0,
        language_instruction: str = "",
        max_episode_bytes: int = DEFAULT_MAX_EPISODE_BYTES,
        max_episode_frames: int = DEFAULT_MAX_EPISODE_FRAMES,
    ):
        self.episode_index = int(episode_index)
        self.language_instruction = str(language_instruction)
        self.max_episode_bytes = int(max_episode_bytes)
        self.max_episode_frames = int(max_episode_frames)
        self._frames: list[FrameData] = []
        self._image_bytes = 0

    def __len__(self) -> int:
        return len(self._frames)

    @property
    def image_bytes(self) -> int:
        """Total encoded image bytes currently held by this episode."""
        return self._image_bytes

    @property
    def frames(self) -> list[FrameData]:
        """Frames in insertion order for legacy callers."""
        return self._frames

    def iter_frames(self) -> Iterator[FrameData]:
        """Iterate frames without eagerly decoding all stored images."""
        return iter(self._frames)

    def add_frame(
        self,
        state: np.ndarray | Sequence[float],
        action: np.ndarray | Sequence[float],
        images: dict[str, np.ndarray | bytes],
        timestamp: float,
        *,
        effort: np.ndarray | Sequence[float] | None = None,
        contact_forces: np.ndarray | Sequence[float] | None = None,
        image_timestamps: dict[str, float] | None = None,
        phase_label: str = "",
        language_instruction: str | None = None,
    ) -> None:
        """Append and validate one sample without retaining image arrays."""
        if len(self._frames) >= self.max_episode_frames:
            raise BufferCapacityError(
                f"frame limit reached: {self.max_episode_frames} frames"
            )

        s = np.asarray(state, dtype=np.float32).ravel()
        a = np.asarray(action, dtype=np.float32).ravel()
        if s.shape != a.shape:
            raise ValueError(
                "state and action must have same dimension, "
                f"got state={s.shape}, action={a.shape}"
            )
        e = (
            np.zeros_like(s, dtype=np.float32)
            if effort is None
            else np.asarray(effort, dtype=np.float32).ravel()
        )
        if e.shape != s.shape:
            raise ValueError(
                f"effort must match state dim {s.shape}, got {e.shape}"
            )
        f = (
            np.zeros(4, dtype=np.float32)
            if contact_forces is None
            else np.asarray(contact_forces, dtype=np.float32).ravel()
        )
        if f.shape != (4,):
            raise ValueError(f"contact_forces must be (4,), got {f.shape}")

        jpeg: dict[str, bytes] = {}
        for cam, payload in images.items():
            if isinstance(payload, (bytes, bytearray)):
                jpeg[cam] = bytes(payload)
            else:
                jpeg[cam] = encode_rgb_to_jpeg(payload)

        new_bytes = sum(len(payload) for payload in jpeg.values())
        if self._image_bytes + new_bytes > self.max_episode_bytes:
            raise BufferCapacityError(
                "byte limit reached: "
                f"{self._image_bytes + new_bytes} > {self.max_episode_bytes}"
            )

        ts0 = float(timestamp)
        ts_map = {cam: ts0 for cam in jpeg}
        if image_timestamps:
            ts_map.update(
                {
                    cam: float(ts)
                    for cam, ts in image_timestamps.items()
                    if cam in jpeg
                }
            )

        self._frames.append(
            FrameData(
                state=s,
                action=a,
                effort=e,
                contact_forces=f,
                images_jpeg=jpeg,
                image_timestamps=ts_map,
                timestamp=ts0,
                frame_index=len(self._frames),
                episode_index=self.episode_index,
                next_done=False,
                phase_label=str(phase_label),
                language_instruction=(
                    self.language_instruction
                    if language_instruction is None
                    else str(language_instruction)
                ),
            )
        )
        self._image_bytes += new_bytes

    def seal_episode(self) -> None:
        """Mark the last frame as terminal."""
        if not self._frames:
            raise ValueError("Cannot seal an empty episode")
        self._frames[-1] = replace(self._frames[-1], next_done=True)

    def validate_continuity(self, max_gap_s: float = 0.1) -> list[str]:
        """Return continuity defects; an empty list means validation passed."""
        issues: list[str] = []
        for i, frame in enumerate(self._frames):
            if frame.frame_index != i:
                issues.append(f"frame_index {frame.frame_index} != position {i}")
            if not np.isfinite(frame.state).all():
                issues.append(f"non-finite state at frame {i}")
            if i > 0:
                prev = self._frames[i - 1].timestamp
                if frame.timestamp <= prev:
                    issues.append(
                        "non-monotonic timestamp at frame "
                        f"{i}: {frame.timestamp} <= {prev}"
                    )
                elif frame.timestamp - prev > max_gap_s:
                    issues.append(
                        f"gap {frame.timestamp - prev:.3f}s > "
                        f"{max_gap_s}s at frame {i}"
                    )
        return issues

    def clear(self) -> None:
        """Discard all frames and release their stored byte count."""
        self._frames.clear()
        self._image_bytes = 0

    def to_dict(self) -> dict[str, Any]:
        """Aggregate frames for debugging, including legacy array keys."""
        if not self._frames:
            return {
                "state": np.empty((0, 0), dtype=np.float32),
                "action": np.empty((0, 0), dtype=np.float32),
                "effort": np.empty((0, 0), dtype=np.float32),
                "contact_forces": np.empty((0, 4), dtype=np.float32),
                "timestamp": np.empty((0,), dtype=np.float32),
                "frame_index": np.empty((0,), dtype=np.int64),
                "episode_index": np.empty((0,), dtype=np.int64),
                "next_done": np.empty((0,), dtype=bool),
                "phase_label": [],
                "language_instruction": self.language_instruction,
                "images": {},
            }

        cams = list(self._frames[0].images_jpeg)
        return {
            "state": np.stack([frame.state for frame in self._frames]),
            "action": np.stack([frame.action for frame in self._frames]),
            "effort": np.stack([frame.effort for frame in self._frames]),
            "contact_forces": np.stack(
                [frame.contact_forces for frame in self._frames]
            ),
            "timestamp": np.array(
                [frame.timestamp for frame in self._frames], dtype=np.float32
            ),
            "frame_index": np.array(
                [frame.frame_index for frame in self._frames], dtype=np.int64
            ),
            "episode_index": np.array(
                [frame.episode_index for frame in self._frames], dtype=np.int64
            ),
            "next_done": np.array(
                [frame.next_done for frame in self._frames], dtype=bool
            ),
            "phase_label": np.asarray(
                [frame.phase_label for frame in self._frames], dtype=str
            ),
            "language_instruction": self.language_instruction,
            "images": {
                cam: np.stack(
                    [frame.decode_image(cam) for frame in self._frames]
                )
                for cam in cams
            },
        }
