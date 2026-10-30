"""camera_specs.py - standard camera optics and mount topology."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class CameraMountType(Enum):
    """Camera mount topology type."""

    EYE_TO_HAND = auto()
    EYE_IN_HAND = auto()


@dataclass(frozen=True)
class CameraSpec:
    """Optical specification and MJCF name mapping for one camera."""

    name: str
    mjcf_name: str
    mount_type: CameraMountType
    width: int = 640
    height: int = 480
    fovy: float = 60.0


STANDARD_CAMERA_SPECS: dict[str, CameraSpec] = {
    "overhead": CameraSpec(
        name="overhead",
        mjcf_name="overhead_cam",
        mount_type=CameraMountType.EYE_TO_HAND,
        fovy=60.0,
    ),
    "wrist": CameraSpec(
        name="wrist",
        mjcf_name="wrist_cam",
        mount_type=CameraMountType.EYE_IN_HAND,
        fovy=75.0,
    ),
    "side": CameraSpec(
        name="side",
        mjcf_name="side_cam",
        mount_type=CameraMountType.EYE_TO_HAND,
        fovy=45.0,
    ),
}
