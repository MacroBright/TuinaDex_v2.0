"""Standard three-camera specifications and simulation capture."""
from .camera_specs import STANDARD_CAMERA_SPECS, CameraMountType, CameraSpec
from .sim_camera_rig import (
    CameraProducerThread,
    SimCameraRig,
    decode_jpeg_to_rgb,
    encode_rgb_to_jpeg,
)

__all__ = [
    "CameraSpec",
    "CameraMountType",
    "STANDARD_CAMERA_SPECS",
    "SimCameraRig",
    "CameraProducerThread",
    "encode_rgb_to_jpeg",
    "decode_jpeg_to_rgb",
]
