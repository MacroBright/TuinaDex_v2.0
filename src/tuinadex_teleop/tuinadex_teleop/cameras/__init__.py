"""Standard three-camera specifications and simulation capture."""
from .camera_specs import STANDARD_CAMERA_SPECS, CameraMountType, CameraSpec
from .real_camera_rig import (
    RealCameraConfig,
    RealCameraProducerThread,
    discover_uvc_devices,
    encode_bgr_to_jpeg,
    load_real_camera_configs,
    make_unbound_jpeg_frame,
    parse_camera_map,
    resolve_default_camera_config_path,
    save_real_camera_device,
)
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
    "RealCameraConfig",
    "RealCameraProducerThread",
    "discover_uvc_devices",
    "encode_bgr_to_jpeg",
    "load_real_camera_configs",
    "make_unbound_jpeg_frame",
    "parse_camera_map",
    "resolve_default_camera_config_path",
    "save_real_camera_device",
    "encode_rgb_to_jpeg",
    "decode_jpeg_to_rgb",
]
