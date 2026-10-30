"""Perception subpackage: RealSense streaming, table ArUco calibration, and 3D wrist tracking."""
from .realsense_stream import CameraIntrinsics, RealSenseStream, MockRealSenseStream
from .table_calibrator import (
    TableCalibrator,
    MarkerPoseObservation,
    rotation_matrix_to_quaternion,
    quaternion_to_rotation_matrix,
    average_quaternions,
)
from .wrist_tracker import (
    WristTracker,
    WristPose,
    TrackingState,
    median_depth_at,
    compute_palm_basis,
    rotation_matrix_to_euler_deg,
    draw_wrist_hud,
)
from .imu_stream import (
    ImuReading,
    ImuStream,
    MockImuStream,
    WristImuTracker,
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
    parse_axis_mapping,
    rotation_matrix_to_rotvec,
    solve_imu_alignment_from_gestures,
)

__all__ = [
    "CameraIntrinsics",
    "RealSenseStream",
    "MockRealSenseStream",
    "TableCalibrator",
    "MarkerPoseObservation",
    "rotation_matrix_to_quaternion",
    "quaternion_to_rotation_matrix",
    "average_quaternions",
    "WristTracker",
    "WristPose",
    "TrackingState",
    "median_depth_at",
    "compute_palm_basis",
    "rotation_matrix_to_euler_deg",
    "draw_wrist_hud",
    "ImuReading",
    "ImuStream",
    "MockImuStream",
    "WristImuTracker",
    "quat_to_rotation_matrix",
    "rotation_matrix_to_quat",
    "parse_axis_mapping",
    "rotation_matrix_to_rotvec",
    "solve_imu_alignment_from_gestures",
]

