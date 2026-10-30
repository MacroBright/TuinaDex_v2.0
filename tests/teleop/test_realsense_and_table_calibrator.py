"""Unit tests for RealSense stream abstraction, ArUco detection, and table calibration."""
import os
import cv2
import numpy as np
import pytest

from tuinadex_teleop.perception.realsense_stream import (
    CameraIntrinsics,
    MockRealSenseStream,
)
from tuinadex_teleop.perception.table_calibrator import (
    TableCalibrator,
    rotation_matrix_to_quaternion,
    quaternion_to_rotation_matrix,
    average_quaternions,
)


def test_mock_realsense_stream():
    stream = MockRealSenseStream(width=640, height=480, fps=30)
    assert not stream.is_running

    intrinsics = stream.start()
    assert stream.is_running
    assert intrinsics.width == 640
    assert intrinsics.height == 480
    assert intrinsics.matrix.shape == (3, 3)

    color, depth_m, ts = stream.read_frames()
    assert color.shape == (480, 640, 3)
    assert depth_m.shape == (480, 640)
    assert np.all(depth_m == 0.65)
    assert ts > 0.0

    # Deprojection test
    p_3d = intrinsics.deproject_pixel_to_point(intrinsics.cx, intrinsics.cy, 1.0)
    np.testing.assert_allclose(p_3d, [0.0, 0.0, 1.0], atol=1e-5)

    stream.stop()
    assert not stream.is_running


def test_quaternion_matrix_roundtrip():
    # Test random rotation matrices
    np.random.seed(42)
    for _ in range(10):
        rand_vec = np.random.randn(3)
        R_orig, _ = cv2.Rodrigues(rand_vec)
        q = rotation_matrix_to_quaternion(R_orig)
        R_recon = quaternion_to_rotation_matrix(q)
        np.testing.assert_allclose(R_orig, R_recon, atol=1e-6)


def test_average_quaternions_clustering():
    # Identity quaternion [1, 0, 0, 0] with slight perturbation
    q_base = np.array([1.0, 0.0, 0.0, 0.0])
    noisy_qs = [
        q_base + np.random.normal(0, 0.02, 4) for _ in range(20)
    ]
    avg_q = average_quaternions(noisy_qs)
    # Average should stay close to identity
    assert avg_q[0] > 0.99
    assert np.isclose(np.linalg.norm(avg_q), 1.0)


def test_aruco_detection_and_table_calibration_synthetic(tmp_path):
    calibrator = TableCalibrator(
        marker_size_m=0.080,
        dictionary_name="DICT_4X4_50",
        primary_id=0,
        secondary_id=1,
        min_samples=5,
    )
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)

    # Synthesize image with two ArUco markers: ID0 (left) and ID1 (right)
    canvas = np.full((480, 640, 3), 255, dtype=np.uint8)
    marker0 = cv2.aruco.generateImageMarker(calibrator.dictionary, 0, 100)
    marker1 = cv2.aruco.generateImageMarker(calibrator.dictionary, 1, 100)

    # Place marker 0 at (100, 150)
    canvas[150:250, 100:200] = cv2.cvtColor(marker0, cv2.COLOR_GRAY2BGR)
    # Place marker 1 at (400, 200)
    canvas[200:300, 400:500] = cv2.cvtColor(marker1, cv2.COLOR_GRAY2BGR)

    observations = calibrator.detect_markers(canvas, intrinsics)
    assert 0 in observations
    assert 1 in observations
    assert observations[0].t_cam_to_marker.shape == (4, 4)
    assert observations[1].t_cam_to_marker.shape == (4, 4)

    # Test sample accumulation
    for _ in range(5):
        calibrator.add_frame_sample(observations)

    assert calibrator.sample_count == 5
    assert calibrator.is_ready

    baseline = calibrator.get_baseline_distance()
    assert baseline is not None
    assert baseline > 0.0

    # Compute calibration
    calib = calibrator.compute_calibration()
    assert calib.is_valid
    assert calib.num_samples == 5
    assert calib.t_cam_to_table.shape == (4, 4)
    assert calib.t_table_to_cam.shape == (4, 4)

    # Save and load roundtrip
    calib_file = str(tmp_path / "test_table_calib.yaml")
    calibrator.save_calibration(calib, calib_file)
    assert os.path.exists(calib_file)

    loaded_calib = TableCalibrator.load_calibration(calib_file)
    assert loaded_calib.is_valid
    assert loaded_calib.tag_id == 0
    np.testing.assert_allclose(loaded_calib.t_cam_to_table, calib.t_cam_to_table, atol=1e-5)
    np.testing.assert_allclose(loaded_calib.t_table_to_cam, calib.t_table_to_cam, atol=1e-5)

    # Point at ID0 origin in camera frame should transform to [0, 0, 0] in table frame
    id0_cam_origin = calib.t_cam_to_table[:3, 3]
    p_table = calib.transform_point_to_table(id0_cam_origin)
    np.testing.assert_allclose(p_table, [0.0, 0.0, 0.0], atol=1e-5)

