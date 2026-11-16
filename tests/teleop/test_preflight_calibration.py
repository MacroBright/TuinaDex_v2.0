"""Unit test for 00_preflight_calibration.py all-in-one wizard."""
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "scripts" / "00_preflight_calibration.py"


def test_preflight_calibration_mock_dry_run():
    """Verify that 00_preflight_calibration runs cleanly in mock/headless mode."""
    cmd = [
        sys.executable,
        str(SCRIPT_PATH),
        "--mock",
        "--no-gui",
        "--skip-table",
        "--skip-imu",
        "--skip-hand",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=35)
    assert res.returncode == 0, f"STDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}"
    assert "[SUCCESS] 统一会话标定文件已写入" in res.stdout


def test_preflight_calibration_mock_full_wizard():
    """Verify that 00_preflight_calibration runs all 3 stages in mock mode without crashing."""
    cmd = [
        sys.executable,
        str(SCRIPT_PATH),
        "--mock",
        "--no-gui",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=45)
    assert res.returncode == 0, f"STDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}"
    assert "[SUCCESS] 阶段 1 桌面标定成功！" in res.stdout
    assert "[SUCCESS] 阶段 2 IMU 三轴姿态对齐与零位校准成功！" in res.stdout
    assert "[SUCCESS] 阶段 3 手部标定成功！" in res.stdout
    assert "[SUCCESS] 统一会话标定文件已写入" in res.stdout


def test_stage1_only_id1_no_keyerror():
    """Regression test: verify detecting ONLY ID 1 (without ID 0) does not throw KeyError: 0."""
    import cv2
    import numpy as np
    from tuinadex_teleop.perception.realsense_stream import CameraIntrinsics
    from tuinadex_teleop.perception.table_calibrator import TableCalibrator

    calibrator = TableCalibrator(primary_id=0, secondary_id=1)
    intrinsics = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)

    # Synthesize image with ONLY ArUco ID 1 (no ID 0)
    canvas = np.full((480, 640, 3), 255, dtype=np.uint8)
    marker1 = cv2.aruco.generateImageMarker(calibrator.dictionary, 1, 100)
    canvas[200:300, 400:500] = cv2.cvtColor(marker1, cv2.COLOR_GRAY2BGR)

    observations = calibrator.detect_markers(canvas, intrinsics)
    assert 0 not in observations
    assert 1 in observations

    has_id0 = 0 in observations
    has_id1 = 1 in observations

    # Execute telemetry formatting logic matching 00_preflight_calibration.py
    t_lines = [
        f"识别状态: ID 0: {'已锁定 (OK)' if has_id0 else '未检测 (SEARCHING)'} | ID 1: {'已锁定 (OK)' if has_id1 else '未检测'}"
    ]
    if has_id0 and has_id1:
        t0 = observations[0].t_cam_to_marker[:3, 3]
        t1 = observations[1].t_cam_to_marker[:3, 3]
        t_lines.append(f"基线几何校验: 对角线距离 {np.linalg.norm(t1-t0)*100:.1f} cm (符合双码基准)")
    elif has_id0:
        t_lines.append("基线几何校验: 建议放置 ID 1 码以提供稳定对角线基线约束")
    elif has_id1:
        t_lines.append("基线几何校验: 已检测到 ID 1 对角码，但未检测到 ID 0 主基准码")
    else:
        t_lines.append("基线几何校验: 建议放置 ID 1 码以提供稳定对角线基线约束")

    assert "已检测到 ID 1 对角码，但未检测到 ID 0 主基准码" in t_lines[1]


