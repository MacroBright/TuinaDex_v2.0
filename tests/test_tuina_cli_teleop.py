"""test_tuina_cli_teleop.py — Comprehensive tests for TuinaDex Teleop CLI commands.

Covers:
- tuina teleop hand (direct & ros2 backends)
- tuina teleop arm (single arm reserved interface)
- tuina teleop composite (22-DOF real arm+hand)
- tuina teleop real (alias for composite)
- dry-run dispatch verification
"""
from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "tuinadex_bringup"))

from tuinadex_bringup.cli.main import build_parser, main


def test_teleop_hand_direct_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop",
        "hand",
        "--backend", "direct",
        "--port", "/dev/ttyUSB1",
        "--camera-id", "4",
        "--rate", "60.0",
        "--mock",
        "--mock-camera",
        "--no-preview",
        "--handedness", "Right",
        "--publish-ros2",
        "--max-frames", "50",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "hand"
    assert args.backend == "direct"
    assert args.port == "/dev/ttyUSB1"
    assert args.camera_id == 4
    assert args.rate == 60.0
    assert args.mock is True
    assert args.mock_camera is True
    assert args.no_preview is True
    assert args.handedness == "Right"
    assert args.publish_ros2 is True
    assert args.max_frames == 50


def test_teleop_hand_ros2_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop",
        "hand",
        "--backend", "ros2",
        "--port", "/dev/ttyUSB0",
        "--camera-id", "2",
        "--rate", "50.0",
        "--mock",
        "--no-preview",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "hand"
    assert args.backend == "ros2"
    assert args.port == "/dev/ttyUSB0"
    assert args.camera_id == 2
    assert args.rate == 50.0
    assert args.mock is True
    assert args.no_preview is True


def test_teleop_arm_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop",
        "arm",
        "--channel", "can1",
        "--camera-id", "4",
        "--rate", "50.0",
        "--mock",
        "--no-preview",
        "--max-frames", "10",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "arm"
    assert args.channel == "can1"
    assert args.camera_id == 4
    assert args.rate == 50.0
    assert args.mock is True
    assert args.no_preview is True
    assert args.max_frames == 10


def test_teleop_composite_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop",
        "composite",
        "--arm-channel", "can0",
        "--hand-port", "/dev/ttyUSB0",
        "--camera-id", "4",
        "--rate-hz", "60.0",
        "--mock",
        "--no-imu",
        "--no-gui",
        "--max-frames", "100",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "composite"
    assert args.arm_channel == "can0"
    assert args.hand_port == "/dev/ttyUSB0"
    assert args.camera_id == 4
    assert args.rate_hz == 60.0
    assert args.mock is True
    assert args.no_imu is True
    assert args.no_gui is True
    assert args.max_frames == 100


def test_teleop_real_alias_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop",
        "real",
        "--arm-channel", "can1",
        "--hand-port", "/dev/ttyUSB1",
        "--camera-id", "2",
        "--mock",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "real"
    assert args.arm_channel == "can1"
    assert args.hand_port == "/dev/ttyUSB1"
    assert args.camera_id == 2
    assert args.mock is True


@patch("subprocess.run")
def test_dispatch_teleop_hand_direct_dry_run(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)
    ret = main(["teleop", "hand", "--backend", "direct", "--port", "/dev/ttyUSB0", "--camera-id", "4", "--mock", "--dry-run"])
    assert ret == 0
    mock_run.assert_not_called()
    out = capsys.readouterr().out
    assert "[DRY-RUN]" in out
    assert "tuinadex_hand.cli.teleop" in out
    assert "--port /dev/ttyUSB0" in out
    assert "--camera-id 4" in out
    assert "--mock" in out


@patch("subprocess.run")
def test_dispatch_teleop_hand_ros2_dry_run(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)
    ret = main(["teleop", "hand", "--backend", "ros2", "--port", "/dev/ttyUSB0", "--camera-id", "4", "--dry-run"])
    assert ret == 0
    mock_run.assert_not_called()
    out = capsys.readouterr().out
    assert "[DRY-RUN]" in out
    assert "ros2 launch tuinadex_teleop teleop_hand_real.launch.py" in out
    assert "port:=/dev/ttyUSB0" in out
    assert "camera_id:=4" in out


def test_dispatch_teleop_arm_dry_run(capsys):
    ret = main(["teleop", "arm", "--channel", "can0", "--camera-id", "4", "--dry-run"])
    assert ret == 0
    out = capsys.readouterr().out
    assert "6-DOF reBot 机械臂遥操作控制接口" in out
    assert "can0" in out


def test_dispatch_teleop_arm_mock(capsys):
    ret = main(["teleop", "arm", "--channel", "can0", "--mock"])
    assert ret == 0
    out = capsys.readouterr().out
    assert "Mock 遥操就绪" in out


@patch("subprocess.run")
def test_dispatch_teleop_composite_dry_run(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)
    ret = main(["teleop", "composite", "--arm-channel", "can0", "--hand-port", "/dev/ttyUSB0", "--camera-id", "4", "--mock", "--dry-run"])
    assert ret == 0
    mock_run.assert_not_called()
    out = capsys.readouterr().out
    assert "[DRY-RUN]" in out
    assert "ros2 launch tuinadex_teleop teleop_composite_real.launch.py" in out
    assert "arm_channel:=can0" in out
    assert "hand_port:=/dev/ttyUSB0" in out


@patch("subprocess.run")
def test_dispatch_teleop_real_alias_dry_run(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)
    ret = main(["teleop", "real", "--arm-channel", "can0", "--hand-port", "/dev/ttyUSB0", "--camera-id", "4", "--mock", "--dry-run"])
    assert ret == 0
    mock_run.assert_not_called()
    out = capsys.readouterr().out
    assert "[DRY-RUN]" in out
    assert "ros2 launch tuinadex_teleop teleop_composite_real.launch.py" in out


def test_teleop_invalid_subcommand(capsys):
    with pytest.raises(SystemExit) as exc:
        main(["teleop", "non_existent_command"])
    assert exc.value.code != 0
    captured = capsys.readouterr()
    assert "invalid choice" in captured.err


def test_resolve_hand_port_avoids_imu_collision(tmp_path):
    from tuinadex_bringup.cli.teleop import resolve_hand_port

    fake_imu_dev = tmp_path / "ttyUSB0"
    fake_imu_dev.write_text("", encoding="utf-8")
    imu_link = tmp_path / "myimu"
    imu_link.symlink_to(fake_imu_dev)

    # Explicit port matching IMU warns cleanly
    resolved, msg = resolve_hand_port(str(fake_imu_dev), imu_symlink=str(imu_link))
    assert resolved == str(fake_imu_dev)
    assert msg is not None and "冲突" in msg

    # Distinct custom port has zero warning
    other_dev = tmp_path / "ttyUSB9"
    other_dev.write_text("", encoding="utf-8")
    resolved2, msg2 = resolve_hand_port(str(other_dev), imu_symlink=str(imu_link))
    assert resolved2 == str(other_dev)
    assert msg2 is None
