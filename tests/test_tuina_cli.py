"""test_tuina_cli.py — Tests for TuinaDex Unified CLI (tuina teleop & tuina collect)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Ensure tuinadex_bringup can be imported
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "tuinadex_bringup"))

from tuinadex_bringup.cli.env_detector import get_conda_python, get_repo_root
from tuinadex_bringup.cli.main import build_parser, main


def test_get_repo_root():
    root = get_repo_root()
    assert root.is_dir()
    assert (root / "scripts").exists()
    assert (root / "src").exists()


def test_get_conda_python():
    # Test fallback or valid conda python detection
    python_bin = get_conda_python("tuinadex-control")
    assert python_bin.exists() or Path(sys.executable).exists()


def test_cli_help(capsys):
    parser = build_parser()
    with pytest.raises(SystemExit) as exc:
        parser.parse_args(["--help"])
    assert exc.value.code == 0
    captured = capsys.readouterr()
    assert "tuina" in captured.out
    assert "teleop" in captured.out
    assert "collect" in captured.out


def test_teleop_sim_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop", "sim",
        "--mock",
        "--no-gui",
        "--solver-mode", "hybrid",
        "--axis-mapping=-y,+x,+z",
        "--max-frames", "10",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "sim"
    assert args.mock is True
    assert args.no_gui is True
    assert args.solver_mode == "hybrid"
    assert args.axis_mapping == "-y,+x,+z"
    assert args.max_frames == 10


def test_teleop_calibrate_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop", "calibrate",
        "--imu-only",
        "--camera-id", "2",
        "--imu-port", "/dev/ttyUSB1",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "calibrate"
    assert args.imu_only is True
    assert args.camera_id == 2
    assert args.imu_port == "/dev/ttyUSB1"


def test_teleop_node_args():
    parser = build_parser()
    args = parser.parse_args([
        "teleop", "node",
        "--rate-hz", "60.0",
        "--mock",
        "--headless",
    ])
    assert args.command == "teleop"
    assert args.teleop_subcommand == "node"
    assert args.rate_hz == 60.0
    assert args.mock is True
    assert args.headless is True


def test_collect_hub_args():
    parser = build_parser()
    args = parser.parse_args([
        "collect", "hub",
        "--env", "sim",
        "--mode", "arm_hand",
        "--port", "8088",
        "--host", "0.0.0.0",
        "--cameras", "overhead,wrist",
        "--fps", "60",
        "--task", "acupoint press",
    ])
    assert args.command == "collect"
    assert args.collect_subcommand == "hub"
    assert args.env == "sim"
    assert args.mode == "arm_hand"
    assert args.port == 8088
    assert args.host == "0.0.0.0"
    assert args.cameras == "overhead,wrist"
    assert args.fps == 60
    assert args.task == "acupoint press"


def test_collect_terminal_args():
    parser = build_parser()
    args = parser.parse_args([
        "collect", "terminal",
        "--env", "real",
        "--episodes", "5",
        "--frames-per-episode", "200",
        "--mock-teleop",
    ])
    assert args.command == "collect"
    assert args.collect_subcommand == "terminal"
    assert args.env == "real"
    assert args.episodes == 5
    assert args.frames_per_episode == 200
    assert args.mock_teleop is True


def test_collect_check_args():
    parser = build_parser()
    args = parser.parse_args(["collect", "check"])
    assert args.command == "collect"
    assert args.collect_subcommand == "check"


def test_launch_sim_args():
    parser = build_parser()
    args = parser.parse_args([
        "launch",
        "--sim",
        "--gui",
        "--mode", "arm_hand",
        "--record",
    ])
    assert args.command == "launch"
    assert args.sim is True
    assert args.gui is True
    assert args.mode == "arm_hand"
    assert args.record is True


def test_launch_real_args():
    parser = build_parser()
    args = parser.parse_args([
        "launch",
        "--real",
        "--arm-channel", "can1",
        "--hand-port", "/dev/ttyUSB1",
    ])
    assert args.command == "launch"
    assert args.real is True
    assert args.arm_channel == "can1"
    assert args.hand_port == "/dev/ttyUSB1"


def test_launch_mock_args():
    parser = build_parser()
    args = parser.parse_args([
        "launch",
        "--mock",
        "--mode", "hand_only",
        "--no-gui",
    ])
    assert args.command == "launch"
    assert args.mock is True
    assert args.mode == "hand_only"
    assert args.gui is False


@patch("subprocess.run")
def test_dispatch_launch_dry_run(mock_run):
    mock_run.return_value = MagicMock(returncode=0)
    ret = main(["launch", "--sim", "--gui", "--dry-run"])
    assert ret == 0
    mock_run.assert_not_called()


@patch("subprocess.run")
def test_dispatch_teleop_sim(mock_run):
    mock_run.return_value = MagicMock(returncode=0)
    ret = main(["teleop", "sim", "--mock", "--no-gui", "--dry-run"])
    assert ret == 0
    # In dry-run mode, subprocess.run should not be called
    mock_run.assert_not_called()


@patch("subprocess.run")
def test_dispatch_collect_hub(mock_run):
    mock_run.return_value = MagicMock(returncode=0)
    ret = main(["collect", "hub", "--dry-run"])
    assert ret == 0
    mock_run.assert_not_called()
