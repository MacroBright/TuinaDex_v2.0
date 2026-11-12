"""test_tuina_cli_system.py — Full 8-domain tests, runtime isolation & safety gate checks for tuina CLI."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "tuinadex_bringup"))

from tuinadex_bringup.cli.env_detector import build_runtime_env
from tuinadex_bringup.cli.main import build_parser, main
from tuinadex_bringup.cli.safety import check_serial_device, confirm_hardware_action


def test_all_eight_domains_registered_in_help(capsys):
    parser = build_parser()
    with pytest.raises(SystemExit) as exc:
        parser.parse_args(["--help"])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    for domain in ("launch", "teleop", "collect", "calibrate", "arm", "hand", "data", "sys"):
        assert domain in out


def test_runtime_env_isolation_profiles():
    contaminated_env = {
        "PATH": "/home/user/win_office/conda/envs/tuinadex-control/bin:/opt/ros/lyrical/bin:/usr/bin",
        "PYTHONPATH": "/home/user/win_office/conda/envs/tuinadex-vla/lib/python3.10/site-packages:/opt/ros/lyrical/lib/python3.14/site-packages",
        "LD_LIBRARY_PATH": "/home/user/win_office/conda/envs/tuinadex-control/lib/python3.10/site-packages/cmeel.prefix/lib:/usr/lib",
        "CONDA_PREFIX": "/home/user/win_office/conda/envs/tuinadex-control",
        "CONDA_DEFAULT_ENV": "tuinadex-control",
        "ROS_DISTRO": "lyrical",
    }

    ros2_env = build_runtime_env(contaminated_env, profile="ros2")
    assert "CONDA_PREFIX" not in ros2_env
    assert "cmeel.prefix" not in ros2_env.get("LD_LIBRARY_PATH", "")
    assert "tuinadex-control" not in ros2_env.get("PATH", "")
    assert "tuinadex-vla" not in ros2_env.get("PYTHONPATH", "")
    assert "/opt/ros/lyrical/lib/python3.14/site-packages" in ros2_env.get("PYTHONPATH", "")

    vla_env = build_runtime_env(contaminated_env, profile="vla")
    assert "/opt/ros/lyrical" not in vla_env.get("PYTHONPATH", "")
    assert "cmeel.prefix" not in vla_env.get("LD_LIBRARY_PATH", "")
    assert "tuinadex-control" not in vla_env.get("PYTHONPATH", "")
    assert str(REPO_ROOT / "dataset") in vla_env.get("PYTHONPATH", "")

    ctrl_env = build_runtime_env(contaminated_env, profile="control")
    assert "tuinadex-vla" not in ctrl_env.get("PYTHONPATH", "")
    assert str(REPO_ROOT / "src" / "tuinadex_core") in ctrl_env.get("PYTHONPATH", "")


def test_safety_serial_collision_detection(tmp_path):
    usb0 = tmp_path / "ttyUSB0"
    usb0.write_text("", encoding="utf-8")
    imu = tmp_path / "myimu"
    imu.symlink_to(usb0)

    ok, msg = check_serial_device(str(usb0), label="灵巧手串口", imu_symlink=str(imu))
    assert ok is False
    assert "冲突" in msg

    imu_ok, imu_msg = check_serial_device(str(imu), label="IMU", imu_symlink=str(imu))
    assert imu_ok is True
    assert "[OK]" in imu_msg


def test_safety_gate_confirmation_modes():
    with patch("builtins.input", return_value="n"):
        assert confirm_hardware_action("test real", skip_confirm=False, dry_run=False) is False

    with patch("builtins.input", return_value="yes"):
        assert confirm_hardware_action("test real", skip_confirm=False, dry_run=False) is True

    with patch("builtins.input", side_effect=EOFError):
        assert confirm_hardware_action("test real", skip_confirm=False, dry_run=False) is False

    assert confirm_hardware_action("test real", skip_confirm=True, dry_run=False) is True
    assert confirm_hardware_action("test real", skip_confirm=False, dry_run=True) is True


@patch("subprocess.run")
def test_launch_real_safety_gate_and_targets(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)

    # User rejects real hardware prompt -> subprocess must NOT run
    with patch("builtins.input", return_value="n"):
        rc = main(["launch", "--real"])
    assert rc == 1
    mock_run.assert_not_called()

    # --yes skips prompt even when placed after subcommand
    rc = main(["launch", "--real", "-y"])
    assert rc == 0
    assert mock_run.call_count == 1
    launched_cmd = mock_run.call_args[0][0]
    launched_env = mock_run.call_args[1]["env"]
    assert "robot.launch.py" in " ".join(launched_cmd)
    assert "use_sim:=false" in launched_cmd
    assert "cmeel.prefix" not in launched_env.get("LD_LIBRARY_PATH", "")

    mock_run.reset_mock()
    # Sim and mock skip prompt automatically without stdin interaction
    rc_sim = main(["launch", "--target", "sim", "--no-scene-world", "--physics-substeps", "12", "--dry-run"])
    assert rc_sim == 0
    out = capsys.readouterr().out
    assert "tuinadex_sim sim.launch.py" in out
    assert "use_scene_world:=false" in out
    assert "physics_substeps:=12" in out

    rc_arm = main(["launch", "--target", "arm", "--mock", "--arm-channel", "can1", "--dry-run"])
    assert rc_arm == 0
    out = capsys.readouterr().out
    assert "tuinadex_arm arm.launch.py" in out
    assert "channel:=can1" in out
    assert "mock:=true" in out


@patch("subprocess.run")
def test_teleop_calibrate_drift_fixes_and_collect_terminal(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)

    # Default teleop calibrate must use /dev/myimu and MUST NOT hardcode --camera-id 0
    assert main(["teleop", "calibrate", "--mock", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "00_preflight_calibration.py" in out
    assert "--imu-port /dev/myimu" in out
    assert "--camera-id" not in out

    # --imu-only with --output must map to --output-imu (never invalid --output)
    assert main(["teleop", "calibrate", "--imu-only", "--output", "/tmp/imu_cal.yaml", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "calibrate_imu.py" in out
    assert "--output-imu /tmp/imu_cal.yaml" in out
    assert "--output /tmp/imu_cal.yaml" not in out

    # collect terminal real env must NOT emit --standalone and must pass real camera flags
    assert main([
        "collect", "terminal",
        "--env", "real",
        "--camera-config", "cams.yaml",
        "--camera-map", "overhead=/dev/video2",
        "--max-staleness", "0.25",
        "--headless",
        "--mock-teleop",
        "--dry-run",
    ]) == 0
    out = capsys.readouterr().out
    assert "--env real" in out
    assert "--camera-config cams.yaml" in out
    assert "--camera-map overhead=/dev/video2" in out
    assert "--max-staleness 0.25" in out
    assert "--headless" in out
    assert "--standalone" not in out


@patch("subprocess.run")
def test_calibrate_arm_and_hand_domains_dry_run(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)

    assert main(["calibrate", "all", "--camera-id", "4", "--skip-table", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "00_preflight_calibration.py" in out
    assert "--camera-id 4" in out
    assert "--skip-table" in out

    assert main(["calibrate", "hand-limits", "--motor", "1", "4", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "tuinadex_hand.cli.diagnostics" in out
    assert "--motor 1 4" in out

    assert main(["calibrate", "arm-ready", "--dry-run"]) == 0
    assert "teach_arm_ready_pose.py" in capsys.readouterr().out

    # rebot.cli expects --config/-c and --speed/-s BEFORE subcommand 'move'
    assert main(["arm", "move", "-c", "custom.yaml", "-s", "15", "--joint", "2", "--angle", "30", "--wait", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "-m rebot.cli --config custom.yaml --speed 15.0 move --joint 2 --angle 30.0 --wait" in out

    # High risk arm move without --yes must prompt and abort on 'n'
    with patch("builtins.input", return_value="n"):
        assert main(["arm", "move", "--joint", "1", "--angle", "5"]) == 1
    mock_run.assert_not_called()

    # arm can up uses 1000000 bitrate by default
    assert main(["arm", "can", "up", "--channel", "can0", "--dry-run"]) == 0
    assert "bitrate 1000000" in capsys.readouterr().out

    # hand control/calibrate/diagnose/teleop
    assert main(["hand", "control", "--port", "/dev/ttyUSB7", "--mock", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "tuinadex_hand.cli.control" in out
    assert "--port /dev/ttyUSB7" in out
    assert "--mock" in out


@patch("subprocess.run")
def test_data_and_sys_domains(mock_run, tmp_path, capsys):
    mock_run.return_value = MagicMock(returncode=0)

    # data convert routes cleanly to vla runtime
    assert main(["data", "convert", "--bag", "/tmp/bag1", "--out", "/tmp/ds1", "--fps", "30", "--allow-invalid"]) == 0
    cmd = mock_run.call_args[0][0]
    env = mock_run.call_args[1]["env"]
    assert "tuinadex_dataset.cli" in cmd
    assert "--allow-invalid" in cmd
    assert "tuinadex-vla" in env.get("CONDA_DEFAULT_ENV", "")
    assert "/opt/ros/lyrical" not in env.get("PYTHONPATH", "")

    mock_run.reset_mock()

    # Real filesystem dataset inspection test (no mocks needed)
    meta_dir = tmp_path / "meta"
    meta_dir.mkdir(parents=True)
    (meta_dir / "info.json").write_text(
        json.dumps({
            "codebase_version": "v3.0",
            "robot_type": "tuinadex_22dof",
            "fps": 50,
            "total_episodes": 2,
            "total_frames": 300,
            "features": {"observation.state": {}, "action": {}},
        }),
        encoding="utf-8",
    )
    (tmp_path / "ep0.mp4").write_bytes(b"")
    (tmp_path / "ep0.parquet").write_bytes(b"")
    assert main(["data", "inspect", str(tmp_path)]) == 0
    inspect_out = capsys.readouterr().out
    assert "tuinadex_22dof" in inspect_out
    assert "total_episodes   : 2" in inspect_out
    assert "mp4_files        : 1" in inspect_out

    # sys commands
    assert main(["sys", "build", "-p", "tuinadex_bringup", "--dry-run"]) == 0
    assert "colcon build --symlink-install --packages-select tuinadex_bringup" in capsys.readouterr().out

    assert main(["sys", "e2e", "--suite", "unified", "--dry-run"]) == 0
    assert "e2e_unified_robot_check.sh" in capsys.readouterr().out

    assert main(["sys", "rosbridge", "--port", "9091", "--dry-run"]) == 0
    assert "start_rosbridge.sh 9091 0.0.0.0" in capsys.readouterr().out

    assert main(["sys", "probe", "--position", "0.3", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "sim_loop_probe.py" in out
    assert "--position 0.3" in out

    assert main(["sys", "completion", "--install-hint", "--shell", "bash"]) == 0
    assert "tuina-completion.bash" in capsys.readouterr().out


@patch("subprocess.run")
def test_additional_subcommands_and_composite_safety(mock_run, capsys):
    mock_run.return_value = MagicMock(returncode=0)

    # Real composite teleop aborts without 'y' / '--yes'
    with patch("builtins.input", return_value="no"):
        assert main(["teleop", "composite", "--arm-channel", "can0"]) == 1
    mock_run.assert_not_called()

    # With --yes, composite teleop executes cleanly using ROS2 runtime
    assert main(["teleop", "composite", "--arm-channel", "can0", "--yes"]) == 0
    assert mock_run.call_count == 1
    env = mock_run.call_args[1]["env"]
    assert "cmeel.prefix" not in env.get("LD_LIBRARY_PATH", "")

    # Cover remaining dry-run command mappings cleanly
    dry_cases = [
        (["launch", "--target", "hand", "--real", "--hand-port", "/dev/ttyUSB0", "--dry-run"], "tuinadex_hand hand.launch.py"),
        (["calibrate", "imu", "--auto", "--output-imu", "/tmp/imu.yaml", "--dry-run"], "--output-imu /tmp/imu.yaml"),
        (["calibrate", "hand", "--action", "1", "--mock", "--dry-run"], "tuinadex_hand.cli.calibrate"),
        (["calibrate", "arm-goto", "--dry-run"], "test_goto_ready_pose.py"),
        (["arm", "status", "--dry-run"], "-m rebot.cli status"),
        (["arm", "watch", "--joint", "3", "--hz", "30", "--dry-run"], "-m rebot.cli watch --joint 3 --hz 30.0"),
        (["arm", "zero", "--dry-run"], "-m rebot.cli zero"),
        (["arm", "goto-ready", "--dry-run"], "test_goto_ready_pose.py"),
        (["arm", "can", "vcan", "--channel", "vcan0", "--dry-run"], "type vcan"),
        (["hand", "diagnose", "--motor", "0", "1", "--mock", "--dry-run"], "tuinadex_hand.cli.diagnostics"),
        (["hand", "teleop", "--mock", "--mock-camera", "--no-preview", "--dry-run"], "tuinadex_hand.cli.teleop"),
        (["data", "convert-docker", "--build", "--dry-run"], "run_data_convert.sh --build"),
        (["data", "vla-cloud", "--dry-run"], "launch_vla_cloud.sh"),
        (["sys", "env", "--dry-run"], "check_env.sh"),
        (["sys", "verify", "--dry-run"], "verify_all_envs.sh"),
        (["sys", "init-pc", "--setup-vcan", "--dry-run"], "init_dev_pc.sh --setup-vcan"),
        (["sys", "init-edge", "--check-only", "--dry-run"], "init_orangepi.sh --check-only"),
        (["sys", "dds", "--generate-xml", "--dry-run"], "setup_cyclonedds.sh --generate-xml"),
        (["sys", "test", "--suite", "dataset", "--dry-run"], "tuinadex-vla"),
        (["sys", "teleop-bridge", "--dry-run"], "ros2_teleop_bridge.py"),
        (["sys", "pub", "--mode", "arm_hand", "--dry-run"], "mock_command_publisher.py --mode arm_hand"),
        (["sys", "docker", "--dry-run"], "ros2_docker.sh"),
    ]
    for argv, expected in dry_cases:
        assert main(argv) == 0
        out = capsys.readouterr().out
        assert expected in out, f"Missing '{expected}' in output for {argv}: {out}"
