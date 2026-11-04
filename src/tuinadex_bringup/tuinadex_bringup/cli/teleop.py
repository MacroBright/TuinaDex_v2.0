"""teleop.py — Handlers for 'tuina teleop' subcommands."""

from __future__ import annotations

import argparse
import sys
from typing import List

from .env_detector import execute_subcommand, get_conda_python, get_repo_root


def register_teleop_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'teleop' subcommands."""
    teleop_parser = subparsers.add_parser(
        "teleop",
        help="Teleoperation, vision tracking and calibration tools",
        description="TuinaDex Teleoperation Subsystem.",
    )
    teleop_subs = teleop_parser.add_subparsers(dest="teleop_subcommand")

    # 1. tuina teleop sim
    sim_parser = teleop_subs.add_parser(
        "sim",
        help="Launch real-time 22-DOF simulation teleoperation preview",
        description="Fuses RealSense D455 3D hand tracking, IMU attitude, and MuJoCo digital twin.",
    )
    sim_parser.add_argument("--mock", action="store_true", help="Run with mock camera stream")
    sim_parser.add_argument("--no-gui", action="store_true", help="Headless terminal mode (no OpenCV/Viewer)")
    sim_parser.add_argument("--max-frames", type=int, default=0, help="Exit after N frames")
    sim_parser.add_argument(
        "--solver-mode",
        type=str,
        default="hybrid",
        choices=["hybrid", "geometric", "dex_retargeting"],
        help="Hand retargeting solver mode",
    )
    sim_parser.add_argument("--axis-mapping", type=str, default=None, help="Direction alignment mapping (e.g. -y,+x,+z)")
    sim_parser.add_argument("--session-config", type=str, default=None, help="Unified session calibration YAML path")
    sim_parser.add_argument("--model", type=str, default=None, help="Custom MuJoCo scene XML path")

    # 2. tuina teleop calibrate
    calib_parser = teleop_subs.add_parser(
        "calibrate",
        help="Interactive pre-flight calibration wizard (table + IMU + hand)",
        description="Calibrates tabletop plane, wrist IMU neutral/axes, and hand ROM.",
    )
    calib_parser.add_argument("--imu-only", action="store_true", help="Only calibrate wrist IMU 3-gesture alignment")
    calib_parser.add_argument("--camera-id", type=int, default=0, help="RealSense / V4L2 camera device ID")
    calib_parser.add_argument("--imu-port", type=str, default="/dev/ttyUSB0", help="IMU serial port device")
    calib_parser.add_argument("--output", type=str, default=None, help="Output calibration YAML path")

    # 3. tuina teleop node
    node_parser = teleop_subs.add_parser(
        "node",
        help="Launch ROS 2 composite teleoperation node via teleop_sim.launch.py",
        description="Publishes /arm/joint_command and /hand/joint_command to ROS 2.",
    )
    node_parser.add_argument("--rate-hz", type=float, default=60.0, help="Publish rate in Hz")
    node_parser.add_argument("--camera-fps", type=int, default=60, help="Camera acquisition FPS")
    node_parser.add_argument("--mock", action="store_true", help="Run mock tracking")
    node_parser.add_argument("--headless", action="store_true", help="Do not open OpenCV display window")


def handle_teleop(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Route teleop subcommands to underlying scripts."""
    root = get_repo_root()
    python_bin = get_conda_python("tuinadex-control")
    dry_run = getattr(args, "dry_run", False)

    subcmd = getattr(args, "teleop_subcommand", None)

    if subcmd == "sim":
        script = root / "scripts" / "run_sim_teleop.py"
        cmd = [str(python_bin), str(script)]
        if args.mock:
            cmd.append("--mock")
        if args.no_gui:
            cmd.append("--no-gui")
        if args.max_frames > 0:
            cmd.extend(["--max-frames", str(args.max_frames)])
        if args.solver_mode:
            cmd.extend(["--solver-mode", args.solver_mode])
        if args.axis_mapping:
            cmd.extend(["--axis-mapping", args.axis_mapping])
        if args.session_config:
            cmd.extend(["--session-config", args.session_config])
        if args.model:
            cmd.extend(["--model", args.model])
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run)

    elif subcmd == "calibrate":
        if args.imu_only:
            script = root / "scripts" / "calibrate_imu.py"
            cmd = [str(python_bin), str(script), "--port", args.imu_port]
            if args.output:
                cmd.extend(["--output", args.output])
        else:
            script = root / "scripts" / "00_preflight_calibration.py"
            cmd = [
                str(python_bin),
                str(script),
                "--camera-id",
                str(args.camera_id),
                "--imu-port",
                args.imu_port,
            ]
            if args.output:
                cmd.extend(["--output", args.output])
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run)

    elif subcmd == "node":
        # Launch via ros2 launch
        cmd = [
            "ros2",
            "launch",
            "tuinadex_teleop",
            "teleop_sim.launch.py",
            f"publish_rate_hz:={args.rate_hz}",
            f"camera_fps:={args.camera_fps}",
            f"mock:={'true' if args.mock else 'false'}",
            f"headless:={'true' if args.headless else 'false'}",
        ]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run)

    else:
        print("Usage: tuina teleop {sim|calibrate|node} [OPTIONS]")
        return 1

