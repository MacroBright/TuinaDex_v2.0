"""hand.py — Handlers for 'tuina hand' 16-DOF LEAP Hand hardware & diagnostic domain."""

from __future__ import annotations

import argparse
from typing import List

from .env_detector import execute_subcommand, get_conda_python
from .safety import confirm_hardware_action
from .teleop import resolve_hand_port


def register_hand_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'hand' subcommand wrapping tuinadex_hand.cli utilities."""
    hand_parser = subparsers.add_parser(
        "hand",
        help="16-DOF LEAP Hand interactive control, zero calibration, limits & teleop",
        description="Unified wrapper for leap-control, leap-calibrate, leap-diagnose, and leap-teleop.",
    )
    hand_subs = hand_parser.add_subparsers(dest="hand_subcommand", metavar="<subcommand>")

    # 1. tuina hand control
    ctrl_parser = hand_subs.add_parser("control", help="Interactive joint control & safe home console (leap-control)")
    ctrl_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="LEAP Hand serial port")
    ctrl_parser.add_argument("--mock", action="store_true", help="Run in offline mock mode")

    # 2. tuina hand calibrate
    cal_parser = hand_subs.add_parser("calibrate", help="Calibrate LEAP Hand OPEN_POSE zero offsets (leap-calibrate)")
    cal_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="LEAP Hand serial port")
    cal_parser.add_argument("--action", choices=["1", "2"], default=None, help="1=Calibrate open pose, 2=Record custom pose")
    cal_parser.add_argument("--mock", action="store_true", help="Run in offline mock mode")

    # 3. tuina hand diagnose (alias: diagnostics)
    diag_parser = hand_subs.add_parser(
        "diagnose",
        aliases=["diagnostics"],
        help="Measure motor hardware limits and health diagnostics (leap-diagnose)",
    )
    diag_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="LEAP Hand serial port")
    diag_parser.add_argument("--out", type=str, default=None, help="Output limits JSON path")
    diag_parser.add_argument("--motor", type=int, nargs="+", default=None, help="Specific motor ID(s) to measure")
    diag_parser.add_argument("--mock", action="store_true", help="Run in offline mock mode")

    # 4. tuina hand teleop
    teleop_parser = hand_subs.add_parser("teleop", help="Direct vision teleoperation for 16-DOF hand (leap-teleop)")
    teleop_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="LEAP Hand serial port")
    teleop_parser.add_argument("--camera-id", type=int, default=0, help="Camera device index (default: 0)")
    teleop_parser.add_argument("--rate", type=float, default=50.0, help="Control loop rate in Hz (default: 50.0)")
    teleop_parser.add_argument("--filter-cutoff", type=float, default=1.0, help="OneEuro filter min cutoff")
    teleop_parser.add_argument("--filter-beta", type=float, default=0.05, help="OneEuro filter beta")
    teleop_parser.add_argument("--handedness", choices=["any", "Right", "Left"], default="Right", help="Handedness filter")
    teleop_parser.add_argument("--no-preview", action="store_true", help="Disable OpenCV preview window")
    teleop_parser.add_argument("--mock", action="store_true", help="Mock hardware LEAP Hand")
    teleop_parser.add_argument("--mock-camera", action="store_true", help="Use synthetic mock camera frames")
    teleop_parser.add_argument("--publish-ros2", action="store_true", help="Mirror /hand/joint_command over ROS 2")
    teleop_parser.add_argument("--max-frames", type=int, default=0, help="Auto-exit after N frames")


def handle_hand(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Dispatch 'tuina hand <subcommand>' to tuinadex_hand.cli modules."""
    python_bin = str(get_conda_python("tuinadex-control"))
    dry_run = getattr(args, "dry_run", False)
    skip_confirm = getattr(args, "yes", False)
    subcmd = getattr(args, "hand_subcommand", None)

    if subcmd not in ("control", "calibrate", "diagnose", "diagnostics", "teleop"):
        print("Usage: tuina hand {control|calibrate|diagnose|teleop} [OPTIONS]")
        return 1

    resolved_port, port_msg = resolve_hand_port(args.port)
    if port_msg:
        print(port_msg)

    if not args.mock:
        high_risk = subcmd in ("diagnose", "diagnostics")
        if not confirm_hardware_action(
            action_name=f"启动 16-DOF LEAP Hand 真机工具 ({subcmd})",
            hand_port=resolved_port,
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=high_risk,
        ):
            return 1

    if subcmd == "control":
        cmd = [python_bin, "-m", "tuinadex_hand.cli.control", "--port", resolved_port]
        if args.mock:
            cmd.append("--mock")
    elif subcmd == "calibrate":
        cmd = [python_bin, "-m", "tuinadex_hand.cli.calibrate", "--port", resolved_port]
        if args.action:
            cmd.extend(["--action", args.action])
        if args.mock:
            cmd.append("--mock")
    elif subcmd in ("diagnose", "diagnostics"):
        cmd = [python_bin, "-m", "tuinadex_hand.cli.diagnostics", "--port", resolved_port]
        if args.out:
            cmd.extend(["--out", args.out])
        if args.motor:
            cmd.extend(["--motor", *[str(m) for m in args.motor]])
        if args.mock:
            cmd.append("--mock")
    else:  # teleop
        cmd = [
            python_bin,
            "-m",
            "tuinadex_hand.cli.teleop",
            "--port",
            resolved_port,
            "--camera-id",
            str(args.camera_id),
            "--rate",
            str(args.rate),
            "--filter-cutoff",
            str(args.filter_cutoff),
            "--filter-beta",
            str(args.filter_beta),
            "--handedness",
            args.handedness,
        ]
        if args.no_preview:
            cmd.append("--no-preview")
        if args.mock:
            cmd.append("--mock")
        if args.mock_camera:
            cmd.append("--mock-camera")
        if args.publish_ros2:
            cmd.append("--publish-ros2")
        if args.max_frames > 0:
            cmd.extend(["--max-frames", str(args.max_frames)])

    cmd.extend(extra_args)
    return execute_subcommand(cmd, dry_run=dry_run, profile="control")
