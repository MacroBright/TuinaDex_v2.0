"""launch.py — Handlers for 'tuina launch' command."""

from __future__ import annotations

import argparse
from typing import List

from .env_detector import execute_subcommand


def register_launch_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'launch' subcommand."""
    launch_parser = subparsers.add_parser(
        "launch",
        help="Unified system-level robot bringup (robot.launch.py)",
        description="Brings up composite 22-DOF robot in MuJoCo simulation, safe Mock, or Real hardware.",
    )

    # Environment & Hardware Mode Group
    mode_group = launch_parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--sim",
        action="store_true",
        default=None,
        help="Launch MuJoCo digital twin simulation (default mode if --real/--mock omitted)",
    )
    mode_group.add_argument(
        "--real",
        action="store_true",
        default=False,
        help="Launch real physical hardware drivers (SocketCAN 6-DOF arm + Dynamixel 16-DOF hand)",
    )
    mode_group.add_argument(
        "--mock",
        action="store_true",
        default=False,
        help="Run in offline mock mode with simulated hardware nodes",
    )

    # Robot Joint Morphology Mode
    launch_parser.add_argument(
        "--mode",
        choices=["arm_hand", "hand_only", "arm_only"],
        default="arm_hand",
        help="Joint degrees-of-freedom mode (default: arm_hand 22D)",
    )

    # GUI / Headless display
    gui_group = launch_parser.add_mutually_exclusive_group()
    gui_group.add_argument(
        "--gui",
        dest="gui",
        action="store_true",
        default=True,
        help="Launch MuJoCo 3D interactive viewer window (default in sim)",
    )
    gui_group.add_argument(
        "--no-gui",
        "--headless",
        dest="gui",
        action="store_false",
        help="Run in headless terminal background mode",
    )

    # Recording
    launch_parser.add_argument(
        "--record",
        action="store_true",
        default=False,
        help="Launch tuinadex_recorder_node to record sensor_msgs topics to rosbag2",
    )

    # Hardware bus parameters
    launch_parser.add_argument(
        "--arm-channel",
        type=str,
        default="can0",
        help="SocketCAN interface for 6-DOF reBot arm (default: can0)",
    )
    launch_parser.add_argument(
        "--hand-port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for 16-DOF LEAP hand (default: /dev/ttyUSB0)",
    )

    # Rates & Safety Watchdog
    launch_parser.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Control and telemetry rate in Hz (default: 50.0)",
    )
    launch_parser.add_argument(
        "--watchdog-timeout",
        type=float,
        default=0.30,
        help="Watchdog timeout before smooth ramp to home in seconds (default: 0.30)",
    )


def handle_launch(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Translate 'tuina launch' flags into ros2 launch arguments and execute."""
    dry_run = getattr(args, "dry_run", False)

    # Determine runtime mode:
    # If neither --real nor --mock is requested, default to --sim
    if args.real:
        use_sim = False
        mock = False
    elif args.mock:
        use_sim = False
        mock = True
    else:
        # Default or explicit --sim
        use_sim = True
        mock = False

    cmd = [
        "ros2",
        "launch",
        "tuinadex_bringup",
        "robot.launch.py",
        f"mode:={args.mode}",
        f"use_sim:={'true' if use_sim else 'false'}",
        f"mock:={'true' if mock else 'false'}",
        f"gui:={'true' if args.gui else 'false'}",
        f"record:={'true' if args.record else 'false'}",
        f"arm_channel:={args.arm_channel}",
        f"hand_port:={args.hand_port}",
        f"rate_hz:={args.rate_hz}",
        f"watchdog_timeout_sec:={args.watchdog_timeout}",
    ]

    cmd.extend(extra_args)
    return execute_subcommand(cmd, dry_run=dry_run)
