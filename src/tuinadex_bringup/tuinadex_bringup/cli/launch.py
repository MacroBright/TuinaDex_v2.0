"""launch.py — Handlers for 'tuina launch' command."""

from __future__ import annotations

import argparse
from typing import List, Optional

from .env_detector import execute_subcommand
from .safety import confirm_hardware_action


def register_launch_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'launch' subcommand."""
    launch_parser = subparsers.add_parser(
        "launch",
        help="Unified system-level robot bringup (robot/hand/arm/sim.launch.py)",
        description="Brings up composite 22-DOF robot or subsystems in MuJoCo simulation, safe Mock, or Real hardware.",
    )

    launch_parser.add_argument(
        "--target",
        choices=["robot", "hand", "arm", "sim"],
        default="robot",
        help="Launch target package/file (default: robot -> robot.launch.py)",
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
        help="Joint degrees-of-freedom mode for target=robot (default: arm_hand 22D)",
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

    # Scene world toggle & physics substeps
    scene_group = launch_parser.add_mutually_exclusive_group()
    scene_group.add_argument(
        "--use-scene-world",
        dest="use_scene_world",
        action="store_true",
        default=None,
        help="Load full Tuina scene with bed and patient dummy in simulation",
    )
    scene_group.add_argument(
        "--no-scene-world",
        dest="use_scene_world",
        action="store_false",
        help="Load hand-only simulation world without Tuina scene",
    )
    launch_parser.add_argument(
        "--physics-substeps",
        type=int,
        default=None,
        help="MuJoCo physics substeps per ROS 2 step (supported for hand/sim targets)",
    )

    # Recording
    launch_parser.add_argument(
        "--record",
        action="store_true",
        default=False,
        help="Launch tuinadex_recorder_node to record sensor_msgs topics to rosbag2 (target=robot)",
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
    skip_confirm = getattr(args, "yes", False)
    target = getattr(args, "target", "robot")

    # Determine runtime mode:
    # For target=sim always run MuJoCo sim; otherwise default to --sim if --real/--mock omitted.
    if target == "sim":
        use_sim = True
        mock = False
    elif args.real:
        use_sim = False
        mock = False
    elif args.mock:
        use_sim = False
        mock = True
    else:
        use_sim = True
        mock = False

    # Safety gate for real physical hardware bringup
    if not use_sim and not mock:
        need_arm = target == "arm" or (target == "robot" and args.mode in ("arm_hand", "arm_only"))
        need_hand = target == "hand" or (target == "robot" and args.mode in ("arm_hand", "hand_only"))
        arm_ch: Optional[str] = args.arm_channel if need_arm else None
        hand_pt: Optional[str] = args.hand_port if need_hand else None
        if not confirm_hardware_action(
            action_name=f"启动真机 ROS2 控制节点 (target={target}, mode={args.mode})",
            arm_channel=arm_ch,
            hand_port=hand_pt,
            skip_confirm=skip_confirm,
            dry_run=dry_run,
        ):
            return 1

    use_scene_world = getattr(args, "use_scene_world", None)
    physics_substeps = getattr(args, "physics_substeps", None)

    if target == "robot":
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
        if use_scene_world is not None:
            cmd.append(f"use_scene_world:={'true' if use_scene_world else 'false'}")
    elif target == "hand":
        cmd = [
            "ros2",
            "launch",
            "tuinadex_hand",
            "hand.launch.py",
            f"port:={args.hand_port}",
            f"use_sim:={'true' if use_sim else 'false'}",
            f"mock:={'true' if mock else 'false'}",
            f"publish_rate_hz:={args.rate_hz}",
            f"watchdog_timeout_sec:={args.watchdog_timeout}",
            f"gui:={'true' if args.gui else 'false'}",
        ]
        if use_scene_world is not None:
            cmd.append(f"use_scene_world:={'true' if use_scene_world else 'false'}")
        if physics_substeps is not None:
            cmd.append(f"physics_substeps:={physics_substeps}")
    elif target == "arm":
        cmd = [
            "ros2",
            "launch",
            "tuinadex_arm",
            "arm.launch.py",
            f"channel:={args.arm_channel}",
            f"mock:={'true' if mock else 'false'}",
            f"control_rate_hz:={args.rate_hz}",
            f"watchdog_timeout_sec:={args.watchdog_timeout}",
        ]
    else:  # target == "sim"
        cmd = [
            "ros2",
            "launch",
            "tuinadex_sim",
            "sim.launch.py",
            f"publish_rate_hz:={args.rate_hz}",
            f"watchdog_timeout_sec:={args.watchdog_timeout}",
            f"gui:={'true' if args.gui else 'false'}",
        ]
        if use_scene_world is not None:
            cmd.append(f"use_scene_world:={'true' if use_scene_world else 'false'}")
        if physics_substeps is not None:
            cmd.append(f"physics_substeps:={physics_substeps}")

    cmd.extend(extra_args)
    return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")
