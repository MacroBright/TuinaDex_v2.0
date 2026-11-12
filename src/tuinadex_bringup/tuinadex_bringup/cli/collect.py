"""collect.py — Handlers for 'tuina collect' subcommands (hub, terminal, check)."""

from __future__ import annotations

import argparse
from typing import List

from .env_detector import execute_subcommand, get_conda_python, get_repo_root
from .safety import confirm_hardware_action


def _add_common_collect_args(parser: argparse.ArgumentParser) -> None:
    """Register shared simulation/real collection options across hub and terminal modes."""
    parser.add_argument("--env", choices=["sim", "real"], default="sim", help="Environment type (default: sim)")
    parser.add_argument(
        "--mode",
        choices=["arm_hand", "hand_only"],
        default="arm_hand",
        help="Joint control mode (default: arm_hand 22D)",
    )
    parser.add_argument(
        "--cameras",
        type=str,
        default="overhead,wrist,side",
        help="Comma-separated camera list (default: overhead,wrist,side)",
    )
    parser.add_argument("--fps", type=int, default=50, help="Control & recording FPS (default: 50)")
    parser.add_argument("--task", type=str, default="tuina acupoint massage", help="Task description prompt")
    parser.add_argument("--repo-id", type=str, default=None, help="Target LeRobotDataset directory")
    parser.add_argument("--mock-teleop", action="store_true", help="Use synthetic mock teleoperation")
    standalone_group = parser.add_mutually_exclusive_group()
    standalone_group.add_argument(
        "--standalone",
        dest="standalone",
        action="store_true",
        default=True,
        help="Run MuJoCo in standalone in-process mode when --env sim (default: True)",
    )
    standalone_group.add_argument(
        "--no-standalone",
        dest="standalone",
        action="store_false",
        help="Connect to external ROS 2 simulation nodes instead of in-process MuJoCo",
    )
    parser.add_argument(
        "--camera-config",
        type=str,
        default=None,
        help="Physical camera YAML config path (real env)",
    )
    parser.add_argument(
        "--camera-map",
        type=str,
        default=None,
        help="Comma-separated camera overrides, e.g. overhead=/dev/video8,side=/dev/video6",
    )
    parser.add_argument(
        "--max-staleness",
        type=float,
        default=None,
        help="Max camera frame age in seconds before staleness abort",
    )


def register_collect_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'collect' subcommand and its nested subcommands."""
    collect_parser = subparsers.add_parser(
        "collect",
        help="Multi-modal teleoperation data collection & Web UI Hub",
        description="Manages Web UI collection workbench, terminal episodes, and E2E data verification.",
    )
    collect_subs = collect_parser.add_subparsers(dest="collect_subcommand", metavar="<subcommand>")

    # 1. tuina collect hub
    hub_parser = collect_subs.add_parser(
        "hub",
        help="Start Web UI Collection Hub workbench (http://localhost:8080)",
        description="Zero-dependency Web UI for multi-camera streaming, audio cues and teleop data recording.",
    )
    _add_common_collect_args(hub_parser)
    hub_parser.add_argument("--host", type=str, default="127.0.0.1", help="Bind IP address (default: 127.0.0.1)")
    hub_parser.add_argument("--port", type=int, default=8080, help="Web Hub port (default: 8080)")
    hub_parser.add_argument("--ui-fps", type=int, default=None, help="Web UI MJPEG/SSE stream frame rate")

    # 2. tuina collect terminal
    term_parser = collect_subs.add_parser(
        "terminal",
        help="Collect episodes in terminal interactive/loop mode",
    )
    _add_common_collect_args(term_parser)
    term_parser.add_argument("--episodes", type=int, default=1, help="Number of episodes to record")
    term_parser.add_argument("--frames-per-episode", type=int, default=150, help="Frames per episode")
    term_parser.add_argument("--headless", action="store_true", help="Disable OpenCV preview window")

    # 3. tuina collect check
    check_parser = collect_subs.add_parser(
        "check",
        help="Run automated E2E multi-camera collection & H.264/LeRobot verification check",
    )
    check_parser.add_argument(
        "--legacy",
        action="store_true",
        help="Run legacy collection script e2e_collection_check.sh instead of e2e_hub_collection_check.sh",
    )


def _append_common_collect_flags(cmd: List[str], args: argparse.Namespace) -> None:
    if args.repo_id:
        cmd.extend(["--repo-id", args.repo_id])
    if args.mock_teleop:
        cmd.append("--mock-teleop")
    if getattr(args, "camera_config", None):
        cmd.extend(["--camera-config", args.camera_config])
    if getattr(args, "camera_map", None):
        cmd.extend(["--camera-map", args.camera_map])
    if getattr(args, "max_staleness", None) is not None:
        cmd.extend(["--max-staleness", str(args.max_staleness)])
    if getattr(args, "standalone", False) and args.env == "sim":
        cmd.append("--standalone")


def handle_collect(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Dispatch 'tuina collect <subcommand>' to scripts/collect_teleop_data.py or E2E checks."""
    root = get_repo_root()
    python_bin = get_conda_python("tuinadex-control")
    dry_run = getattr(args, "dry_run", False)
    skip_confirm = getattr(args, "yes", False)

    subcmd = getattr(args, "collect_subcommand", None)
    if subcmd == "hub":
        if args.env == "real" and not args.mock_teleop:
            if not confirm_hardware_action(
                action_name=f"启动真机 Web UI 数据采集 Hub (mode={args.mode}, cameras={args.cameras})",
                arm_channel="can0" if args.mode == "arm_hand" else None,
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        script = root / "scripts" / "collect_teleop_data.py"
        cmd = [
            str(python_bin),
            str(script),
            "--env",
            args.env,
            "--mode",
            args.mode,
            "--hub",
            "--host",
            args.host,
            "--port",
            str(args.port),
            "--cameras",
            args.cameras,
            "--fps",
            str(args.fps),
            "--task",
            args.task,
        ]
        if getattr(args, "ui_fps", None) is not None:
            cmd.extend(["--ui-fps", str(args.ui_fps)])
        _append_common_collect_flags(cmd, args)
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "terminal":
        if args.env == "real" and not args.mock_teleop:
            if not confirm_hardware_action(
                action_name=f"启动真机终端数据采集 (mode={args.mode}, episodes={args.episodes})",
                arm_channel="can0" if args.mode == "arm_hand" else None,
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        script = root / "scripts" / "collect_teleop_data.py"
        cmd = [
            str(python_bin),
            str(script),
            "--env",
            args.env,
            "--mode",
            args.mode,
            "--cameras",
            args.cameras,
            "--fps",
            str(args.fps),
            "--task",
            args.task,
            "--episodes",
            str(args.episodes),
            "--frames-per-episode",
            str(args.frames_per_episode),
        ]
        if getattr(args, "headless", False):
            cmd.append("--headless")
        _append_common_collect_flags(cmd, args)
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "check":
        script_name = "e2e_collection_check.sh" if getattr(args, "legacy", False) else "e2e_hub_collection_check.sh"
        script = root / "scripts" / script_name
        cmd = ["bash", str(script)]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="bash")

    else:
        print("Usage: tuina collect {hub|terminal|check} [OPTIONS]")
        return 1
