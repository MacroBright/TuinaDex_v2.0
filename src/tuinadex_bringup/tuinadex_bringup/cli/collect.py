"""collect.py — Handlers for 'tuina collect' subcommands."""

from __future__ import annotations

import argparse
from typing import List

from .env_detector import execute_subcommand, get_conda_python, get_repo_root


def register_collect_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'collect' subcommands."""
    collect_parser = subparsers.add_parser(
        "collect",
        help="Multi-modal data collection and LeRobot dataset tools",
        description="TuinaDex Multi-Modal Data Collection Subsystem.",
    )
    collect_subs = collect_parser.add_subparsers(dest="collect_subcommand")

    # 1. tuina collect hub
    hub_parser = collect_subs.add_parser(
        "hub",
        help="Launch the immersive Web UI collection hub (default port 8080)",
        description="Zero-dependency Web UI for multi-camera streaming, audio cues and teleop data recording.",
    )
    hub_parser.add_argument("--env", choices=["sim", "real"], default="sim", help="Environment type (default: sim)")
    hub_parser.add_argument(
        "--mode",
        choices=["arm_hand", "hand_only"],
        default="arm_hand",
        help="Joint control mode (default: arm_hand 22D)",
    )
    hub_parser.add_argument("--host", type=str, default="127.0.0.1", help="Bind IP address (default: 127.0.0.1)")
    hub_parser.add_argument("--port", type=int, default=8080, help="Web Hub port (default: 8080)")
    hub_parser.add_argument(
        "--cameras",
        type=str,
        default="overhead,wrist,side",
        help="Comma-separated camera list (default: overhead,wrist,side)",
    )
    hub_parser.add_argument("--fps", type=int, default=50, help="Control & recording FPS (default: 50)")
    hub_parser.add_argument("--task", type=str, default="tuina acupoint massage", help="Task description prompt")
    hub_parser.add_argument("--repo-id", type=str, default=None, help="Target LeRobotDataset directory")
    hub_parser.add_argument("--mock-teleop", action="store_true", help="Use synthetic mock teleoperation")
    hub_parser.add_argument(
        "--standalone",
        action="store_true",
        default=True,
        help="Run MuJoCo in standalone in-process mode (default: True for sim)",
    )

    # 2. tuina collect terminal
    term_parser = collect_subs.add_parser(
        "terminal",
        help="Terminal interactive multi-modal collection (no browser required)",
        description="Records synchronized video and robot joint states directly from terminal.",
    )
    term_parser.add_argument("--env", choices=["sim", "real"], default="sim", help="Environment type (default: sim)")
    term_parser.add_argument(
        "--mode",
        choices=["arm_hand", "hand_only"],
        default="arm_hand",
        help="Joint control mode (default: arm_hand 22D)",
    )
    term_parser.add_argument("--episodes", type=int, default=1, help="Number of episodes to record")
    term_parser.add_argument("--frames-per-episode", type=int, default=150, help="Frames per episode")
    term_parser.add_argument(
        "--cameras",
        type=str,
        default="overhead,wrist,side",
        help="Comma-separated camera list",
    )
    term_parser.add_argument("--fps", type=int, default=50, help="FPS")
    term_parser.add_argument("--task", type=str, default="tuina acupoint massage", help="Task description")
    term_parser.add_argument("--repo-id", type=str, default=None, help="Output directory")
    term_parser.add_argument("--mock-teleop", action="store_true", help="Synthetic mock teleoperation")
    term_parser.add_argument(
        "--standalone",
        action="store_true",
        default=True,
        help="Run MuJoCo in standalone in-process mode (default: True for sim)",
    )

    # 3. tuina collect check
    check_parser = collect_subs.add_parser(
        "check",
        help="Run end-to-end regression check for Web Hub and dataset exporter",
        description="Automated E2E check with real EGL offscreen rendering and H.264 MP4 export.",
    )


def handle_collect(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Route collect subcommands to underlying scripts."""
    root = get_repo_root()
    python_bin = get_conda_python("tuinadex-control")
    dry_run = getattr(args, "dry_run", False)

    subcmd = getattr(args, "collect_subcommand", None)

    if subcmd == "hub":
        script = root / "scripts" / "collect_teleop_data.py"
        cmd = [
            str(python_bin),
            str(script),
            "--hub",
            "--env",
            args.env,
            "--mode",
            args.mode,
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
        if args.repo_id:
            cmd.extend(["--repo-id", args.repo_id])
        if args.mock_teleop:
            cmd.append("--mock-teleop")
        if getattr(args, "standalone", False) and args.env == "sim":
            cmd.append("--standalone")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run)

    elif subcmd == "terminal":
        script = root / "scripts" / "collect_teleop_data.py"
        cmd = [
            str(python_bin),
            str(script),
            "--env",
            args.env,
            "--mode",
            args.mode,
            "--episodes",
            str(args.episodes),
            "--frames-per-episode",
            str(args.frames_per_episode),
            "--cameras",
            args.cameras,
            "--fps",
            str(args.fps),
            "--task",
            args.task,
        ]
        if args.repo_id:
            cmd.extend(["--repo-id", args.repo_id])
        if args.mock_teleop:
            cmd.append("--mock-teleop")
        if getattr(args, "standalone", False) and args.env == "sim":
            cmd.append("--standalone")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run)

    elif subcmd == "check":
        script = root / "scripts" / "e2e_hub_collection_check.sh"
        cmd = ["bash", str(script)]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run)

    else:
        print("Usage: tuina collect {hub|terminal|check} [OPTIONS]")
        return 1

