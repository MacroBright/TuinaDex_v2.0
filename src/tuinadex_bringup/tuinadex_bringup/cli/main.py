"""main.py — Main Entrypoint for TuinaDex Unified CLI (tuina)."""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

from .collect import handle_collect, register_collect_subparsers
from .env_detector import get_active_env_info, get_repo_root
from .launch import handle_launch, register_launch_subparsers
from .teleop import handle_teleop, register_teleop_subparsers

__version__ = "2.0.0"

BANNER = r"""
======================================================================
  TuinaDex v2.0 — 中医推拿机械臂-灵巧手具身智能系统 CLI 控制台
======================================================================
"""


def build_parser() -> argparse.ArgumentParser:
    """Build the root parser for the tuina CLI."""
    parser = argparse.ArgumentParser(
        prog="tuina",
        description="TuinaDex v2.0 Unified Command Line Interface (CLI).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Quick Examples:
  tuina launch --sim --gui                # 启动 22D 整机物理仿真并打开 3D 视窗
  tuina launch --sim --no-gui             # 启动无头后台仿真
  tuina launch --real                     # 启动真实硬件底座 (SocketCAN + 串口)
  tuina teleop sim --mock                 # 启动 22D 仿真遥操预览 (Mock 模式)
  tuina teleop calibrate                  # 启动起飞前三合一标定向导
  tuina collect hub --env sim             # 启动 Web UI 数据采集工作台 (http://localhost:8080)
  tuina collect terminal --episodes 5     # 终端控制台交互式数据采集
  tuina collect check                     # 运行三机位 EGL+H.264 导出自动化自检
""",
    )
    parser.add_argument("-v", "--version", action="version", version=f"tuina {__version__}")
    parser.add_argument("--dry-run", action="store_true", help="Print command without executing")
    parser.add_argument("--env-info", action="store_true", help="Show environment and runtime diagnostic info")

    subparsers = parser.add_subparsers(dest="command", metavar="<command>")
    register_launch_subparsers(subparsers)
    register_teleop_subparsers(subparsers)
    register_collect_subparsers(subparsers)

    return parser


def print_env_info() -> None:
    """Print diagnosed environment status."""
    root = get_repo_root()
    print("\n--- TuinaDex Runtime Diagnostic ---")
    print(f"Repository Root: {root}")
    print(f"Python Running:  {sys.executable} ({sys.version.split()[0]})")
    # Check conda environments
    from .env_detector import get_conda_python

    control_py = get_conda_python("tuinadex-control")
    vla_py = get_conda_python("tuinadex-vla")
    print(f"tuinadex-control Python: {control_py}")
    print(f"tuinadex-vla Python:     {vla_py}")
    print("-----------------------------------\n")


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point for tuina CLI."""
    if argv is None:
        argv = sys.argv[1:]

    parser = build_parser()

    # If no arguments provided, show banner and help
    if not argv:
        print(BANNER)
        parser.print_help()
        return 0

    if "--env-info" in argv:
        print_env_info()
        return 0

    # Parse known args to allow seamless passthrough of unknown flags
    args, extra_args = parser.parse_known_args(argv)

    # Propagate global flags like dry_run even if placed after subcommands
    if "--dry-run" in argv:
        args.dry_run = True
        if "--dry-run" in extra_args:
            extra_args.remove("--dry-run")

    if args.command == "launch":
        return handle_launch(args, extra_args)
    elif args.command == "teleop":
        return handle_teleop(args, extra_args)
    elif args.command == "collect":
        return handle_collect(args, extra_args)
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())
