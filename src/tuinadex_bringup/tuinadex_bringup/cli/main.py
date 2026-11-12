"""main.py — Main Entrypoint for TuinaDex Unified CLI (tuina)."""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

from .arm import handle_arm, register_arm_subparsers
from .calibrate import handle_calibrate, register_calibrate_subparsers
from .collect import handle_collect, register_collect_subparsers
from .data import handle_data, register_data_subparsers
from .env_detector import get_active_env_info, get_repo_root
from .hand import handle_hand, register_hand_subparsers
from .launch import handle_launch, register_launch_subparsers
from .sys_tools import handle_sys, register_sys_subparsers
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
  tuina launch --real                     # 启动真实硬件底座 (SocketCAN + 串口)
  tuina teleop hand                       # 启动 16-DOF LEAP Hand 真机视觉遥操作 (低延迟直驱)
  tuina teleop hand --backend ros2        # 启动 16-DOF 灵巧手 ROS 2 节点化真机遥操
  tuina teleop arm --mock                 # 启动 6-DOF reBot 机械臂遥操接口 (预留单臂)
  tuina teleop composite                  # 启动 22-DOF 臂手协同真机视觉遥操 (或 tuina teleop real)
  tuina teleop sim --mock                 # 启动 22D 仿真遥操预览 (Mock 模式)
  tuina teleop calibrate                  # 启动起飞前三合一标定向导
  tuina collect hub --env sim             # 启动 Web UI 数据采集工作台 (http://localhost:8080)
  tuina collect terminal --episodes 5     # 终端控制台交互式数据采集
  tuina collect check                     # 运行三机位 EGL+H.264 导出自动化自检
""",
    )
    parser.add_argument("-v", "--version", action="version", version=f"tuina {__version__}")
    parser.add_argument("--dry-run", action="store_true", help="Print command without executing")
    parser.add_argument("-y", "--yes", action="store_true", help="Skip interactive hardware safety prompts")
    parser.add_argument("--env-info", action="store_true", help="Show environment and runtime diagnostic info")

    subparsers = parser.add_subparsers(dest="command", metavar="<command>")
    register_launch_subparsers(subparsers)
    register_teleop_subparsers(subparsers)
    register_collect_subparsers(subparsers)
    register_calibrate_subparsers(subparsers)
    register_arm_subparsers(subparsers)
    register_hand_subparsers(subparsers)
    register_data_subparsers(subparsers)
    register_sys_subparsers(subparsers)

    return parser


def print_env_info() -> None:
    """Print diagnosed environment, ROS 2, Conda, and hardware bus status."""
    info = get_active_env_info()
    print("\n--- TuinaDex Runtime Diagnostic ---")
    print(f"Repository Root:         {info['repo_root']}")
    print(f"CLI Python Running:      {info['cli_python']}")
    print(f"ROS 2 System Python:     {info['ros2_python']}")
    print(f"tuinadex-control Python: {info['tuinadex_control_python']}")
    print(f"tuinadex-vla Python:     {info['tuinadex_vla_python']}")
    print(f"ROS Distro / Domain ID:  {info['ros_distro']} / {info['ros_domain_id']}")
    print(f"RMW / CycloneDDS Ready:  {info['rmw_implementation']} / {info['cyclonedds_installed']}")
    print(f"SocketCAN (can0):        {info['can0_status']}")
    print(f"Wrist IMU (/dev/myimu):  {info['myimu_device']}")
    print(f"USB Serial Ports:        {info['usb_serial_ports']}")
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

    # Propagate global flags like --dry-run and -y/--yes even if placed after subcommands
    raw_argv = argv if argv is not None else sys.argv[1:]
    if "--dry-run" in raw_argv or getattr(args, "dry_run", False):
        args.dry_run = True
        extra_args = [a for a in extra_args if a != "--dry-run"]
    if "--yes" in raw_argv or "-y" in raw_argv or getattr(args, "yes", False):
        args.yes = True
        extra_args = [a for a in extra_args if a not in ("-y", "--yes")]

    handlers = {
        "launch": handle_launch,
        "teleop": handle_teleop,
        "collect": handle_collect,
        "calibrate": handle_calibrate,
        "arm": handle_arm,
        "hand": handle_hand,
        "data": handle_data,
        "sys": handle_sys,
    }
    handler = handlers.get(args.command)
    if handler is not None:
        return handler(args, extra_args)
    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
