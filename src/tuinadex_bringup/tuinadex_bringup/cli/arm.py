"""arm.py — Handlers for 'tuina arm' 6-DOF reBot robotic arm & SocketCAN domain."""

from __future__ import annotations

import argparse
import shlex
from typing import List

from .env_detector import execute_subcommand, get_conda_python, get_repo_root
from .safety import confirm_hardware_action


def _add_rebot_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", "-c", type=str, default=None, help="reBot YAML config path")
    parser.add_argument("--speed", "-s", type=float, default=None, help="Motion speed limit (deg/s)")
    parser.add_argument("--channel", type=str, default="can0", help="SocketCAN interface for preflight check (default: can0)")


def register_arm_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'arm' subcommand and its subcommands."""
    arm_parser = subparsers.add_parser(
        "arm",
        help="6-DOF reBot arm control, ready-pose teaching & SocketCAN management",
        description="Wraps rebot.cli, arm posture teaching/verification, and SocketCAN interface operations.",
    )
    arm_subs = arm_parser.add_subparsers(dest="arm_subcommand", metavar="<subcommand>")

    # 1. tuina arm cli (interactive control console)
    cli_parser = arm_subs.add_parser("cli", help="Launch interactive rebot_cli control console")
    _add_rebot_common_args(cli_parser)

    # 2. tuina arm status
    status_parser = arm_subs.add_parser("status", help="Read current arm joint positions and motor status")
    _add_rebot_common_args(status_parser)

    # 3. tuina arm watch (aliases: drag, read)
    watch_parser = arm_subs.add_parser(
        "watch",
        aliases=["drag", "read"],
        help="Real-time unpowered joint angle stream monitor",
    )
    _add_rebot_common_args(watch_parser)
    watch_parser.add_argument("--joint", "-j", type=int, default=None, help="Watch a single joint (1~6)")
    watch_parser.add_argument("--hz", type=float, default=20.0, help="Refresh frequency in Hz (default: 20.0)")

    # 4. tuina arm zero
    zero_parser = arm_subs.add_parser("zero", help="Set current physical pose as encoder zero offset")
    _add_rebot_common_args(zero_parser)

    # 5. tuina arm move
    move_parser = arm_subs.add_parser("move", help="Move specified arm joint(s) to target degree(s)")
    _add_rebot_common_args(move_parser)
    move_parser.add_argument("--joint", "-j", type=int, default=None, help="Target joint ID (1~6)")
    move_parser.add_argument("--angle", "-a", type=float, default=None, help="Target angle in degrees")
    move_parser.add_argument("--angles", nargs="+", type=float, default=None, help="All 6 target angles in degrees")
    move_parser.add_argument("--wait", action="store_true", help="Block until smooth motion settles")
    move_parser.add_argument("--hold", action="store_true", help="Hold final pose instead of homing on exit")

    # 6. tuina arm teach
    teach_parser = arm_subs.add_parser("teach", help="Drag-teach arm ready_pose to teleop_real_posture.yaml")
    teach_parser.add_argument("--channel", type=str, default="can0", help="SocketCAN interface (default: can0)")

    # 7. tuina arm goto-ready
    goto_parser = arm_subs.add_parser("goto-ready", help="Move arm smoothly to calibrated ready_pose")
    goto_parser.add_argument("--channel", type=str, default="can0", help="SocketCAN interface (default: can0)")

    # 8. tuina arm can
    can_parser = arm_subs.add_parser("can", help="Manage SocketCAN interface (status, up, down, vcan)")
    can_subs = can_parser.add_subparsers(dest="can_action", metavar="<action>")

    can_status = can_subs.add_parser("status", help="Inspect SocketCAN link status and bitrate")
    can_status.add_argument("--channel", type=str, default="can0", help="CAN interface name (default: can0)")

    can_up = can_subs.add_parser("up", help="Bring up physical SocketCAN interface (1Mbps default)")
    can_up.add_argument("--channel", type=str, default="can0", help="CAN interface name (default: can0)")
    can_up.add_argument("--bitrate", type=int, default=1000000, help="CAN bitrate in bps (default: 1000000)")

    can_down = can_subs.add_parser("down", help="Bring down SocketCAN interface")
    can_down.add_argument("--channel", type=str, default="can0", help="CAN interface name (default: can0)")

    can_vcan = can_subs.add_parser("vcan", help="Create and bring up virtual vcan interface for offline testing")
    can_vcan.add_argument("--channel", type=str, default="vcan0", help="Virtual CAN interface (default: vcan0)")


def _build_rebot_base_cmd(python_bin: str, args: argparse.Namespace) -> List[str]:
    cmd = [python_bin, "-m", "rebot.cli"]
    if getattr(args, "config", None):
        cmd.extend(["--config", args.config])
    if getattr(args, "speed", None) is not None:
        cmd.extend(["--speed", str(args.speed)])
    return cmd


def handle_arm(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Dispatch 'tuina arm <subcommand>' safely."""
    root = get_repo_root()
    python_bin = str(get_conda_python("tuinadex-control"))
    dry_run = getattr(args, "dry_run", False)
    skip_confirm = getattr(args, "yes", False)
    subcmd = getattr(args, "arm_subcommand", None)

    if subcmd == "cli":
        if not confirm_hardware_action(
            action_name="启动 reBot 机械臂交互控制控制台",
            arm_channel=args.channel,
            warnings=["控制台支持上使能及点位控制命令，请确认急停就位！"],
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=True,
        ):
            return 1
        cmd = _build_rebot_base_cmd(python_bin, args) + ["interactive"]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "status":
        cmd = _build_rebot_base_cmd(python_bin, args) + ["status"]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd in ("watch", "drag", "read"):
        cmd = _build_rebot_base_cmd(python_bin, args) + ["watch"]
        if args.joint is not None:
            cmd.extend(["--joint", str(args.joint)])
        if args.hz is not None:
            cmd.extend(["--hz", str(args.hz)])
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "zero":
        if not confirm_hardware_action(
            action_name="写入 reBot 机械臂当前零位偏移 (zero)",
            arm_channel=args.channel,
            warnings=["该操作将覆写电机编码器零点，务必确认臂体处于标准机械零位！"],
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=True,
        ):
            return 1
        cmd = _build_rebot_base_cmd(python_bin, args) + ["zero"]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "move":
        if not confirm_hardware_action(
            action_name="驱动 reBot 机械臂关节目标运动 (move)",
            arm_channel=args.channel,
            warnings=["电机即将上使能运动至目标角度，请确认工作空间净空！"],
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=True,
        ):
            return 1
        cmd = _build_rebot_base_cmd(python_bin, args) + ["move"]
        if args.joint is not None:
            cmd.extend(["--joint", str(args.joint)])
        if args.angle is not None:
            cmd.extend(["--angle", str(args.angle)])
        if args.angles:
            cmd.extend(["--angles", *[str(val) for val in args.angles]])
        if args.wait:
            cmd.append("--wait")
        if args.hold:
            cmd.append("--hold")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "teach":
        if not confirm_hardware_action(
            action_name="启动机械臂 ready_pose 拖动示教 (teach_arm_ready_pose.py)",
            arm_channel=args.channel,
            warnings=["机械臂将进入拖动示教状态，请始终扶稳机械臂避免重力下坠！"],
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=True,
        ):
            return 1
        cmd = [python_bin, str(root / "scripts" / "teach_arm_ready_pose.py")]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "goto-ready":
        if not confirm_hardware_action(
            action_name="驱动机械臂平滑移动至标定 ready_pose (test_goto_ready_pose.py)",
            arm_channel=args.channel,
            warnings=["机械臂将上使能并运动至 ready_pose，请确认急停开关在握！"],
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=True,
        ):
            return 1
        cmd = [python_bin, str(root / "scripts" / "test_goto_ready_pose.py")]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "can":
        action = getattr(args, "can_action", None)
        if action == "status":
            cmd = ["ip", "-details", "link", "show", args.channel]
            cmd.extend(extra_args)
            return execute_subcommand(cmd, dry_run=dry_run, profile="bash")
        elif action == "up":
            if not confirm_hardware_action(
                action_name=f"开启物理 SocketCAN 接口 {args.channel} (bitrate={args.bitrate})",
                warnings=["该操作需要 sudo 权限配置内核网络接口并接通硬件 CAN 总线！"],
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
            ch = shlex.quote(args.channel)
            br = str(args.bitrate)
            shell_cmd = f"sudo ip link set {ch} down || true; sudo ip link set {ch} type can bitrate {br} && sudo ip link set {ch} up"
            return execute_subcommand(["bash", "-c", shell_cmd], dry_run=dry_run, profile="bash")
        elif action == "down":
            if not confirm_hardware_action(
                action_name=f"关闭 SocketCAN 接口 {args.channel}",
                arm_channel=args.channel,
                warnings=["关闭 CAN 总线后机械臂将失去周期通信与指令更新！"],
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
            cmd = ["sudo", "ip", "link", "set", args.channel, "down"]
            return execute_subcommand(cmd, dry_run=dry_run, profile="bash")
        elif action == "vcan":
            ch = shlex.quote(args.channel)
            shell_cmd = f"sudo modprobe vcan && (sudo ip link add dev {ch} type vcan || true) && sudo ip link set up {ch}"
            return execute_subcommand(["bash", "-c", shell_cmd], dry_run=dry_run, profile="bash")
        else:
            print("Usage: tuina arm can {status|up|down|vcan} [OPTIONS]")
            return 1

    else:
        print("Usage: tuina arm {cli|status|watch|zero|move|teach|goto-ready|can} [OPTIONS]")
        return 1
