"""calibrate.py — Handlers for unified 'tuina calibrate' system calibration domain."""

from __future__ import annotations

import argparse
from typing import List

from .env_detector import execute_subcommand, get_conda_python, get_repo_root
from .safety import confirm_hardware_action
from .teleop import resolve_hand_port


def register_calibrate_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'calibrate' subcommand and its system-wide calibration wizards."""
    calib_parser = subparsers.add_parser(
        "calibrate",
        help="Unified calibration domain (preflight, IMU, LEAP hand, reBot arm)",
        description="Orchestrates tabletop/camera/IMU/hand/arm calibrations across the TuinaDex system.",
    )
    calib_subs = calib_parser.add_subparsers(dest="calibrate_subcommand", metavar="<subcommand>")

    # 1. tuina calibrate all | preflight
    all_parser = calib_subs.add_parser(
        "all",
        aliases=["preflight"],
        help="Run 3-in-1 preflight calibration wizard (tabletop + wrist IMU + hand ROM)",
    )
    all_parser.add_argument("--camera-id", type=str, default=None, help="RealSense serial or V4L2 index (auto if omitted)")
    all_parser.add_argument("--imu-port", type=str, default="/dev/myimu", help="IMU serial port (default: /dev/myimu)")
    all_parser.add_argument("--imu-axis-mapping", type=str, default=None, help="Override IMU-to-camera axis mapping")
    all_parser.add_argument("--axis-mapping", type=str, default=None, help="Override teleop axis mapping")
    all_parser.add_argument("--skip-table", action="store_true", help="Skip Stage 1 tabletop calibration")
    all_parser.add_argument("--skip-imu", action="store_true", help="Skip Stage 2 IMU calibration")
    all_parser.add_argument("--skip-hand", action="store_true", help="Skip Stage 3 hand ROM calibration")
    all_parser.add_argument("--no-gui", action="store_true", help="Run headless without OpenCV GUI")
    all_parser.add_argument("--mock", action="store_true", help="Run offline with mock devices")
    all_parser.add_argument("--output", type=str, default=None, help="Output preflight calibration YAML path")

    # 2. tuina calibrate imu
    imu_parser = calib_subs.add_parser(
        "imu",
        help="Calibrate wrist IMU neutral quaternion & 3-gesture axis alignment",
    )
    imu_parser.add_argument("--port", type=str, default="/dev/myimu", help="IMU serial port (default: /dev/myimu)")
    imu_parser.add_argument("--baudrate", type=int, default=115200, help="IMU baudrate (default: 115200)")
    imu_parser.add_argument("--mock", action="store_true", help="Use synthetic mock IMU data")
    imu_parser.add_argument("--auto", action="store_true", help="Automatic countdown without keypress wait")
    imu_parser.add_argument("--output-session", type=str, default=None, help="Output session YAML path")
    imu_parser.add_argument("--output-imu", "--output", dest="output_imu", type=str, default=None, help="Output IMU YAML path")

    # 3. tuina calibrate hand
    hand_parser = calib_subs.add_parser(
        "hand",
        help="Calibrate 16-DOF LEAP Hand open/custom reference zero poses",
    )
    hand_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="LEAP Hand serial port")
    hand_parser.add_argument("--action", choices=["1", "2"], default=None, help="1=Open pose zeroing, 2=Record custom pose")
    hand_parser.add_argument("--mock", action="store_true", help="Run in offline mock mode")

    # 4. tuina calibrate hand-limits
    limits_parser = calib_subs.add_parser(
        "hand-limits",
        help="Sweep & record 16-DOF LEAP Hand physical motor joint limits",
    )
    limits_parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="LEAP Hand serial port")
    limits_parser.add_argument("--out", type=str, default=None, help="Output motor limits JSON path")
    limits_parser.add_argument("--motor", type=int, nargs="+", default=None, help="Test specific motor IDs only")
    limits_parser.add_argument("--mock", action="store_true", help="Run in offline mock mode")

    # 5. tuina calibrate arm-ready
    arm_ready_parser = calib_subs.add_parser(
        "arm-ready",
        help="Interactive compliant drag-teaching wizard for arm ready_pose",
    )
    arm_ready_parser.add_argument("--channel", type=str, default="can0", help="SocketCAN interface (default: can0)")

    # 6. tuina calibrate arm-goto
    arm_goto_parser = calib_subs.add_parser(
        "arm-goto",
        help="Smooth quintic verification move to calibrated arm ready_pose",
    )
    arm_goto_parser.add_argument("--channel", type=str, default="can0", help="SocketCAN interface (default: can0)")


def handle_calibrate(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Dispatch 'tuina calibrate <subcommand>' cleanly across calibrated subsystems."""
    root = get_repo_root()
    python_bin = get_conda_python("tuinadex-control")
    dry_run = getattr(args, "dry_run", False)
    skip_confirm = getattr(args, "yes", False)
    subcmd = getattr(args, "calibrate_subcommand", None)

    if subcmd in ("all", "preflight"):
        if not args.mock:
            if not confirm_hardware_action(
                action_name="执行三合一遥操作起飞前标定 (桌面 + 手腕IMU + 灵巧手)",
                imu_port=None if args.skip_imu else args.imu_port,
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        cmd = [str(python_bin), str(root / "scripts" / "00_preflight_calibration.py"), "--imu-port", args.imu_port]
        if args.camera_id is not None:
            cmd.extend(["--camera-id", str(args.camera_id)])
        if args.imu_axis_mapping:
            cmd.extend(["--imu-axis-mapping", args.imu_axis_mapping])
        if args.axis_mapping:
            cmd.extend(["--axis-mapping", args.axis_mapping])
        if args.skip_table:
            cmd.append("--skip-table")
        if args.skip_imu:
            cmd.append("--skip-imu")
        if args.skip_hand:
            cmd.append("--skip-hand")
        if args.no_gui:
            cmd.append("--no-gui")
        if args.mock:
            cmd.append("--mock")
        if args.output:
            cmd.extend(["--output", args.output])
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "imu":
        if not args.mock:
            if not confirm_hardware_action(
                action_name="执行手腕 IMU 零位与三姿态轴向标定",
                imu_port=args.port,
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        cmd = [
            str(python_bin),
            str(root / "scripts" / "calibrate_imu.py"),
            "--port",
            args.port,
            "--baudrate",
            str(args.baudrate),
        ]
        if args.mock:
            cmd.append("--mock")
        if args.auto:
            cmd.append("--auto")
        if args.output_session:
            cmd.extend(["--output-session", args.output_session])
        if args.output_imu:
            cmd.extend(["--output-imu", args.output_imu])
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "hand":
        resolved_port, port_msg = resolve_hand_port(args.port)
        if port_msg:
            print(port_msg)
        if not args.mock:
            if not confirm_hardware_action(
                action_name="执行 16-DOF LEAP Hand 零位标定",
                hand_port=resolved_port,
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        cmd = [str(python_bin), "-m", "tuinadex_hand.cli.calibrate", "--port", resolved_port]
        if args.action:
            cmd.extend(["--action", args.action])
        if args.mock:
            cmd.append("--mock")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "hand-limits":
        resolved_port, port_msg = resolve_hand_port(args.port)
        if port_msg:
            print(port_msg)
        if not args.mock:
            if not confirm_hardware_action(
                action_name="执行 16-DOF LEAP Hand 电机物理限位扫描",
                hand_port=resolved_port,
                warnings=["将会逐个驱动舵机检测堵转电流极限，请保证手指周围无障碍物！"],
                skip_confirm=skip_confirm,
                dry_run=dry_run,
                high_risk=True,
            ):
                return 1
        cmd = [str(python_bin), "-m", "tuinadex_hand.cli.diagnostics", "--port", resolved_port]
        if args.out:
            cmd.extend(["--out", args.out])
        if args.motor:
            cmd.extend(["--motor", *[str(m) for m in args.motor]])
        if args.mock:
            cmd.append("--mock")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "arm-ready":
        if not confirm_hardware_action(
            action_name="执行 6-DOF reBot 机械臂就绪姿态拖动示教",
            arm_channel=args.channel,
            warnings=["机械臂将进入低阻力示教模式，请托扶末端防止重力下坠！"],
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=True,
        ):
            return 1
        cmd = [str(python_bin), str(root / "scripts" / "teach_arm_ready_pose.py")]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "arm-goto":
        if not confirm_hardware_action(
            action_name="执行 6-DOF reBot 机械臂回归 ready_pose 验证运动",
            arm_channel=args.channel,
            warnings=["机械臂将上使能并按五次多项式插值平缓运动至标定 ready_pose！"],
            skip_confirm=skip_confirm,
            dry_run=dry_run,
            high_risk=True,
        ):
            return 1
        cmd = [str(python_bin), str(root / "scripts" / "test_goto_ready_pose.py")]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    else:
        print("Usage: tuina calibrate {all|preflight|imu|hand|hand-limits|arm-ready|arm-goto} [OPTIONS]")
        return 1
