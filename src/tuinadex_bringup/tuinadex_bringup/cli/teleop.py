"""teleop.py — Handlers for 'tuina teleop' subcommands."""

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import List, Optional

from .env_detector import execute_subcommand, get_conda_python, get_repo_root
from .safety import confirm_hardware_action


def resolve_hand_port(requested_port: str, imu_symlink: str = "/dev/myimu") -> tuple[str, Optional[str]]:
    """Resolve the LEAP Hand U2D2 serial port while avoiding collisions with the IMU port.

    Returns:
        (resolved_port, warning_or_info_message)
    """
    imu_real = os.path.realpath(imu_symlink) if os.path.exists(imu_symlink) else None
    req_real = os.path.realpath(requested_port) if os.path.exists(requested_port) else None

    def _find_ftdi_candidate() -> Optional[str]:
        for path in sorted(glob.glob("/dev/serial/by-id/*")):
            if "ftdi" in os.path.basename(path).lower() and os.path.exists(path):
                real = os.path.realpath(path)
                if imu_real is None or real != imu_real:
                    return real
        for path in sorted(glob.glob("/dev/ttyUSB*")):
            if os.path.exists(path):
                real = os.path.realpath(path)
                if imu_real is None or real != imu_real:
                    return real
        return None

    # Auto-detect when using default port and it is missing or collides with IMU,
    # or when an FTDI U2D2 adapter is explicitly present at another ttyUSB node.
    if requested_port == "/dev/ttyUSB0":
        if imu_real is not None and req_real == imu_real:
            alt = _find_ftdi_candidate()
            if alt:
                return alt, (
                    f"[INFO] 默认端口 {requested_port} 当前被 IMU ({imu_symlink} -> {req_real}) 占用，"
                    f"已自动切换灵巧手端口为 {alt}。"
                )
            return requested_port, (
                f"[WARN] 默认端口 {requested_port} 与 IMU ({imu_symlink} -> {req_real}) 冲突且未发现备用串口，将回退 mock_hand。"
            )
        if not os.path.exists(requested_port):
            alt = _find_ftdi_candidate()
            if alt:
                return alt, f"[INFO] 未检测到 {requested_port}，已自动发现灵巧手串口 {alt}。"
        return requested_port, None

    if imu_real is not None and req_real == imu_real:
        return requested_port, (
            f"[WARN] 指定端口 {requested_port} ({req_real}) 与 IMU ({imu_symlink}) 冲突！请检查串口分配。"
        )
    return requested_port, None


def register_teleop_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'teleop' subcommands."""
    teleop_parser = subparsers.add_parser(
        "teleop",
        help="Teleoperation, vision tracking and calibration tools",
        description="TuinaDex Teleoperation Subsystem.",
    )
    teleop_subs = teleop_parser.add_subparsers(dest="teleop_subcommand")

    # 1. tuina teleop hand (16-DOF 灵巧手真机遥操)
    hand_parser = teleop_subs.add_parser(
        "hand",
        help="16-DOF LEAP Hand real hardware teleoperation",
        description="Low-latency vision teleoperation for 16-DOF LEAP Hand (direct or ROS 2).",
    )
    hand_parser.add_argument(
        "--backend",
        type=str,
        choices=["direct", "ros2"],
        default="direct",
        help="Execution backend: 'direct' (Python + 4Mbps serial, default) or 'ros2' (ROS 2 launch)",
    )
    hand_parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for LEAP Hand (default: /dev/ttyUSB0)",
    )
    hand_parser.add_argument(
        "--camera-id",
        type=int,
        default=4,
        help="Camera device ID (default: 4 for RealSense RGB)",
    )
    hand_parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Control loop rate in Hz (default: 50.0)",
    )
    hand_parser.add_argument(
        "--mock",
        action="store_true",
        help="Run in offline mock mode (mock hand & mock camera)",
    )
    hand_parser.add_argument(
        "--mock-camera",
        action="store_true",
        help="Use synthetic frames instead of physical camera",
    )
    hand_parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Headless mode without OpenCV tracking window",
    )
    hand_parser.add_argument(
        "--handedness",
        choices=["Right", "Left", "any"],
        default="Right",
        help="Filter handedness (default: Right)",
    )
    hand_parser.add_argument(
        "--publish-ros2",
        action="store_true",
        help="Also publish /hand/joint_command to ROS 2 in direct mode",
    )
    hand_parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Exit after N frames (useful for automated testing)",
    )

    # 2. tuina teleop arm (6-DOF 机械臂单臂遥操预留)
    arm_parser = teleop_subs.add_parser(
        "arm",
        help="6-DOF reBot arm teleoperation (reserved for single arm control)",
        description="Teleoperation interface reserved for 6-DOF reBot robotic arm.",
    )
    arm_parser.add_argument(
        "--channel",
        type=str,
        default="can0",
        help="SocketCAN interface for reBot arm (default: can0)",
    )
    arm_parser.add_argument(
        "--camera-id",
        type=int,
        default=4,
        help="Camera device ID (default: 4 for RealSense RGB)",
    )
    arm_parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Control loop rate in Hz (default: 50.0)",
    )
    arm_parser.add_argument(
        "--mock",
        action="store_true",
        help="Run in offline mock mode",
    )
    arm_parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Headless mode without OpenCV tracking window",
    )
    arm_parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Exit after N frames",
    )

    # 3. tuina teleop composite (22-DOF 臂手联合真机遥操)
    composite_parser = teleop_subs.add_parser(
        "composite",
        help="22-DOF Composite (Arm + Hand) Real Teleoperation",
        description="Fuses RealSense 3D tracking, IMU attitude, reBot arm and LEAP hand.",
    )
    _add_composite_arguments(composite_parser)

    # 4. tuina teleop real (别名入口)
    real_parser = teleop_subs.add_parser(
        "real",
        help="Alias for 'tuina teleop composite' (22-DOF Real Hardware Teleoperation)",
        description="22-DOF Real Hardware Teleoperation shortcut.",
    )
    _add_composite_arguments(real_parser)

    # 5. tuina teleop sim (22-DOF 纯仿真遥操)
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

    # 6. tuina teleop calibrate
    calib_parser = teleop_subs.add_parser(
        "calibrate",
        help="Interactive pre-flight calibration wizard (table + IMU + hand)",
        description="Calibrates tabletop plane, wrist IMU neutral/axes, and hand ROM.",
    )
    calib_parser.add_argument("--imu-only", action="store_true", help="Only calibrate wrist IMU 3-gesture alignment")
    calib_parser.add_argument(
        "--camera-id",
        type=int,
        default=None,
        help="RealSense / V4L2 camera device ID (omit for auto-detection)",
    )
    calib_parser.add_argument("--imu-port", type=str, default="/dev/myimu", help="IMU serial port device (default: /dev/myimu)")
    calib_parser.add_argument("--baudrate", type=int, default=115200, help="IMU serial baudrate (default: 115200)")
    calib_parser.add_argument("--mock", action="store_true", help="Run calibration in mock mode")
    calib_parser.add_argument("--auto", action="store_true", help="Automatic countdown mode for IMU calibration")
    calib_parser.add_argument("--no-gui", action="store_true", help="Headless terminal mode")
    calib_parser.add_argument("--skip-table", action="store_true", help="Skip Stage 1 table calibration")
    calib_parser.add_argument("--skip-imu", action="store_true", help="Skip Stage 2 IMU calibration")
    calib_parser.add_argument("--skip-hand", action="store_true", help="Skip Stage 3 hand calibration")
    calib_parser.add_argument("--output-session", type=str, default=None, help="Output session YAML path (with --imu-only)")
    calib_parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output calibration YAML path (maps to --output-imu when --imu-only)",
    )

    # 7. tuina teleop node
    node_parser = teleop_subs.add_parser(
        "node",
        help="Launch ROS 2 composite teleoperation node via teleop_sim.launch.py",
        description="Publishes /arm/joint_command and /hand/joint_command to ROS 2.",
    )
    node_parser.add_argument("--rate-hz", type=float, default=60.0, help="Publish rate in Hz")
    node_parser.add_argument("--camera-fps", type=int, default=60, help="Camera acquisition FPS")
    node_parser.add_argument("--mock", action="store_true", help="Run mock tracking")
    node_parser.add_argument("--headless", action="store_true", help="Do not open OpenCV display window")


def _add_composite_arguments(parser: argparse.ArgumentParser) -> None:
    """Add arguments common to composite / real teleoperation commands."""
    parser.add_argument(
        "--arm-channel",
        type=str,
        default="can0",
        help="SocketCAN interface for reBot arm (default: can0)",
    )
    parser.add_argument(
        "--hand-port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for LEAP Hand (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--camera-id",
        type=int,
        default=4,
        help="Camera device ID (default: 4 for RealSense RGB)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=60.0,
        help="Unified control and teleoperation rate in Hz (default: 60.0)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run both arm/hand drivers and camera in offline mock mode",
    )
    parser.add_argument(
        "--mock-hand",
        action="store_true",
        help="Run hand driver in mock mode while arm uses real hardware",
    )
    parser.add_argument(
        "--no-imu",
        action="store_true",
        help="Disable IMU attitude tracking",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Headless mode without OpenCV tracking window",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Exit after N frames (useful for testing)",
    )


def handle_teleop(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Route teleop subcommands to underlying scripts or ROS 2 launches."""
    root = get_repo_root()
    python_bin = get_conda_python("tuinadex-control")
    dry_run = getattr(args, "dry_run", False)
    skip_confirm = getattr(args, "yes", False)

    subcmd = getattr(args, "teleop_subcommand", None)

    if subcmd == "hand":
        resolved_port, port_msg = resolve_hand_port(args.port)
        if port_msg:
            print(port_msg)
        if not args.mock:
            if not confirm_hardware_action(
                action_name=f"启动 16-DOF LEAP Hand 真机视觉遥操作 (backend={args.backend})",
                hand_port=resolved_port,
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        if args.backend == "direct":
            cmd = [
                str(python_bin),
                "-m",
                "tuinadex_hand.cli.teleop",
                "--port",
                resolved_port,
                "--camera-id",
                str(args.camera_id),
                "--rate",
                str(args.rate),
            ]
            if args.mock:
                cmd.append("--mock")
            if args.mock_camera:
                cmd.append("--mock-camera")
            if args.no_preview:
                cmd.append("--no-preview")
            if args.handedness:
                cmd.extend(["--handedness", args.handedness])
            if args.publish_ros2:
                cmd.append("--publish-ros2")
            if args.max_frames > 0:
                cmd.extend(["--max-frames", str(args.max_frames)])
            cmd.extend(extra_args)
            return execute_subcommand(cmd, dry_run=dry_run, profile="control")
        else:
            # Backend: ros2
            cmd = [
                "ros2",
                "launch",
                "tuinadex_teleop",
                "teleop_hand_real.launch.py",
                f"port:={resolved_port}",
                f"camera_id:={args.camera_id}",
                f"rate_hz:={args.rate}",
                f"mock:={'true' if args.mock else 'false'}",
                f"show_preview:={'false' if args.no_preview else 'true'}",
            ]
            cmd.extend(extra_args)
            return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    elif subcmd == "arm":
        # 6-DOF reBot 机械臂遥操接口预留
        print("\n" + "=" * 65)
        print("      TuinaDex v2.0 — 6-DOF reBot 机械臂遥操作控制接口")
        print(f"      (SocketCAN 通道: {args.channel}, Rate: {args.rate}Hz, Mock: {args.mock})")
        print("=" * 65)
        if dry_run:
            print(f"[DRY-RUN] SocketCAN 通道={args.channel}, Camera ID={args.camera_id}, Rate={args.rate}Hz")
            return 0
        if args.mock:
            print(f"[TuinaDex] reBot 机械臂 Mock 遥操就绪。已连接虚拟 SocketCAN 通道 ({args.channel})。")
            return 0
        else:
            print(f"[TuinaDex] 正在监听 SocketCAN 通道 ({args.channel})...")
            print("[提示] 机械臂已连接至 TuinaDex 控制总线。若需臂手协同，请运行 'tuina teleop composite'。")
            return 0

    elif subcmd in ("composite", "real"):
        resolved_hand_port, port_msg = resolve_hand_port(args.hand_port)
        if port_msg:
            print(port_msg)
        imu_real = os.path.realpath("/dev/myimu") if os.path.exists("/dev/myimu") else None
        port_collides_imu = (
            imu_real is not None
            and os.path.exists(resolved_hand_port)
            and os.path.realpath(resolved_hand_port) == imu_real
        )
        use_mock_hand = (
            getattr(args, "mock_hand", False)
            or not os.path.exists(resolved_hand_port)
            or port_collides_imu
        )
        if not args.mock:
            if not confirm_hardware_action(
                action_name="启动 22-DOF 臂手协同真机遥操作",
                arm_channel=args.arm_channel,
                hand_port=None if use_mock_hand else resolved_hand_port,
                imu_port=None if args.no_imu else "/dev/myimu",
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        cmd = [
            "ros2",
            "launch",
            "tuinadex_teleop",
            "teleop_composite_real.launch.py",
            f"arm_channel:={args.arm_channel}",
            f"hand_port:={resolved_hand_port}",
            f"camera_id:={args.camera_id}",
            f"rate_hz:={args.rate_hz}",
            f"mock:={'true' if args.mock else 'false'}",
            f"mock_hand:={'true' if use_mock_hand else 'false'}",
            f"no_imu:={'true' if args.no_imu else 'false'}",
            f"gui:={'false' if args.no_gui else 'true'}",
        ]
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    elif subcmd == "sim":
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
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "calibrate":
        if not args.mock:
            if not confirm_hardware_action(
                action_name="执行视-惯-手遥操作起飞前标定",
                imu_port=None if args.skip_imu else args.imu_port,
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
        if args.imu_only:
            script = root / "scripts" / "calibrate_imu.py"
            cmd = [
                str(python_bin),
                str(script),
                "--port",
                args.imu_port,
                "--baudrate",
                str(getattr(args, "baudrate", 115200)),
            ]
            if args.mock:
                cmd.append("--mock")
            if getattr(args, "auto", False):
                cmd.append("--auto")
            if getattr(args, "output_session", None):
                cmd.extend(["--output-session", args.output_session])
            if args.output:
                cmd.extend(["--output-imu", args.output])
        else:
            script = root / "scripts" / "00_preflight_calibration.py"
            cmd = [
                str(python_bin),
                str(script),
                "--imu-port",
                args.imu_port,
            ]
            if args.camera_id is not None:
                cmd.extend(["--camera-id", str(args.camera_id)])
            if args.mock:
                cmd.append("--mock")
            if args.no_gui:
                cmd.append("--no-gui")
            if args.skip_table:
                cmd.append("--skip-table")
            if args.skip_imu:
                cmd.append("--skip-imu")
            if args.skip_hand:
                cmd.append("--skip-hand")
            if args.output:
                cmd.extend(["--output", args.output])
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="control")

    elif subcmd == "node":
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
        return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    else:
        print("Usage: tuina teleop {hand|arm|composite|real|sim|calibrate|node} [OPTIONS]")
        return 1
