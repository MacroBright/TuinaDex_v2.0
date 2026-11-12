"""sys_tools.py — Handlers for 'tuina sys' environment, build, test, bridge & Docker domain."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Tuple

from .env_detector import (
    execute_subcommand,
    get_conda_python,
    get_repo_root,
    get_ros2_python,
)
from .safety import confirm_hardware_action

_E2E_SCRIPTS = {
    "hub": "e2e_hub_collection_check.sh",
    "unified": "e2e_unified_robot_check.sh",
    "sim": "e2e_sim_check.sh",
    "mock": "e2e_mock_check.sh",
    "collection": "e2e_collection_check.sh",
}


def _suite_test_specs(root: Path) -> dict[str, Tuple[str, str, List[str]]]:
    control_py = str(get_conda_python("tuinadex-control"))
    vla_py = str(get_conda_python("tuinadex-vla"))
    ros2_py = str(get_ros2_python())

    return {
        "cli": (
            "ros2",
            ros2_py,
            [
                "tests/test_tuina_cli.py",
                "tests/test_tuina_cli_teleop.py",
                "tests/test_tuina_cli_system.py",
            ],
        ),
        "core": (
            "control",
            control_py,
            [
                "tests/test_tuinadex_core.py",
                "tests/test_tuinadex_arm_core.py",
                "tests/test_command_processing.py",
                "tests/test_watchdog_fsm.py",
            ],
        ),
        "teleop": (
            "control",
            control_py,
            ["tests/teleop/"],
        ),
        "hub": (
            "control",
            control_py,
            [
                "tests/test_camera_rig.py",
                "tests/test_episode_buffer.py",
                "tests/test_data_channel.py",
                "tests/test_data_hub_controller.py",
                "tests/test_collection_hub_api.py",
            ],
        ),
        "dataset": (
            "vla",
            vla_py,
            [
                "tests/test_dataset_adapter.py",
                "tests/test_lerobot_v3_exporter.py",
            ],
        ),
        "ros2": (
            "ros2",
            ros2_py,
            [
                "tests/test_leap_hand_node_integration.py",
                "tests/test_arm_node_integration.py",
                "tests/test_sim_node_integration.py",
                "tests/test_tuinadex_bringup_launch.py",
            ],
        ),
    }


def register_sys_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'sys' operational subcommand."""
    sys_parser = subparsers.add_parser(
        "sys",
        help="System environment checks, colcon build, tests, E2E, bridges & completions",
        description="Operational utilities spanning host/edge setup, colcon builds, isolated test suites and bridges.",
    )
    sys_subs = sys_parser.add_subparsers(dest="sys_subcommand", metavar="<subcommand>")

    # 1. tuina sys env / verify / init-pc / init-edge / dds
    sys_subs.add_parser("env", help="Run host environment diagnostic check (check_env.sh)")
    sys_subs.add_parser("verify", help="Run full 10-point environment verification gate (verify_all_envs.sh)")

    init_pc = sys_subs.add_parser("init-pc", help="Initialize development PC setup (init_dev_pc.sh)")
    init_pc.add_argument("--install-rules", action="store_true", help="Install udev rules (requires sudo)")
    init_pc.add_argument("--setup-vcan", action="store_true", help="Create virtual CAN interface vcan0")

    init_edge = sys_subs.add_parser("init-edge", help="Initialize Orange Pi AI Pro edge target (init_orangepi.sh)")
    edge_group = init_edge.add_mutually_exclusive_group()
    edge_group.add_argument("--force", action="store_true", help="Force execution on non-aarch64 host")
    edge_group.add_argument("--check-only", action="store_true", help="Check edge dependencies without changes")

    dds = sys_subs.add_parser("dds", help="Generate or install CycloneDDS XML config (setup_cyclonedds.sh)")
    dds_group = dds.add_mutually_exclusive_group()
    dds_group.add_argument("--install", action="store_true", help="Install CycloneDDS profile")
    dds_group.add_argument("--generate-xml", action="store_true", help="Regenerate cyclonedds.xml only")

    # 2. tuina sys build
    build = sys_subs.add_parser("build", help="Build ROS 2 workspace via colcon build --symlink-install")
    build.add_argument(
        "--packages",
        "-p",
        nargs="+",
        default=None,
        help="Optional package names for --packages-select",
    )

    # 3. tuina sys test
    test = sys_subs.add_parser("test", help="Run pytest suites in matching isolated Python runtimes")
    test.add_argument(
        "--suite",
        choices=["cli", "core", "teleop", "hub", "dataset", "ros2", "all"],
        default="cli",
        help="Target test suite (default: cli)",
    )

    # 4. tuina sys e2e
    e2e = sys_subs.add_parser("e2e", help="Run automated end-to-end verification script")
    e2e.add_argument(
        "--suite",
        choices=list(_E2E_SCRIPTS.keys()),
        default="hub",
        help="E2E suite name (default: hub)",
    )

    # 5. tuina sys rosbridge / teleop-bridge / pub / probe / docker
    rb = sys_subs.add_parser("rosbridge", help="Start rosbridge WebSocket server for MCP / web tooling")
    rb.add_argument("--port", type=int, default=9090, help="WebSocket port (default: 9090)")
    rb.add_argument("--address", type=str, default="0.0.0.0", help="Bind IP address (default: 0.0.0.0)")

    sys_subs.add_parser("teleop-bridge", help="Start ROS 2 <-> UDP bridge (scripts/ros2_teleop_bridge.py)")

    pub = sys_subs.add_parser("pub", help="Run safe mock ROS 2 joint command publisher (mock_command_publisher.py)")
    pub.add_argument("--mode", choices=["hand_only", "arm_hand", "arm_only"], default="hand_only")
    pub.add_argument("--position", type=float, default=0.5, help="Hand joint position (rad)")
    pub.add_argument("--arm-position", type=float, default=0.2, help="Arm joint position (rad)")
    pub.add_argument("--rate-hz", type=float, default=50.0, help="Publish rate in Hz")
    pub.add_argument("--duration-sec", type=float, default=3.0, help="Duration in seconds")

    probe = sys_subs.add_parser("probe", help="Run MuJoCo closed-loop sim probe (sim_loop_probe.py)")
    probe.add_argument("--position", type=float, default=0.2, help="Target joint position (rad)")
    probe.add_argument("--rate-hz", type=float, default=50.0, help="Probe command rate (Hz)")
    probe.add_argument("--active-sec", type=float, default=1.5, help="Active command duration (s)")
    probe.add_argument("--settle-sec", type=float, default=2.0, help="Watchdog settle duration (s)")

    sys_subs.add_parser("docker", help="Launch ros:lyrical container shell (scripts/ros2_docker.sh)")

    # 6. tuina sys completion
    comp = sys_subs.add_parser("completion", help="Show or print shell Tab completion script for Bash/Zsh")
    comp.add_argument("--shell", choices=["bash", "zsh"], default="bash", help="Target shell (default: bash)")
    comp.add_argument(
        "--install-hint",
        action="store_true",
        help="Print one-line eval/source snippet for ~/.bashrc or ~/.zshrc",
    )


def handle_sys(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Dispatch 'tuina sys <subcommand>' safely across host, ROS 2, and conda runtimes."""
    root = get_repo_root()
    ros2_py = str(get_ros2_python())
    dry_run = getattr(args, "dry_run", False)
    skip_confirm = getattr(args, "yes", False)
    subcmd = getattr(args, "sys_subcommand", None)

    if subcmd == "env":
        return execute_subcommand(["bash", str(root / "scripts" / "env" / "check_env.sh"), *extra_args], dry_run=dry_run, profile="bash")

    elif subcmd == "verify":
        return execute_subcommand(["bash", str(root / "scripts" / "verify_all_envs.sh"), *extra_args], dry_run=dry_run, profile="bash")

    elif subcmd == "init-pc":
        cmd = ["bash", str(root / "scripts" / "env" / "init_dev_pc.sh")]
        if args.install_rules:
            if not confirm_hardware_action(
                action_name="安装系统级 udev 硬件规则 (需要 sudo 权限)",
                warnings=["将会写入 /etc/udev/rules.d/ 并重载 udev 规则"],
                skip_confirm=skip_confirm,
                dry_run=dry_run,
            ):
                return 1
            cmd = ["sudo", "bash", str(root / "scripts" / "env" / "init_dev_pc.sh"), "--install-rules"]
        elif args.setup_vcan:
            cmd.append("--setup-vcan")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="bash")

    elif subcmd == "init-edge":
        cmd = ["bash", str(root / "scripts" / "env" / "init_orangepi.sh")]
        if args.force:
            cmd.append("--force")
        elif args.check_only:
            cmd.append("--check-only")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="bash")

    elif subcmd == "dds":
        cmd = ["bash", str(root / "scripts" / "env" / "setup_cyclonedds.sh")]
        if args.install:
            cmd.append("--install")
        elif args.generate_xml:
            cmd.append("--generate-xml")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="bash")

    elif subcmd == "build":
        cmd = ["colcon", "build", "--symlink-install"]
        if args.packages:
            cmd.extend(["--packages-select", *args.packages])
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    elif subcmd == "test":
        specs = _suite_test_specs(root)
        suite_names = list(specs.keys()) if args.suite == "all" else [args.suite]
        for suite_name in suite_names:
            profile, py_bin, files = specs[suite_name]
            existing_or_spec = [str(root / f) if not f.endswith("/") else str(root / f.rstrip("/")) for f in files]
            cmd = [py_bin, "-m", "pytest", *existing_or_spec, "-q", *extra_args]
            rc = execute_subcommand(cmd, dry_run=dry_run, profile=profile)
            if rc != 0:
                return rc
        return 0

    elif subcmd == "e2e":
        script_file = root / "scripts" / _E2E_SCRIPTS[args.suite]
        return execute_subcommand(["bash", str(script_file), *extra_args], dry_run=dry_run, profile="bash")

    elif subcmd == "rosbridge":
        cmd = ["bash", str(root / "scripts" / "start_rosbridge.sh"), str(args.port), args.address, *extra_args]
        return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    elif subcmd == "teleop-bridge":
        cmd = [ros2_py, str(root / "scripts" / "ros2_teleop_bridge.py"), *extra_args]
        return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    elif subcmd == "pub":
        cmd = [
            ros2_py,
            str(root / "scripts" / "mock_command_publisher.py"),
            "--mode",
            args.mode,
            "--position",
            str(args.position),
            "--arm-position",
            str(args.arm_position),
            "--rate-hz",
            str(args.rate_hz),
            "--duration-sec",
            str(args.duration_sec),
            *extra_args,
        ]
        return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    elif subcmd == "probe":
        cmd = [
            ros2_py,
            str(root / "scripts" / "sim_loop_probe.py"),
            "--position",
            str(args.position),
            "--rate-hz",
            str(args.rate_hz),
            "--active-sec",
            str(args.active_sec),
            "--settle-sec",
            str(args.settle_sec),
            *extra_args,
        ]
        return execute_subcommand(cmd, dry_run=dry_run, profile="ros2")

    elif subcmd == "docker":
        return execute_subcommand(["bash", str(root / "scripts" / "ros2_docker.sh"), *extra_args], dry_run=dry_run, profile="bash")

    elif subcmd == "completion":
        comp_path = root / "bin" / "tuina-completion.bash"
        if args.install_hint:
            if args.shell == "zsh":
                print(f'autoload -U +X bashcompinit && bashcompinit && source "{comp_path}"')
            else:
                print(f'source "{comp_path}"')
            return 0
        if not comp_path.is_file():
            print(f"[TUINA ERROR] Completion script not found: {comp_path}")
            return 1
        print(comp_path.read_text(encoding="utf-8"))
        return 0

    else:
        print("Usage: tuina sys {env|verify|init-pc|init-edge|dds|build|test|e2e|rosbridge|teleop-bridge|pub|probe|docker|completion} [OPTIONS]")
        return 1
