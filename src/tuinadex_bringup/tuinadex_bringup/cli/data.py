"""data.py — Handlers for 'tuina data' dataset conversion, VLA containers & inspection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List

from .env_detector import execute_subcommand, get_conda_python, get_repo_root


def register_data_subparsers(subparsers: argparse._SubParsersAction) -> None:
    """Register 'data' subcommand for LeRobot v3.0 dataset and VLA runtime workflows."""
    data_parser = subparsers.add_parser(
        "data",
        help="Offline rosbag2 -> LeRobotDataset v3.0 conversion, inspection & VLA containers",
        description="Manages dataset conversion in isolated tuinadex-vla runtime and VLA Docker containers.",
    )
    data_subs = data_parser.add_subparsers(dest="data_subcommand", metavar="<subcommand>")

    # 1. tuina data convert
    conv_parser = data_subs.add_parser(
        "convert",
        help="Convert rosbag2 recording to LeRobotDataset v3.0 using tuinadex-vla",
    )
    conv_parser.add_argument("--bag", required=True, type=str, help="Input rosbag2 directory path")
    conv_parser.add_argument("--out", required=True, type=str, help="Output LeRobotDataset repo_id / path")
    conv_parser.add_argument("--fps", type=float, default=50.0, help="Target resampled frequency in Hz (default: 50.0)")
    conv_parser.add_argument(
        "--allow-invalid",
        action="store_true",
        help="Proceed with export even if sync/range validation warns",
    )

    # 2. tuina data convert-docker
    conv_docker_parser = data_subs.add_parser(
        "convert-docker",
        help="Run isolated Docker LeRobot data converter container (scripts/run_data_convert.sh)",
    )
    conv_docker_parser.add_argument("--build", action="store_true", help="Build/rebuild the data Docker image first")

    # 3. tuina data vla-cloud
    vla_parser = data_subs.add_parser(
        "vla-cloud",
        help="Launch cloud GPU VLA training & inference container (scripts/launch_vla_cloud.sh)",
    )
    vla_parser.add_argument("--build", action="store_true", help="Build/rebuild the VLA cloud Docker image")

    # 4. tuina data inspect
    inspect_parser = data_subs.add_parser(
        "inspect",
        help="Inspect a converted LeRobotDataset directory metadata and video tracks",
    )
    inspect_parser.add_argument("dataset_dir", type=str, help="Path to LeRobotDataset root directory")


def _inspect_dataset(dataset_dir: Path, dry_run: bool = False) -> int:
    if dry_run:
        print(f"[DRY-RUN][vla] inspect dataset metadata under {dataset_dir}")
        return 0

    if not dataset_dir.exists():
        print(f"[DATA ERROR] Dataset directory does not exist: {dataset_dir}")
        return 1

    info_file = dataset_dir / "meta" / "info.json"
    print(f"=== LeRobotDataset Summary: {dataset_dir} ===")
    if info_file.is_file():
        try:
            info = json.loads(info_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[DATA ERROR] Failed to parse {info_file}: {exc}")
            return 1
        print(f"  codebase_version : {info.get('codebase_version', 'unknown')}")
        print(f"  robot_type       : {info.get('robot_type', 'unknown')}")
        print(f"  fps              : {info.get('fps', 'unknown')}")
        print(f"  total_episodes   : {info.get('total_episodes', 'unknown')}")
        print(f"  total_frames     : {info.get('total_frames', 'unknown')}")
        features = list((info.get("features") or {}).keys())
        print(f"  features ({len(features)})    : {', '.join(features)}")
    else:
        print("  [WARN] meta/info.json not found.")

    mp4_count = len(list(dataset_dir.rglob("*.mp4")))
    parquet_count = len(list(dataset_dir.rglob("*.parquet")))
    print(f"  mp4_files        : {mp4_count}")
    print(f"  parquet_files    : {parquet_count}")
    return 0


def handle_data(args: argparse.Namespace, extra_args: List[str]) -> int:
    """Dispatch 'tuina data <subcommand>' to tuinadex-vla or Docker runners."""
    root = get_repo_root()
    dry_run = getattr(args, "dry_run", False)
    subcmd = getattr(args, "data_subcommand", None)

    if subcmd == "convert":
        vla_py = get_conda_python("tuinadex-vla")
        cmd = [
            str(vla_py),
            "-m",
            "tuinadex_dataset.cli",
            "--bag",
            args.bag,
            "--out",
            args.out,
            "--fps",
            str(args.fps),
        ]
        if args.allow_invalid:
            cmd.append("--allow-invalid")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="vla")

    elif subcmd == "convert-docker":
        cmd = ["bash", str(root / "scripts" / "run_data_convert.sh")]
        if args.build:
            cmd.append("--build")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="bash")

    elif subcmd == "vla-cloud":
        cmd = ["bash", str(root / "scripts" / "launch_vla_cloud.sh")]
        if args.build:
            cmd.append("--build")
        cmd.extend(extra_args)
        return execute_subcommand(cmd, dry_run=dry_run, profile="bash")

    elif subcmd == "inspect":
        return _inspect_dataset(Path(args.dataset_dir), dry_run=dry_run)

    else:
        print("Usage: tuina data {convert|convert-docker|vla-cloud|inspect} [OPTIONS]")
        return 1
