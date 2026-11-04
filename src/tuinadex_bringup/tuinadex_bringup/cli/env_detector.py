"""env_detector.py — Environment and runtime detection for TuinaDex CLI."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


def get_repo_root() -> Path:
    """Resolve the TuinaDex repository root directory."""
    # Priority: TUINADEX_ROOT env -> marker files check (.agent or scripts/activate_tuinadex.sh)
    if "TUINADEX_ROOT" in os.environ and Path(os.environ["TUINADEX_ROOT"]).is_dir():
        return Path(os.environ["TUINADEX_ROOT"]).resolve()

    # Traverse upward from this file
    current = Path(__file__).resolve().parent
    for _ in range(5):
        if (current / "scripts" / "activate_tuinadex.sh").exists() or (current / ".agent").exists():
            return current
        if current.parent == current:
            break
        current = current.parent

    # Default fallback to cwd
    return Path.cwd().resolve()


def get_conda_python(env_name: str = "tuinadex-control") -> Path:
    """Find the specific python executable for a conda environment."""
    env_override_key = f"{env_name.upper().replace('-', '_')}_PYTHON"
    if env_override_key in os.environ:
        p = Path(os.environ[env_override_key])
        if p.exists():
            return p

    home = Path.home()
    candidate_paths = [
        home / "win_office" / "conda" / "envs" / env_name / "bin" / "python",
        home / "miniconda3" / "envs" / env_name / "bin" / "python",
        home / ".conda" / "envs" / env_name / "bin" / "python",
        Path("/opt/conda/envs") / env_name / "bin" / "python",
    ]

    for candidate in candidate_paths:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return candidate

    # If already inside the desired env
    if env_name in sys.executable:
        return Path(sys.executable)

    # Fallback to current sys.executable
    return Path(sys.executable)


def get_active_env_info() -> Dict[str, str]:
    """Gather diagnostic environment information dictionary."""
    root = get_repo_root()
    return {
        "repo_root": str(root),
        "current_python": sys.executable,
        "python_version": sys.version.split()[0],
        "tuinadex_control_python": str(get_conda_python("tuinadex-control")),
        "tuinadex_vla_python": str(get_conda_python("tuinadex-vla")),
    }


def build_runtime_env(base_env: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    """Construct environment dictionary with CycloneDDS and ROS 2 baselines."""
    env = dict(os.environ if base_env is None else base_env)
    root = get_repo_root()

    # Invariant 19: Eclipse CycloneDDS Baseline
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "42")

    cyclonedds_xml = root / "scripts" / "env" / "cyclonedds.xml"
    if cyclonedds_xml.exists():
        env.setdefault("CYCLONEDDS_URI", f"file://{cyclonedds_xml}")

    # Fix Pinocchio / Boost Python 3.10 dynamic library priority over host Python 3.14
    control_py = get_conda_python("tuinadex-control")
    conda_base = control_py.parent.parent
    cmeel_lib = conda_base / "lib" / "python3.10" / "site-packages" / "cmeel.prefix" / "lib"
    if cmeel_lib.is_dir():
        curr_ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = f"{cmeel_lib}:{conda_base / 'lib'}:{curr_ld}"

    # Set PYTHONPATH to include src packages
    src_dir = root / "src"
    py_paths = [
        str(src_dir / "tuinadex_core"),
        str(src_dir / "tuinadex_teleop"),
        str(src_dir / "tuinadex_sim"),
        str(src_dir / "tuinadex_bringup"),
        str(src_dir / "tuinadex_hand"),
        str(src_dir / "tuinadex_arm"),
        str(src_dir / "rebot_control"),
        str(root / "dataset"),
    ]
    existing_pp = env.get("PYTHONPATH", "")
    if existing_pp:
        py_paths.append(existing_pp)
    env["PYTHONPATH"] = ":".join(py_paths)

    return env


def execute_subcommand(
    cmd: List[str],
    env: Optional[Dict[str, str]] = None,
    dry_run: bool = False,
    cwd: Optional[Path] = None,
) -> int:
    """Execute a subprocess command cleanly, handling SIGINT gracefully."""
    root = get_repo_root()
    working_dir = cwd or root
    runtime_env = build_runtime_env(env)

    if dry_run:
        print("[DRY-RUN] Executing command:")
        print(f"  Working Dir: {working_dir}")
        print(f"  Command:     {' '.join(cmd)}")
        return 0

    try:
        res = subprocess.run(cmd, cwd=str(working_dir), env=runtime_env)
        return res.returncode
    except KeyboardInterrupt:
        print("\n[tuina] Interrupted by user.")
        return 130
