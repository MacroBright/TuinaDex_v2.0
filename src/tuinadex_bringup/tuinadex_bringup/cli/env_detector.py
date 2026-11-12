"""env_detector.py — Environment auto-detection and isolated runtime routing for TuinaDex CLI."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional

VALID_RUNTIMES = ("control", "vla", "ros2", "bash")

_SRC_PY_PACKAGES = (
    "tuinadex_bringup",
    "tuinadex_core",
    "tuinadex_teleop",
    "tuinadex_sim",
    "tuinadex_hand",
    "tuinadex_arm",
    "rebot_control",
)


def get_repo_root() -> Path:
    """Resolve the absolute path of the TuinaDex_v2.0 repository root."""
    env_root = os.environ.get("TUINADEX_ROOT")
    if env_root:
        candidate = Path(env_root).resolve()
        if (candidate / "src").is_dir() and (candidate / "scripts").is_dir():
            return candidate

    here = Path(__file__).resolve()
    for parent in [here.parent] + list(here.parents):
        if (parent / "src").is_dir() and (parent / "scripts").is_dir():
            return parent

    return here.parents[4]


def get_conda_python(env_name: str = "tuinadex-control") -> Path:
    """Find the Python executable for a given Conda environment."""
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    if conda_prefix and Path(conda_prefix).name == env_name:
        py = Path(conda_prefix) / "bin" / "python"
        if py.exists():
            return py

    home = Path.home()
    candidates = [
        home / "win_office" / "conda" / "envs" / env_name / "bin" / "python",
        home / "miniconda3" / "envs" / env_name / "bin" / "python",
        home / "anaconda3" / "envs" / env_name / "bin" / "python",
        home / "mambaforge" / "envs" / env_name / "bin" / "python",
        home / "miniforge3" / "envs" / env_name / "bin" / "python",
        Path("/opt/conda/envs") / env_name / "bin" / "python",
        Path("/opt/miniforge3/envs") / env_name / "bin" / "python",
    ]
    for path in candidates:
        if path.exists():
            return path

    return Path(sys.executable)


def get_ros2_python() -> Path:
    """Resolve the native system Python 3 interpreter used by ROS 2 Lyrical."""
    sys_py = Path("/usr/bin/python3")
    if sys_py.exists():
        return sys_py
    which_py = shutil.which("python3")
    if which_py:
        return Path(which_py)
    return Path(sys.executable)


def _split_unique_paths(raw: Optional[str]) -> List[str]:
    if not raw:
        return []
    seen = set()
    items: List[str] = []
    for entry in raw.split(os.pathsep):
        cleaned = entry.strip()
        if not cleaned or cleaned in seen:
            continue
        seen.add(cleaned)
        items.append(cleaned)
    return items


def _filter_paths(entries: Iterable[str], drop_substrings: Iterable[str]) -> List[str]:
    drops = tuple(drop_substrings)
    kept: List[str] = []
    seen = set()
    for entry in entries:
        if not entry or entry in seen:
            continue
        if any(token in entry for token in drops):
            continue
        seen.add(entry)
        kept.append(entry)
    return kept


def _cyclonedds_lib_exists(ros_distro: str) -> bool:
    candidates = (
        Path(f"/opt/ros/{ros_distro}/lib/librmw_cyclonedds_cpp.so"),
        Path("/usr/lib/librmw_cyclonedds_cpp.so"),
        Path(f"/opt/ros/{ros_distro}/lib/x86_64-linux-gnu/librmw_cyclonedds_cpp.so"),
        Path(f"/opt/ros/{ros_distro}/lib/aarch64-linux-gnu/librmw_cyclonedds_cpp.so"),
        Path("/usr/lib/x86_64-linux-gnu/librmw_cyclonedds_cpp.so"),
        Path("/usr/lib/aarch64-linux-gnu/librmw_cyclonedds_cpp.so"),
    )
    return any(path.exists() for path in candidates)


def _configure_cyclonedds(env: Dict[str, str], root: Path, ros_distro: str) -> None:
    if _cyclonedds_lib_exists(ros_distro):
        env.setdefault("ROS_DOMAIN_ID", "42")
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        cyclonedds_xml = root / "scripts" / "env" / "cyclonedds.xml"
        if cyclonedds_xml.exists():
            env["CYCLONEDDS_URI"] = f"file://{cyclonedds_xml}"
    else:
        env.pop("RMW_IMPLEMENTATION", None)
        env.pop("CYCLONEDDS_URI", None)


def _repo_src_python_paths(root: Path) -> List[str]:
    src_dir = root / "src"
    paths: List[str] = []
    for pkg in _SRC_PY_PACKAGES:
        pkg_dir = src_dir / pkg
        if pkg_dir.exists():
            paths.append(str(pkg_dir))
    dataset_dir = root / "dataset"
    if dataset_dir.exists():
        paths.append(str(dataset_dir))
    return paths


def build_runtime_env(
    base_env: Optional[Dict[str, str]] = None,
    profile: str = "control",
) -> Dict[str, str]:
    """Construct an isolated runtime environment dictionary for the target profile.

    Profiles:
        - 'control': tuinadex-control Python 3.10 + cmeel.prefix first + src/* PYTHONPATH.
        - 'vla': tuinadex-vla Python 3.10 + dataset/ PYTHONPATH; strips ROS2 Python & cmeel.
        - 'ros2': native ROS 2 Lyrical (Python 3.14); strips conda PYTHONPATH/cmeel/CONDA_PREFIX.
        - 'bash': clean host shell with CycloneDDS baseline for scripts/*.sh.
    """
    if profile not in VALID_RUNTIMES:
        raise ValueError(f"Unsupported runtime profile '{profile}', expected one of {VALID_RUNTIMES}")

    env = dict(base_env) if base_env is not None else dict(os.environ)
    root = get_repo_root()
    env["TUINADEX_ROOT"] = str(root)

    ros_distro = env.get("ROS_DISTRO", "lyrical")
    _configure_cyclonedds(env, root, ros_distro)

    orig_py_path = _split_unique_paths(env.get("PYTHONPATH"))
    orig_ld_path = _split_unique_paths(env.get("LD_LIBRARY_PATH"))
    orig_bin_path = _split_unique_paths(env.get("PATH"))

    if profile == "control":
        control_py = get_conda_python("tuinadex-control")
        conda_dir = control_py.parent.parent
        cmeel_lib = conda_dir / "lib" / "python3.10" / "site-packages" / "cmeel.prefix" / "lib"

        ld_prefix: List[str] = []
        if cmeel_lib.exists():
            ld_prefix.append(str(cmeel_lib))
        conda_lib = conda_dir / "lib"
        if conda_lib.exists():
            ld_prefix.append(str(conda_lib))
        ld_rest = _filter_paths(orig_ld_path, ("tuinadex-vla", "cmeel.prefix"))
        merged_ld = ld_prefix + [p for p in ld_rest if p not in ld_prefix]
        if merged_ld:
            env["LD_LIBRARY_PATH"] = os.pathsep.join(merged_ld)

        bin_rest = _filter_paths(orig_bin_path, ("tuinadex-vla",))
        conda_bin = str(control_py.parent)
        if Path(conda_bin).exists():
            env["PATH"] = os.pathsep.join([conda_bin] + [p for p in bin_rest if p != conda_bin])

        py_rest = _filter_paths(orig_py_path, ("tuinadex-vla",))
        src_paths = _repo_src_python_paths(root)
        merged_py = src_paths + [p for p in py_rest if p not in src_paths]
        if merged_py:
            env["PYTHONPATH"] = os.pathsep.join(merged_py)

        if conda_dir.exists() and conda_dir.name == "tuinadex-control":
            env["CONDA_PREFIX"] = str(conda_dir)
            env["CONDA_DEFAULT_ENV"] = "tuinadex-control"

    elif profile == "vla":
        vla_py = get_conda_python("tuinadex-vla")
        conda_dir = vla_py.parent.parent

        ld_rest = _filter_paths(orig_ld_path, ("tuinadex-control", "cmeel.prefix", "/opt/ros/"))
        conda_lib = conda_dir / "lib"
        ld_prefix = [str(conda_lib)] if conda_lib.exists() else []
        merged_ld = ld_prefix + [p for p in ld_rest if p not in ld_prefix]
        if merged_ld:
            env["LD_LIBRARY_PATH"] = os.pathsep.join(merged_ld)
        else:
            env.pop("LD_LIBRARY_PATH", None)

        bin_rest = _filter_paths(orig_bin_path, ("tuinadex-control",))
        conda_bin = str(vla_py.parent)
        if Path(conda_bin).exists():
            env["PATH"] = os.pathsep.join([conda_bin] + [p for p in bin_rest if p != conda_bin])

        py_rest = _filter_paths(
            orig_py_path,
            ("tuinadex-control", "cmeel.prefix", "/opt/ros/", str(root / "src")),
        )
        dataset_dir = str(root / "dataset")
        merged_py = ([dataset_dir] if Path(dataset_dir).exists() else []) + [
            p for p in py_rest if p != dataset_dir
        ]
        if merged_py:
            env["PYTHONPATH"] = os.pathsep.join(merged_py)
        else:
            env.pop("PYTHONPATH", None)

        if conda_dir.exists() and conda_dir.name == "tuinadex-vla":
            env["CONDA_PREFIX"] = str(conda_dir)
            env["CONDA_DEFAULT_ENV"] = "tuinadex-vla"

    elif profile in ("ros2", "bash"):
        conda_markers = ("tuinadex-control", "tuinadex-vla", "cmeel.prefix", "/conda/envs/")
        clean_ld = _filter_paths(orig_ld_path, conda_markers)
        if clean_ld:
            env["LD_LIBRARY_PATH"] = os.pathsep.join(clean_ld)
        else:
            env.pop("LD_LIBRARY_PATH", None)

        clean_bin = _filter_paths(orig_bin_path, conda_markers)
        if clean_bin:
            env["PATH"] = os.pathsep.join(clean_bin)

        clean_py = _filter_paths(
            orig_py_path,
            conda_markers + ("python3.10", str(root / "src")),
        )
        if clean_py:
            env["PYTHONPATH"] = os.pathsep.join(clean_py)
        else:
            env.pop("PYTHONPATH", None)

        for var in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "CONDA_PROMPT_MODIFIER", "PYTHONHOME"):
            env.pop(var, None)

    return env


def get_active_env_info() -> Dict[str, str]:
    """Return runtime, ROS 2, Conda, and hardware bus summary for diagnostics."""
    root = get_repo_root()
    ros_distro = os.environ.get("ROS_DISTRO", "lyrical")
    control_py = get_conda_python("tuinadex-control")
    vla_py = get_conda_python("tuinadex-vla")
    ros2_py = get_ros2_python()

    tty_ports = sorted(str(p) for p in Path("/dev").glob("ttyUSB*"))
    can0_flags = Path("/sys/class/net/can0/flags")
    can0_status = "missing"
    if can0_flags.parent.exists():
        try:
            up = bool(int(can0_flags.read_text(encoding="utf-8").strip(), 16) & 0x1)
            can0_status = "UP" if up else "DOWN"
        except OSError:
            can0_status = "present"

    myimu_path = Path("/dev/myimu")
    if myimu_path.exists():
        myimu_status = f"{myimu_path} -> {os.path.realpath(myimu_path)}"
    else:
        myimu_status = "not detected"

    return {
        "repo_root": str(root),
        "cli_python": f"{sys.executable} ({sys.version.split()[0]})",
        "ros2_python": str(ros2_py),
        "tuinadex_control_python": f"{control_py} ({'OK' if control_py.exists() else 'MISSING'})",
        "tuinadex_vla_python": f"{vla_py} ({'OK' if vla_py.exists() else 'MISSING'})",
        "ros_distro": os.environ.get("ROS_DISTRO", "unset"),
        "ament_prefix_path": os.environ.get("AMENT_PREFIX_PATH", "unset"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "default"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "cyclonedds_installed": "YES" if _cyclonedds_lib_exists(ros_distro) else "NO",
        "can0_status": can0_status,
        "myimu_device": myimu_status,
        "usb_serial_ports": ", ".join(tty_ports) if tty_ports else "none",
    }


def _needs_ros2_shell_bootstrap(cmd: List[str], runtime_env: Dict[str, str]) -> bool:
    if not cmd:
        return False
    exe = cmd[0]
    if exe not in ("ros2", "colcon"):
        return False
    return (
        shutil.which(exe, path=runtime_env.get("PATH")) is None
        or "AMENT_PREFIX_PATH" not in runtime_env
    )


def execute_subcommand(
    cmd: List[str],
    env: Optional[Dict[str, str]] = None,
    dry_run: bool = False,
    profile: str = "control",
) -> int:
    """Execute a subprocess command cleanly under an isolated runtime profile."""
    runtime_env = build_runtime_env(env, profile=profile)
    cmd_str = shlex.join(cmd)

    if dry_run:
        print(f"[DRY-RUN][{profile}] {cmd_str}")
        return 0

    print(f"[TUINA][{profile}] {cmd_str}")

    exec_cmd = cmd
    if profile == "ros2" and _needs_ros2_shell_bootstrap(cmd, runtime_env):
        root = get_repo_root()
        ros_distro = runtime_env.get("ROS_DISTRO", "lyrical")
        install_setup = root / "install" / "setup.bash"
        sources = [
            f'[ -f "/opt/ros/{shlex.quote(ros_distro)}/setup.bash" ] && source "/opt/ros/{shlex.quote(ros_distro)}/setup.bash"',
        ]
        if install_setup.exists():
            sources.append(f'source {shlex.quote(str(install_setup))}')
        bootstrap = "set +u; " + "; ".join(sources) + "; set -u; exec " + cmd_str
        exec_cmd = ["bash", "-c", bootstrap]

    try:
        proc = subprocess.run(exec_cmd, env=runtime_env, check=False)
        return int(proc.returncode)
    except KeyboardInterrupt:
        print("\n[TUINA] Interrupted by user (Ctrl+C).")
        return 130
    except FileNotFoundError as exc:
        print(f"[TUINA ERROR] Executable not found: {exc}", file=sys.stderr)
        return 127
