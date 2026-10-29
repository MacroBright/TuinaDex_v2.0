"""test_tuinadex_bringup_launch.py — 验证整机统一启动 Launch 文件生成与参数自适应."""
import sys
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("tuinadex_core", "tuinadex_hand", "tuinadex_arm", "tuinadex_sim", "tuinadex_bringup"):
    p = str(ROOT / "src" / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

pytest.importorskip("launch")
import importlib.util
from ament_index_python.packages import get_package_share_directory


def _get_launch_module():
    launch_path = ROOT / "src" / "tuinadex_bringup" / "launch" / "robot.launch.py"
    spec = importlib.util.spec_from_file_location("robot_launch", str(launch_path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_robot_launch_description_generation():
    """验证 robot.launch.py 能正确生成 LaunchDescription 且包含预期实体."""
    mod = _get_launch_module()
    ld = mod.generate_launch_description()
    assert ld is not None

    entity_names = [e.__class__.__name__ for e in ld.entities]
    assert "DeclareLaunchArgument" in entity_names
    assert "Node" in entity_names

    # 检查核心参数名是否均已声明
    declared_args = [
        e.name for e in ld.entities if e.__class__.__name__ == "DeclareLaunchArgument"
    ]
    expected_args = [
        "mode",
        "use_sim",
        "mock",
        "arm_channel",
        "hand_port",
        "rate_hz",
        "gui",
        "record",
        "use_scene_world",
        "watchdog_timeout_sec",
    ]
    for arg in expected_args:
        assert arg in declared_args, f"Argument '{arg}' not found in declared args"


def test_robot_launch_nodes_configuration():
    """验证不同模式下 Node 实体的包名与可执行文件契约."""
    mod = _get_launch_module()
    ld = mod.generate_launch_description()
    nodes = [e for e in ld.entities if e.__class__.__name__ == "Node"]

    node_pkgs = [(getattr(n, "node_package", None), getattr(n, "node_executable", None)) for n in nodes]
    # 确保包含了 hand, arm, sim 以及 recorder 候选
    packages_included = {p[0] for p in node_pkgs}
    assert "tuinadex_hand" in packages_included
    assert "tuinadex_arm" in packages_included
    assert "tuinadex_sim" in packages_included
    assert "tuinadex_recorder" in packages_included
