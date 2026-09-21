"""test_sim_bridge.py — MuJoCo 仿真桥接器单元测试."""
import os
import sys
from pathlib import Path
import pytest
import numpy as np

SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_sim"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

try:
    import mujoco
except ImportError:
    mujoco = None

if mujoco is None:
    pytest.skip("mujoco is not installed", allow_module_level=True)

from tuinadex_sim.sim_bridge import MujocoSimBridge, CANONICAL_JOINT_NAMES


@pytest.fixture
def scene_xml_path():
    base = Path(__file__).resolve().parents[1]
    return str(base / "src" / "tuinadex_sim" / "worlds" / "tuina_scene.xml")


@pytest.fixture
def hand_xml_path():
    base = Path(__file__).resolve().parents[1]
    return str(base / "src" / "tuinadex_sim" / "worlds" / "leap_hand_right.xml")


def test_sim_bridge_init_scene(scene_xml_path):
    bridge = MujocoSimBridge(scene_xml_path)
    assert len(bridge.joint_ids) == 16
    assert len(bridge.actuator_ids) == 16
    assert bridge.timestep == pytest.approx(0.002)


def test_sim_bridge_init_hand(hand_xml_path):
    bridge = MujocoSimBridge(hand_xml_path)
    assert len(bridge.joint_ids) == 16
    assert len(bridge.actuator_ids) == 16


def test_sim_bridge_get_state_dimensions(scene_xml_path):
    bridge = MujocoSimBridge(scene_xml_path)
    state = bridge.get_state()
    assert state.positions.shape == (16,)
    assert state.velocities.shape == (16,)
    assert state.efforts.shape == (16,)
    assert np.all(np.isfinite(state.positions))


def test_sim_bridge_control_step(scene_xml_path):
    bridge = MujocoSimBridge(scene_xml_path)
    targets = np.zeros(16, dtype=np.float64)
    targets[1] = 0.5  # act_index_mcp
    bridge.set_joint_targets(targets)

    # 步进 500 次 (1.0 秒物理时间)
    bridge.step(500)
    state = bridge.get_state()

    assert state.time == pytest.approx(1.0, abs=1e-3)
    # 验证 index_mcp 产生屈曲位移 (> 0.3 rad)
    assert state.positions[1] > 0.3
    # 验证其余关节仍稳定在 0 附近
    assert abs(state.positions[0]) < 0.1


def test_sim_bridge_reset(scene_xml_path):
    bridge = MujocoSimBridge(scene_xml_path)
    targets = np.zeros(16, dtype=np.float64)
    targets[1] = 0.5
    bridge.set_joint_targets(targets)
    bridge.step(250)

    bridge.reset()
    state = bridge.get_state()
    assert state.time == pytest.approx(0.0)
    assert np.allclose(state.positions, 0.0, atol=1e-3)
