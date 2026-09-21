"""test_leap_driver.py — 验证 16-DOF LEAP Hand 驱动层不变量与双向坐标转换."""
import sys
from pathlib import Path
import numpy as np
import pytest

# 添加 src 到导入路径
WORKSPACE_SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_hand"
if str(WORKSPACE_SRC) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_SRC))

WORKSPACE_CORE = Path(__file__).resolve().parents[1] / "src" / "tuinadex_core"
if str(WORKSPACE_CORE) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_CORE))

from tuinadex_hand.driver.leap_driver import (
    LeapHandDriver,
    CANONICAL_JOINT_NAMES,
    DEFAULT_OPEN_POSE,
    DEFAULT_JOINT_DIR,
)


def test_canonical_joint_names_count():
    """验证 16 维 Canonical 命名完整性."""
    assert len(CANONICAL_JOINT_NAMES) == 16
    assert CANONICAL_JOINT_NAMES[0] == "index_yaw"
    assert CANONICAL_JOINT_NAMES[12] == "thumb_yaw"
    assert CANONICAL_JOINT_NAMES[15] == "thumb_ip"


def test_coordinate_transform_invariants():
    """验证双向数学转换契约: 0 rad -> OPEN_POSE, OPEN_POSE -> 0 rad."""
    driver = LeapHandDriver(mock_mode=True)
    
    # 1. 0 rad (全开位) -> 舵机绝对弧度必须精确等于 OPEN_POSE
    zero_cmd = np.zeros(16, dtype=np.float64)
    dxl_target = driver.robot_joint_to_hardware_dxl(zero_cmd)
    np.testing.assert_allclose(dxl_target, DEFAULT_OPEN_POSE, atol=1e-5)

    # 2. 舵机绝对弧度 OPEN_POSE -> 转换回关节空间必须精确等于 0 rad
    q_recovered = driver.hardware_dxl_to_robot_joint(DEFAULT_OPEN_POSE)
    np.testing.assert_allclose(q_recovered, zero_cmd, atol=1e-5)


def test_positive_flexion_direction():
    """验证屈曲为正 (q > 0) 的方向映射逻辑."""
    driver = LeapHandDriver(mock_mode=True)
    
    # 手指弯曲 0.5 rad
    bend_cmd = np.ones(16, dtype=np.float64) * 0.5
    dxl_target = driver.robot_joint_to_hardware_dxl(bend_cmd)
    
    # 前 12 轴 JOINT_DIR 为 -1，舵机值应减小
    for i in range(12):
        assert dxl_target[i] < DEFAULT_OPEN_POSE[i]
    
    # 拇指根部 JOINT_DIR 为 +1，舵机值应增大
    assert dxl_target[12] > DEFAULT_OPEN_POSE[12]

    # 双向还原一致性
    q_recovered = driver.hardware_dxl_to_robot_joint(dxl_target)
    np.testing.assert_allclose(q_recovered, bend_cmd, atol=1e-5)


def test_mock_driver_lifecycle_and_relax():
    """验证驱动在 Mock 模式下的生命周期与看门狗 Relax-to-OPEN."""
    driver = LeapHandDriver(mock_mode=True)
    assert not driver.is_connected()
    
    # 连接
    assert driver.connect() is True
    assert driver.is_connected() is True
    
    # 初始状态为 0 rad
    pos, vel, eff = driver.read_states()
    np.testing.assert_allclose(pos, np.zeros(16), atol=1e-5)
    
    # 写入动作
    target = np.ones(16, dtype=np.float64) * 0.8
    driver.write_command(target)
    pos_after, _, _ = driver.read_states()
    np.testing.assert_allclose(pos_after, target, atol=1e-5)
    
    # 测试看门狗平滑过渡回 0 rad
    # 1) hold 期 (0.1s < 0.3s)
    held = driver.relax_to_open_step(now=0.1, hold_time=0.30, ramp_time=0.60)
    np.testing.assert_allclose(held, target, atol=1e-5)
    
    # 2) ramp 期 (0.6s)
    ramped = driver.relax_to_open_step(now=0.6, hold_time=0.30, ramp_time=0.60)
    assert np.all(ramped < target)
    assert np.all(ramped > 0.0)
    
    # 3) 完成期 (1.0s >= 0.9s)
    finished = driver.relax_to_open_step(now=1.0, hold_time=0.30, ramp_time=0.60)
    np.testing.assert_allclose(finished, np.zeros(16), atol=1e-5)
    
    # 断开
    driver.disconnect()
    assert not driver.is_connected()
