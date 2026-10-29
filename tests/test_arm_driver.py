"""test_arm_driver.py — 验证 6-DOF reBot B601-RS 机械臂驱动层不变量与 Mock 行为."""
import sys
from pathlib import Path
import numpy as np
import pytest

WORKSPACE_ARM = Path(__file__).resolve().parents[1] / "src" / "tuinadex_arm"
if str(WORKSPACE_ARM) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ARM))

WORKSPACE_CORE = Path(__file__).resolve().parents[1] / "src" / "tuinadex_core"
if str(WORKSPACE_CORE) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_CORE))

from tuinadex_core.names import CANONICAL_ARM_JOINT_NAMES
from tuinadex_arm.driver.arm_driver import ArmDriver


def test_canonical_arm_joint_names_count():
    """验证 6 维机械臂命名完整性."""
    assert len(CANONICAL_ARM_JOINT_NAMES) == 6
    assert CANONICAL_ARM_JOINT_NAMES == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ]


def test_mock_driver_lifecycle_and_relax():
    """验证 ArmDriver Mock 模式生命周期与看门狗平滑回零."""
    driver = ArmDriver(mock_mode=True)
    assert driver.connect() is True
    assert driver.is_connected is True
    assert driver.is_enabled is True

    # 1. 初始状态为 0
    pos, vel, eff = driver.read_states()
    assert pos.shape == (6,)
    assert np.allclose(pos, 0.0)

    # 2. 写入指令并多步更新
    cmd = np.array([0.5, 1.0, 1.2, -0.4, 0.2, -1.0], dtype=np.float64)
    for _ in range(20):
        driver.write_command(cmd)
    pos, _, _ = driver.read_states()
    assert np.allclose(pos, cmd, atol=0.05)

    # 3. 看门狗回零步进 (Hold 0.1s + Ramp 0.2s)
    t0 = 100.0
    # Hold 期保持原位置
    driver.relax_to_open_step(now=t0 + 0.05, hold_time=0.1, ramp_time=0.2)
    pos_hold, _, _ = driver.read_states()
    assert np.allclose(pos_hold, pos, atol=0.01)

    # Ramp 期渐变过渡
    driver.relax_to_open_step(now=t0 + 0.20, hold_time=0.1, ramp_time=0.2)
    pos_mid, _, _ = driver.read_states()
    assert np.all(np.abs(pos_mid) < np.abs(pos_hold) + 1e-4)

    # 结束阶段归零
    driver.relax_to_open_step(now=t0 + 0.40, hold_time=0.1, ramp_time=0.2)
    for _ in range(15):
        driver.relax_to_open_step(now=t0 + 0.50, hold_time=0.1, ramp_time=0.2)
    pos_end, _, _ = driver.read_states()
    assert np.allclose(pos_end, 0.0, atol=0.01)

    # 4. 安全断开
    driver.disconnect()
    assert driver.is_connected is False
    assert driver.is_enabled is False

