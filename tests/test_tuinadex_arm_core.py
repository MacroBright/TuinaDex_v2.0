"""tests/test_tuinadex_arm_core.py — 机械臂规范命名与软限位安全门单元测试."""
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src" / "tuinadex_core") not in sys.path:
    sys.path.insert(0, str(ROOT / "src" / "tuinadex_core"))

from tuinadex_core import (
    CANONICAL_ARM_JOINT_NAMES,
    ARM_SPEC_LIMITS,
    align_arm_command,
)



def test_arm_canonical_joint_names():
    assert len(CANONICAL_ARM_JOINT_NAMES) == 6
    assert CANONICAL_ARM_JOINT_NAMES == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ]


def test_arm_spec_limits_shape():
    assert ARM_SPEC_LIMITS.shape == (6, 2)
    # 验证下界小于等于上界
    assert np.all(ARM_SPEC_LIMITS[:, 0] <= ARM_SPEC_LIMITS[:, 1])


def test_align_arm_command_valid():
    target = [0.0, 1.0, 1.5, 0.5, -0.5, 2.0]
    aligned, err = align_arm_command(target)
    assert err == ""
    assert aligned is not None
    assert np.allclose(aligned, target)


def test_align_arm_command_dim_mismatch():
    aligned, err = align_arm_command([0.0, 1.0, 2.0])
    assert aligned is None
    assert "dim_mismatch" in err


def test_align_arm_command_non_finite():
    target = [0.0, 1.0, float("nan"), 0.5, -0.5, 2.0]
    aligned, err = align_arm_command(target)
    assert aligned is None
    assert "non_finite" in err


def test_align_arm_command_reorder_names():
    names = ["joint6", "joint5", "joint4", "joint3", "joint2", "joint1"]
    values = [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]
    aligned, err = align_arm_command(values, names=names)
    assert err == ""
    assert aligned is not None
    # 期望按 joint1..joint6 排列并夹取到限位
    assert aligned[0] == 1.0  # joint1
    assert aligned[1] == 2.0  # joint2


def test_align_arm_command_clamping():
    # 传入超限值：joint1 上限 2.80，传 5.0 应被夹至 2.80
    target = [5.0, -1.0, 4.0, -2.0, 2.0, -5.0]
    aligned, err = align_arm_command(target)
    assert err == ""
    assert aligned is not None
    assert np.isclose(aligned[0], 2.80)
    assert np.isclose(aligned[1], 0.00)  # joint2 下限 0.0
    assert np.isclose(aligned[2], 3.14)  # joint3 上限 3.14
    assert np.isclose(aligned[3], -1.57) # joint4 下限 -1.57
    assert np.isclose(aligned[4], 1.57)  # joint5 上限 1.57
    assert np.isclose(aligned[5], -3.14) # joint6 下限 -3.14
