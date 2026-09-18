"""test_command_processing.py — /hand/joint_command 报文校验与重排."""
import sys
from pathlib import Path
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_hand"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_hand.command_processing import align_command, JOINT_SPEC_LIMITS
from tuinadex_hand.driver.leap_driver import CANONICAL_JOINT_NAMES


def test_valid_passthrough_canonical_order():
    q = np.arange(16, dtype=np.float64) * 0.02   # 0..0.30 全在 Spec 限位内, clamp 退化为恒等
    out, reason = align_command(q.tolist(), CANONICAL_JOINT_NAMES)
    assert out is not None and reason == ""
    np.testing.assert_allclose(out, q)


def test_valid_no_names():
    out, reason = align_command([0.1] * 16)
    assert out is not None and reason == ""


def test_reorders_noncanonical_names():
    # 交换 index_yaw (0) 与 thumb_ip (15) 的位置; 取 0.02*i -> 全在限位内, 免被 clamp 干扰
    names = list(CANONICAL_JOINT_NAMES)
    names[0], names[15] = names[15], names[0]
    pos = [0.02 * i for i in range(16)]
    out, reason = align_command(pos, names)
    assert out is not None and reason == ""
    assert abs(out[0] - 0.30) < 1e-9     # thumb_ip 位置值(0.30) 进入 index_yaw 槽
    assert out[15] == 0.0


def test_dim_mismatch_rejected():
    out, reason = align_command([0.0] * 15)
    assert out is None and "dim_mismatch" in reason


def test_nan_rejected_with_index():
    pos = [0.0] * 16
    pos[7] = float("nan")
    out, reason = align_command(pos)
    assert out is None and "non_finite" in reason and "7" in reason


def test_unknown_joint_name_rejected():
    names = list(CANONICAL_JOINT_NAMES)
    names[3] = "wrist_yaw"   # 不存在的关节
    out, reason = align_command([0.0] * 16, names)
    assert out is None and "unknown_joint_names" in reason


def test_name_count_mismatch_rejected():
    out, reason = align_command([0.0] * 16, ["a", "b"])
    assert out is None and "name_dim_mismatch" in reason


def test_out_of_spec_clamped_to_upper_boundary():
    out, reason = align_command([2.5] * 16)   # 全部超出各关节上限 -> 夹到边界, 不拒绝
    assert out is not None and reason == ""
    np.testing.assert_allclose(out, JOINT_SPEC_LIMITS[:, 1])


def test_negative_thumb_yaw_kept_within_lower_boundary():
    pos = [0.0] * 16
    pos[12] = -0.30   # thumb_yaw 合法负区间下边沿, 不应被夹走
    out, reason = align_command(pos)
    assert out is not None and reason == ""
    assert out[12] == -0.30
