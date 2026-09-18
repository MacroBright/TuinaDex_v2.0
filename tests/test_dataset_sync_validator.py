"""test_dataset_sync_validator.py — 验证示教数据同步器与物理质检器."""
import sys
from pathlib import Path
import numpy as np
import pytest

WORKSPACE = Path(__file__).resolve().parents[1]
if str(WORKSPACE / "dataset") not in sys.path:
    sys.path.insert(0, str(WORKSPACE / "dataset"))

from tuinadex_dataset.synchronizer import TimeSynchronizer
from tuinadex_dataset.validator import DatasetValidator


def test_time_synchronizer():
    sync = TimeSynchronizer(target_fps=50.0)
    
    # 模拟 1 秒的数据
    t_state = np.linspace(0.0, 1.0, 60)
    s_state = np.sin(t_state)[:, None].repeat(16, axis=1)
    
    t_action = np.linspace(0.05, 0.95, 45)
    a_action = np.cos(t_action)[:, None].repeat(16, axis=1)
    
    grid, aligned_s, aligned_a = sync.align_series(t_state, s_state, t_action, a_action)
    
    assert len(grid) > 0
    assert aligned_s.shape == (len(grid), 16)
    assert aligned_a.shape == (len(grid), 16)
    # 验证时间间隔大约为 0.02s
    dt = np.diff(grid)
    np.testing.assert_allclose(dt, 0.02, atol=1e-5)


def test_dataset_validator():
    val = DatasetValidator()
    
    # 正常数据测试
    t = np.linspace(0.0, 1.0, 50)
    s = np.zeros((50, 16), dtype=np.float32)
    a = np.zeros((50, 16), dtype=np.float32)
    
    ok, errors = val.validate_episode(t, s, a)
    assert ok is True
    assert len(errors) == 0
    
    # 注入 NaN 测试
    s_nan = s.copy()
    s_nan[10, 0] = np.nan
    ok_nan, errors_nan = val.validate_episode(t, s_nan, a)
    assert ok_nan is False
    assert any("NaN" in e for e in errors_nan)


def test_validator_spec_per_joint_limits():
    """规范逐关节限位: thumb_yaw (idx 12) 上限 1.80, 超出即拒."""
    from tuinadex_dataset.validator import SPEC_JOINT_LIMITS

    assert len(SPEC_JOINT_LIMITS) == 16
    assert SPEC_JOINT_LIMITS[12] == (-0.30, 1.80)
    assert SPEC_JOINT_LIMITS[0] == (-0.35, 0.35)

    val = DatasetValidator()
    t = np.linspace(0.0, 1.0, 20)
    s = np.zeros((20, 16)); a = np.zeros((20, 16))
    s[5, 12] = 1.9   # thumb_yaw 超上限 (旧标量限位 2.5 会漏检)
    ok, errors = val.validate_episode(t, s, a)
    assert ok is False and any("限位" in e for e in errors)


def test_validator_still_passes_spec_boundary_values():
    val = DatasetValidator()
    t = np.linspace(0.0, 1.0, 20)
    s = np.zeros((20, 16)); a = np.zeros((20, 16))
    s[:, 12] = 1.80   # 恰好在上限
    s[:, 0] = -0.35   # 侧摆下限
    ok, errors = val.validate_episode(t, s, a)
    assert ok is True, errors
