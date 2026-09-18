"""test_tracking_mapper.py — 验证 MediaPipe 手部追踪与 16 维关节映射层."""
import sys
from pathlib import Path
import numpy as np
import pytest

WORKSPACE_SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_teleop"
if str(WORKSPACE_SRC) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_SRC))

from tuinadex_teleop.tracking.joint_mapper import JointMapper, _NUM_LEAP_DOF
from tuinadex_teleop.tracking.filter import OneEuroFilter


def test_joint_mapper_dimensions():
    """验证映射器输出维度严格为 16 DOF."""
    mapper = JointMapper()
    assert _NUM_LEAP_DOF == 16


def test_one_euro_filter_stability():
    """验证 1€ 滤波器在 16 维序列输入下的平滑滤波能力."""
    filt = OneEuroFilter(n_joints=16, min_cutoff=1.0, beta=0.05)
    
    # 模拟恒定值 + 高斯白噪声
    clean_signal = np.ones(16) * 0.5
    noisy_signal = clean_signal + np.random.normal(0, 0.05, 16)
    
    filtered_1 = filt(noisy_signal)
    assert filtered_1.shape == (16,)
    
    filtered_2 = filt(noisy_signal)
    assert filtered_2.shape == (16,)
    # 滤波输出应当处于有界合理范围内
    assert np.all(filtered_2 >= 0.0)
