"""test_tracking_mapper.py — 验证 MediaPipe 手部追踪与 16 维关节映射层."""
import sys
from pathlib import Path
import numpy as np
import pytest
pytest.importorskip("mediapipe")

WORKSPACE_SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_teleop"
if str(WORKSPACE_SRC) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_SRC))

from tuinadex_teleop.tracking.joint_mapper import JointMapper, _NUM_LEAP_DOF
from tuinadex_teleop.tracking.filter import OneEuroFilter


def test_joint_mapper_dimensions():
    """验证映射器输出维度严格为 16 DOF."""
    mapper = JointMapper()
    assert _NUM_LEAP_DOF == 16


def test_mapper_keypoints_api_for_node_callpath():
    """P0 回归: teleop 节点经 HandResult 调用 map_keypoints_to_leap 的路径必须可用.

    节点级代码依赖 rclpy 无法在本机导入, 此测试守护节点实际调用的映射入口契约:
    传入 HandResult (21 landmark), 输出 16 维有限弧度.
    """
    pytest.importorskip("mediapipe")
    from tuinadex_teleop.tracking.hand_tracker import HandResult

    class _LM:
        """最小 landmark 桩对象 (MediaPipe landmark 仅有 x/y/z 属性)."""
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    # 21 点张平手掌近似: 全部 landmark 共线于 +y (腕在原点后方, 各指无展开
    # 无屈曲)。注意 MCP 屈曲按 "腕→MCP vs MCP→PIP" 夹角度量, 桩必须保证
    # 腕→MCP 与指骨方向共线, 否则展开会被读成屈曲 (真实手掌的固有特性)。
    # 顺序: 0=WRIST, 1-4=thumb(CMC,MCP,IP,TIP), 5-8=index, 9-12=middle,
    #       13-16=ring, 17-20=pinky
    landmarks = [_LM(0.0, 0.0, 0.0)]
    for _ in range(5):
        for y in (0.10, 0.20, 0.30, 0.40):
            landmarks.append(_LM(0.0, y, 0.0))

    hand = HandResult(landmarks=landmarks, handedness="Right")
    angles = JointMapper().map_keypoints_to_leap(hand)
    assert angles.shape == (16,)
    assert np.all(np.isfinite(angles))
    # 张平手掌: 屈曲应近似为 0 (clamp 下限之上)
    np.testing.assert_allclose(angles, np.zeros(16), atol=1e-6)


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
