"""test_leap_teleop_cli.py — 针对 leap-teleop 交互遥操 CLI 的全量单元测试."""
import sys
from pathlib import Path
import numpy as np
import pytest

from tuinadex_hand.cli.teleop import draw_teleop_hud, main as teleop_main
from tuinadex_hand.core.pose_manager import DEFAULT_POSES


class TestLeapTeleopHUD:
    """测试 HUD 仪表盘图像绘制与数据覆盖."""

    def test_draw_teleop_hud_render_success(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        open_pose = DEFAULT_POSES["全开/平伸"].copy()
        target_angles = np.zeros(16, dtype=np.float64)
        actual_pos = open_pose.copy()

        # 状态 1: 正常追踪
        annotated = draw_teleop_hud(
            frame=frame,
            target_angles=target_angles,
            actual_pos=actual_pos,
            open_pose=open_pose,
            is_paused=False,
            is_locked=False,
            hand_seen=True,
            fps=30.0,
            mock_mode=True,
        )
        assert annotated.shape == (480, 640, 3)
        assert annotated.dtype == np.uint8
        # 确认 HUD 面板被成功绘制 (非全黑)
        assert np.any(annotated > 0)

        # 状态 2: 离合暂停
        annotated_paused = draw_teleop_hud(
            frame=frame,
            target_angles=None,
            actual_pos=actual_pos,
            open_pose=open_pose,
            is_paused=True,
            is_locked=False,
            hand_seen=True,
            fps=25.0,
            mock_mode=False,
        )
        assert annotated_paused.shape == (480, 640, 3)

        # 状态 3: 手势锁定且未读到电机位置
        annotated_locked = draw_teleop_hud(
            frame=frame,
            target_angles=target_angles,
            actual_pos=None,
            open_pose=open_pose,
            is_paused=False,
            is_locked=True,
            hand_seen=False,
            fps=0.0,
            mock_mode=True,
        )
        assert annotated_locked.shape == (480, 640, 3)


class TestLeapTeleopCLI:
    """测试 leap-teleop CLI 启动与生命周期."""

    def test_cli_help_flag(self):
        with pytest.raises(SystemExit) as exc_info:
            teleop_main(["--help"])
        assert exc_info.value.code == 0

    def test_cli_mock_lifecycle_headless(self):
        # 验证脱机 Mock 摄像头 + Mock 手 + 无窗口快速 3 帧退出
        ret = teleop_main([
            "--mock",
            "--mock-camera",
            "--no-preview",
            "--max-frames", "3",
            "--rate", "50",
        ])
        assert ret == 0

