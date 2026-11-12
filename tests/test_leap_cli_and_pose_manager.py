"""test_leap_cli_and_pose_manager.py — 针对 LEAP Hand 姿态管理、核心控制器与 CLI 的单元测试."""

import json
from pathlib import Path
import numpy as np
import pytest

from tuinadex_hand.core.pose_manager import (
    DEFAULT_POSES,
    OPEN_POSE_DEFAULT,
    get_default_config_path,
    is_valid_pose,
    load_poses,
    save_poses,
    unwrap_to_limits,
)
from tuinadex_hand.core.kinematics_limits import (
    angle_safety_clip,
    LEAPsim_limits,
    LEAPsim_to_LEAPhand,
    LEAPhand_to_LEAPsim,
)
from tuinadex_hand.core.hand_controller import LeapNode, HandController
from tuinadex_hand.driver.leap_driver import LeapHandDriver
from tuinadex_hand.cli.control import main as control_main
from tuinadex_hand.cli.calibrate import main as calibrate_main
from tuinadex_hand.cli.diagnostics import main as diagnostics_main


class TestPoseManager:
    """测试姿态持久化、验证与 2pi 回绕解算."""

    def test_is_valid_pose(self):
        # 正常全开位有效
        assert is_valid_pose(OPEN_POSE_DEFAULT) is True

        # 全零数据无效 (读取失败典型特征)
        assert is_valid_pose(np.zeros(16)) is False

        # 常数全相同无效
        assert is_valid_pose(np.ones(16) * 3.14) is False

        # 包含非数 NaN/Inf 无效
        bad_nan = OPEN_POSE_DEFAULT.copy()
        bad_nan[0] = np.nan
        assert is_valid_pose(bad_nan) is False

        # 超出全局物理范围无效
        bad_range = OPEN_POSE_DEFAULT.copy()
        bad_range[2] = 12.0
        assert is_valid_pose(bad_range) is False

        # 维度不符无效
        assert is_valid_pose(np.zeros(15)) is False

    def test_load_and_save_poses(self, tmp_path):
        # 从默认路径加载
        poses, valid = load_poses()
        assert valid is True
        assert "全开/平伸" in poses
        assert "揉法" in poses  # 中医手法姿态存在
        assert poses["全开/平伸"].shape == (16,)

        # 保存到临时文件并读回验证
        test_file = tmp_path / "test_poses.json"
        saved_path = save_poses(poses, test_file)
        assert saved_path.exists()

        reloaded, reloaded_valid = load_poses(test_file)
        assert reloaded_valid is True
        assert np.allclose(reloaded["揉法"], poses["揉法"], atol=1e-4)

    def test_unwrap_to_limits(self, tmp_path):
        # 模拟中间锚点为 [3.0] * 16
        limits_file = tmp_path / "mock_limits.json"
        with open(limits_file, "w") as f:
            json.dump({"min": [2.0] * 16, "max": [4.0] * 16}, f)

        # 输入一个跨越了 +2pi 的位置
        wrapped_pos = np.array([3.0] * 16)
        wrapped_pos[0] += 2.0 * np.pi  # 3.0 + 6.28318 = 9.28318

        unwrapped = unwrap_to_limits(wrapped_pos, limits_file)
        assert np.isclose(unwrapped[0], 3.0, atol=1e-5)
        assert np.allclose(unwrapped[1:], 3.0, atol=1e-5)


class TestKinematicsLimits:
    """测试运动学限位转换与安全截断."""

    def test_sim_real_conversion_invertibility(self):
        sim_q = np.array([0.1, -0.2, 0.5, 0.3] * 4, dtype=np.float64)
        real_q = LEAPsim_to_LEAPhand(sim_q)
        recovered_sim_q = LEAPhand_to_LEAPsim(real_q)
        assert np.allclose(sim_q, recovered_sim_q, atol=1e-6)

    def test_angle_safety_clip(self):
        # 极大值输入
        extreme_high = np.ones(16) * 10.0
        clipped_high = angle_safety_clip(extreme_high)
        assert np.all(clipped_high <= 6.5)

        # 极小值输入
        extreme_low = np.ones(16) * -10.0
        clipped_low = angle_safety_clip(extreme_low)
        assert np.all(clipped_low >= -0.8)


class TestHandControllerAndNode:
    """测试 LeapNode 与 HandController 在 Mock 模式下的控制逻辑."""

    def test_leap_node_mock_lifecycle(self):
        node = LeapNode(mock=True)
        assert node.curr_lim <= 150  # 黄金不变量: 电流硬锁定 150mA

        # 读取初始位置
        pos = node.read_pos()
        assert len(pos) == 16

        # 设置姿态
        assert node.set_pose("全握拳") is True
        assert np.allclose(node.read_pos(), node.poses["全握拳"])

        # 设置单个关节 (JOINT_DIR[0] = -1.0)
        node.set_joint(0, 0.2)
        assert np.isclose(node.read_pos()[0], node.open_pose[0] - 0.2, atol=1e-2)

        # 释放断开
        node.disconnect()

    def test_hand_controller_relative_joint_control(self):
        ctrl = HandController(mock=True)

        # 食指控制 (ID 0..3: side, mcp, pip, dip)
        # JOINT_DIR[1] = -1.0, 弯曲 0.3 rad -> target = open_pose - 0.3
        res = ctrl.set_joint_relative("index", "mcp", 0.3)
        assert res is True
        expected_index_mcp = ctrl.open_pose[1] - 0.3
        assert np.isclose(ctrl.target[1], expected_index_mcp, atol=1e-4)

        # 拇指控制 (ID 12..15: mcp, side, pip, dip)
        # JOINT_DIR[12] = +1.0, 弯曲 0.2 rad -> target = open_pose + 0.2
        res_thumb = ctrl.set_joint_relative("thumb", "mcp", 0.2)
        assert res_thumb is True
        expected_thumb_mcp = ctrl.open_pose[12] + 0.2
        assert np.isclose(ctrl.target[12], expected_thumb_mcp, atol=1e-4)

        ctrl.disconnect()


class TestLeapDriverIntegration:
    """测试 LeapHandDriver 自动加载 poses.json 与实测限位."""

    def test_driver_loads_calibration_files(self):
        driver = LeapHandDriver(mock_mode=True)
        # 确认 open_pose 来自 poses.json 中的实测数据
        poses, valid = load_poses()
        if valid and "全开/平伸" in poses:
            assert np.allclose(driver.open_pose, poses["全开/平伸"])

        # 测试正反向转换契约 (0 rad 对应 open_pose)
        zero_joint = np.zeros(16, dtype=np.float64)
        dxl_open = driver.robot_joint_to_hardware_dxl(zero_joint)
        assert np.allclose(dxl_open, driver.open_pose, atol=1e-4)

        recovered_zero = driver.hardware_dxl_to_robot_joint(dxl_open)
        assert np.allclose(recovered_zero, zero_joint, atol=1e-4)


class TestCliParsers:
    """测试 CLI 命令行解析与入口."""

    def test_cli_help_flags(self, capsys):
        with pytest.raises(SystemExit) as exc1:
            control_main(["--help"])
        assert exc1.value.code == 0

        with pytest.raises(SystemExit) as exc2:
            calibrate_main(["--help"])
        assert exc2.value.code == 0

        with pytest.raises(SystemExit) as exc3:
            diagnostics_main(["--help"])
        assert exc3.value.code == 0

