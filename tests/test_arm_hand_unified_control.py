"""test_arm_hand_unified_control.py — 22 维（6 臂 + 16 手）协同控制与双节点看门狗集成闭环测试."""
import sys
import time
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("tuinadex_core", "tuinadex_hand", "tuinadex_arm", "tuinadex_teleop"):
    p = str(ROOT / "src" / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor

from tuinadex_arm.arm_node import ArmNode
from tuinadex_hand.leap_hand_node import LeapHandNode
from tuinadex_sim.leap_hand_sim_node import LeapHandSimNode
from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs.ros2_robot_env import ROS2RobotEnv


@pytest.fixture
def rclpy_session():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_22d_arm_hand_coordinated_control_and_watchdog(rclpy_session):
    """验证 22 维（6 臂 + 16 手）统一指令分发、状态聚合以及 300ms 看门狗联合回零."""
    watchdog_params = dict(
        mock_mode=True,
        control_rate_hz=50.0,
        watchdog_timeout_sec=0.10,
        watchdog_hold_sec=0.10,
        watchdog_ramp_sec=0.20,
    )

    # 1. 创建并初始化机械臂与灵巧手两个主控节点 (Mock 模式)
    arm_node = ArmNode(parameter_overrides=watchdog_params)
    hand_node = LeapHandNode(parameter_overrides=watchdog_params)

    # 2. 实例化 22 维 ROS2 机器人环境
    cfg = RobotConfig(mode="arm_hand", camera_names=["head"])
    env = ROS2RobotEnv(cfg, node_name="test_22d_coord_env")

    # 辅助 executor 统一调度事件循环
    executor = SingleThreadedExecutor()
    executor.add_node(arm_node)
    executor.add_node(hand_node)
    executor.add_node(env._node)

    def spin_steps(duration_sec: float, dt: float = 0.01):
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            executor.spin_once(timeout_sec=dt)
            env.spin_once()

    # 初始刷新
    spin_steps(0.1)

    # 3. 构造 22 维目标动作并持续下发
    expected_arm = np.array([0.2, 0.4, 0.8, -0.3, 0.5, -0.2], dtype=np.float32)
    expected_hand = np.full(16, 0.4, dtype=np.float32)
    expected_action = np.concatenate([expected_arm, expected_hand]).astype(np.float32)

    assert expected_action.shape == (22,)

    # 持续下发 0.5 秒 (确保 50Hz 滤波充分收敛)
    t_end = time.monotonic() + 0.5
    while time.monotonic() < t_end:
        env.step(expected_action)
        executor.spin_once(timeout_sec=0.01)

    obs = env.get_observation()
    assert obs["state"].shape == (22,), f"期望 22 维状态，实际获得 {obs['state'].shape}"

    # 验证机械臂 6 维状态收敛
    np.testing.assert_allclose(
        obs["state"][:6],
        expected_arm,
        atol=0.08,
        err_msg="机械臂 6-DOF 状态未正确跟随指令",
    )

    # 验证灵巧手 16 维状态收敛
    np.testing.assert_allclose(
        obs["state"][6:],
        expected_hand,
        atol=0.08,
        err_msg="灵巧手 16-DOF 状态未正确跟随指令",
    )

    # 4. 停发指令，等待看门狗超时 (0.1s 超时 + 0.1s 保持 + 0.2s 回退 = 0.4s)
    # 等待 0.8s 确保平滑阻尼充分回退至 0 rad
    spin_steps(0.8)

    # 5. 校验看门狗回退结果: 臂 (0 rad HOME) 与手 (0 rad OPEN)
    relaxed_obs = env.get_observation()
    np.testing.assert_allclose(
        relaxed_obs["state"][:6],
        0.0,
        atol=1e-2,
        err_msg="看门狗超时后机械臂未平滑回零",
    )
    np.testing.assert_allclose(
        relaxed_obs["state"][6:],
        0.0,
        atol=1e-2,
        err_msg="看门狗超时后灵巧手未平滑回零",
    )

    # 6. 安全清理
    env.close()
    arm_node.destroy_node()
    hand_node.destroy_node()


def test_22d_sim_arm_hand_coordination(rclpy_session):
    """验证仿真模式下 16D 动力学孪生手与 6D 机械臂协同运行与看门狗."""
    watchdog_params = dict(
        mock_mode=True,
        control_rate_hz=50.0,
        publish_rate_hz=50.0,
        watchdog_timeout_sec=0.10,
        watchdog_hold_sec=0.10,
        watchdog_ramp_sec=0.20,
        gui=False,
    )

    sim_hand = LeapHandSimNode(parameter_overrides=watchdog_params, node_name="sim_coord_hand")
    sim_arm = ArmNode(parameter_overrides=watchdog_params)

    cfg = RobotConfig(mode="arm_hand", camera_names=["head"])
    env = ROS2RobotEnv(cfg, node_name="test_22d_sim_env")

    executor = SingleThreadedExecutor()
    executor.add_node(sim_arm)
    executor.add_node(sim_hand)
    executor.add_node(env._node)

    def spin_steps(duration_sec: float, dt: float = 0.01):
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            executor.spin_once(timeout_sec=dt)
            env.spin_once()

    spin_steps(0.1)

    # 下发 22 维指令
    expected_arm = np.array([0.15, 0.35, 0.70, -0.25, 0.45, -0.15], dtype=np.float32)
    expected_hand = np.full(16, 0.35, dtype=np.float32)
    expected_action = np.concatenate([expected_arm, expected_hand]).astype(np.float32)

    t_end = time.monotonic() + 0.5
    while time.monotonic() < t_end:
        env.step(expected_action)
        executor.spin_once(timeout_sec=0.01)

    obs = env.get_observation()
    assert obs["state"].shape == (22,)
    np.testing.assert_allclose(obs["state"][:6], expected_arm, atol=0.08)
    np.testing.assert_allclose(obs["state"][6:], expected_hand, atol=0.15)

    # 看门狗回退
    spin_steps(0.8)

    relaxed_obs = env.get_observation()
    np.testing.assert_allclose(relaxed_obs["state"][:6], 0.0, atol=1e-2)
    np.testing.assert_allclose(relaxed_obs["state"][6:], 0.0, atol=0.05)

    env.close()
    sim_arm.destroy_node()
    sim_hand.destroy_node()

