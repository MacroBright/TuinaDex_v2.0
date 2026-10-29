"""test_arm_node_integration.py — tuinadex_arm 节点级闭环验证 (与 test_leap_hand_node_integration 严格对称)."""
import sys
import time
from pathlib import Path
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

ROOT = Path(__file__).resolve().parents[1]
for package in ("tuinadex_core", "tuinadex_arm"):
    path = str(ROOT / "src" / package)
    if path not in sys.path:
        sys.path.insert(0, path)


from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tuinadex_core.names import CANONICAL_ARM_JOINT_NAMES
from tuinadex_arm.arm_node import ArmNode


@pytest.fixture
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_arm_mock_loop_and_watchdog(rclpy_ctx):
    """发布机械臂指令后 joint_states 跟随; 停发后看门狗平滑回 0 rad HOME."""
    overrides = dict(
        mock_mode=True,
        watchdog_timeout_sec=0.10,
        watchdog_hold_sec=0.10,
        watchdog_ramp_sec=0.20,
    )
    node = ArmNode(parameter_overrides=overrides)

    received = []
    node.create_subscription(
        JointState,
        "/arm/joint_states",
        lambda m: received.append(np.asarray(m.position)),
        10,
    )

    # 1) 正常跟随: 6 个关节在规范限位内的目标
    pub = node.create_publisher(JointState, "/arm/joint_command", 10)
    expected = np.asarray([0.2, 0.5, 1.0, -0.3, 0.4, -0.5], dtype=np.float64)
    cmd = JointState()
    cmd.name = list(CANONICAL_ARM_JOINT_NAMES)
    cmd.position = expected.tolist()

    t_end = time.monotonic() + 0.4
    while time.monotonic() < t_end:
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.02)

    assert len(received) > 0, "未收到机械臂状态"
    assert np.allclose(received[-1], expected, atol=0.05), (
        f"期望状态接近 {expected}, 实际收到 {received[-1]}"
    )

    # 2) 停发指令 -> 看门狗 (0.1+0.1+0.2s) 平滑回 0 rad HOME
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)

    assert np.allclose(received[-1], 0.0, atol=1e-3), (
        f"看门狗回退失败: 最新位置 {received[-1]}"
    )

    # 3) 基础服务调用验证
    test_client = rclpy.create_node("test_arm_svc_client")
    disable_cli = test_client.create_client(Trigger, "/arm/disable")
    enable_cli = test_client.create_client(Trigger, "/arm/enable")
    home_cli = test_client.create_client(Trigger, "/arm/safe_home")

    assert disable_cli.wait_for_service(timeout_sec=1.0)
    assert enable_cli.wait_for_service(timeout_sec=1.0)
    assert home_cli.wait_for_service(timeout_sec=1.0)

    req = Trigger.Request()
    for cli in (disable_cli, enable_cli, home_cli):
        fut = cli.call_async(req)
        t_wait = time.monotonic() + 0.5
        while time.monotonic() < t_wait and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.02)
            rclpy.spin_once(test_client, timeout_sec=0.02)
        assert fut.done()
        assert fut.result().success is True

    test_client.destroy_node()
    node.destroy_node()
