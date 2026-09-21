"""leap_hand_sim_node.py — TuinaDex 16-DOF LEAP Hand MuJoCo 仿真 ROS2 节点.

在系统中扮演与真实硬件完全相同的数字孪生替代品角色:
- 订阅: /hand/joint_command (sensor_msgs/JointState)
- 发布: /hand/joint_states (sensor_msgs/JointState)
- 集成 Watchdog: 超时 300ms 自动平滑回退 Relax-to-OPEN
"""
from __future__ import annotations

import os
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:
    Node = object
    JointState = None

from tuinadex_core.command_processing import align_command
from tuinadex_core.names import CANONICAL_JOINT_NAMES
from .sim_bridge import MujocoSimBridge
from .sim_core import SimulationCore


class LeapHandSimNode(Node):
    """LEAP Hand MuJoCo 仿真节点: 与真机节点通信契约完全一致的薄控制循环."""

    def __init__(
        self,
        parameter_overrides: Optional[dict] = None,
        bridge=None,
        node_name: str = "leap_hand_sim_node",
    ):
        super().__init__(node_name)
        overrides = parameter_overrides or {}

        def declare(name: str, value):
            self.declare_parameter(name, overrides.get(name, value))

        def gp(name: str):
            return self.get_parameter(name).value

        declare("xml_path", "")
        declare("scene_file", "worlds/tuina_scene.xml")
        declare("hand_only_file", "worlds/leap_hand_right.xml")
        declare("use_scene_world", True)
        declare("physics_timestep", 0.002)
        declare("publish_rate_hz", 50.0)
        declare("watchdog_timeout_sec", 0.30)
        declare("watchdog_hold_sec", 0.30)
        declare("watchdog_ramp_sec", 0.60)
        declare("physics_substeps", 10)
        declare("command_topic", "/hand/joint_command")
        declare("state_topic", "/hand/joint_states")

        self.rate_hz = float(gp("publish_rate_hz"))
        if self.rate_hz <= 0:
            raise ValueError(f"publish_rate_hz must be > 0, got {self.rate_hz}")

        if bridge is None:
            xml_path = str(gp("xml_path"))
            if not xml_path:
                relative_path = (
                    str(gp("scene_file"))
                    if bool(gp("use_scene_world"))
                    else str(gp("hand_only_file"))
                )
                from ament_index_python.packages import get_package_share_directory

                package_share = get_package_share_directory("tuinadex_sim")
                xml_path = (
                    relative_path
                    if os.path.isabs(relative_path)
                    else os.path.join(package_share, relative_path)
                )
            bridge = MujocoSimBridge(xml_path)

        self.simulation_core = SimulationCore(
            bridge,
            timeout_sec=float(gp("watchdog_timeout_sec")),
            hold_sec=float(gp("watchdog_hold_sec")),
            ramp_sec=float(gp("watchdog_ramp_sec")),
            physics_substeps=int(gp("physics_substeps")),
        )

        self.state_pub = self.create_publisher(
            JointState, str(gp("state_topic")), 10
        )
        self.cmd_sub = self.create_subscription(
            JointState,
            str(gp("command_topic")),
            self._on_joint_command,
            10,
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self._control_loop)
        self.get_logger().info(
            f"LeapHandSimNode ready: rate={self.rate_hz:.1f}Hz, "
            f"substeps={self.simulation_core.physics_substeps}"
        )

    def _now(self) -> float:
        """Node-clock seconds; simulation and hardware must share this time contract."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joint_command(self, msg: JointState) -> None:
        names = list(msg.name) if msg.name else None
        q_target, reason = align_command(msg.position, names)
        if q_target is None:
            self.get_logger().warning(f"指令被拒绝 ({reason})，已丢弃。")
            return
        self.simulation_core.apply_command(q_target, t_now=self._now())

    def _control_loop(self) -> None:
        state = self.simulation_core.tick(self._now())
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(CANONICAL_JOINT_NAMES)
        state_msg.position = state.positions.tolist()
        state_msg.velocity = state.velocities.tolist()
        state_msg.effort = state.efforts.tolist()
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeapHandSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
