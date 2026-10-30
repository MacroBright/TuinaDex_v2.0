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

from tuinadex_core.command_processing import align_command, align_arm_command
from tuinadex_core.names import CANONICAL_JOINT_NAMES, CANONICAL_ARM_JOINT_NAMES
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
        declare("arm_command_topic", "/arm/joint_command")
        declare("arm_state_topic", "/arm/joint_states")
        declare("publish_arm_states", False)
        declare("gui", False)

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
            bridge = MujocoSimBridge(xml_path, timestep=float(gp("physics_timestep")))

        self.simulation_core = SimulationCore(
            bridge,
            timeout_sec=float(gp("watchdog_timeout_sec")),
            hold_sec=float(gp("watchdog_hold_sec")),
            ramp_sec=float(gp("watchdog_ramp_sec")),
            physics_substeps=int(gp("physics_substeps")),
        )

        self.gui_enabled = bool(gp("gui"))
        self.viewer = None
        if self.gui_enabled:
            if hasattr(bridge, "model") and hasattr(bridge, "data"):
                try:
                    import mujoco.viewer

                    self.viewer = mujoco.viewer.launch_passive(bridge.model, bridge.data)
                    self.get_logger().info("MuJoCo 被动 3D 可视化窗口 (GUI) 已启动。")
                except Exception as e:
                    self.get_logger().warning(f"无法启动 MuJoCo Viewer: {e}")
            else:
                self.get_logger().warning("Bridge 没有 model/data 属性，跳过启动 GUI。")

        self.state_pub = self.create_publisher(
            JointState, str(gp("state_topic")), 10
        )
        self.cmd_sub = self.create_subscription(
            JointState,
            str(gp("command_topic")),
            self._on_joint_command,
            10,
        )

        # 6-DOF 机械臂接口 (当 MuJoCo 模型包含机械臂时按参数协调)
        # 默认模式 (publish_arm_states=False): 由 ArmNode 独占发布 /arm/joint_states，
        # LeapHandSimNode 仅订阅同步至 MuJoCo 动力学模型，彻底消除双发话题调度竞争。
        # 独立运行模式 (publish_arm_states=True): 允许无 ArmNode 时由 LeapHandSimNode 独占接管。
        self.arm_state_pub = None
        self.arm_cmd_sub = None
        self.arm_sync_sub = None
        self.publish_arm_states = bool(gp("publish_arm_states"))

        if getattr(bridge, "has_arm", False):
            if self.publish_arm_states:
                self.arm_state_pub = self.create_publisher(
                    JointState, str(gp("arm_state_topic")), 10
                )
                self.arm_cmd_sub = self.create_subscription(
                    JointState,
                    str(gp("arm_command_topic")),
                    self._on_arm_joint_command,
                    10,
                )
                self.get_logger().info("检测到 6-DOF 机械臂模型，LeapHandSimNode 已启用独占机械臂发布与控制话题。")
            else:
                self.arm_sync_sub = self.create_subscription(
                    JointState,
                    str(gp("arm_state_topic")),
                    self._on_arm_joint_state_sync,
                    10,
                )
                self.get_logger().info("检测到 6-DOF 机械臂模型，已启用跟随同步订阅模式 (话题竞争防护生效，由 ArmNode 主导发布)。")

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
            self.get_logger().warning(f"灵巧手指令被拒绝 ({reason})，已丢弃。")
            return
        # 此处已校验/clamp 过；core.apply_command 按契约幂等复检一次，保证 core 独立可测。
        self.simulation_core.apply_command(q_target, t_now=self._now())

    def _on_arm_joint_command(self, msg: JointState) -> None:
        names = list(msg.name) if msg.name else None
        q_target, reason = align_arm_command(msg.position, names)
        if q_target is None:
            self.get_logger().warning(f"机械臂指令被拒绝 ({reason})，已丢弃。")
            return
        if hasattr(self.simulation_core.bridge, "set_arm_targets"):
            self.simulation_core.bridge.set_arm_targets(q_target)

    def _on_arm_joint_state_sync(self, msg: JointState) -> None:
        names = list(msg.name) if msg.name else None
        q_pos, reason = align_arm_command(msg.position, names)
        if q_pos is not None and hasattr(self.simulation_core.bridge, "set_arm_targets"):
            self.simulation_core.bridge.set_arm_targets(q_pos)

    def _control_loop(self) -> None:
        state = self.simulation_core.tick(self._now())
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(CANONICAL_JOINT_NAMES)
        state_msg.position = state.positions.tolist()
        state_msg.velocity = state.velocities.tolist()
        state_msg.effort = state.efforts.tolist()
        self.state_pub.publish(state_msg)

        # 发布 6-DOF 机械臂状态
        if self.arm_state_pub is not None and hasattr(self.simulation_core.bridge, "get_arm_state"):
            arm_state = self.simulation_core.bridge.get_arm_state()
            if arm_state is not None:
                arm_msg = JointState()
                arm_msg.header.stamp = state_msg.header.stamp
                arm_msg.name = list(CANONICAL_ARM_JOINT_NAMES)
                arm_msg.position = arm_state.positions.tolist()
                arm_msg.velocity = arm_state.velocities.tolist()
                arm_msg.effort = arm_state.efforts.tolist()
                self.arm_state_pub.publish(arm_msg)

    def destroy_node(self) -> bool:
        if getattr(self, "viewer", None) is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
        return super().destroy_node()


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
