"""leap_hand_sim_node.py — TuinaDex 16-DOF LEAP Hand MuJoCo 仿真 ROS2 节点.

在系统中扮演与真实硬件完全相同的数字孪生替代品角色:
- 订阅: /hand/joint_command (sensor_msgs/JointState)
- 发布: /hand/joint_states (sensor_msgs/JointState)
- 集成 Watchdog: 超时 300ms 自动平滑回退 Relax-to-OPEN
"""
from __future__ import annotations

import os
import sys
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:
    Node = object
    JointState = None

from .sim_bridge import CANONICAL_JOINT_NAMES, MujocoSimBridge


class LeapHandSimNode(Node):
    """LEAP Hand MuJoCo 仿真节点 (Phase 1 基础脚手架)."""

    def __init__(self, node_name: str = "leap_hand_sim_node"):
        super().__init__(node_name)
        self.get_logger().info("Initializing LeapHandSimNode (MuJoCo Digital Twin)...")


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
