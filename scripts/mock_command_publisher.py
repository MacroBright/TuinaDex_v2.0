#!/usr/bin/env python3
"""mock_command_publisher.py — 模拟发布 16D 手部 / 6D 机械臂 / 22D 臂手协同 JointState 指令."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src" / "tuinadex_core") not in sys.path:
    sys.path.insert(0, str(ROOT / "src" / "tuinadex_core"))

from tuinadex_core.names import CANONICAL_JOINT_NAMES, CANONICAL_ARM_JOINT_NAMES


class MockCommandPublisher(Node):
    def __init__(
        self,
        mode: str = "hand_only",
        hand_position: float = 0.5,
        arm_position: float = 0.2,
        rate_hz: float = 50.0,
    ):
        super().__init__("mock_command_publisher")
        self.mode = mode
        self.hand_pos = hand_position
        self.arm_pos = arm_position

        self.hand_pub = None
        self.arm_pub = None

        if self.mode in ("hand_only", "arm_hand"):
            self.hand_pub = self.create_publisher(JointState, "/hand/joint_command", 10)
        if self.mode in ("arm_only", "arm_hand"):
            self.arm_pub = self.create_publisher(JointState, "/arm/joint_command", 10)

        self.timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f"MockCommandPublisher running in mode='{self.mode}' at {rate_hz}Hz "
            f"(arm_pos={self.arm_pos}, hand_pos={self.hand_pos})"
        )

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()

        if self.hand_pub is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(CANONICAL_JOINT_NAMES)
            msg.position = [float(self.hand_pos)] * 16
            self.hand_pub.publish(msg)

        if self.arm_pub is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(CANONICAL_ARM_JOINT_NAMES)
            msg.position = [float(self.arm_pos)] * 6
            self.arm_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="TuinaDex Mock Command Publisher")
    parser.add_argument(
        "--mode",
        choices=["hand_only", "arm_hand", "arm_only"],
        default="hand_only",
        help="Target control mode",
    )
    parser.add_argument("--position", type=float, default=0.5, help="Hand joint position (rad)")
    parser.add_argument("--arm-position", type=float, default=0.2, help="Arm joint position (rad)")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Publish rate (Hz)")
    parser.add_argument("--duration-sec", type=float, default=3.0, help="Run duration in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = MockCommandPublisher(
        mode=args.mode,
        hand_position=args.position,
        arm_position=args.arm_position,
        rate_hz=args.rate_hz,
    )
    timer = node.create_timer(args.duration_sec, rclpy.shutdown)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
