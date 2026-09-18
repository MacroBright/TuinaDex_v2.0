#!/usr/bin/env python3
"""Publish timed absolute JointState commands for mock closed-loop checks."""
from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]


class MockCommandPublisher(Node):
    def __init__(self, position: float, rate_hz: float):
        super().__init__("mock_command_publisher")
        self.position = position
        self.pub = self.create_publisher(JointState, "/hand/joint_command", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(CANONICAL_JOINT_NAMES)
        msg.position = [self.position] * 16
        self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--position", type=float, default=0.5)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = MockCommandPublisher(args.position, args.rate_hz)
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
