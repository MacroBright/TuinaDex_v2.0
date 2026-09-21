#!/usr/bin/env python3
"""ROS2 loopback probe for LeapHandSimNode: follow, rate, and relax checks."""
from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--position", type=float, default=0.2)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--active-sec", type=float, default=1.5)
    parser.add_argument("--settle-sec", type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("tuinadex_sim_loop_probe")
    received = []
    node.create_subscription(
        JointState,
        "/hand/joint_states",
        lambda msg: received.append(msg),
        20,
    )
    publisher = node.create_publisher(JointState, "/hand/joint_command", 10)

    try:
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not received:
            raise SystemExit("E2E failed: no /hand/joint_states within 5 seconds")

        command = [args.position] * len(CANONICAL_JOINT_NAMES)
        message = JointState()
        message.name = list(CANONICAL_JOINT_NAMES)
        active_deadline = time.monotonic() + args.active_sec
        period = 1.0 / args.rate_hz
        next_publish = time.monotonic()
        while time.monotonic() < active_deadline:
            now = time.monotonic()
            if now >= next_publish:
                message.header.stamp = node.get_clock().now().to_msg()
                message.position = command
                publisher.publish(message)
                next_publish += period
            rclpy.spin_once(node, timeout_sec=0.002)

        last_positions = list(received[-1].position)
        if len(last_positions) != 16 or not all(
            abs(value - args.position) < 0.08 for value in last_positions
        ):
            raise SystemExit(
                f"E2E failed: command did not converge, got {last_positions}"
            )

        stamps = [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for msg in received[-40:]
        ]
        interval = statistics.median(
            right - left for left, right in zip(stamps, stamps[1:])
        )
        if not 0.015 <= interval <= 0.025:
            raise SystemExit(
                f"E2E failed: median state interval {interval:.4f}s is not 50Hz"
            )

        settle_deadline = time.monotonic() + args.settle_sec
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if received and all(abs(value) < 0.05 for value in received[-1].position):
                break
        relaxed = list(received[-1].position)
        if not all(abs(value) < 0.05 for value in relaxed):
            raise SystemExit(f"E2E failed: watchdog did not relax to OPEN, got {relaxed}")

        print(
            f"[E2E SIM] PASS: {len(received)} states, median interval="
            f"{interval * 1000:.2f}ms, converged, relaxed to OPEN"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
