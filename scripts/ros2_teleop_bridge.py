#!/usr/bin/env python3
"""ros2_teleop_bridge.py — Lightweight ROS 2 <-> UDP Teleoperation Bridge.

Bridges ROS 2 topics (/arm/joint_command, /hand/joint_command, /teleop/home)
to a local UDP bus (port 9876/9877) so that Python 3.10 environments (which
cannot directly load ROS 2 Lyrical Python 3.14 C-extensions) receive real-time
teleoperation commands with zero latency.
"""
from __future__ import annotations

import signal
import socket
import struct
import sys
import threading
import time
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Empty
except ImportError as e:
    print(f"[ERROR] ros2_teleop_bridge requires rclpy: {e}", file=sys.stderr)
    sys.exit(1)

UDP_TARGET_PORT = 9876
UDP_HOME_PORT = 9877


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__("tuinadex_teleop_bridge")
        self.lock = threading.Lock()
        self.q_22 = np.zeros(22, dtype=np.float32)

        # Default arm ready posture (DEFAULT_READY_ARM_Q)
        self.q_22[:6] = [0.1511, 1.0429, 0.7581, -0.2432, 1.4212, 0.0]

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest_addr = ("127.0.0.1", UDP_TARGET_PORT)

        self.create_subscription(JointState, "/arm/joint_command", self._arm_cb, 10)
        self.create_subscription(JointState, "/hand/joint_command", self._hand_cb, 10)
        self.home_pub = self.create_publisher(Empty, "/teleop/home", 10)

        # UDP Home Receiver loop
        self.home_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.home_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.home_sock.bind(("127.0.0.1", UDP_HOME_PORT))
            self.home_sock.settimeout(0.2)
            self.home_thread = threading.Thread(target=self._home_listener, daemon=True)
            self.home_thread.start()
        except Exception as exc:
            self.get_logger().debug(f"Home UDP port already bound: {exc}")

        self.get_logger().info(
            f"TeleopBridge active. Bridging ROS 2 -> UDP 127.0.0.1:{UDP_TARGET_PORT} (Home on {UDP_HOME_PORT})"
        )

    def _arm_cb(self, msg: JointState):
        with self.lock:
            if len(msg.position) >= 6:
                self.q_22[:6] = np.array(msg.position[:6], dtype=np.float32)
                self._broadcast()

    def _hand_cb(self, msg: JointState):
        with self.lock:
            if len(msg.position) >= 16:
                self.q_22[6:22] = np.array(msg.position[:16], dtype=np.float32)
                self._broadcast()

    def _broadcast(self):
        try:
            pkt = struct.pack("<4s22f", b"TD22", *self.q_22)
            self.udp_sock.sendto(pkt, self.dest_addr)
        except Exception:
            pass

    def _home_listener(self):
        while rclpy.ok():
            try:
                data, _ = self.home_sock.recvfrom(32)
                if data == b"HOME":
                    self.home_pub.publish(Empty())
                    self.get_logger().info("Forwarded UDP HOME -> /teleop/home")
            except socket.timeout:
                continue
            except Exception:
                break


def main():
    rclpy.init()
    node = TeleopBridgeNode()
    try:
        rclpy.spin(node)
    except BaseException:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
