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
    from std_msgs.msg import Bool, Empty
except ImportError as e:
    print(f"[ERROR] ros2_teleop_bridge requires rclpy: {e}", file=sys.stderr)
    sys.exit(1)

UDP_TARGET_PORT = 9876
UDP_HOME_PORT = 9877
UDP_STATE_PORT = 9878

_ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
_HAND_JOINT_NAMES = tuple(f"joint_{i}" for i in range(16))


def _extract_joint_arrays(
    msg: JointState,
    expected_names: tuple[str, ...],
) -> tuple[np.ndarray, np.ndarray] | None:
    """Extract (position, effort) aligned to expected_names if sufficient length."""
    n = len(expected_names)
    if msg.name:
        idx_map = {name: idx for idx, name in enumerate(msg.name)}
        if all(name in idx_map for name in expected_names):
            pos = np.array(
                [
                    msg.position[idx_map[name]]
                    if idx_map[name] < len(msg.position)
                    else 0.0
                    for name in expected_names
                ],
                dtype=np.float32,
            )
            eff = np.array(
                [
                    msg.effort[idx_map[name]]
                    if idx_map[name] < len(msg.effort)
                    else 0.0
                    for name in expected_names
                ],
                dtype=np.float32,
            )
            return pos, eff

    if len(msg.position) < n:
        return None
    pos = np.array(msg.position[:n], dtype=np.float32)
    eff = (
        np.array(msg.effort[:n], dtype=np.float32)
        if len(msg.effort) >= n
        else np.zeros(n, dtype=np.float32)
    )
    return pos, eff


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__("tuinadex_teleop_bridge")
        self.lock = threading.Lock()
        self.q_22 = np.zeros(22, dtype=np.float32)
        self.obs_q_22 = np.zeros(22, dtype=np.float32)
        self.obs_effort_22 = np.zeros(22, dtype=np.float32)

        # Arm ready posture from teleop_real_posture.yaml
        self.q_22[:6] = [0.0, 1.2217, 0.9599, -0.7854, 0.0, 0.0]
        try:
            import yaml
            from pathlib import Path
            cfg_p = Path(__file__).resolve().parent.parent / "src" / "tuinadex_teleop" / "configs" / "teleop_real_posture.yaml"
            if cfg_p.is_file():
                with open(cfg_p, "r", encoding="utf-8") as f:
                    cfg_data = yaml.safe_load(f) or {}
                r_dict = cfg_data.get("arm", {}).get("ready_pose_deg", {})
                if isinstance(r_dict, dict) and len(r_dict) >= 6:
                    degs = [float(r_dict.get(f"joint{i+1}", 0.0)) for i in range(6)]
                    self.q_22[:6] = np.radians(degs).astype(np.float32)
        except Exception:
            pass

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest_addr = ("127.0.0.1", UDP_TARGET_PORT)
        self.state_dest_addr = ("127.0.0.1", UDP_STATE_PORT)

        self.create_subscription(JointState, "/arm/joint_command", self._arm_cb, 10)
        self.create_subscription(JointState, "/hand/joint_command", self._hand_cb, 10)
        self.create_subscription(JointState, "/arm/joint_states", self._arm_state_cb, 10)
        self.create_subscription(JointState, "/hand/joint_states", self._hand_state_cb, 10)
        self.home_pub = self.create_publisher(Empty, "/teleop/home", 10)
        self.hand_hold_pub = self.create_publisher(Bool, "/teleop/hand_hold", 10)

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
            "TeleopBridge active. Bridging ROS 2 -> UDP "
            f"127.0.0.1:{UDP_TARGET_PORT} (cmd), 127.0.0.1:{UDP_STATE_PORT} (state), "
            f"Home on {UDP_HOME_PORT}"
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

    def _arm_state_cb(self, msg: JointState):
        parsed = _extract_joint_arrays(msg, _ARM_JOINT_NAMES)
        if parsed is None:
            return
        pos, eff = parsed
        with self.lock:
            self.obs_q_22[:6] = pos
            self.obs_effort_22[:6] = eff
            self._broadcast_state()

    def _hand_state_cb(self, msg: JointState):
        parsed = _extract_joint_arrays(msg, _HAND_JOINT_NAMES)
        if parsed is None:
            return
        pos, eff = parsed
        with self.lock:
            self.obs_q_22[6:22] = pos
            self.obs_effort_22[6:22] = eff
            self._broadcast_state()

    def _broadcast(self):
        try:
            pkt = struct.pack("<4s22f", b"TD22", *self.q_22)
            self.udp_sock.sendto(pkt, self.dest_addr)
        except Exception:
            pass

    def _broadcast_state(self):
        try:
            pkt = struct.pack("<4s22f22f", b"TS22", *self.obs_q_22, *self.obs_effort_22)
            self.udp_sock.sendto(pkt, self.state_dest_addr)
        except Exception:
            pass

    def _home_listener(self):
        while rclpy.ok():
            try:
                data, _ = self.home_sock.recvfrom(64)
                if data == b"HOME":
                    self.home_pub.publish(Empty())
                    self.get_logger().info("Forwarded UDP HOME -> /teleop/home")
                elif data == b"HAND_HOLD:1":
                    self.hand_hold_pub.publish(Bool(data=True))
                    self.get_logger().info("Forwarded UDP HAND_HOLD:1 -> /teleop/hand_hold")
                elif data == b"HAND_HOLD:0":
                    self.hand_hold_pub.publish(Bool(data=False))
                    self.get_logger().info("Forwarded UDP HAND_HOLD:0 -> /teleop/hand_hold")
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
