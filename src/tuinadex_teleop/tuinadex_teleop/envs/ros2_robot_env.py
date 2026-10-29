"""ros2_robot_env.py — 基于 ROS 2 话题的多模态机器人环境封装."""
from __future__ import annotations

import threading
import time
from typing import Any, Sequence
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, Image
    HAS_RCLPY = True
except (ImportError, ModuleNotFoundError):
    rclpy = None
    Node = object  # type: ignore
    JointState = None  # type: ignore
    Image = None  # type: ignore
    HAS_RCLPY = False

from tuinadex_core.names import CANONICAL_JOINT_NAMES, CANONICAL_ARM_JOINT_NAMES
from tuinadex_core.command_processing import align_command, JOINT_SPEC_LIMITS

from .base_env import BaseTuinaEnv
from .config import RobotConfig


def _decode_image_msg(msg: Image) -> np.ndarray:
    """将 ROS 2 sensor_msgs/Image 解析为 (H, W, 3) 的 RGB uint8 数组."""
    h, w = msg.height, msg.width
    encoding = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding == "rgb8":
        return raw.reshape((h, w, 3))
    elif encoding == "bgr8":
        bgr = raw.reshape((h, w, 3))
        return bgr[..., ::-1]  # 转为 RGB
    elif encoding in ("rgba8", "rgb8a"):
        rgba = raw.reshape((h, w, 4))
        return rgba[..., :3]
    elif encoding in ("bgra8", "bgr8a"):
        bgra = raw.reshape((h, w, 4))
        return bgra[..., 2::-1]  # BGR -> RGB
    elif encoding == "mono8":
        gray = raw.reshape((h, w))
        return np.stack([gray, gray, gray], axis=-1)
    else:
        # 兼容未知 3 通道
        if len(raw) == h * w * 3:
            return raw.reshape((h, w, 3))
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")


class ROS2RobotEnv(BaseTuinaEnv):
    """通过 ROS 2 话题交互的多模态环境基类.
    
    无论是真机 (RealRobotEnv) 还是仿真孪生 (MuJoCoEnv)，均统一以此桥接
    ROS 2 通信，保持数据采集与训练完全解耦。
    """

    def __init__(
        self,
        config: RobotConfig,
        node_name: str = "tuina_robot_env",
        existing_node: Node | None = None,
        spin_timeout_sec: float = 0.005,
    ):
        super().__init__(config)
        if not HAS_RCLPY and existing_node is None:
            raise RuntimeError(
                "ROS 2 (rclpy/sensor_msgs) is not available in the current Python environment. "
                "Please run with system ROS 2 environment (Python 3.14) or pass an existing mock node."
            )
        self._owns_node = existing_node is None
        if self._owns_node:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node(node_name)
        else:
            self._node = existing_node

        self._spin_timeout = spin_timeout_sec
        self._lock = threading.Lock()

        # 缓存状态
        self._latest_hand_state = np.zeros(16, dtype=np.float32)
        self._latest_arm_state = np.zeros(6, dtype=np.float32)
        self._latest_images: dict[str, np.ndarray] = {}
        self._latest_timestamp = time.time()

        # 创建发布者
        self._hand_cmd_pub = self._node.create_publisher(
            JointState, "/hand/joint_command", 10
        )
        if self.config.mode == "arm_hand":
            self._arm_cmd_pub = self._node.create_publisher(
                JointState, "/arm/joint_command", 10
            )
        else:
            self._arm_cmd_pub = None

        # 创建订阅者
        self._subs = []
        self._subs.append(
            self._node.create_subscription(
                JointState, "/hand/joint_states", self._on_hand_state, 10
            )
        )
        if self.config.mode == "arm_hand":
            self._subs.append(
                self._node.create_subscription(
                    JointState, "/arm/joint_states", self._on_arm_state, 10
                )
            )

        # 摄像头订阅
        for cam_name in self.config.camera_names:
            topic = f"/camera/{cam_name}/image_raw"
            sub = self._node.create_subscription(
                Image,
                topic,
                lambda msg, name=cam_name: self._on_image(name, msg),
                10,
            )
            self._subs.append(sub)

    def _on_hand_state(self, msg: JointState) -> None:
        """手部 16-DOF 状态回调."""
        pos = np.array(msg.position, dtype=np.float32)
        if pos.shape[0] >= 16:
            with self._lock:
                self._latest_hand_state = pos[:16]
                if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
                    self._latest_timestamp = (
                        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    )
                else:
                    self._latest_timestamp = time.time()

    def _on_arm_state(self, msg: JointState) -> None:
        """机械臂 6-DOF 状态回调."""
        pos = np.array(msg.position, dtype=np.float32)
        if pos.shape[0] >= 6:
            with self._lock:
                self._latest_arm_state = pos[:6]

    def _on_image(self, cam_name: str, msg: Image) -> None:
        """摄像头图像回调 (解析为 RGB uint8)."""
        try:
            rgb_arr = _decode_image_msg(msg)
            with self._lock:
                self._latest_images[cam_name] = rgb_arr
                if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
                    self._latest_timestamp = (
                        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    )
        except Exception as e:
            self._node.get_logger().warn(f"Failed to decode image from {cam_name}: {e}")

    def reset(self) -> dict[str, Any]:
        """重置并获取初始观测."""
        self.spin_once()
        return self.get_observation()

    def spin_once(self) -> None:
        """微步执行 ROS2 事件循环以刷新话题."""
        if rclpy.ok() and self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=self._spin_timeout)

    def get_observation(self) -> dict[str, Any]:
        """获取当前最新多模态观测字典."""
        with self._lock:
            if self.config.mode == "hand_only":
                state = self._latest_hand_state.copy()
            else:
                state = np.concatenate(
                    [self._latest_arm_state, self._latest_hand_state]
                ).astype(np.float32)

            images = {k: v.copy() for k, v in self._latest_images.items()}
            timestamp = self._latest_timestamp

        return {
            "state": state,
            "images": images,
            "timestamp": timestamp,
        }

    def step(self, action: np.ndarray) -> tuple[dict[str, Any], dict[str, Any]]:
        """执行动作并返回 (obs, info)."""
        action = self.validate_action(action)

        if self.config.mode == "hand_only":
            hand_act = action
            arm_act = None
        else:
            arm_act = action[:6]
            hand_act = action[6:]

        # 发布手部控制指令 (经安全限位对齐)
        cmd_msg = JointState()
        cmd_msg.header.stamp = self._node.get_clock().now().to_msg()
        cmd_msg.name = list(CANONICAL_JOINT_NAMES)
        cmd_msg.position = [float(p) for p in hand_act]
        self._hand_cmd_pub.publish(cmd_msg)

        # 发布机械臂控制指令 (若处于臂手模式)
        if arm_act is not None and self._arm_cmd_pub is not None:
            arm_msg = JointState()
            arm_msg.header.stamp = cmd_msg.header.stamp
            arm_msg.name = list(CANONICAL_ARM_JOINT_NAMES)
            arm_msg.position = [float(p) for p in arm_act]
            self._arm_cmd_pub.publish(arm_msg)

        # 刷新话题回调
        self.spin_once()
        obs = self.get_observation()
        return obs, {}

    def close(self) -> None:
        """销毁节点资源."""
        if self._node is not None:
            for sub in self._subs:
                self._node.destroy_subscription(sub)
            self._subs.clear()
            if self._owns_node:
                self._node.destroy_node()
                self._node = None
