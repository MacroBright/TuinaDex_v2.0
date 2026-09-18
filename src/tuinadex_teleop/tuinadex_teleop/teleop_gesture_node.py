"""teleop_gesture_node.py — MediaPipe 视觉手势遥操作 ROS2 节点.

架构规范:
1. 彻底解耦硬件驱动与控制逻辑，仅负责视觉手势感知并向 /hand/joint_command 发布目标指令；
2. 输出 16 维相对全开位的目标关节弧度 (0 为全开平伸，弯曲为正)；
3. 遵循 Canonical 16-DOF 命名与顺序；
4. 集成 1€ 指尖滤波，杜绝抖动。
"""
from __future__ import annotations

import time
from typing import Optional
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from tuinadex_teleop.tracking import HandTracker, JointMapper, OneEuroFilter

CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]


class TeleopGestureNode(Node):
    """MediaPipe 视觉手势遥操作主控节点."""

    def __init__(self):
        super().__init__("teleop_gesture_node")

        # 声明参数
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("show_preview", False)
        self.declare_parameter("filter_min_cutoff", 1.0)
        self.declare_parameter("filter_beta", 0.05)

        self.camera_id = self.get_parameter("camera_id").get_parameter_value().integer_value
        self.rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.show_preview = self.get_parameter("show_preview").get_parameter_value().bool_value
        min_cutoff = self.get_parameter("filter_min_cutoff").get_parameter_value().double_value
        beta = self.get_parameter("filter_beta").get_parameter_value().double_value

        # 初始化视觉追踪与映射器
        self.tracker = HandTracker(max_num_hands=1)
        self.mapper = JointMapper()
        self.filter = OneEuroFilter(n_joints=16, min_cutoff=min_cutoff, beta=beta)

        # 话题发布
        self.cmd_pub = self.create_publisher(JointState, "/hand/joint_command", 10)

        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开摄像头 ID={self.camera_id}!")
            raise SystemExit("Camera open failed")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.get_logger().info(
            f"手势遥操作节点就绪! [Camera={self.camera_id}, Rate={self.rate_hz}Hz, Preview={self.show_preview}]"
        )

        # 离合状态
        self.is_paused = False
        self.is_locked = False
        self.locked_angles = np.zeros(16, dtype=np.float64)

        # 定时采集与发布循环
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self._loop)

    def _loop(self) -> None:
        """视觉推理与发布主循环."""
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        now = time.time()
        results = self.tracker.detect(frame)

        if results and len(results) > 0:
            hand = results[0]
            # 1. 映射为 16 维相对弧度 (0 为全开，>0 为弯曲)
            raw_angles = self.mapper.map_keypoints_to_leap(hand)
            # 2. 1€ 滤波平滑
            filtered_angles = self.filter(raw_angles)
        else:
            # 未检测到手部: 不发送或保持
            filtered_angles = None

        # 3. 构造并发布 ROS2 指令
        if filtered_angles is not None and not self.is_paused:
            if self.is_locked:
                out_angles = self.locked_angles
            else:
                out_angles = filtered_angles

            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = list(CANONICAL_JOINT_NAMES)
            cmd_msg.position = out_angles.tolist()

            self.cmd_pub.publish(cmd_msg)

        # 4. 可选 OpenCV 画面渲染与快捷键
        if self.show_preview:
            annotated = self.tracker.draw_landmarks(frame, results) if results else frame
            status_text = "PAUSED" if self.is_paused else ("LOCKED" if self.is_locked else "TRACKING")
            cv2.putText(
                annotated, f"STATUS: {status_text}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
            )
            cv2.imshow("TuinaDex Teleop HUD", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == 32:  # SPACE 离合切换
                self.is_paused = not self.is_paused
            elif key == ord('l') or key == ord('L'):  # L 锁定手势
                self.is_locked = not self.is_locked
                if self.is_locked and filtered_angles is not None:
                    self.locked_angles = filtered_angles.copy()
            elif key == ord('q') or key == 27:  # Q / ESC 退出
                rclpy.shutdown()

    def destroy_node(self) -> None:
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeleopGestureNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
