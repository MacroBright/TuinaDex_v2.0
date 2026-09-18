"""recorder_node.py — rosbag2 进程内采集节点.

generic subscription 直收序列化字节 (零反序列化开销), Trigger 服务控制起停;
bag 输出 bags/session_YYYYmmdd_HHMMSS (sqlite3)。
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import rosbag2_py


class RecorderNode(Node):
    def __init__(self):
        super().__init__("tuinadex_recorder")
        self.declare_parameter(
            "topics", ["/hand/joint_states", "/hand/joint_command"])
        self.declare_parameter(
            "topic_types",
            ["sensor_msgs/msg/JointState", "sensor_msgs/msg/JointState"])
        self.declare_parameter("bag_root", "bags")

        self._topics = list(self.get_parameter("topics").value)
        self._types = list(self.get_parameter("topic_types").value)
        if len(self._topics) != len(self._types):
            raise SystemExit(
                f"topics({len(self._topics)}) 与 topic_types({len(self._types)}) 数量不一致")
        self._bag_root = Path(str(self.get_parameter("bag_root").value))

        self._writer: rosbag2_py.SequentialWriter | None = None
        self._subs = []

        self.create_service(Trigger, "/recorder/start", self._on_start)
        self.create_service(Trigger, "/recorder/stop", self._on_stop)
        self.get_logger().info(f"采集节点就绪: topics={self._topics}")

    def _on_start(self, request, response) -> Trigger.Response:
        if self._writer is not None:
            response.success, response.message = False, "already recording"
            return response
        bag_dir = str(self._bag_root / f"session_{datetime.now():%Y%m%d_%H%M%S}")
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                        output_serialization_format="cdr"))
        for topic_id, (topic, ttype) in enumerate(zip(self._topics, self._types)):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=topic_id, name=topic, type=ttype, serialization_format="cdr"))
            self._subs.append(self.create_subscription(
                JointState, topic,
                lambda data, _t=topic: self._write(_t, data), 10,
                raw=True,
            ))
        self._writer = writer
        self.get_logger().info(f"录制开始: {bag_dir}")
        response.success, response.message = True, bag_dir
        return response

    def _write(self, topic: str, data: bytes) -> None:
        if self._writer is not None:
            self._writer.write(topic, data, self.get_clock().now().nanoseconds)

    def _on_stop(self, request, response) -> Trigger.Response:
        if self._writer is None:
            response.success, response.message = False, "not recording"
            return response
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs.clear()
        self._writer = None   # SequentialWriter 析构时 flush 并关闭
        self.get_logger().info("录制停止。")
        response.success, response.message = True, "stopped"
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RecorderNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
