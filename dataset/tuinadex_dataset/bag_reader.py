"""bag_reader.py — rosbag2 读取与 JointState 反序列化 (依赖 ROS2 环境)."""
from __future__ import annotations

from typing import Tuple

import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


def read_joint_state_bag(bag_dir, topic: str) -> Tuple[np.ndarray, np.ndarray]:
    """读取 bag 中指定 JointState 话题 -> (header.stamp 秒 (N,), position (N,16)).

    非法帧 (维度 != 16) 跳过并计数; 时间戳取 header.stamp
    (规范: 硬件回读时刻 / 指令生成时刻)。
    """
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    known = {m.name: m.type for m in reader.get_all_topics_and_types()}
    if topic not in known:
        raise ValueError(f"话题 {topic} 不在 bag 中 (现有: {sorted(known)})")
    if known[topic] != "sensor_msgs/msg/JointState":
        raise ValueError(f"话题 {topic} 类型为 {known[topic]}, 仅支持 JointState")

    ts_list, pos_list, skipped = [], [], 0
    while reader.has_next():
        topic_name, data, _recv = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, JointState)
        pos = np.asarray(msg.position, dtype=np.float64)
        if pos.shape != (16,):
            skipped += 1
            continue
        ts_list.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        pos_list.append(pos)
    if skipped:
        print(f"[bag_reader] 跳过 {skipped} 个非法维度帧")
    if not ts_list:
        raise ValueError(f"话题 {topic} 无有效帧")
    return np.asarray(ts_list, dtype=np.float64), np.stack(pos_list)
