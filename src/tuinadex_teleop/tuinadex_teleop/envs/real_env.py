"""real_env.py — 真实物理机器人推拿环境 (RealRobotEnv)."""
from __future__ import annotations

import logging
from typing import Any
import numpy as np

from .config import RobotConfig
from .ros2_robot_env import ROS2RobotEnv

logger = logging.getLogger(__name__)


class RealRobotEnv(ROS2RobotEnv):
    """物理灵巧手与机械臂环境封装.
    
    通过 ROS 2 话题与底层串口总线 (Dynamixel) 和 SocketCAN (reBot) 节点通信。
    在启动时核验连接状态，并受 300ms 物理看门狗严格保护。
    """

    def __init__(
        self,
        config: RobotConfig,
        node_name: str = "real_tuina_robot_env",
        existing_node: Any = None,
        spin_timeout_sec: float = 0.005,
    ):
        super().__init__(
            config=config,
            node_name=node_name,
            existing_node=existing_node,
            spin_timeout_sec=spin_timeout_sec,
        )
        logger.info(f"RealRobotEnv initialized in mode '{self.config.mode}'.")
