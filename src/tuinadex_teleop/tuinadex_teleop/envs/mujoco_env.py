"""mujoco_env.py — MuJoCo 数字孪生仿真推拿环境 (MuJoCoEnv)."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any
import numpy as np

from .config import RobotConfig
from .ros2_robot_env import ROS2RobotEnv, BaseTuinaEnv

try:
    from tuinadex_sim.sim_bridge import MujocoSimBridge
    HAS_SIM_BRIDGE = True
except (ImportError, ModuleNotFoundError):
    MujocoSimBridge = None
    HAS_SIM_BRIDGE = False


def _get_default_tuina_scene_xml() -> str:
    """获取默认推拿仿真场景 XML 路径."""
    root = Path(__file__).resolve().parents[4]
    candidates = [
        root / "src" / "tuinadex_sim" / "worlds" / "tuina_scene.xml",
        root / "src" / "tuinadex_sim" / "urdf" / "tuina_scene.xml",
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return ""


class MuJoCoEnv(BaseTuinaEnv):
    """MuJoCo 数字孪生推拿环境.
    
    支持两种运行模式:
    1. ROS 2 桥接模式 (standalone=False): 订阅/发布仿真节点话题 (/hand/joint_command, /hand/joint_states)；
    2. 进程内独立直驱模式 (standalone=True): 纯 Python 内核，直驱 MujocoSimBridge，
       适用于快速单测、离线自动化回归及无 ROS 2 守护进程环境。
    """

    def __init__(
        self,
        config: RobotConfig,
        standalone: bool = False,
        scene_xml: str | None = None,
        node_name: str = "mujoco_tuina_sim_env",
        existing_node: Any = None,
        spin_timeout_sec: float = 0.005,
    ):
        super().__init__(config)
        self.standalone = standalone
        self._ros_env: ROS2RobotEnv | None = None
        self._bridge: Any | None = None
        self._xml_path = scene_xml or _get_default_tuina_scene_xml()

        if self.standalone:
            if not HAS_SIM_BRIDGE or MujocoSimBridge is None:
                raise RuntimeError("tuinadex_sim.sim_bridge is required for standalone MuJoCoEnv")
            if not os.path.exists(self._xml_path):
                raise FileNotFoundError(f"Simulation scene XML not found: {self._xml_path}")
            self._bridge = MujocoSimBridge(self._xml_path, timestep=self.config.physics_dt)
            self._current_time = 0.0
        else:
            self._ros_env = ROS2RobotEnv(
                config=config,
                node_name=node_name,
                existing_node=existing_node,
                spin_timeout_sec=spin_timeout_sec,
            )

    def reset(self) -> dict[str, Any]:
        """重置仿真世界并返回初始观测."""
        if self.standalone:
            self._bridge.reset()
            self._current_time = 0.0
            return self.get_observation()
        assert self._ros_env is not None
        return self._ros_env.reset()

    def get_observation(self) -> dict[str, Any]:
        """获取当前最新多模态观测."""
        if self.standalone:
            sim_state = self._bridge.get_state()
            if self.config.mode == "hand_only":
                state = sim_state.positions.copy().astype(np.float32)
            else:
                arm_zeros = np.zeros(6, dtype=np.float32)
                state = np.concatenate([arm_zeros, sim_state.positions]).astype(np.float32)

            # 生成摄像头画面 (独立模式下生成标准测试图案或虚拟画面)
            images: dict[str, np.ndarray] = {}
            for cam in self.config.camera_names:
                # 默认尺寸 240x320 RGB
                images[cam] = np.full((240, 320, 3), 128, dtype=np.uint8)

            return {
                "state": state,
                "images": images,
                "timestamp": self._current_time,
            }

        assert self._ros_env is not None
        return self._ros_env.get_observation()

    def step(self, action: np.ndarray) -> tuple[dict[str, Any], dict[str, Any]]:
        """执行一个动作步进."""
        action = self.validate_action(action)

        if self.standalone:
            if self.config.mode == "hand_only":
                hand_act = action
            else:
                hand_act = action[6:]

            # 设置目标并前向物理步进 (10 * 0.002s = 0.02s = 50Hz)
            self._bridge.set_joint_targets(hand_act)
            substeps = max(1, int(round((1.0 / self.config.fps) / self.config.physics_dt)))
            self._bridge.step(substeps)
            self._current_time += 1.0 / self.config.fps
            return self.get_observation(), {}

        assert self._ros_env is not None
        return self._ros_env.step(action)

    def close(self) -> None:
        """关闭并释放资源."""
        if self._ros_env is not None:
            self._ros_env.close()
            self._ros_env = None
        self._bridge = None
