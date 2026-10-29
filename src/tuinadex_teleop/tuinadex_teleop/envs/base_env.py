"""base_env.py — 统一机器人推拿环境抽象基类 (BaseTuinaEnv)."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any
import numpy as np

from .config import RobotConfig


class BaseTuinaEnv(ABC):
    """TuinaDex 统一环境抽象基类 (Gym-like API).
    
    所有仿真环境 (MuJoCoEnv) 与物理环境 (RealRobotEnv) 均必须继承该接口，
    保证上层示教采集、质检状态机和策略推理完全透明无感。
    """

    def __init__(self, config: RobotConfig):
        self._config = config

    @property
    def config(self) -> RobotConfig:
        return self._config

    def validate_action(self, action: np.ndarray | Sequence[float]) -> np.ndarray:
        """校验动作维度与数据类型，返回 float32 一维 NumPy 数组."""
        arr = np.asarray(action, dtype=np.float32).ravel()
        expected = self.config.action_dim
        if arr.shape[0] != expected:
            raise ValueError(
                f"Expected action shape ({expected},), got shape {arr.shape} for mode '{self.config.mode}'"
            )
        return arr

    @abstractmethod
    def reset(self) -> dict[str, Any]:
        """重置环境，返回初始观测字典:
        
        {
            "state": np.ndarray (float32 rad, 形状为 (action_dim,)),
            "images": dict[str, np.ndarray] (各个视角的 uint8 RGB 图像),
            "timestamp": float (秒),
        }
        """

    @abstractmethod
    def step(self, action: np.ndarray) -> tuple[dict[str, Any], dict[str, Any]]:
        """执行一个动作，步进物理/硬件，返回 (observation, info)."""

    @abstractmethod
    def get_observation(self) -> dict[str, Any]:
        """非破坏性获取当前最新观测字典."""

    @abstractmethod
    def close(self) -> None:
        """释放底层节点或物理资源."""
