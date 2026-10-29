"""collector.py — 交互式数据采集会话编排器与状态机 (TuinaCollector)."""
from __future__ import annotations

from enum import Enum, auto
import logging
from typing import Any, Callable, Optional
import numpy as np

from ..envs.base_env import BaseTuinaEnv
from .episode_buffer import EpisodeBuffer

logger = logging.getLogger(__name__)


class CollectionState(Enum):
    """数据采集交互状态机枚举."""
    IDLE = auto()        # 空闲就绪状态 (不记录数据)
    RECORDING = auto()   # 正在示教录制中
    PAUSED = auto()      # 示教中断暂停
    REVIEWING = auto()   # 单个试次完成，等待评审确认 (Save / Discard / Restart)


class TuinaCollector:
    """数据采集器主控制器.
    
    统一编排 BaseTuinaEnv 步进、EpisodeBuffer 缓冲与交互状态机流转。
    """

    def __init__(
        self,
        env: BaseTuinaEnv,
        on_save: Optional[Callable[[EpisodeBuffer], None]] = None,
    ):
        self.env = env
        self.buffer = EpisodeBuffer()
        self.state = CollectionState.IDLE
        self.on_save = on_save

        self.total_episodes_saved = 0
        self.total_episodes_discarded = 0

    def start_recording(self) -> None:
        """从 IDLE 或 PAUSED 状态进入 RECORDING 状态."""
        if self.state in (CollectionState.IDLE, CollectionState.PAUSED):
            self.state = CollectionState.RECORDING
            logger.info("Collector entered RECORDING state.")
        elif self.state == CollectionState.REVIEWING:
            logger.warning("Cannot start recording while in REVIEWING state. Please confirm or discard first.")

    def pause_recording(self) -> None:
        """暂停当前录制."""
        if self.state == CollectionState.RECORDING:
            self.state = CollectionState.PAUSED
            logger.info("Collector PAUSED.")

    def stop_recording(self) -> None:
        """完成当前试次，进入 REVIEWING 评审确认状态."""
        if self.state in (CollectionState.RECORDING, CollectionState.PAUSED):
            self.state = CollectionState.REVIEWING
            logger.info(f"Collector entered REVIEWING state with {len(self.buffer)} frames.")

    def restart_episode(self) -> None:
        """放弃当前未完试次并立即重新开始录制."""
        self.buffer.clear()
        self.state = CollectionState.RECORDING
        logger.info("Collector restarted current episode.")

    def review_decision(self, save: bool) -> bool:
        """评审当前试次: save=True 保存, save=False 丢弃."""
        if self.state != CollectionState.REVIEWING:
            logger.warning(f"review_decision called while in state {self.state}, expected REVIEWING.")
            return False

        if save:
            if len(self.buffer) > 0:
                if self.on_save is not None:
                    self.on_save(self.buffer)
                self.total_episodes_saved += 1
                logger.info(f"Episode approved and saved ({len(self.buffer)} frames). Total saved: {self.total_episodes_saved}")
            self.buffer.clear()
            self.state = CollectionState.IDLE
            return True
        else:
            dropped_frames = len(self.buffer)
            self.buffer.clear()
            self.total_episodes_discarded += 1
            self.state = CollectionState.IDLE
            logger.info(f"Episode discarded ({dropped_frames} frames). Total discarded: {self.total_episodes_discarded}")
            return False

    def step_once(self, action: np.ndarray) -> tuple[dict[str, Any], dict[str, Any]]:
        """单步执行动作，若处于 RECORDING 状态则自动记入缓存."""
        obs, info = self.env.step(action)

        if self.state == CollectionState.RECORDING:
            self.buffer.add_frame(
                state=obs["state"],
                action=action,
                images=obs.get("images", {}),
                timestamp=obs.get("timestamp", 0.0),
            )

        return obs, info
