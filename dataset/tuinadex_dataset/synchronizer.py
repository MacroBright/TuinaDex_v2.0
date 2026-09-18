"""synchronizer.py — 示教时间序列 50Hz 网格对齐与插值同步器."""
from __future__ import annotations

from typing import List, Tuple
import numpy as np


class TimeSynchronizer:
    """负责将异步 State 与 Command 采样序列对齐到统一固定时间网格."""

    def __init__(self, target_fps: float = 50.0):
        self.target_fps = target_fps
        self.dt = 1.0 / target_fps

    def align_series(
        self,
        state_timestamps: np.ndarray,
        state_positions: np.ndarray,
        action_timestamps: np.ndarray,
        action_positions: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """将状态与动作严格对齐至 target_fps 规则时钟网格.
        
        Args:
            state_timestamps: (N,) 状态采集时间戳 (秒)
            state_positions: (N, 16) 16 维实测关节位置 (rad)
            action_timestamps: (M,) 动作发布时间戳 (秒)
            action_positions: (M, 16) 16 维目标关节动作 (rad)
            
        Returns:
            grid_times: (T,) 规则网格时间戳
            aligned_states: (T, 16) 对齐后的状态
            aligned_actions: (T, 16) 对齐后的动作
        """
        # 1. 确定重叠时间有效区间
        t_start = max(state_timestamps[0], action_timestamps[0])
        t_end = min(state_timestamps[-1], action_timestamps[-1])

        if t_start >= t_end:
            raise ValueError(
                f"时间戳无重叠区间: state [{state_timestamps[0]:.3f}, {state_timestamps[-1]:.3f}], "
                f"action [{action_timestamps[0]:.3f}, {action_timestamps[-1]:.3f}]"
            )

        # 2. 生成规则时间网格
        grid_times = np.arange(t_start, t_end, self.dt)
        T = len(grid_times)
        if T == 0:
            raise ValueError("对齐后的有效帧数为 0。")

        # 3. 线性插值对齐 16 维状态与动作
        aligned_states = np.zeros((T, 16), dtype=np.float32)
        aligned_actions = np.zeros((T, 16), dtype=np.float32)

        for dim in range(16):
            aligned_states[:, dim] = np.interp(
                grid_times, state_timestamps, state_positions[:, dim]
            )
            aligned_actions[:, dim] = np.interp(
                grid_times, action_timestamps, action_positions[:, dim]
            )

        return grid_times, aligned_states, aligned_actions
