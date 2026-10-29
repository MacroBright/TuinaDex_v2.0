"""episode_buffer.py — 多模态试次内存暂存缓存 (EpisodeBuffer)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np


@dataclass
class FrameData:
    """单个控制周期的多模态采样数据帧."""
    state: np.ndarray          # float32 (D,)
    action: np.ndarray         # float32 (D,)
    images: dict[str, np.ndarray]  # cam_name -> uint8 (H, W, 3)
    timestamp: float


class EpisodeBuffer:
    """单个推拿试次 (Episode) 的多模态内存缓存.
    
    支持动态追加、向量化转换与内存重置。
    """

    def __init__(self):
        self._frames: list[FrameData] = []

    def __len__(self) -> int:
        return len(self._frames)

    def add_frame(
        self,
        state: np.ndarray | Sequence[float],
        action: np.ndarray | Sequence[float],
        images: dict[str, np.ndarray],
        timestamp: float,
    ) -> None:
        """追加一帧采样数据."""
        s = np.asarray(state, dtype=np.float32).ravel()
        a = np.asarray(action, dtype=np.float32).ravel()
        if s.shape != a.shape:
            raise ValueError(
                f"state and action must have same dimension, got state={s.shape}, action={a.shape}"
            )

        # 拷贝图像以避免引用污染
        img_dict = {k: np.asarray(v, dtype=np.uint8).copy() for k, v in images.items()}
        self._frames.append(
            FrameData(state=s, action=a, images=img_dict, timestamp=float(timestamp))
        )

    def clear(self) -> None:
        """清空当前试次缓存."""
        self._frames.clear()

    @property
    def frames(self) -> list[FrameData]:
        return self._frames

    def to_dict(self) -> dict[str, Any]:
        """批量聚合为 NumPy 数组字典."""
        if not self._frames:
            return {
                "state": np.empty((0, 0), dtype=np.float32),
                "action": np.empty((0, 0), dtype=np.float32),
                "timestamp": np.empty((0,), dtype=np.float32),
                "images": {},
            }

        states = np.stack([f.state for f in self._frames], axis=0)
        actions = np.stack([f.action for f in self._frames], axis=0)
        timestamps = np.array([f.timestamp for f in self._frames], dtype=np.float32)

        # 聚合并堆叠各相机视角
        cam_keys = self._frames[0].images.keys()
        batched_images = {}
        for cam in cam_keys:
            batched_images[cam] = np.stack([f.images[cam] for f in self._frames], axis=0)

        return {
            "state": states,
            "action": actions,
            "timestamp": timestamps,
            "images": batched_images,
        }
