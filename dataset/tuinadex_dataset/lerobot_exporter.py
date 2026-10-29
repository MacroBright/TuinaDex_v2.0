"""lerobot_exporter.py — LeRobotDataset v3.0 多模态轨迹导出器."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any
import numpy as np

logger = logging.getLogger(__name__)


class LeRobotDatasetWriter:
    """将 EpisodeBuffer 导出为原生 LeRobotDataset v3.0 (含 Parquet 与 MP4 视频分块)."""

    def __init__(
        self,
        repo_id: str,
        config: Any,
        fps: float = 50.0,
        use_videos: bool = True,
        vcodec: str = "h264",
    ):
        self.repo_id = repo_id
        self.config = config
        self.fps = float(fps)
        self.use_videos = use_videos
        self.vcodec = vcodec
        self.output_root = Path(repo_id).resolve()
        self._dataset = None

    def _init_dataset_if_needed(self, first_frame: Any) -> Any:
        """根据第一帧图像尺寸与配置初始化或加载 LeRobotDataset."""
        import torch
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        info_path = self.output_root / "meta" / "info.json"
        if info_path.exists():
            return LeRobotDataset(self.output_root.name, root=self.output_root)

        features: dict[str, Any] = {
            "observation.state": {
                "dtype": "float32",
                "shape": (self.config.action_dim,),
                "names": {"joints": list(self.config.joint_names)},
            },
            "action": {
                "dtype": "float32",
                "shape": (self.config.action_dim,),
                "names": {"joints": list(self.config.joint_names)},
            },
        }

        # 动态根据第一帧注入相机图像/视频特征
        for cam in self.config.camera_names:
            key = f"observation.images.{cam}"
            if cam in first_frame.images:
                img = first_frame.images[cam]
                h, w = img.shape[:2]
            else:
                h, w = 240, 320

            features[key] = {
                "dtype": "video" if self.use_videos else "image",
                "shape": (3, h, w),
                "names": ["channels", "height", "width"],
            }

        logger.info(f"Creating new LeRobotDataset v3.0 at {self.output_root} (use_videos={self.use_videos}).")
        return LeRobotDataset.create(
            repo_id=self.output_root.name,
            fps=int(self.fps),
            features=features,
            root=self.output_root,
            use_videos=self.use_videos,
            vcodec=self.vcodec,
            metadata_buffer_size=1,
        )

    def flush(self) -> None:
        """刷新并关闭底层写入器，保证 Parquet 与视频数据完整落盘."""
        if self._dataset is not None:
            if hasattr(self._dataset, "_close_writer"):
                self._dataset._close_writer()
            if hasattr(self._dataset, "meta"):
                if hasattr(self._dataset.meta, "_flush_metadata_buffer"):
                    self._dataset.meta._flush_metadata_buffer()
                if hasattr(self._dataset.meta, "_close_writer"):
                    self._dataset.meta._close_writer()

    def save_episode(self, buffer: Any, task: str = "tuina_massage") -> str:
        """将一个完整的 EpisodeBuffer 序列写入数据集并保存."""
        import torch

        if len(buffer) == 0:
            raise ValueError("Cannot export empty EpisodeBuffer.")

        if self._dataset is None:
            self._dataset = self._init_dataset_if_needed(buffer.frames[0])

        for frame in buffer.frames:
            frame_dict: dict[str, Any] = {
                "observation.state": torch.from_numpy(frame.state.astype(np.float32)),
                "action": torch.from_numpy(frame.action.astype(np.float32)),
                "task": str(task),
            }

            for cam in self.config.camera_names:
                key = f"observation.images.{cam}"
                if cam in frame.images:
                    img = frame.images[cam]
                    if img.ndim == 3 and img.shape[2] == 3:
                        # (H, W, 3) -> (3, H, W)
                        img_chw = np.transpose(img, (2, 0, 1))
                    else:
                        img_chw = img
                    frame_dict[key] = torch.from_numpy(img_chw.astype(np.uint8))

            self._dataset.add_frame(frame=frame_dict)

        self._dataset.save_episode()
        self.flush()
        logger.info(f"Successfully saved episode with {len(buffer)} frames to {self.output_root}.")
        return str(self.output_root)
