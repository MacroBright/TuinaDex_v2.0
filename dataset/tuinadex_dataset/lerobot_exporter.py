"""Export multimodal episodes to LeRobotDataset v3.0."""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

CONTACT_FORCE_NAMES = ["index", "middle", "ring", "thumb"]


class LeRobotDatasetWriter:
    """Write episodes as native LeRobot v3.0 Parquet and video chunks."""

    def __init__(
        self,
        repo_id: str,
        config: Any,
        fps: float = 50.0,
        use_videos: bool = True,
        vcodec: str = "h264",
        streaming_encoding: bool = True,
    ):
        self.repo_id = repo_id
        self.config = config
        self.fps = float(fps)
        self.use_videos = use_videos
        self.vcodec = vcodec
        self.streaming_encoding = bool(streaming_encoding)
        self.output_root = Path(repo_id).resolve()
        self._dataset = None
        self._phase_index: dict[str, int] = {}

    def _init_dataset_if_needed(self, first_frame: Any) -> Any:
        """Create a dataset from the first frame without fabricating images."""
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        info_path = self.output_root / "meta" / "info.json"
        if info_path.exists():
            return LeRobotDataset(self.output_root.name, root=self.output_root)

        dim = int(self.config.action_dim)
        joint_names = list(self.config.joint_names)
        features: dict[str, Any] = {
            "observation.state": {
                "dtype": "float32",
                "shape": (dim,),
                "names": {"joints": joint_names},
            },
            "observation.effort": {
                "dtype": "float32",
                "shape": (dim,),
                "names": {"joints": joint_names},
            },
            "observation.contact_forces": {
                "dtype": "float32",
                "shape": (4,),
                "names": {"fingers": CONTACT_FORCE_NAMES},
            },
            "action": {
                "dtype": "float32",
                "shape": (dim,),
                "names": {"joints": joint_names},
            },
            "phase_index": {
                "dtype": "int64",
                "shape": (1,),
                "names": None,
            },
        }

        for camera in self.config.camera_names:
            key = f"observation.images.{camera}"
            if camera not in first_frame.images_jpeg:
                raise ValueError(
                    f"camera '{camera}' missing from first frame; "
                    "refusing to fabricate a shape"
                )
            height, width = first_frame.decode_image(camera).shape[:2]
            features[key] = {
                "dtype": "video" if self.use_videos else "image",
                "shape": (3, height, width),
                "names": ["channels", "height", "width"],
            }

        logger.info(
            "Creating LeRobotDataset v3.0 at %s (videos=%s)",
            self.output_root,
            self.use_videos,
        )
        return LeRobotDataset.create(
            repo_id=self.output_root.name,
            fps=int(self.fps),
            features=features,
            root=self.output_root,
            use_videos=self.use_videos,
            vcodec=self.vcodec,
            metadata_buffer_size=1,
            streaming_encoding=self.streaming_encoding,
            encoder_queue_maxsize=30,
        )

    def _phase_id(self, label: str) -> int:
        key = label or "unspecified"
        if key not in self._phase_index:
            self._phase_index[key] = len(self._phase_index)
        return self._phase_index[key]

    def _write_phase_labels(self) -> None:
        path = self.output_root / "meta" / "phase_labels.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            str(index): name
            for name, index in self._phase_index.items()
        }
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def flush(self) -> None:
        """Flush and close the underlying dataset writers."""
        if self._dataset is None:
            return
        from lerobot.datasets.utils import load_episodes

        if hasattr(self._dataset, "_writer_closed_for_reading"):
            self._dataset._writer_closed_for_reading = True
        if hasattr(self._dataset, "_close_writer"):
            self._dataset._close_writer()
        if hasattr(self._dataset, "meta"):
            if hasattr(self._dataset.meta, "_flush_metadata_buffer"):
                self._dataset.meta._flush_metadata_buffer()
            if hasattr(self._dataset.meta, "_close_writer"):
                self._dataset.meta._close_writer()
            self._dataset.meta.episodes = load_episodes(self.output_root)
            self._dataset.meta.latest_episode = None
        self._dataset.latest_episode = None

    def save_episode(
        self, buffer: Any, task: str = "tuina_massage"
    ) -> str:
        """Write one episode while decoding only one image frame at a time."""
        import torch

        if len(buffer) == 0:
            raise ValueError("Cannot export empty EpisodeBuffer.")

        first_frame = next(buffer.iter_frames())
        if self._dataset is None:
            self._dataset = self._init_dataset_if_needed(first_frame)

        for frame in buffer.iter_frames():
            frame_dict: dict[str, Any] = {
                "observation.state": torch.from_numpy(
                    frame.state.astype(np.float32)
                ),
                "observation.effort": torch.from_numpy(
                    frame.effort.astype(np.float32)
                ),
                "observation.contact_forces": torch.from_numpy(
                    frame.contact_forces.astype(np.float32)
                ),
                "action": torch.from_numpy(
                    frame.action.astype(np.float32)
                ),
                "phase_index": torch.tensor(
                    [self._phase_id(frame.phase_label)],
                    dtype=torch.int64,
                ),
                "task": str(frame.language_instruction or task),
            }

            for camera in self.config.camera_names:
                if camera not in frame.images_jpeg:
                    raise ValueError(
                        f"camera '{camera}' missing at frame "
                        f"{frame.frame_index}"
                    )
                image = frame.decode_image(camera)
                frame_dict[f"observation.images.{camera}"] = (
                    torch.from_numpy(
                        np.transpose(image, (2, 0, 1)).astype(np.uint8)
                    )
                )

            self._dataset.add_frame(frame=frame_dict)

        self._dataset.save_episode(parallel_encoding=False)
        self._write_phase_labels()
        self.flush()
        logger.info(
            "Saved episode with %d frames to %s",
            len(buffer),
            self.output_root,
        )
        return str(self.output_root)

    def close(self) -> None:
        """Flush phase metadata and remaining writer state."""
        self._write_phase_labels()
        self.flush()
