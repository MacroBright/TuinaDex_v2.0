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
        self._active_cameras: list[str] = []

    def _detect_active_cameras(self, first_frame: Any) -> list[str]:
        """Detect which cameras actually contain non-empty JPEG payloads in the recorded buffer.

        If camera_names was configured, preserve that preferred ordering.
        Include any active cameras present in first_frame.images_jpeg.
        """
        candidates: list[str] = []
        if hasattr(self.config, "camera_names") and self.config.camera_names:
            for cam in self.config.camera_names:
                if cam not in candidates:
                    candidates.append(cam)
        if hasattr(first_frame, "images_jpeg") and isinstance(first_frame.images_jpeg, dict):
            for cam in first_frame.images_jpeg.keys():
                if cam not in candidates:
                    candidates.append(cam)

        active: list[str] = []
        for cam in candidates:
            if hasattr(first_frame, "images_jpeg") and cam in first_frame.images_jpeg:
                payload = first_frame.images_jpeg[cam]
                if payload is not None and len(payload) > 0:
                    active.append(cam)
        return active

    def _init_dataset_if_needed(self, first_frame: Any) -> Any:
        """Create a dataset from the first frame without fabricating images."""
        import re
        import shutil
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self._active_cameras = self._detect_active_cameras(first_frame)
        logger.info(
            "Detected %d active camera(s) for export: %s",
            len(self._active_cameras),
            self._active_cameras,
        )

        info_path = self.output_root / "meta" / "info.json"
        if info_path.exists():
            try:
                meta = json.loads(info_path.read_text(encoding="utf-8"))
                existing_cams = [
                    k.replace("observation.images.", "")
                    for k in meta.get("features", {}).keys()
                    if k.startswith("observation.images.")
                ]
                total_episodes = int(meta.get("total_episodes", 0))
            except Exception as e:
                logger.warning("Failed to parse existing %s: %s", info_path, e)
                existing_cams = []
                total_episodes = 0

            if sorted(existing_cams) == sorted(self._active_cameras):
                logger.info(
                    "Loading existing dataset at %s with matching cameras: %s",
                    self.output_root,
                    existing_cams,
                )
                return LeRobotDataset(self.output_root.name, root=self.output_root)

            # Camera schema mismatch
            if total_episodes == 0:
                logger.info(
                    "Existing dataset at %s has 0 episodes but mismatched camera schema (%s vs %s). Recreating.",
                    self.output_root,
                    existing_cams,
                    self._active_cameras,
                )
                shutil.rmtree(self.output_root)
            else:
                # Route to separate camera-count directory to preserve existing dataset
                suffix = f"_{len(self._active_cameras)}cam" if self._active_cameras else "_nocam"
                cur_name = self.output_root.name
                base_name = re.sub(r"_\d+cam$", "", cur_name)
                routed_name = f"{base_name}{suffix}"
                routed_root = self.output_root.parent / routed_name
                logger.warning(
                    "Schema mismatch at %s: existing dataset has %d cameras (%s) with %d episodes, "
                    "but incoming episode has %d cameras (%s). Auto-routing to %s to prevent feature mismatch.",
                    self.output_root,
                    len(existing_cams),
                    existing_cams,
                    total_episodes,
                    len(self._active_cameras),
                    self._active_cameras,
                    routed_root,
                )
                self.output_root = routed_root
                routed_info = self.output_root / "meta" / "info.json"
                if routed_info.exists():
                    try:
                        routed_meta = json.loads(routed_info.read_text(encoding="utf-8"))
                        routed_cams = [
                            k.replace("observation.images.", "")
                            for k in routed_meta.get("features", {}).keys()
                            if k.startswith("observation.images.")
                        ]
                        if sorted(routed_cams) == sorted(self._active_cameras):
                            return LeRobotDataset(self.output_root.name, root=self.output_root)
                    except Exception:
                        pass
                    shutil.rmtree(self.output_root)

        # LeRobotDatasetMetadata.create invokes root.mkdir(exist_ok=False).
        # If output_root already exists without info.json (e.g. empty or only partial meta),
        # clean it up so create() does not fail with FileExistsError.
        if self.output_root.exists() and not (self.output_root / "meta" / "info.json").exists():
            logger.warning(
                "Cleaning pre-existing incomplete dataset directory %s to avoid LeRobot FileExistsError",
                self.output_root,
            )
            shutil.rmtree(self.output_root)

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

        for camera in self._active_cameras:
            key = f"observation.images.{camera}"
            height, width = first_frame.decode_image(camera).shape[:2]
            features[key] = {
                "dtype": "video" if self.use_videos else "image",
                "shape": (3, height, width),
                "names": ["channels", "height", "width"],
            }

        logger.info(
            "Creating LeRobotDataset v3.0 at %s (cameras=%s, videos=%s)",
            self.output_root,
            self._active_cameras,
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

    def _save_via_subprocess(self, buffer: Any, task: str) -> str:
        """Fallback to export via Python 3.10 Conda environment when torch is missing."""
        import os
        import pickle
        import subprocess
        import tempfile

        conda_py = Path.home() / "win_office/conda/envs/tuinadex-control/bin/python"
        if not conda_py.exists():
            conda_py = Path.home() / "win_office/conda/envs/tuinadex-vla/bin/python"
        if not conda_py.exists():
            raise RuntimeError(
                "PyTorch / LeRobot not available in current Python and no conda environment found at "
                f"{conda_py}."
            )

        with tempfile.NamedTemporaryFile(suffix=".pkl", delete=False) as f:
            tmp_path = Path(f.name)
            pickle.dump(buffer, f)

        repo_root = Path(__file__).resolve().parents[2]
        env = dict(os.environ)
        extra_paths = [
            str(repo_root / "dataset"),
            str(repo_root / "src/tuinadex_core"),
            str(repo_root / "src/tuinadex_teleop"),
            str(repo_root / "src/tuinadex_sim"),
        ]
        current_pp = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = ":".join(extra_paths) + (f":{current_pp}" if current_pp else "")

        try:
            active_cams = (
                getattr(self, "_active_cameras", None)
                or (self._detect_active_cameras(next(buffer.iter_frames())) if len(buffer) > 0 else [])
                or list(self.config.camera_names)
            )
            cmd = [
                str(conda_py),
                "-m",
                "tuinadex_dataset.lerobot_exporter",
                "--pickle-path",
                str(tmp_path),
                "--repo-id",
                str(self.output_root),
                "--task",
                str(task),
                "--fps",
                str(self.fps),
                "--mode",
                str(getattr(self.config, "mode", "arm_hand")),
                "--cameras",
                ",".join(active_cams),
            ]
            res = subprocess.run(cmd, env=env, cwd=str(repo_root), capture_output=True, text=True)
            if res.returncode != 0:
                raise RuntimeError(
                    f"Subprocess LeRobot export failed: {res.stderr or res.stdout}"
                )
            logger.info("Subprocess LeRobot export succeeded: %s", res.stdout.strip())
            return str(self.output_root)
        finally:
            if tmp_path.exists():
                tmp_path.unlink()

    def save_episode(
        self, buffer: Any, task: str = "tuina_massage"
    ) -> str:
        """Write one episode while decoding only one image frame at a time."""
        try:
            import torch
        except (ImportError, ModuleNotFoundError):
            return self._save_via_subprocess(buffer, task)

        if len(buffer) == 0:
            raise ValueError("Cannot export empty EpisodeBuffer.")

        first_frame = next(buffer.iter_frames())
        if self._dataset is None:
            self._dataset = self._init_dataset_if_needed(first_frame)
        elif not hasattr(self, "_active_cameras") or not self._active_cameras:
            self._active_cameras = self._detect_active_cameras(first_frame)

        last_valid_decoded: dict[str, np.ndarray] = {}
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

            for camera in self._active_cameras:
                if (
                    hasattr(frame, "images_jpeg")
                    and camera in frame.images_jpeg
                    and frame.images_jpeg[camera]
                ):
                    image = frame.decode_image(camera)
                    last_valid_decoded[camera] = image
                elif camera in last_valid_decoded:
                    logger.warning(
                        "Camera '%s' missing at frame %d; reusing previous frame",
                        camera,
                        frame.frame_index,
                    )
                    image = last_valid_decoded[camera]
                else:
                    logger.warning(
                        "Camera '%s' missing at frame %d without prior frame; creating blank fallback",
                        camera,
                        frame.frame_index,
                    )
                    image = np.zeros((480, 640, 3), dtype=np.uint8)
                    last_valid_decoded[camera] = image

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


if __name__ == "__main__":
    import argparse
    import pickle
    import sys

    root_p = str(Path(__file__).resolve().parents[2])
    for sub in ("src/tuinadex_core", "src/tuinadex_teleop", "src/tuinadex_sim", "dataset"):
        p = f"{root_p}/{sub}"
        if p not in sys.path:
            sys.path.insert(0, p)

    parser = argparse.ArgumentParser()
    parser.add_argument("--pickle-path", required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--task", default="tuina_massage")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--mode", default="arm_hand")
    parser.add_argument("--cameras", default="overhead,wrist,side")
    cli_args = parser.parse_args()

    from tuinadex_teleop.envs.config import RobotConfig

    with open(cli_args.pickle_path, "rb") as pf:
        loaded_buf = pickle.load(pf)

    cams = [c.strip() for c in cli_args.cameras.split(",") if c.strip()]
    cfg = RobotConfig(mode=cli_args.mode, camera_names=cams, fps=cli_args.fps)
    w = LeRobotDatasetWriter(repo_id=cli_args.repo_id, config=cfg, fps=cli_args.fps)
    w.save_episode(loaded_buf, task=cli_args.task)
    print("SAVED_OK")
