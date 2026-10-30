"""test_lerobot_v3_exporter.py — LeRobotDataset v3.0 多模态导出器单元测试 (需 lerobot)."""
import os

import pytest
import numpy as np
from pathlib import Path
import json

os.environ.setdefault("HF_HUB_OFFLINE", "1")

pytest.importorskip("lerobot")


def test_lerobot_v3_multi_modal_export(tmp_path):
    from tuinadex_teleop.envs.config import RobotConfig
    from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer
    from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    buf = EpisodeBuffer()

    # 构造 15 帧合成多模态数据 (含 120x160 RGB 图像)
    for i in range(15):
        img = np.full((120, 160, 3), i * 15, dtype=np.uint8)
        buf.add_frame(
            state=np.full(16, i * 0.05, dtype=np.float32),
            action=np.full(16, i * 0.05, dtype=np.float32),
            images={"head": img},
            timestamp=float(i) * 0.02,
        )

    output_dir = tmp_path / "test_v3_dataset"
    writer = LeRobotDatasetWriter(repo_id=str(output_dir), config=cfg, fps=50.0, use_videos=True)
    res_path = writer.save_episode(buf, task="tuina_demo")

    # 1. 验证元数据 info.json
    info_path = Path(res_path) / "meta" / "info.json"
    assert info_path.exists(), f"Missing {info_path}"
    with open(info_path) as f:
        meta = json.load(f)

    assert meta["codebase_version"] == "v3.0"
    assert meta["total_frames"] == 15
    assert meta["total_episodes"] == 1
    assert "observation.images.head" in meta["features"]
    assert meta["features"]["observation.images.head"]["dtype"] == "video"

    # 2. 验证 Parquet 数据文件存在
    data_files = list((Path(res_path) / "data").rglob("*.parquet"))
    assert len(data_files) > 0

    # 3. 验证视频 MP4 分块文件存在
    video_files = list((Path(res_path) / "videos").rglob("*.mp4"))
    assert len(video_files) > 0

    # 4. 验证 LeRobot 原生读取
    ds = LeRobotDataset(Path(res_path).name, root=res_path)
    assert len(ds) == 15
    first_frame = ds[0]
    assert first_frame["observation.state"].shape == (16,)
    assert first_frame["action"].shape == (16,)
    assert "observation.images.head" in first_frame


def test_multi_modal_v3_export_with_three_cameras(tmp_path):
    from tuinadex_teleop.envs.config import RobotConfig
    from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer
    from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    cfg = RobotConfig(
        mode="arm_hand", camera_names=["overhead", "wrist", "side"]
    )
    writer = LeRobotDatasetWriter(
        repo_id=str(tmp_path / "ds3cam"), config=cfg, fps=50.0
    )
    for episode_index in range(2):
        buffer = EpisodeBuffer(
            episode_index=episode_index,
            language_instruction="按压肺俞穴",
        )
        for i in range(12):
            image = np.full((96, 128, 3), 40 + i, dtype=np.uint8)
            buffer.add_frame(
                state=np.full(22, i * 0.01, np.float32),
                action=np.full(22, i * 0.02, np.float32),
                effort=np.full(22, i * 0.1, np.float32),
                contact_forces=np.array(
                    [1.0, 2.0, 3.0, 4.0], np.float32
                ),
                images={
                    "overhead": image,
                    "wrist": image,
                    "side": image,
                },
                timestamp=i * 0.02,
                phase_label="press" if i < 6 else "release",
            )
        buffer.seal_episode()
        assert buffer.frames[-1].next_done is True
        writer.save_episode(buffer, task="tuina_massage")

    info = json.loads(
        (tmp_path / "ds3cam/meta/info.json").read_text()
    )
    assert info["codebase_version"] == "v3.0"
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 24
    features = info["features"]
    for key in (
        "observation.state",
        "observation.effort",
        "observation.contact_forces",
        "action",
        "phase_index",
        "observation.images.overhead",
        "observation.images.wrist",
        "observation.images.side",
    ):
        assert key in features, key
    assert features["observation.images.wrist"]["dtype"] == "video"

    for camera in ("overhead", "wrist", "side"):
        assert list(
            (
                tmp_path
                / f"ds3cam/videos/observation.images.{camera}"
            ).rglob("*.mp4")
        )

    phase_map = json.loads(
        (tmp_path / "ds3cam/meta/phase_labels.json").read_text()
    )
    assert sorted(phase_map.values()) == ["press", "release"]

    dataset = LeRobotDataset("ds3cam", root=tmp_path / "ds3cam")
    assert len(dataset) == 24
    frame = dataset[0]
    assert frame["observation.state"].shape == (22,)
    assert frame["observation.effort"].shape == (22,)
    assert frame["observation.contact_forces"].shape == (4,)
    assert "observation.images.side" in frame
