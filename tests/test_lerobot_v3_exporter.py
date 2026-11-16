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


def test_dynamic_camera_detection_two_cameras_with_missing_wrist(tmp_path):
    """When config has 3 cameras but buffer only has 2 (missing wrist), writer must auto-detect and save 2 cameras."""
    from tuinadex_teleop.envs.config import RobotConfig
    from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer
    from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    cfg = RobotConfig(
        mode="arm_hand", camera_names=["overhead", "wrist", "side"]
    )
    ds_root = tmp_path / "ds_2cam"
    writer = LeRobotDatasetWriter(
        repo_id=str(ds_root), config=cfg, fps=50.0
    )

    buffer = EpisodeBuffer(episode_index=0, language_instruction="双相机推拿采集")
    for i in range(10):
        img_overhead = np.full((96, 128, 3), 50 + i, dtype=np.uint8)
        img_side = np.full((96, 128, 3), 100 + i, dtype=np.uint8)
        buffer.add_frame(
            state=np.full(22, i * 0.01, np.float32),
            action=np.full(22, i * 0.02, np.float32),
            effort=np.full(22, i * 0.1, np.float32),
            contact_forces=np.array([1.0, 2.0, 3.0, 4.0], np.float32),
            images={
                "overhead": img_overhead,
                "side": img_side,
                # 'wrist' is intentionally missing
            },
            timestamp=i * 0.02,
            phase_label="press",
        )
    buffer.seal_episode()

    saved_path = writer.save_episode(buffer, task="tuina_massage")
    assert Path(saved_path).resolve() == ds_root.resolve()

    info = json.loads((ds_root / "meta" / "info.json").read_text())
    features = info["features"]

    # Active cameras must be present
    assert "observation.images.overhead" in features
    assert "observation.images.side" in features
    # Absent wrist camera must NOT be present in features
    assert "observation.images.wrist" not in features

    # Verify videos exist for overhead and side, but not wrist
    assert list((ds_root / "videos/observation.images.overhead").rglob("*.mp4"))
    assert list((ds_root / "videos/observation.images.side").rglob("*.mp4"))
    assert not (ds_root / "videos/observation.images.wrist").exists()

    # Load with LeRobot native reader
    dataset = LeRobotDataset("ds_2cam", root=ds_root)
    assert len(dataset) == 10
    frame0 = dataset[0]
    assert "observation.images.overhead" in frame0
    assert "observation.images.side" in frame0
    assert "observation.images.wrist" not in frame0


def test_schema_mismatch_auto_routing_protects_existing_dataset(tmp_path):
    """When target dataset has 3 cameras and 1 episode, saving a 2-camera episode must auto-route to _2cam."""
    from tuinadex_teleop.envs.config import RobotConfig
    from tuinadex_teleop.collection.episode_buffer import EpisodeBuffer
    from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    cfg3 = RobotConfig(
        mode="arm_hand", camera_names=["overhead", "wrist", "side"]
    )
    orig_root = tmp_path / "my_demo"

    # 1. Create original 3-camera dataset with 1 episode
    writer3 = LeRobotDatasetWriter(repo_id=str(orig_root), config=cfg3, fps=50.0)
    buf3 = EpisodeBuffer(episode_index=0)
    for i in range(5):
        img = np.full((96, 128, 3), 80, dtype=np.uint8)
        buf3.add_frame(
            state=np.full(22, 0.1, np.float32),
            action=np.full(22, 0.1, np.float32),
            effort=np.full(22, 0.1, np.float32),
            contact_forces=np.array([1.0, 1.0, 1.0, 1.0], np.float32),
            images={"overhead": img, "wrist": img, "side": img},
            timestamp=i * 0.02,
        )
    buf3.seal_episode()
    writer3.save_episode(buf3)

    info3 = json.loads((orig_root / "meta" / "info.json").read_text())
    assert info3["total_episodes"] == 1
    assert "observation.images.wrist" in info3["features"]

    # 2. Now attempt to write a 2-camera episode targeting the SAME repo_id
    writer2 = LeRobotDatasetWriter(repo_id=str(orig_root), config=cfg3, fps=50.0)
    buf2 = EpisodeBuffer(episode_index=0)
    for i in range(5):
        img = np.full((96, 128, 3), 90, dtype=np.uint8)
        buf2.add_frame(
            state=np.full(22, 0.2, np.float32),
            action=np.full(22, 0.2, np.float32),
            effort=np.full(22, 0.2, np.float32),
            contact_forces=np.array([1.0, 1.0, 1.0, 1.0], np.float32),
            images={"overhead": img, "side": img},  # no wrist
            timestamp=i * 0.02,
        )
    buf2.seal_episode()

    # Must NOT crash with Feature mismatch; must auto-route to my_demo_2cam
    saved_path = Path(writer2.save_episode(buf2))
    assert saved_path == tmp_path / "my_demo_2cam"

    # Original 3-camera dataset is intact!
    orig_ds = LeRobotDataset("my_demo", root=orig_root)
    assert len(orig_ds) == 5
    assert "observation.images.wrist" in orig_ds[0]

    # New 2-camera dataset is created and healthy!
    routed_ds = LeRobotDataset("my_demo_2cam", root=saved_path)
    assert len(routed_ds) == 5
    assert "observation.images.wrist" not in routed_ds[0]
    assert "observation.images.overhead" in routed_ds[0]
    assert "observation.images.side" in routed_ds[0]

