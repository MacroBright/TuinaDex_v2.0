"""test_lerobot_v3_exporter.py — LeRobotDataset v3.0 多模态导出器单元测试 (需 lerobot)."""
import pytest
import numpy as np
from pathlib import Path
import json

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
