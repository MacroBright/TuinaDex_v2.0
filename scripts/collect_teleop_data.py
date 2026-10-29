#!/usr/bin/env python3
"""collect_teleop_data.py — TuinaDex v2.0 统一多模态数据采集终端入口.

支持:
1. 仿真/真机一行切换 (--env sim|real)
2. 自适应 16D 手部 / 22D 臂手协同 (--mode hand_only|arm_hand)
3. 多摄像头 RGB 图像流同步 (--cameras head,wrist)
4. 直通 LeRobotDataset v3.0 视频分块与 Parquet 导出
5. 自动化 Mock 示教与交互式遥操示教
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
import numpy as np

# 确保源码路径可导入
ROOT = Path(__file__).resolve().parents[1]
for p in ("src/tuinadex_core", "src/tuinadex_teleop", "src/tuinadex_sim", "dataset"):
    full_p = str(ROOT / p)
    if full_p not in sys.path:
        sys.path.insert(0, full_p)

from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs import make_tuina_env
from tuinadex_teleop.collection.collector import TuinaCollector, CollectionState
from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("collect_teleop")


def parse_args():
    parser = argparse.ArgumentParser(description="TuinaDex Multi-Modal Data Collector")
    parser.add_argument("--env", choices=["sim", "real"], default="sim", help="Environment type (sim or real)")
    parser.add_argument("--mode", choices=["hand_only", "arm_hand"], default="hand_only", help="Robot joint mode")
    parser.add_argument("--cameras", default="head", help="Comma-separated camera names (e.g. head,wrist)")
    parser.add_argument("--fps", type=float, default=50.0, help="Sampling & control frequency in Hz")
    parser.add_argument("--repo-id", default="data/tuina_demo", help="Target LeRobot dataset directory")
    parser.add_argument("--task", default="tuina_massage", help="Task prompt description")
    parser.add_argument("--mock-teleop", action="store_true", help="Use synthetic mock teleoperation for testing")
    parser.add_argument("--episodes", type=int, default=1, help="Number of episodes to collect in mock mode")
    parser.add_argument("--frames-per-episode", type=int, default=50, help="Frames per episode in mock mode")
    parser.add_argument("--standalone", action="store_true", help="Run MuJoCo sim in standalone in-process mode")
    return parser.parse_args()


def run_mock_collection(collector: TuinaCollector, config: RobotConfig, episodes: int, frames: int, fps: float):
    """自动化生成测试示教轨迹并保存."""
    logger.info(f"Starting automated mock collection: {episodes} episode(s), {frames} frames/ep at {fps}Hz...")
    dt = 1.0 / fps

    for ep in range(episodes):
        collector.start_recording()
        logger.info(f">>> Recording Episode {ep + 1}/{episodes}...")
        for k in range(frames):
            # 生成平滑屈伸动作 (0.1 ~ 0.5 rad)
            if config.mode == "hand_only":
                action = 0.3 + 0.2 * np.sin(0.1 * k + np.linspace(0, np.pi, 16)).astype(np.float32)
            else:
                arm_act = 0.2 * np.sin(0.05 * k + np.arange(6)).astype(np.float32)
                hand_act = 0.3 + 0.2 * np.sin(0.1 * k + np.linspace(0, np.pi, 16)).astype(np.float32)
                action = np.concatenate([arm_act, hand_act])

            collector.step_once(action)
            # 在独立模式下无需高延迟 sleep，轻微休眠保证稳定
            time.sleep(0.001)

        collector.stop_recording()
        collector.review_decision(save=True)

    logger.info(f"Automated collection completed. Total saved: {collector.total_episodes_saved}.")


def main():
    args = parse_args()
    cam_list = [c.strip() for c in args.cameras.split(",") if c.strip()]
    config = RobotConfig(
        mode=args.mode,
        camera_names=cam_list,
        fps=args.fps,
    )

    logger.info("============================================================")
    logger.info(f" TuinaDex v2.0 Data Collection Session")
    logger.info(f" - Environment: {args.env.upper()} (standalone={args.standalone})")
    logger.info(f" - Mode:        {args.mode} ({config.action_dim}D)")
    logger.info(f" - Cameras:     {config.camera_names}")
    logger.info(f" - Target FPS:  {config.fps} Hz")
    logger.info(f" - Dataset Dir: {args.repo_id}")
    logger.info("============================================================")

    # 1. 实例化统一环境
    env = make_tuina_env(
        env_type=args.env,
        config=config,
        standalone=args.standalone,
    )

    # 2. 实例化 LeRobot v3.0 导出器
    writer = LeRobotDatasetWriter(
        repo_id=args.repo_id,
        config=config,
        fps=args.fps,
        use_videos=True,
    )

    # 3. 实例化采集编排器
    collector = TuinaCollector(
        env=env,
        on_save=lambda buf: writer.save_episode(buf, task=args.task),
    )

    try:
        if args.mock_teleop:
            run_mock_collection(
                collector=collector,
                config=config,
                episodes=args.episodes,
                frames=args.frames_per_episode,
                fps=args.fps,
            )
        else:
            logger.info("Interactive teleop mode. (Use --mock-teleop for automated self-test)")
            # 交互模式骨架
            collector.start_recording()
            try:
                while True:
                    action = np.zeros(config.action_dim, dtype=np.float32)
                    collector.step_once(action)
                    time.sleep(1.0 / config.fps)
            except KeyboardInterrupt:
                logger.info("Interrupted by user.")
                collector.stop_recording()
                collector.review_decision(save=True)
    finally:
        env.close()
        logger.info("Collection session closed.")


if __name__ == "__main__":
    main()
