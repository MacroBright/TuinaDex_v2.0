"""test_collector_fsm.py — TuinaCollector 试次采集状态机单元测试."""
import pytest
import numpy as np

from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs.base_env import BaseTuinaEnv
from tuinadex_teleop.collection.collector import TuinaCollector, CollectionState


class MockEnv(BaseTuinaEnv):
    def __init__(self, config):
        super().__init__(config)
        self.step_count = 0

    def reset(self):
        self.step_count = 0
        return self.get_observation()

    def step(self, action):
        self.step_count += 1
        return self.get_observation(), {"count": self.step_count}

    def get_observation(self):
        return {
            "state": np.zeros(self.config.action_dim, dtype=np.float32),
            "images": {"head": np.zeros((60, 80, 3), dtype=np.uint8)},
            "timestamp": float(self.step_count) * 0.02,
        }

    def close(self):
        pass


def test_collector_fsm_transitions():
    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    env = MockEnv(cfg)
    collector = TuinaCollector(env)

    # 1. 初始状态为 IDLE
    assert collector.state == CollectionState.IDLE
    assert len(collector.buffer) == 0

    # IDLE 下 step 不记录数据
    collector.step_once(np.zeros(16, dtype=np.float32))
    assert len(collector.buffer) == 0

    # 2. 启动录制 -> RECORDING
    collector.start_recording()
    assert collector.state == CollectionState.RECORDING

    for _ in range(5):
        collector.step_once(np.full(16, 0.1, dtype=np.float32))
    assert len(collector.buffer) == 5

    # 3. 暂停录制 -> PAUSED
    collector.pause_recording()
    assert collector.state == CollectionState.PAUSED
    collector.step_once(np.full(16, 0.1, dtype=np.float32))
    assert len(collector.buffer) == 5  # 暂停期间不入缓存

    # 4. 恢复录制 -> RECORDING
    collector.start_recording()
    collector.step_once(np.full(16, 0.1, dtype=np.float32))
    assert len(collector.buffer) == 6

    # 5. 停止录制 -> 进入 REVIEWING 质检状态
    collector.stop_recording()
    assert collector.state == CollectionState.REVIEWING

    # 6. 评审通过 (保存)
    saved_episodes = []
    collector.on_save = lambda buf: saved_episodes.append(len(buf))
    collector.review_decision(save=True)
    assert collector.state == CollectionState.IDLE
    assert len(saved_episodes) == 1
    assert saved_episodes[0] == 6
    assert len(collector.buffer) == 0


def test_collector_fsm_discard():
    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    env = MockEnv(cfg)
    collector = TuinaCollector(env)

    collector.start_recording()
    for _ in range(3):
        collector.step_once(np.zeros(16, dtype=np.float32))
    assert len(collector.buffer) == 3

    collector.stop_recording()
    assert collector.state == CollectionState.REVIEWING

    # 示教失误 -> 丢弃
    collector.review_decision(save=False)
    assert collector.state == CollectionState.IDLE
    assert len(collector.buffer) == 0
