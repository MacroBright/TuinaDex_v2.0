"""test_robot_env_spec.py — RobotConfig 与 BaseTuinaEnv 规范与自适应维度测试."""
import pytest
import numpy as np

from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs.base_env import BaseTuinaEnv


def test_robot_config_adaptive_dimensions():
    # 模式 1: hand_only (16D)
    cfg_hand = RobotConfig(mode="hand_only")
    assert cfg_hand.action_dim == 16
    assert cfg_hand.state_dim == 16
    assert len(cfg_hand.joint_names) == 16
    assert cfg_hand.joint_names[0] == "index_yaw"

    # 模式 2: arm_hand (22D = 6 臂 + 16 手)
    cfg_arm_hand = RobotConfig(mode="arm_hand")
    assert cfg_arm_hand.action_dim == 22
    assert cfg_arm_hand.state_dim == 22
    assert len(cfg_arm_hand.joint_names) == 22
    assert list(cfg_arm_hand.joint_names[:6]) == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert cfg_arm_hand.joint_names[6:] == cfg_hand.joint_names


def test_robot_config_cameras_validation():
    cfg = RobotConfig(mode="hand_only", camera_names=["head", "wrist"])
    assert cfg.camera_names == ("head", "wrist")

    with pytest.raises(ValueError, match="Invalid mode"):
        RobotConfig(mode="invalid_mode")


def test_base_tuina_env_contract():
    class DummyEnv(BaseTuinaEnv):
        def reset(self):
            return self.get_observation()

        def step(self, action):
            action = self.validate_action(action)
            return self.get_observation(), {"step": 1}

        def get_observation(self):
            return {
                "state": np.zeros(self.config.action_dim, dtype=np.float32),
                "images": {"head": np.zeros((480, 640, 3), dtype=np.uint8)},
                "timestamp": 1000.0,
            }

        def close(self):
            pass

    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    env = DummyEnv(cfg)
    obs = env.reset()
    assert obs["state"].shape == (16,)
    assert "head" in obs["images"]
    assert obs["images"]["head"].shape == (480, 640, 3)

    # 正常 step
    obs2, info = env.step(np.zeros(16, dtype=np.float32))
    assert obs2["state"].shape == (16,)
    assert info["step"] == 1

    # 异常动作维度拒绝
    with pytest.raises(ValueError, match="Expected action shape"):
        env.step(np.zeros(10, dtype=np.float32))

    env.close()
