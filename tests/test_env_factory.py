"""test_env_factory.py — 环境工厂函数与 MuJoCoEnv / RealRobotEnv 特化测试."""
import pytest
import numpy as np

from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs import make_tuina_env, BaseTuinaEnv, MuJoCoEnv, RealRobotEnv


def test_make_tuina_env_sim_standalone():
    cfg = RobotConfig(mode="hand_only", camera_names=["head"])
    env = make_tuina_env("sim", cfg, standalone=True)
    assert isinstance(env, MuJoCoEnv)
    assert isinstance(env, BaseTuinaEnv)

    obs = env.reset()
    assert obs["state"].shape == (16,)
    assert "head" in obs["images"]
    assert obs["images"]["head"].shape[2] == 3

    # 执行 step 动作
    act = np.full(16, 0.2, dtype=np.float32)
    next_obs, info = env.step(act)
    assert next_obs["state"].shape == (16,)
    assert "head" in next_obs["images"]
    env.close()


def test_make_tuina_env_invalid_type():
    cfg = RobotConfig(mode="hand_only")
    with pytest.raises(ValueError, match="Unknown env_type"):
        make_tuina_env("unsupported_env", cfg)
