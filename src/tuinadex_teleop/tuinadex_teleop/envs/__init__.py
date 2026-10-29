"""tuinadex_teleop.envs — 统一机器人环境抽象与工厂."""
from .config import RobotConfig
from .base_env import BaseTuinaEnv
from .ros2_robot_env import ROS2RobotEnv
from .mujoco_env import MuJoCoEnv
from .real_env import RealRobotEnv


def make_tuina_env(env_type: str, config: RobotConfig, **kwargs) -> BaseTuinaEnv:
    """环境工厂方法.
    
    Args:
        env_type: "sim" (MuJoCo 仿真) 或 "real" (真机运行时).
        config: RobotConfig 配置实例.
        **kwargs: 传给对应环境构造函数的附加参数 (如 standalone, scene_xml 等).
        
    Returns:
        BaseTuinaEnv 子类实例.
    """
    env_type_lower = env_type.lower()
    if env_type_lower in ("sim", "simulation", "mujoco"):
        return MuJoCoEnv(config, **kwargs)
    elif env_type_lower in ("real", "robot", "hardware"):
        return RealRobotEnv(config, **kwargs)
    else:
        raise ValueError(
            f"Unknown env_type '{env_type}'. Expected one of ('sim', 'real')."
        )


__all__ = [
    "RobotConfig",
    "BaseTuinaEnv",
    "ROS2RobotEnv",
    "MuJoCoEnv",
    "RealRobotEnv",
    "make_tuina_env",
]
