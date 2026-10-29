"""tests/test_rebot_control_regression.py — 机械臂 rebot_control 核心功能回归单测."""
import sys
from pathlib import Path
import pytest

# 确保 src/rebot_control 处于 sys.path 中
REBOT_CONTROL_ROOT = Path(__file__).resolve().parent.parent / "src" / "rebot_control"
if str(REBOT_CONTROL_ROOT) not in sys.path:
    sys.path.insert(0, str(REBOT_CONTROL_ROOT))

from rebot import ReBotRSMITController, load_config
from rebot.config import ControllerConfig, TemperatureThresholds


def test_rebot_config_loading():
    config_path = REBOT_CONTROL_ROOT / "config" / "rebotarm_rs.yaml"
    cfg = load_config(config_path)
    assert cfg.channel == "can0"
    assert cfg.control_hz == 200.0
    assert cfg.temperatures.alarm_c == 110.0
    assert cfg.temperatures.return_zero_c == 125.0
    assert cfg.temperatures.disconnect_c == 140.0


def test_rebot_smoothstep_math():
    assert ReBotRSMITController._smoothstep(0.0) == 0.0
    assert ReBotRSMITController._smoothstep(1.0) == 1.0
    assert ReBotRSMITController._smoothstep(0.5) == 0.5
    # 单调性
    val1 = ReBotRSMITController._smoothstep(0.2)
    val2 = ReBotRSMITController._smoothstep(0.8)
    assert 0.0 < val1 < val2 < 1.0



def test_rebot_controller_api_initialization():
    config_path = REBOT_CONTROL_ROOT / "config" / "rebotarm_rs.yaml"
    cfg = load_config(config_path)
    arm = ReBotRSMITController(cfg)
    assert arm.started is False
    assert arm.motor_count == 6
    assert arm.get_target_angles() == [0.0] * 6
    assert arm.get_command_angles() == [0.0] * 6

