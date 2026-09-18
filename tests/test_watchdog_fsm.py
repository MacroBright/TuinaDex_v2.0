"""test_watchdog_fsm.py — 看门狗安全状态机全状态转移验证."""
import sys
from pathlib import Path
import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_hand"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_hand.watchdog import (
    WatchdogStateMachine, WatchdogState, WatchdogAction,
)


def test_never_commanded_is_none():
    """从未收到指令: 不动作 (手保持全开), 也不触发 relax."""
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    assert fsm.state is WatchdogState.FOLLOWING
    assert fsm.tick(t_now=100.0) is WatchdogAction.NONE


def test_following_within_timeout_writes():
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    fsm.on_command(t_now=1.0)
    assert fsm.tick(t_now=1.0) is WatchdogAction.WRITE
    assert fsm.tick(t_now=1.30) is WatchdogAction.WRITE  # 边界 <=


def test_timeout_transitions_to_relaxing():
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    fsm.on_command(t_now=1.0)
    assert fsm.tick(t_now=1.31) is WatchdogAction.RELAX_STEP
    assert fsm.state is WatchdogState.RELAXING
    # 持续 RELAX_STEP 直到新指令
    assert fsm.tick(t_now=2.0) is WatchdogAction.RELAX_STEP


def test_new_command_rearms_following():
    fsm = WatchdogStateMachine(timeout_sec=0.30)
    fsm.on_command(t_now=1.0)
    fsm.tick(t_now=1.31)  # 进入 RELAXING
    fsm.on_command(t_now=2.0)
    assert fsm.state is WatchdogState.FOLLOWING
    assert fsm.tick(t_now=2.0) is WatchdogAction.WRITE


def test_invalid_timeout_rejected():
    with pytest.raises(ValueError):
        WatchdogStateMachine(timeout_sec=0.0)
