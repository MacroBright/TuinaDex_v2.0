"""test_teleop_logic.py — 遥操纯逻辑: 利手过滤与重捕获判定."""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_teleop"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_teleop.tracking.teleop_logic import handedness_ok, regained_after_gap


def test_handedness_any_accepts_both():
    assert handedness_ok("Left", "any") is True
    assert handedness_ok("Right", "any") is True


def test_handedness_strict():
    assert handedness_ok("Right", "Right") is True
    assert handedness_ok("Left", "Right") is False
    assert handedness_ok("Right", "any") is True


def test_regained_after_gap():
    assert regained_after_gap(prev_seen=False, now_seen=True) is True
    assert regained_after_gap(prev_seen=True, now_seen=True) is False
    assert regained_after_gap(prev_seen=True, now_seen=False) is False
    assert regained_after_gap(prev_seen=False, now_seen=False) is False
