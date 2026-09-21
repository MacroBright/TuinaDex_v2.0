"""test_tuinadex_core.py — shared pure-logic package contract."""
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src" / "tuinadex_core") not in sys.path:
    sys.path.insert(0, str(ROOT / "src" / "tuinadex_core"))

from tuinadex_core import command_processing
from tuinadex_core import names
from tuinadex_core import watchdog


def test_core_exposes_shared_pure_logic():
    assert names.CANONICAL_JOINT_NAMES[0] == "index_yaw"
    assert names.CANONICAL_JOINT_NAMES[-1] == "thumb_ip"
    assert command_processing.JOINT_SPEC_LIMITS.shape == (16, 2)
    assert watchdog.WatchdogStateMachine(timeout_sec=0.3)


def test_command_processing_accepts_in_limit_target():
    q = [0.0] * 16
    q[1] = 0.5
    out, reason = command_processing.align_command(q)
    assert out is not None and reason == ""
    np.testing.assert_allclose(out[1], 0.5)
