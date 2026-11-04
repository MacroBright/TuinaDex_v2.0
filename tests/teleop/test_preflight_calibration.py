"""Unit test for 00_preflight_calibration.py all-in-one wizard."""
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "scripts" / "00_preflight_calibration.py"


def test_preflight_calibration_mock_dry_run():
    """Verify that 00_preflight_calibration runs cleanly in mock/headless mode."""
    cmd = [
        sys.executable,
        str(SCRIPT_PATH),
        "--mock",
        "--no-gui",
        "--skip-table",
        "--skip-imu",
        "--skip-hand",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=35)
    assert res.returncode == 0, f"STDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}"
    assert "[SUCCESS] 统一会话标定文件已写入" in res.stdout

