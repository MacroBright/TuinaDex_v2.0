"""conftest.py — pytest configuration and fixtures."""
import os
import sys
from pathlib import Path

# If running under Python != 3.14 (e.g. Conda Python 3.10), clean incompatible ROS2 3.14 paths from sys.path
if sys.version_info < (3, 14):
    sys.path = [p for p in sys.path if not ("/opt/ros/lyrical/lib/python3.14" in p)]

ROOT = Path(__file__).resolve().parents[1]
for p in ("tuinadex_core", "tuinadex_hand", "tuinadex_sim", "tuinadex_teleop", "tuinadex_arm", "tuinadex_recorder"):
    pkg_path = str(ROOT / "src" / p)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)

dataset_path = str(ROOT / "dataset")
if dataset_path not in sys.path:
    sys.path.insert(0, dataset_path)
