"""conftest.py — pytest configuration and fixtures."""
import os
import sys
from pathlib import Path

# Isolate ROS 2 domain during testing to prevent crosstalk with running daemon/GUI nodes
os.environ.setdefault("ROS_DOMAIN_ID", "42")

ROOT = Path(__file__).resolve().parents[1]
for p in ("tuinadex_core", "tuinadex_hand", "tuinadex_sim", "tuinadex_teleop", "tuinadex_arm", "tuinadex_recorder"):
    pkg_path = str(ROOT / "src" / p)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)

dataset_path = str(ROOT / "dataset")
if dataset_path not in sys.path:
    sys.path.insert(0, dataset_path)
