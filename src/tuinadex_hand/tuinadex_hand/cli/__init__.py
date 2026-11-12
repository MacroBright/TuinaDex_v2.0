"""tuinadex_hand.cli — LEAP Hand 视觉遥操、交互控制、零点标定与限位体检诊断命令行工具集."""

from .control import main as control_main
from .calibrate import main as calibrate_main
from .diagnostics import main as diagnostics_main
from .teleop import main as teleop_main

__all__ = ["control_main", "calibrate_main", "diagnostics_main", "teleop_main"]
