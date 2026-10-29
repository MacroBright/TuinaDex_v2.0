"""tuinadex_core — 纯逻辑核心层，定义系统契约不变量与安全看门狗."""
from .names import CANONICAL_JOINT_NAMES, CANONICAL_ARM_JOINT_NAMES
from .command_processing import (
    JOINT_SPEC_LIMITS,
    ARM_SPEC_LIMITS,
    align_command,
    align_arm_command,
)
from .watchdog import WatchdogStateMachine, WatchdogState

__all__ = [
    "CANONICAL_JOINT_NAMES",
    "CANONICAL_ARM_JOINT_NAMES",
    "JOINT_SPEC_LIMITS",
    "ARM_SPEC_LIMITS",
    "align_command",
    "align_arm_command",
    "WatchdogStateMachine",
    "WatchdogState",
]
