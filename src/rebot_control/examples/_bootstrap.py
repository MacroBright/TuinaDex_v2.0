"""Add project root to sys.path and shared helpers for examples."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rebot.controller import ReBotRSMITController

PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Motor / joint count (default 6 for pure arm, see config/rebotarm_rs.yaml).
# 电机/关节数量（默认 6 轴纯臂，见 config/rebotarm_rs.yaml）。
MOTOR_COUNT = 6
GRIPPER_JOINT_ID = 7


def setup_project_path() -> Path:
    """Insert project root into sys.path if missing."""

    root = str(PROJECT_ROOT)

    if root not in sys.path:
        sys.path.insert(0, root)

    return PROJECT_ROOT


def wait_for_command_targets(
    arm: ReBotRSMITController,
    *,
    tolerance_deg: float = 0.5,
    timeout_s: float = 60.0,
    poll_s: float = 0.05,
    show_progress: bool = False,
) -> bool:
    """
    Wait until smoothed command angles reach the current targets.

    set_joint_angles() only updates the target; the control thread ramps
    command_positions at set_max_speeds(). Call this before stop() if the
    example should finish a visible move first.

    等待平滑后的发送角度到达当前目标。
    set_joint_angles() 只改目标，需等控制环限速逼近后再 stop()。
    """

    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline and not arm.is_stopped:
        targets = arm.get_target_angles()
        sent = arm.get_command_angles()

        if all(
            abs(target - command) <= tolerance_deg
            for target, command in zip(targets, sent)
        ):
            if show_progress:
                print(
                    f"\r发送角度 / sent: "
                    f"{[round(x, 2) for x in sent]}",
                    flush=True,
                )
                print()
            return True

        if show_progress:
            print(
                f"\r发送角度 / sent: "
                f"{[round(x, 2) for x in sent]}  "
                f"目标 / target: "
                f"{[round(x, 2) for x in targets]}",
                end="",
                flush=True,
            )

        time.sleep(poll_s)

    if show_progress:
        print()

    return False
