#!/usr/bin/env python3
"""
Tutorial 4: Adjust joints one by one with set_joint_angle().
教程 4：用 set_joint_angle() 逐个设置关节目标。

Run / 运行：
    python3 examples/single_joint_adjust.py

Expected motion / 预期动作：
    1. 目标：J1=+25°、J2=+15°、J3=+15°、J4=-15°、J5/J6=0°；
       夹爪 J7（CAN ID 7）= 180°。
       Targets: J1=+25°, J2=+15°, J3=+15°, J4=-15°, J5/J6=0°;
       gripper J7 (CAN ID 7) = 180°.
    2. 先 set_joint_angles 全零，再对各关节 set_joint_angle，最后设夹爪。
       set_joint_angles to zero, then per-joint set_joint_angle, then gripper.
    3. 多关节同时以约 15°/s 向目标运动；终端发送角度逐渐逼近目标列表。
       Multiple joints move together at ~15°/s; sent angles ramp toward targets.
    4. 按 Esc/Ctrl+C 或 finally 中 stop()：全部关节与夹爪缓慢回零后失能。
       Esc/Ctrl+C or finally stop(): arm and gripper return to zero, then disable.
"""

from __future__ import annotations

import time

from _bootstrap import (
    GRIPPER_JOINT_ID,
    MOTOR_COUNT,
    setup_project_path,
    wait_for_command_targets,
)

setup_project_path()

from rebot import ReBotRSMITController

JOINT_SPEEDS_DEG_S = [15.0] * MOTOR_COUNT

ARM_JOINT_TARGETS = {
    1: 25.0,   # J1
    2: 15.0,   # J2
    3: 15.0,   # J3
    4: -15.0,  # J4
    5: 0.0,    # J5
    6: 0.0,    # J6
}

GRIPPER_ANGLE_DEG = 180.0


def apply_joint_targets(arm: ReBotRSMITController) -> None:
    """Set arm joints, and gripper if equipped."""

    arm.set_joint_angles([0.0] * arm.motor_count)

    for joint_id, angle_deg in ARM_JOINT_TARGETS.items():
        if joint_id <= arm.motor_count:
            arm.set_joint_angle(joint_id, angle_deg)

    if arm.motor_count >= GRIPPER_JOINT_ID:
        arm.set_joint_angle(GRIPPER_JOINT_ID, GRIPPER_ANGLE_DEG)


def print_expected_motion() -> None:
    print(
        "[Expected / 预期] "
        "J1→+25°, J2→+15°, J3→+15°, J4→-15°, J5/J6→0° at 15°/s; "
        "then return-to-zero / "
        "J1→+25°、J2→+15°、J3→+15°、J4→-15°、J5/J6→0°（约 15°/s）；随后回零"
    )


def main() -> None:
    print_expected_motion()

    arm = ReBotRSMITController()

    try:
        arm.start(enable_esc=True, install_signal_handlers=True)
        arm.set_max_speeds([15.0] * arm.motor_count)

        apply_joint_targets(arm)

        print(
            "[Target / 目标] "
            f"{arm.get_target_angles()} deg / "
            f"{arm.get_target_angles()} 度"
        )

        wait_for_command_targets(arm, show_progress=True)

        while not arm.is_stopped:
            sent = arm.get_command_angles()
            print(
                f"\r发送角度 / sent: "
                f"{[round(x, 2) for x in sent]}",
                end="",
                flush=True,
            )
            time.sleep(0.1)

    finally:
        if not arm.is_stopped:
            arm.stop(return_to_zero=True, wait=True)
        else:
            arm.wait_until_stopped()


if __name__ == "__main__":
    main()
