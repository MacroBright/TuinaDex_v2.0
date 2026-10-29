#!/usr/bin/env python3
"""
Tutorial 1: Quick start (minimal example).
教程 1：快速开始（最小示例）。

Run / 运行：
    python3 examples/quick_start.py

Expected motion / 预期动作：
    1. 使能后机械臂保持当前姿态，无突然跳动。
       After enable, the arm holds its current pose with no sudden jump.
    2. J1（基座）以约 15°/s 缓慢转向 +20°；J2–J6 与夹爪（J7）保持 0° 不动。
       J1 (base) slowly rotates to +20° at ~15°/s; J2–J6 and gripper (J7) stay at 0°.
    3. 终端会刷新显示发送角度逐渐逼近目标，约 1–2 s 后 J1 接近 20°。
       The terminal shows sent angles ramping toward targets; J1 nears 20°
       in about 1–2 s (from 0°).
    4. 到达目标后再 stop()：各关节沿平滑轨迹缓慢回到 0°，然后失能。
       After the target is reached, stop() slowly returns all joints to 0°,
       then disables the motors.

Note / 注意：
    set_joint_angles() 只设置目标，不会立刻到位；本示例会等待发送角度
    接近目标后再 stop()，否则几乎看不到向 +20° 的运动。
"""

from __future__ import annotations

from _bootstrap import setup_project_path, wait_for_command_targets

setup_project_path()

from rebot import ReBotRSMITController

TARGET_ANGLES = [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
JOINT_SPEEDS_DEG_S = [15.0] * 6


def print_expected_motion() -> None:
    print(
        "[Expected / 预期] J1 → +20° at 15°/s, others stay at 0°; "
        "wait until sent reaches target, then return-to-zero / "
        "J1 转到 +20°（约 15°/s），其余关节保持 0°；"
        "等待发送角度到位后再回零"
    )


def main() -> None:
    print_expected_motion()

    arm = ReBotRSMITController()

    arm.start(enable_esc=True)
    arm.set_max_speeds([15.0] * arm.motor_count)

    start_j1 = arm.get_command_angles()[0]
    print(
        f"J1 起始角度 / J1 start: {start_j1:.2f}°"
    )

    targets = [20.0] + [0.0] * (arm.motor_count - 1)
    arm.set_joint_angles(targets)

    reached = wait_for_command_targets(
        arm,
        show_progress=True,
    )

    if not reached:
        print(
            "[Warn / 警告] Timed out before target reached / "
            "未在超时前到达目标"
        )

    arm.stop()


if __name__ == "__main__":
    main()
