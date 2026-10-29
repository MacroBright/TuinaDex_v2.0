#!/usr/bin/env python3
"""
Tutorial 3: Monitor target, sent command, and MOS temperature while running.
教程 3：运行中监控目标、发送角度与 MOS 温度。

Run / 运行：
    python3 examples/monitor_status.py

Expected motion / 预期动作：
    1. J1 以约 20°/s 向 +30° 运动；其余关节与夹爪 J7 目标为 0° 并保持不动。
       J1 moves toward +30° at ~20°/s; other joints and gripper J7 target 0°.
    2. 终端每 0.1 s 打印一行：目标角度、发送角度、各电机 MOS 温度。
       The terminal prints target, sent, and MOS temps every 0.1 s.
    3. 发送角度应逐渐逼近目标（例如 J1 从当前值慢慢接近 30），不会一步到位。
       Sent angles should ramp toward targets (J1 gradually approaches 30°).
    4. 温度约为室温到运行后的稳定值；正常时各电机应有数值（非 None）。
       Temperatures show room-to-steady values; normally not None per motor.
    5. 程序不会自动结束；按 Esc 或 Ctrl+C 后缓慢回零并失能。
       The program does not exit on its own; press Esc or Ctrl+C to
       return to zero and disable.
"""

from __future__ import annotations

import time

from _bootstrap import setup_project_path

setup_project_path()

from rebot import ReBotRSMITController


def print_expected_motion() -> None:
    print(
        "[Expected / 预期] J1 → +30° at 20°/s; terminal shows target / sent / "
        "temps each line; sent lags target; press Esc or Ctrl+C to exit / "
        "J1 向 +30° 运动；终端持续打印目标/发送/温度；"
        "发送角度滞后于目标；按 Esc 或 Ctrl+C 结束并回零"
    )


def main() -> None:
    print_expected_motion()

    arm = ReBotRSMITController()

    try:
        arm.start(enable_esc=True, install_signal_handlers=True)
        arm.set_max_speeds([20.0] * arm.motor_count)
        arm.set_joint_angles([30.0] + [0.0] * (arm.motor_count - 1))

        while not arm.is_stopped:
            target = arm.get_target_angles()
            sent = arm.get_command_angles()
            temps = arm.last_temperatures

            print(
                f"目标: {target}  发送: {sent}  温度: {temps}"
            )
            time.sleep(0.1)

    finally:
        if not arm.is_stopped:
            arm.stop(return_to_zero=True, wait=True)
        else:
            arm.wait_until_stopped()


if __name__ == "__main__":
    main()
