#!/usr/bin/env python3
"""
Tutorial 5: Read actual mechanical joint angles from the motors.
教程 5：读取电机实际机械位置。

Run / 运行：
    python3 examples/read_joint_angles.py

Expected motion / 预期动作：
    1. connect() 切 MIT 并使能（建立通信、读取当前角度作为目标初值）。
       connect() switches to MIT, enables, and reads current angles.
    2. 立即 disable_motors() 失能，便于手动推动机械臂。
       disable_motors() immediately so the arm can be moved by hand.
    3. 终端以约 30 Hz 持续打印实际角度；手动改变姿态时读数应随之变化。
       Actual angles print at ~30 Hz; values change when you move the arm.
    4. 按 Ctrl+C 结束并关闭 CAN（不回零）。
       Press Ctrl+C to exit and close CAN without return-to-zero.
"""

from __future__ import annotations

import time

from _bootstrap import setup_project_path

setup_project_path()

from rebot import ReBotRSMITController

READ_HZ = 30.0
READ_INTERVAL_S = 1.0 / READ_HZ


def print_expected_motion() -> None:
    print(
        "[Expected / 预期] connect → enable → disable; "
        "terminal keeps printing actual angles while you move the arm; "
        "Ctrl+C to exit / "
        "连接并使能后立刻失能；持续打印实际角度；手动推动观察读数；"
        "按 Ctrl+C 结束"
    )


def main() -> None:
    print_expected_motion()

    arm = ReBotRSMITController()

    try:
        arm.connect()
        arm.disable_motors()

        while True:
            loop_start = time.monotonic()
            actual = arm.read_joint_angles()
            print(
                f"实际角度 / actual: {[round(x, 2) for x in actual]} deg / 度"
            )
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, READ_INTERVAL_S - elapsed))

    except KeyboardInterrupt:
        print()
    finally:
        arm.stop(return_to_zero=False, wait=True)


if __name__ == "__main__":
    main()
