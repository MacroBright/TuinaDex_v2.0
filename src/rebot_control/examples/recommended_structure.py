#!/usr/bin/env python3
"""
Tutorial 7: Recommended try/finally program structure.
教程 7：推荐的 try/finally 程序结构。

Run / 运行：
    python3 examples/recommended_structure.py

Expected motion / 预期动作：
    1. 与典型业务程序相同：start → 设速度 → 设目标 → 循环监控。
       Same as a typical app: start → set speeds → set targets → monitor loop.
    2. J1 以约 20°/s 转到 +25°；其余关节保持 0°。
       J1 moves to +25° at ~20°/s; other arm joints and gripper J7 stay at 0°.
    3. 终端一行刷新显示目标与发送角度；发送逐渐逼近目标。
       One line refreshes target vs sent; sent ramps toward target.
    4. 按 Esc/Ctrl+C 触发回零；若代码抛异常，finally 仍会执行 stop() 回零。
       Esc/Ctrl+C triggers return-to-zero; on exception, finally still stop().
    5. 本示例侧重程序结构；实际角度请在约 70% 工作空间内设置。
       Focus is program structure; keep real targets within ~70% workspace.
"""

from __future__ import annotations

import time

from _bootstrap import setup_project_path

setup_project_path()

from rebot import ReBotRSMITController

# Keep within ~70% of workspace. / 保持在约 70% 工作空间内。
TARGET_ANGLES = [25.0, 0.0, 0.0, 0.0, 0.0, 0.0]
JOINT_SPEEDS_DEG_S = [20.0] * 6


def print_expected_motion() -> None:
    print(
        "[Expected / 预期] J1 → +25° at 20°/s; refresh target/sent; "
        "Esc/Ctrl+C or finally → return-to-zero / "
        "J1 到 +25°（20°/s）；终端刷新目标/发送；"
        "Esc/Ctrl+C 或 finally 中回零"
    )


def main() -> None:
    print_expected_motion()

    arm = ReBotRSMITController()

    try:
        arm.start(
            enable_esc=True,
            install_signal_handlers=True,
        )
        arm.set_max_speeds([20.0] * arm.motor_count)
        targets = [25.0] + [0.0] * (arm.motor_count - 1)
        arm.set_joint_angles(targets)

        while not arm.is_stopped:
            print(
                "\r"
                f"目标 / target: "
                f"{[round(x, 2) for x in arm.get_target_angles()]}  |  "
                f"发送 / sent: "
                f"{[round(x, 2) for x in arm.get_command_angles()]}",
                end="",
                flush=True,
            )
            time.sleep(0.1)

    except Exception as error:
        print(f"\n程序错误 / program error: {error}")

    finally:
        if not arm.is_stopped:
            arm.stop(return_to_zero=True, wait=True)
        else:
            arm.wait_until_stopped()


if __name__ == "__main__":
    main()
