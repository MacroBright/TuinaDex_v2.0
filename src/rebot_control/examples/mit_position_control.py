#!/usr/bin/env python3
"""
Example layer: MIT position control demo for reBot B601-RS.
示例层：reBot B601-RS MIT 位置控制演示。

Features / 功能：
1. Fill in the 7 target angles manually in TARGET_ANGLES (J1–J6 + gripper J7).
   在 TARGET_ANGLES 列表中手动填写 7 个目标角度（J1–J6 + 夹爪 J7）。
2. All joints default to 20 deg/s (7 motors including gripper).
   全部关节（含夹爪 J7）默认速度统一为 20 度/秒。
3. Press Esc, Ctrl+C, or let the program end to slowly return to zero.
   按 Esc、Ctrl+C 或程序结束时缓慢回到零点。

Run / 运行：
    python3 examples/mit_position_control.py

Expected motion / 预期动作（默认 TARGET_ANGLES）：
    1. J1 以 20°/s 转向 +50°；J2–J6 与夹爪 J7 目标为 0° 并保持不动。
       J1 rotates to +50° at 20°/s; J2–J6 and gripper J7 target 0° and stay put.
    2. 终端一行持续刷新：目标角度、发送角度、各电机 MOS 温度。
       One line refreshes: target, sent, and MOS temperatures.
    3. 发送角度从当前值逐渐逼近目标（J1 慢慢靠近 50°）。
       Sent angles ramp from current pose toward targets (J1 toward 50°).
    4. 修改 TARGET_ANGLES 后，机械臂应朝新目标运动；请保持在约 70% 工作空间内。
       After editing TARGET_ANGLES, the arm moves toward new targets;
       stay within ~70% of workspace.
    5. 按 Esc/Ctrl+C 或程序结束：缓慢回零后失能。
       Esc/Ctrl+C or program end: slow return-to-zero, then disable.
"""

from __future__ import annotations

import time

from _bootstrap import setup_project_path

setup_project_path()

from rebot import ReBotRSMITController, load_config


# =============================================================================
# Demo parameters / 演示参数
# =============================================================================

# Controller config file path; None means the default config/rebotarm_rs.yaml.
# 控制器配置文件路径；None 表示使用默认的 config/rebotarm_rs.yaml。
CONFIG_PATH = None

# Target angles for J1-J6, in degrees.
# 目标角度，顺序为 J1–J6，单位：度。
TARGET_ANGLES = [
    50.0,  # J1
    0.0,   # J2
    0.0,   # J3
    0.0,   # J4
    0.0,   # J5
    0.0,   # J6
]

# Max motion speed per joint, in deg/s. / 各关节最大运动速度，单位：度/秒。
JOINT_SPEEDS_DEG_S = [20.0] * 6


def print_expected_motion() -> None:
    print(
        "[Expected / 预期] "
        f"J1 → {TARGET_ANGLES[0]}° at {JOINT_SPEEDS_DEG_S[0]}°/s, "
        "others → 0°; refresh target/sent/temps; Esc/Ctrl+C to exit / "
        f"J1 到 {TARGET_ANGLES[0]}°（{JOINT_SPEEDS_DEG_S[0]}°/s），"
        "其余到 0°；终端刷新状态；Esc/Ctrl+C 结束并回零"
    )


def main() -> None:
    print_expected_motion()

    if CONFIG_PATH is None:
        # Without a config it loads config/rebotarm_rs.yaml automatically.
        # 不传配置时自动加载 config/rebotarm_rs.yaml。
        arm = ReBotRSMITController()
    else:
        arm = ReBotRSMITController(
            load_config(CONFIG_PATH)
        )

    try:
        arm.start(
            enable_esc=True,
            install_signal_handlers=True,
        )

        speeds = (
            JOINT_SPEEDS_DEG_S[:arm.motor_count]
            if len(JOINT_SPEEDS_DEG_S) >= arm.motor_count
            else JOINT_SPEEDS_DEG_S + [20.0] * (arm.motor_count - len(JOINT_SPEEDS_DEG_S))
        )
        arm.set_max_speeds(speeds)

        targets = (
            TARGET_ANGLES[:arm.motor_count]
            if len(TARGET_ANGLES) >= arm.motor_count
            else TARGET_ANGLES + [0.0] * (arm.motor_count - len(TARGET_ANGLES))
        )

        print(
            "[Speed / 速度] "
            f"Speed: {speeds} deg/s / "
            f"速度: {speeds} 度/秒"
        )

        print(
            "[Target / 目标] "
            f"Target: {targets} deg / "
            f"目标: {targets} 度"
        )

        arm.set_joint_angles(targets)

        while not arm.is_stopped:
            target_angles = arm.get_target_angles()
            command_angles = arm.get_command_angles()

            temperature_text = [
                "--"
                if value is None
                else round(value, 1)
                for value in arm.last_temperatures
            ]

            print(
                "\r"
                f"Target/目标: {[round(x, 2) for x in target_angles]}  |  "
                f"Sent/发送: {[round(x, 2) for x in command_angles]}  |  "
                f"MOS temp/MOS温度: {temperature_text}",
                end="",
                flush=True,
            )

            time.sleep(0.10)

    except Exception as error:
        print(
            f"\n[Program error / 程序错误] Error: {error} / 错误: {error}"
        )

    finally:
        if not arm.is_stopped:
            arm.stop(
                return_to_zero=True,
                wait=True,
            )
        else:
            arm.wait_until_stopped()


if __name__ == "__main__":
    main()
