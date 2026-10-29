#!/usr/bin/env python3
"""
Tutorial 2: Load a specific YAML config file.
教程 2：指定配置文件。

Run / 运行：
    python3 examples/custom_config.py
    python3 examples/custom_config.py config/rebotarm_rs.yaml

Expected motion / 预期动作：
    1. 与 quick_start 相同流程，但显式通过 load_config() 加载 YAML。
       Same flow as quick_start, but the YAML is loaded via load_config().
    2. J1 以约 15°/s 转到 +15°；J2–J6 与夹爪（J7）保持 0°。
       J1 rotates to +15° at ~15°/s; J2–J6 and gripper (J7) stay at 0°.
    3. 等待发送角度接近目标后再 stop()，然后缓慢回零并失能。
       Waits until sent angles reach targets, then stop() and return-to-zero.
    4. 若更换 YAML 中的 CAN 通道或电机参数，动作幅度不变，但通信或 MIT 刚度可能不同。
       Changing CAN or motor settings in YAML does not change this demo's
       angles, but communication or MIT stiffness may differ.
"""

from __future__ import annotations

import sys

from _bootstrap import setup_project_path, wait_for_command_targets

setup_project_path()

from rebot import ReBotRSMITController, load_config

TARGET_ANGLES = [15.0, 0.0, 0.0, 0.0, 0.0, 0.0]
JOINT_SPEEDS_DEG_S = [15.0] * 6


def print_expected_motion(config_path: str) -> None:
    print(
        f"[Expected / 预期] Config: {config_path} | "
        "J1 → +15° at 15°/s, others 0°; then return-to-zero / "
        f"配置：{config_path} | "
        "J1 转到 +15°（约 15°/s），其余为 0°；随后回零"
    )


def main() -> None:
    config_path = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "config/rebotarm_rs.yaml"
    )

    print_expected_motion(config_path)

    arm = ReBotRSMITController(load_config(config_path))

    arm.start()
    arm.set_max_speeds([15.0] * arm.motor_count)
    targets = [15.0] + [0.0] * (arm.motor_count - 1)
    arm.set_joint_angles(targets)

    wait_for_command_targets(arm, show_progress=True)

    arm.stop()


if __name__ == "__main__":
    main()
