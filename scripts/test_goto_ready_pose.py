#!/usr/bin/env python3
"""test_goto_ready_pose.py — 6-DOF reBot 机械臂平滑运动至初始准备姿态实测验证脚本.

功能:
1. 自动加载 src/tuinadex_teleop/configs/teleop_real_posture.yaml 中的准备姿态配置;
2. 显示当前配置的目标关节角度与安全限速;
3. 建立 CAN 通信并在使能后，以平滑五次样条/匀加速限制将机械臂从当前姿态驱动至准备姿态;
4. 运动到位后打印实际物理反馈与到位误差;
5. 支持保持在准备姿态退出，或按 [Z] 平滑回零位。
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "rebot_control"))

from rebot import ReBotRSMITController, load_config

CONFIG_FILE = ROOT / "src" / "tuinadex_teleop" / "configs" / "teleop_real_posture.yaml"


def main():
    print("=" * 70)
    print("   TuinaDex v2.0 — 机械臂平滑就位 (Ready Pose) 硬件动作验证")
    print("=" * 70)

    if not CONFIG_FILE.exists():
        print(f"[错误] 找不到配置文件: {CONFIG_FILE}")
        return 1

    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    arm_cfg = cfg.get("arm", {})
    ready_dict = arm_cfg.get("ready_pose_deg", {})
    motion_cfg = arm_cfg.get("motion", {})

    target_deg = [
        float(ready_dict.get("joint1", 0.0)),
        float(ready_dict.get("joint2", 70.0)),
        float(ready_dict.get("joint3", 50.0)),
        float(ready_dict.get("joint4", -70.0)),
        float(ready_dict.get("joint5", 0.0)),
        float(ready_dict.get("joint6", 0.0)),
    ]
    max_speed = float(motion_cfg.get("max_speed_deg_s", 15.0))
    # 为安全起见，验证时速度上限限制在 15°/s 以内
    safe_speed = min(max_speed, 15.0)

    print(f"已加载准备姿态配置: {CONFIG_FILE.name}")
    print(f"  - 目标角度 (deg) : {target_deg}")
    print(f"  - 安全运行限速   : {safe_speed:.1f} °/s (平滑匀速逼近)")
    print("  - 关节分解       :")
    print(f"      J1 (基座偏航)  : {target_deg[0]:>+6.1f}°")
    print(f"      J2 (肩部俯仰)  : {target_deg[1]:>+6.1f}° (向前上方展开)")
    print(f"      J3 (肘部俯仰)  : {target_deg[2]:>+6.1f}° (向前弯折)")
    print(f"      J4 (手腕俯仰)  : {target_deg[3]:>+6.1f}° (向下俯视推拿床)")
    print(f"      J5 (手腕偏航)  : {target_deg[4]:>+6.1f}°")
    print(f"      J6 (手腕翻转)  : {target_deg[5]:>+6.1f}°")
    print("=" * 70)
    print("【安全提醒】请确认机械臂运动空间内无人员与障碍物，随时可按 [Ctrl+C] 减速停止。")
    input("确认安全后，请按 [回车] 开始平滑就位运动...")

    arm = ReBotRSMITController()

    try:
        print("\n[1/3] 正在建立 CAN 通信并使能机械臂...")
        arm.start(enable_esc=True, install_signal_handlers=True)
        arm.set_max_speeds([safe_speed] * arm.motor_count)

        start_angles = arm.get_command_angles()
        print(f"[2/3] 机械臂当前初始姿态: {[round(x, 1) for x in start_angles]}°")
        print(f"[3/3] 开始平滑向准备姿态运动 (限速 {safe_speed}°/s)...")

        arm.set_joint_angles(target_deg)

        # 监控运动过程直至到位
        t_start = time.monotonic()
        while not arm.is_stopped:
            targets = arm.get_target_angles()
            commands = arm.get_command_angles()
            errors = [abs(t - c) for t, c in zip(targets, commands)]
            max_err = max(errors)

            parts = [f"J{i+1}:{commands[i]:>+5.1f}°" for i in range(len(commands))]
            print(f"\r▶ 执行中: " + " ".join(parts) + f" (最大剩余误差: {max_err:4.1f}°)   ", end="", flush=True)

            if max_err <= 0.3:
                print("\n\n✓ 运动完成！机械臂已平滑到达目标准备姿态。")
                break

            time.sleep(0.05)

        # 稳定等待 0.3s
        time.sleep(0.3)
        actual = arm.read_joint_angles()
        final_err = [round(actual[i] - target_deg[i], 2) for i in range(len(target_deg))]
        print("=" * 70)
        print(f"  到位实测角度 (deg) : {[round(x, 2) for x in actual]}")
        print(f"  各轴控制误差 (deg) : {final_err}")
        print("=" * 70)

        # 询问后续退出动作
        print("\n请选择退出方式:")
        print("  [1] (推荐) 保持在此准备姿态并失能退出 (输入 1 或直接按回车)")
        print("  [2] 平滑回归全 0° 零位并失能退出 (输入 2)")
        try:
            choice = input("您的选择 [1]: ").strip()
        except (KeyboardInterrupt, EOFError):
            choice = "1"

        if choice == "2":
            print("\n正在缓慢平滑回全零位...")
            arm.stop(return_to_zero=True, wait=True)
            print("✓ 已回零并安全失能。")
        else:
            print("\n正在保持当前准备姿态并失能...")
            arm.stop(return_to_zero=False, wait=True)
            print("✓ 已保持就位并失能退出。")

    except KeyboardInterrupt:
        print("\n\n[安全响应] 收到用户中断，正在安全停止...")
        arm.stop(return_to_zero=False, wait=True)
        print("✓ 已安全停止并失能。")
    except Exception as e:
        print(f"\n[错误] 运行异常: {e}")
        try:
            arm.stop(return_to_zero=False, wait=False)
        except Exception:
            pass
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
