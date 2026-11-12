#!/usr/bin/env python3
"""teach_arm_ready_pose.py — reBot 机械臂手动拖动示教与初始准备姿态快速标定工具.

功能:
1. 连接 can0 并在开机瞬间将 6 个电机安全失能 (卸除刚度与驱动力矩);
2. 终端高频单行刷新打印当前 6 轴实际机械角度 (J1~J6) 与变化量;
3. 允许用户直接用手托动机械臂至理想的推拿初始准备姿态 (手掌朝下悬停在推拿床上方);
4. 满意后按 [Ctrl+C] 或 [回车] 结束示教，自动打印角度并询问是否一键写入配置文件:
   -> src/tuinadex_teleop/configs/teleop_real_posture.yaml
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import time
import yaml

# 自动定位项目根目录与 rebot_control 路径
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "rebot_control"))

from rebot import ReBotRSMITController, load_config

CONFIG_FILE = ROOT / "src" / "tuinadex_teleop" / "configs" / "teleop_real_posture.yaml"


def main():
    print("=" * 70)
    print("   TuinaDex v2.0 — 6-DOF reBot 机械臂手动拖动示教与就绪姿态标定")
    print("=" * 70)
    print("【安全提示】")
    print("  1. 电机连接后将立刻失能，机械臂可能会因自重下沉，请先用手稳稳托住机械臂！")
    print("  2. 请手动将机械臂调整至您期望的【初始准备姿态】(如悬停在推拿床上方)。")
    print("  3. 姿态满意后，按 [Ctrl+C] 退出监视并完成记录。")
    print("=" * 70 + "\n")

    input("请托扶住机械臂，确认就绪后按 [回车] 开始示教连接...")

    arm = ReBotRSMITController()

    last_actual = None
    try:
        print("[连接中] 正在打开 CAN 总线并失能电机...")
        arm.connect()
        arm.disable_motors()
        print("✓ 电机已完全失能 (无动力下发，可自由推移示教)！\n")
        print("▶ 正在实时读取关节读数 (单位: 角度 deg)，请调整机械臂姿态...\n")

        init_angles = None
        while True:
            t0 = time.monotonic()
            actual = arm.read_joint_angles()
            last_actual = actual
            if init_angles is None:
                init_angles = actual

            parts = [f"J{i+1}:{actual[i]:>+6.1f}°" for i in range(len(actual))]
            print("\r▶ 实时读数: " + " | ".join(parts) + "  (按 Ctrl+C 确定)    ", end="", flush=True)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, 0.05 - elapsed))

    except KeyboardInterrupt:
        print("\n\n✓ 收到退出指令，已锁定当前停留姿态。")
    except Exception as e:
        print(f"\n[错误] 通信异常: {e}")
        return 1
    finally:
        arm.stop(return_to_zero=False, wait=True)

    if last_actual is None or len(last_actual) < 6:
        print("[提示] 未能采集到有效的机械臂角度，操作已取消。")
        return 1

    deg_vals = [round(float(x), 2) for x in last_actual[:6]]
    rad_vals = [round(math.radians(x), 4) for x in deg_vals]

    print("\n" + "=" * 70)
    print("  【示教姿态测定结果 (TEACHING SNAPSHOT)】")
    print("=" * 70)
    print(f"  各关节角度 (deg): {deg_vals}")
    print(f"  各关节弧度 (rad): {rad_vals}")
    print("\n  关节详细分解:")
    print(f"    - J1 (基座偏航 Yaw)  : {deg_vals[0]:>+7.2f}° ({rad_vals[0]:>+7.4f} rad)")
    print(f"    - J2 (肩部俯仰 Pitch): {deg_vals[1]:>+7.2f}° ({rad_vals[1]:>+7.4f} rad)")
    print(f"    - J3 (肘部俯仰 Pitch): {deg_vals[2]:>+7.2f}° ({rad_vals[2]:>+7.4f} rad)")
    print(f"    - J4 (手腕俯仰 Pitch): {deg_vals[3]:>+7.2f}° ({rad_vals[3]:>+7.4f} rad)")
    print(f"    - J5 (手腕偏航 Yaw)  : {deg_vals[4]:>+7.2f}° ({rad_vals[4]:>+7.4f} rad)")
    print(f"    - J6 (手腕翻转 Roll) : {deg_vals[5]:>+7.2f}° ({rad_vals[5]:>+7.4f} rad)")
    print("=" * 70)

    # 询问是否直接写入配置文件
    print(f"\n目标配置文件: {CONFIG_FILE}")
    try:
        ans = input("是否将此姿态保存为系统的【初始准备姿态】(Ready Stance)? [Y/n]: ").strip().lower()
    except (KeyboardInterrupt, EOFError):
        ans = "n"

    if ans in ("", "y", "yes"):
        try:
            if CONFIG_FILE.exists():
                with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                    cfg_data = yaml.safe_load(f) or {}
            else:
                cfg_data = {}

            if "arm" not in cfg_data:
                cfg_data["arm"] = {}

            cfg_data["arm"]["ready_pose_deg"] = {
                "joint1": float(deg_vals[0]),
                "joint2": float(deg_vals[1]),
                "joint3": float(deg_vals[2]),
                "joint4": float(deg_vals[3]),
                "joint5": float(deg_vals[4]),
                "joint6": float(deg_vals[5]),
            }

            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                yaml.dump(cfg_data, f, sort_keys=False, allow_unicode=True)

            print(f"\n[成功] 新准备姿态已成功保存写入配置文件:\n  -> {CONFIG_FILE}")
            print("今后在遥操中按下 [R] 键或一键就位时，机械臂都将平滑运动到此姿态！")
        except Exception as err:
            print(f"[错误] 写入配置文件失败: {err}")
    else:
        print("\n[跳过] 未保存到配置文件。您可以按需手动记录上述数值。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
