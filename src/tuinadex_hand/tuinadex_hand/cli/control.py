#!/usr/bin/env python3
"""LEAP Hand 16-DOF 交互式控制终端 (leap-control).

支持逐指、逐关节相对弧度微调、常用推拿手法姿态切换与电机状态监测.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import numpy as np

from tuinadex_hand.core.hand_controller import HandController, LeapNode
from tuinadex_hand.core.pose_manager import load_poses


def print_help():
    print("""
╔══════════════════════════════════════════════════════════════╗
║              LEAP Hand 交互控制命令帮助                      ║
╠══════════════════════════════════════════════════════════════╣
║  【关节控制】                                                ║
║  格式: <手指> <关节> <角度>                                  ║
║  例如: index mcp 0.5   (食指 MCP 前后向内弯曲 0.5 rad)       ║
║        thumb mcp 0.3   (拇指 MCP 向内弯曲 0.3 rad)           ║
║        middle all 0.4  (中指所有关节弯曲 0.4 rad)             ║
║                                                              ║
║  【手指】index / middle / ring / thumb                       ║
║         食指   中指     无名   拇指                          ║
║                                                              ║
║  【关节】side / mcp / pip / dip / all                        ║
║         侧摆  前后  中关节 远关节 整指                        ║
║                                                              ║
║  【角度】相对位移 (正值=向手心弯曲, 负值=向手背伸展, rad)   ║
║  ⚠ thumb 关节顺序: 12=mcp, 13=side (与其他指相反)           ║
║                                                              ║
║  ─── 手势 ────────────────────────────────────────────────── ║
║  open       全开/平伸                                        ║
║  fist       全握拳                                           ║
║  half       半握                                             ║
║  point      食指指                                           ║
║  peace      比耶                                             ║
║  ok         OK手势                                           ║
║  thumbup    竖拇指                                           ║
║                                                              ║
║  ─── 其他命令 ────────────────────────────────────────────── ║
║  state      显示当前位置 (LEAP角度 + 相对位移 + 电流)         ║
║  raw <ID> <LEAP角度>  直接写舵机绝对弧度                     ║
║  poses      列出所有姿势库中的姿态                           ║
║  help       显示此帮助                                       ║
║  quit / exit 退出                                            ║
╚══════════════════════════════════════════════════════════════╝
""")


def main(argv=None):
    parser = argparse.ArgumentParser(description="LEAP Hand 16-DOF interactive CLI controller (leap-control)")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--mock", action="store_true", help="Run in mock mode without physical hardware")
    args = parser.parse_args(argv)

    print("\n启动 LEAP Hand 控制台...")
    try:
        ctrl = HandController(port=args.port, mock=args.mock)
    except Exception as e:
        print(f"[错误] 无法连接 LEAP Hand: {e}")
        return 1

    print("就绪！输入 'help' 查看帮助，'quit' 退出。\n")

    poses_dict, _ = load_poses()

    try:
        while True:
            try:
                cmd = input("LEAP> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n正在退出...")
                break

            if not cmd:
                continue

            parts = cmd.split()
            c0 = parts[0].lower()

            if c0 in ("quit", "exit", "q"):
                break
            elif c0 == "help":
                print_help()
            elif c0 == "state":
                ctrl.print_state()
            elif c0 == "poses":
                poses_dict, _ = load_poses()
                print("当前已注册姿势:")
                for name in poses_dict.keys():
                    print(f"  - {name}")
            elif c0 == "open":
                ctrl.target = ctrl.open_pose.copy()
                ctrl.leap.set_open()
                print("-> 全开位")
            elif c0 in ("fist", "half", "point", "peace", "ok", "thumbup") or c0 in poses_dict:
                alias_map = {
                    "fist": "全握拳",
                    "half": "半握",
                    "point": "食指指",
                    "peace": "比耶",
                    "ok": "OK手势",
                    "thumbup": "竖拇指",
                }
                target_name = alias_map.get(c0, c0)
                if target_name in poses_dict:
                    ctrl.target = poses_dict[target_name].copy()
                    ctrl.leap.set_leap(ctrl.target)
                    print(f"-> 切换至姿态: {target_name}")
                else:
                    print(f"[错误] 未知姿势: {target_name}")
            elif c0 == "raw":
                if len(parts) == 3:
                    try:
                        mid = int(parts[1])
                        angle = float(parts[2])
                        if 0 <= mid <= 15:
                            ctrl.target[mid] = angle
                            ctrl.leap.set_leap(ctrl.target)
                            print(f"-> 电机 {mid} 绝对弧度置为 {angle:.4f}")
                        else:
                            print("[错误] 电机 ID 必须在 0-15 范围内")
                    except ValueError:
                        print("[错误] 参数格式错误: raw <ID> <LEAP角度>")
                else:
                    print("[用法] raw <ID> <LEAP角度>")
            elif len(parts) == 3:
                # <finger> <joint> <angle>
                finger, joint, angle_str = parts
                try:
                    angle = float(angle_str)
                    if ctrl.set_joint_relative(finger, joint, angle):
                        print(f"-> {finger} {joint} 相对弧度置为 {angle:+.4f} rad")
                except ValueError:
                    print(f"[错误] 无法解析角度: {angle_str}")
            else:
                print(f"[未知命令] {cmd}，输入 'help' 查看帮助。")

    finally:
        ctrl.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())

