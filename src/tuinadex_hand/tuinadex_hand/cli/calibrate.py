#!/usr/bin/env python3
"""LEAP Hand 零点校准与姿态录制工具 (leap-calibrate).

功能:
1. 校准 — 重录全开平伸位 (OPEN_POSE 作为 0 rad 物理基准);
2. 录制 — 录制推拿手法姿态或自定义手势并保存至 poses.json.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from tuinadex_hand.core.hand_controller import LeapNode
from tuinadex_hand.core.pose_manager import (
    get_default_config_path,
    is_valid_pose as _is_valid_pose,
    load_poses,
    save_poses,
    unwrap_to_limits,
)

FINGERS = ["食指", "中指", "无名指", "拇指"]
JOINTS = ["侧摆", "前后", "PIP", "DIP"]


def read_pos_validated(leap: LeapNode, retries: int = 6) -> np.ndarray:
    """读取 16 关节位置，并做有效性校验与重试."""
    for attempt in range(retries):
        pos = leap.read_pos()
        pos = unwrap_to_limits(pos)
        if _is_valid_pose(pos):
            return pos
        time.sleep(0.1)

    print("\n[⚠️ 告警] 连续多次读取到异常数据 (全零或超出物理范围)!")
    print("         可能原因: 舵机未上电 / USB 通信断开 / 串口波特率不匹配。")
    ans = input("是否仍然强制使用当前读数? (y/N): ").strip().lower()
    if ans == "y":
        return pos
    raise RuntimeError("读取舵机位置失败，校准已中止。")


def print_table(positions: np.ndarray, title: str = ""):
    """打印美观的 16 关节弧度表格."""
    if title:
        print(f"\n{title}")
    print(f"{'─'*50}")
    print(f"  {'手指':<8} {'侧摆':>9} {'前后':>9} {'PIP':>9} {'DIP':>9}")
    print(f"{'─'*50}")
    for i, fname in enumerate(FINGERS):
        p = positions[i * 4 : (i + 1) * 4]
        print(f"  {fname:<8} {p[0]:>9.4f} {p[1]:>9.4f} {p[2]:>9.4f} {p[3]:>9.4f}")
    print(f"{'─'*50}\n")


def record_single(leap: LeapNode, pose_name: str) -> List[float]:
    """关闭转矩让用户摆好手势后，按回车采样保存."""
    print(f"\n─── 正在录制: 「{pose_name}」───")
    print("  转矩已临时关闭，请手动将灵巧手摆为目标姿势。")
    print("  摆好并保持住后，按 Enter 键记录。")

    leap.set_torque(False)
    try:
        input("按 Enter 记录当前姿势...")
        pos = read_pos_validated(leap)
        print_table(pos, f"采集到的「{pose_name}」数据")
    finally:
        leap.set_torque(True)

    return [round(float(p), 4) for p in pos]


def do_calibrate(leap: LeapNode) -> Path:
    """校准模式: 重录全开位."""
    print("\n" + "=" * 55)
    print("  📐 零点校准模式 (重录 OPEN_POSE)")
    print("=" * 55)
    print("""
  请将灵巧手完全张开 (五指伸直、并拢、平摊，作为 0 rad 基准)，
  保持住这个姿势，然后按 Enter。
""")
    leap.set_torque(False)
    try:
        input("按 Enter 记录全开位...")
        pos = read_pos_validated(leap)
        print_table(pos, "新全开位 (OPEN_POSE)")
    finally:
        leap.set_torque(True)

    poses, _ = load_poses()
    poses["全开/平伸"] = np.array([round(float(p), 4) for p in pos], dtype=np.float64)
    out_file = save_poses(poses)

    print(f"\n✅ 零点校准完成！数据已持久化写入: {out_file}")
    return out_file


def do_record(leap: LeapNode) -> Optional[Path]:
    """录制模式: 循环选择已有姿势重录或新增自定义手法姿势."""
    out_file = None
    while True:
        poses, _ = load_poses()
        names = list(poses.keys())

        print("\n" + "=" * 55)
        print("  🎬 手势与推拿手法姿态录制")
        print("=" * 55)
        print("\n  已有姿势:")
        for i, name in enumerate(names):
            print(f"    {i+1}. {name}")
        print(f"    {len(names)+1}. ⭐ 新增自定义姿势/推拿手法...")
        print("    0. 返回主菜单")
        print()

        try:
            choice = input(f"请选择 (0-{len(names)+1}): ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == "0":
            break

        try:
            idx = int(choice) - 1
        except ValueError:
            print("[错误] 请输入有效数字")
            continue

        if 0 <= idx < len(names):
            pose_name = names[idx]
            angles = record_single(leap, pose_name)
            poses[pose_name] = np.array(angles, dtype=np.float64)
            out_file = save_poses(poses)
            print(f"\n✅ 姿势「{pose_name}」已更新！")

        elif idx == len(names):
            try:
                pose_name = input("  请输入新姿势名称 (如: 滚法 / 拿法): ").strip()
            except (EOFError, KeyboardInterrupt):
                continue
            if not pose_name:
                print("[错误] 名称不能为空")
                continue
            if pose_name in poses:
                confirm = input(f"  「{pose_name}」已存在，是否覆盖? (y/N): ").strip().lower()
                if confirm != "y":
                    continue

            angles = record_single(leap, pose_name)
            poses[pose_name] = np.array(angles, dtype=np.float64)
            out_file = save_poses(poses)
            print(f"\n✅ 新手法姿态「{pose_name}」已保存！")

        else:
            print("[错误] 无效选择")

    return out_file


def main(argv=None):
    parser = argparse.ArgumentParser(description="LEAP Hand zero-point calibration and pose recording tool (leap-calibrate)")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--action", choices=["1", "2"], default=None, help="1=Calibrate open pose, 2=Record custom pose")
    parser.add_argument("--mock", action="store_true", help="Run in mock mode without physical hardware")
    args = parser.parse_args(argv)

    print("\n╔══════════════════════════════════════╗")
    print("║   LEAP Hand 校准与姿态录制工具       ║")
    print("╠══════════════════════════════════════╣")
    print("║  1. 校准 (重录全开平伸位 0 rad 基准) ║")
    print("║  2. 录制 (重录/新增推拿手法姿势)     ║")
    print("╚══════════════════════════════════════╝")

    choice = args.action
    if choice is None:
        try:
            choice = input("\n请选择 (1/2): ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已取消操作。")
            return 0

    print("\n正在连接 LEAP Hand...")
    try:
        # calib_mode=True 允许在全开位无效时正常连接进行重新标定
        leap = LeapNode(port=args.port, calib_mode=True, mock=args.mock)
    except Exception as e:
        print(f"[错误] 无法连接 LEAP Hand: {e}")
        return 1

    try:
        if choice == "1":
            do_calibrate(leap)
        elif choice == "2":
            do_record(leap)
        else:
            print("[错误] 无效选项，请输入 1 或 2。")
            return 1
    finally:
        leap.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())

