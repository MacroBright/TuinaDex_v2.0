#!/usr/bin/env python3
"""LEAP Hand 16-DOF 舵机物理限位检测与状态诊断工具 (leap-diagnose).

安全设计:
- 测量期间转矩关闭 (set_torque_enabled False)，手指可自由活动，绝无强行驱动至机械硬挡块风险;
- 手动轻推各关节至自然软停档位，采样编码器实际极限;
- 自动进行 2pi 回绕解算与 OPEN_POSE 包含度检测;
- 测量结束安全恢复转矩并回到全开位.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

from tuinadex_hand.core.hand_controller import LeapNode
from tuinadex_hand.core.pose_manager import get_default_config_path

_MOTOR_LABELS = [
    "Idx MCP-side", "Idx MCP", "Idx PIP", "Idx DIP",
    "Mid MCP-side", "Mid MCP", "Mid PIP", "Mid DIP",
    "Rng MCP-side", "Rng MCP", "Rng PIP", "Rng DIP",
    "Thb MCP-flex", "Thb MCP-side", "Thb PIP", "Thb DIP",
]


def main(argv=None):
    parser = argparse.ArgumentParser(description="LEAP Hand 16-motor physical limits diagnostics scanner (leap-diagnose)")
    parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--out", type=str, default=None, help="Output json path (default config/motor_limits.json)")
    parser.add_argument("--motor", type=int, nargs="+", default=None, help="Measure specific motors only, e.g. --motor 4")
    parser.add_argument("--mock", action="store_true", help="Run in mock mode without physical hardware")
    args = parser.parse_args(argv)

    try:
        leap = LeapNode(port=args.port, calib_mode=True, mock=args.mock)
    except Exception as e:
        print(f"[错误] 无法连接 LEAP Hand: {e}")
        return 1

    motors = list(leap.motors)
    dxl = leap.dxl_client
    to_measure = args.motor if args.motor is not None else list(range(16))
    TAU = 2.0 * np.pi

    out_path = Path(args.out) if args.out else get_default_config_path("motor_limits.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 增量安全: 从现有限位表加载已知值, 只更新 to_measure 电机
    limits_min = np.zeros(16, dtype=np.float64)
    limits_max = np.zeros(16, dtype=np.float64)
    if out_path.exists():
        try:
            with open(out_path, "r", encoding="utf-8") as f:
                prev = json.load(f)
            limits_min = np.array(prev["min"], dtype=np.float64)
            limits_max = np.array(prev["max"], dtype=np.float64)
            print(f"[INFO] 载入现有限位表 ({out_path})；本次仅更新电机: {to_measure}")
        except Exception as e:
            print(f"[WARN] 载入现有限位表失败: {e}")

    try:
        # 转矩关闭 -> 手指可手动活动
        dxl.set_torque_enabled(motors, False)
        print("\n[INFO] 16 舵机转矩已安全关闭，手指可手动活动。Ctrl+C 可随时退出。")
        input("    准备就绪后按 Enter 开始逐个电机测量...")

        for mid in to_measure:
            label = _MOTOR_LABELS[mid]
            while True:
                print(f"\n=== 电机 {mid:2d} ({label}) ===")
                input(f"  请推到 {label} 的 MIN 端 (最开/伸直)，按 Enter 记录...")
                time.sleep(0.2)
                pmin = float(dxl.read_pos()[mid])

                input(f"  请推到 {label} 的 MAX 端 (最弯/收紧)，按 Enter 记录...")
                time.sleep(0.2)
                pmax = float(dxl.read_pos()[mid])

                lo, hi = min(pmin, pmax), max(pmin, pmax)
                # 2π 回绕解算: 若 OPEN_POSE 不在 [lo,hi], 整体平移 ±2π 使其落入
                open_val = leap.open_pose[mid]
                for k in (-1, 0, 1):
                    if lo + k * TAU <= open_val <= hi + k * TAU:
                        lo, hi = lo + k * TAU, hi + k * TAU
                        break

                if not (lo <= open_val <= hi):
                    print(f"[⚠️ 告警] 当前 OPEN_POSE ({open_val:.4f}) 无法落入测量范围 [{lo:.4f}, {hi:.4f}] — 建议重测该电机。")
                if hi - lo < 0.05:
                    print(f"[⚠️ 告警] 电机测量范围过小 ({hi - lo:.4f} rad) — 可能未推动，建议重测。")

                print(f"  测得: min={lo:.4f}  max={hi:.4f}  (行程 {hi - lo:.4f} rad)")
                choice = input("  确认? [Enter=保存继续, r=重测该电机]: ").strip().lower()
                if choice != "r":
                    limits_min[mid] = lo
                    limits_max[mid] = hi
                    break

    finally:
        # 恢复: 转矩开启 + 回到全开位 (防止手指瘫软下垂)
        try:
            dxl.set_torque_enabled(motors, True)
            dxl.write_desired_pos(motors, leap.open_pose)
            time.sleep(0.5)
            print("[INFO] 舵机转矩已恢复，灵巧手安全回到全开平伸位。")
        except Exception as e:
            print(f"[WARN] 恢复转矩失败: {e}")
        leap.disconnect()

    data = {
        "min": [round(float(x), 4) for x in limits_min],
        "max": [round(float(x), 4) for x in limits_max],
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"\n✅ 限位诊断完成，数据已保存至: {out_path}")
    print(f"  {'ID':>3s}  {'关节名称':<14s}  {'MIN(rad)':>10s}  {'MAX(rad)':>10s}  {'行程(rad)':>10s}   OPEN位检验")
    print(f"  {'─'*68}")
    for i in range(16):
        is_ok = limits_min[i] <= leap.open_pose[i] <= limits_max[i]
        tag = "✓ 正常" if is_ok else "✗ 超出范围!"
        span = limits_max[i] - limits_min[i]
        print(f"  {i:>3d}  {_MOTOR_LABELS[i]:<14s}  {limits_min[i]:>10.4f}  {limits_max[i]:>10.4f}  {span:>10.4f}   {tag}")

    bad = [i for i in range(16) if not (limits_min[i] <= leap.open_pose[i] <= limits_max[i])]
    if bad:
        print(f"\n[⚠️ 提示] 电机 {bad} 的 OPEN_POSE 不在测得的物理限位内，请检查并重新标定或体检。")

    return 0


if __name__ == "__main__":
    sys.exit(main())

