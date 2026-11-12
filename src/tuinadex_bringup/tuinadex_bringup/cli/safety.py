"""safety.py — Hardware preflight diagnostics and interactive safety confirmation gate."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple


def check_can_interface(channel: str = "can0") -> Tuple[bool, str]:
    """Check whether a SocketCAN network interface exists and is UP."""
    net_dir = Path("/sys/class/net") / channel
    if not net_dir.exists():
        return False, f"[MISSING] SocketCAN 接口 {channel} 不存在（可执行 `tuina arm can up --channel {channel}` 或先启用 vcan）"

    flags_file = net_dir / "flags"
    try:
        flags_val = int(flags_file.read_text(encoding="utf-8").strip(), 16)
        is_up = bool(flags_val & 0x1)
    except (OSError, ValueError):
        operstate_file = net_dir / "operstate"
        try:
            state = operstate_file.read_text(encoding="utf-8").strip().lower()
            is_up = state in ("up", "unknown")
        except OSError:
            is_up = False

    if is_up:
        return True, f"[OK] SocketCAN 接口 {channel} 已就绪 (UP)"
    return False, f"[WARN] SocketCAN 接口 {channel} 处于 DOWN 状态（可执行 `tuina arm can up --channel {channel}`）"


def check_serial_device(port: str, label: str = "串口设备", imu_symlink: str = "/dev/myimu") -> Tuple[bool, str]:
    """Check whether a serial port exists and detect collision with the IMU device."""
    port_path = Path(port)
    if not port_path.exists():
        return False, f"[MISSING] {label} {port} 未连接或不存在"

    real_port = os.path.realpath(port)
    if label != "IMU" and Path(imu_symlink).exists():
        real_imu = os.path.realpath(imu_symlink)
        if real_port == real_imu:
            return False, (
                f"[WARN] {label} {port} 与手腕 IMU ({imu_symlink} -> {real_imu}) 指向同一物理端口，存在抢占冲突风险"
            )

    if real_port != port:
        return True, f"[OK] {label} {port} 已就绪 (-> {real_port})"
    return True, f"[OK] {label} {port} 已就绪"


def run_preflight_checks(
    arm_channel: Optional[str] = None,
    hand_port: Optional[str] = None,
    imu_port: Optional[str] = None,
) -> Tuple[bool, List[str]]:
    """Run hardware readiness checks for requested buses/devices without mutating state."""
    lines: List[str] = []
    all_ok = True

    if arm_channel:
        ok, msg = check_can_interface(arm_channel)
        lines.append(msg)
        if not ok:
            all_ok = False

    if hand_port:
        ok, msg = check_serial_device(hand_port, label="灵巧手串口")
        lines.append(msg)
        if not ok:
            all_ok = False

    if imu_port:
        ok, msg = check_serial_device(imu_port, label="IMU")
        lines.append(msg)
        if not ok:
            all_ok = False

    return all_ok, lines


def confirm_hardware_action(
    action_name: str,
    arm_channel: Optional[str] = None,
    hand_port: Optional[str] = None,
    imu_port: Optional[str] = None,
    warnings: Optional[List[str]] = None,
    skip_confirm: bool = False,
    dry_run: bool = False,
    high_risk: bool = False,
) -> bool:
    """Execute preflight checks and prompt for interactive confirmation on real-hardware actions.

    Returns True when allowed to proceed, False when aborted by user or non-interactive EOF.
    """
    _, preflight_lines = run_preflight_checks(
        arm_channel=arm_channel,
        hand_port=hand_port,
        imu_port=imu_port,
    )

    if preflight_lines:
        print("[SAFETY-PREFLIGHT] 硬件连接状态自检:")
        for line in preflight_lines:
            print(f"  {line}")

    if dry_run:
        print(f"[DRY-RUN][SAFETY] 待确认真机操作: {action_name}")
        return True

    if skip_confirm:
        print(f"[SAFETY] 已通过 --yes 跳过交互确认: {action_name}")
        return True

    banner = "⚠️ [高危真机操作确认]" if high_risk else "⚠️ [真机操作安全确认]"
    print(f"\n{banner} {action_name}")
    print("  1. 请确认机械臂/灵巧手急停开关 (E-Stop) 触手可及")
    print("  2. 请确认工作空间范围内无人员、障碍物或碰撞干涉")
    print("  3. 请确认急停与限位检查就位后再继续")
    for extra in warnings or ():
        print(f"  • {extra}")

    prompt = "输入 yes/y 确认继续，其他任意输入取消 [y/N]: "
    try:
        answer = input(prompt)
    except (EOFError, OSError, KeyboardInterrupt):
        print("\n[SAFETY] 非交互终端或输入中断，已中止真机操作（如需自动放行请加 -y / --yes）。", file=sys.stderr)
        return False

    if answer.strip().lower() in ("y", "yes"):
        return True

    print("[SAFETY] 用户取消操作，已安全中止。")
    return False
