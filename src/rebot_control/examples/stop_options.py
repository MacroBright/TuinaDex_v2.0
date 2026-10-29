#!/usr/bin/env python3
"""
Tutorial 6: Different safe-stop options.
教程 6：安全停止的几种方式。

Run / 运行：
    python3 examples/stop_options.py default
    python3 examples/stop_options.py no_return
    python3 examples/stop_options.py async
    python3 examples/stop_options.py emergency

Expected motion / 预期动作（各模式先相同，再按模式不同）：
    共同前置：
      - 先使能，J1 在约 2 s 内转到约 +20° 并停住。
        Common: enable, then J1 moves to ~+20° within ~2 s and holds.

    default：
      - 从当前姿态缓慢、平滑回到全零，再失能（推荐，最安全）。
        Slow smooth return to all zeros, then disable (recommended).

    no_return：
      - 停在 J1≈+20° 的姿态，不回零，直接失能（臂仍保持在该角度附近）。
        Stays at J1≈+20°, no return-to-zero, immediate disable.

    async：
      - 视觉上与 default 相同：缓慢回零后失能；区别在代码是否阻塞等待。
        Looks like default; difference is non-blocking stop() in code.

    emergency：
      - 在 J1≈+20° 处立即失能，无回零轨迹（运动突然结束，注意安全）。
        Immediate disable at J1≈+20° with no return-to-zero (motion stops abruptly).
"""

from __future__ import annotations

import argparse
import time

from _bootstrap import setup_project_path

setup_project_path()

from rebot import ReBotRSMITController

EXPECTED_BY_MODE = {
    "default": (
        "J1 holds ~+20°, then slow smooth return-to-zero, then disable / "
        "J1 停在约 +20° 后，缓慢平滑回零，再失能"
    ),
    "no_return": (
        "J1 holds ~+20°, disable immediately without return-to-zero / "
        "J1 停在约 +20°，不回零，直接失能"
    ),
    "async": (
        "Same motion as default; stop() returns before shutdown finishes / "
        "动作与 default 相同；stop(wait=False) 先返回，后台继续回零"
    ),
    "emergency": (
        "J1 holds ~+20°, immediate disable, no return-to-zero / "
        "J1 停在约 +20°，立即失能，不回零"
    ),
}


def move_to_demo_pose(arm: ReBotRSMITController) -> None:
    arm.set_max_speeds([15.0] * arm.motor_count)
    arm.set_joint_angles([20.0] + [0.0] * (arm.motor_count - 1))
    time.sleep(2.0)


def print_expected_motion(mode: str) -> None:
    print(
        f"[Expected / 预期] First J1 → ~+20° in ~2 s; then [{mode}]: "
        f"{EXPECTED_BY_MODE[mode]}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Demonstrate arm.stop() and request_stop() options."
    )
    parser.add_argument(
        "mode",
        choices=("default", "no_return", "async", "emergency"),
        default="default",
        nargs="?",
        help="default: slow return-to-zero; no_return: disable only; "
        "async: stop(wait=False); emergency: request_stop(emergency=True)",
    )
    args = parser.parse_args()

    print_expected_motion(args.mode)

    arm = ReBotRSMITController()

    arm.start(enable_esc=False, install_signal_handlers=False)
    move_to_demo_pose(arm)

    if args.mode == "default":
        print("[Stop / 停止] default: slow return-to-zero / 缓慢回零后失能")
        arm.stop()

    elif args.mode == "no_return":
        print(
            "[Stop / 停止] no_return: disable without return-to-zero / "
            "不回零，直接失能"
        )
        arm.stop(return_to_zero=False)

    elif args.mode == "async":
        print("[Stop / 停止] async: stop(wait=False) / 异步停止")
        arm.stop(wait=False)
        finished = arm.wait_until_stopped(timeout=30.0)
        print(
            f"[Stop / 停止] wait_until_stopped: "
            f"{'done' if finished else 'timeout'} / "
            f"{'完成' if finished else '超时'}"
        )

    elif args.mode == "emergency":
        print(
            "[Stop / 停止] emergency: immediate disable / "
            "紧急失能，不回零"
        )
        arm.request_stop(
            return_to_zero=False,
            emergency=True,
            wait=True,
        )


if __name__ == "__main__":
    main()
