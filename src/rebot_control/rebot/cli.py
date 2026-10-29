#!/usr/bin/env python3
"""
Interactive and command-line control tool for reBot Arm B601-RS.
reBot Arm B601-RS 命令行与交互式电机逐关节控制工具。

Features:
- Interactive REPL shell for step-by-step joint tuning and jog control.
- One-shot CLI commands for quick status inspection, joint positioning, and zeroing.
- Safe-by-default: speed limits, smoothstep return-to-zero on exit.
"""

from __future__ import annotations

import argparse
import cmd
import math
import sys
import time
from pathlib import Path
from typing import Sequence

try:
    import readline
except ImportError:
    readline = None

from .config import DEFAULT_CONFIG_PATH, ControllerConfig, load_config
from .controller import ReBotRSMITController


class ReBotShell(cmd.Cmd):
    intro = (
        "\n"
        "======================================================================\n"
        "   reBot Arm B601-RS 机械臂逐关节交互控制控制台 (6-DOF MIT CLI)\n"
        "======================================================================\n"
        " 输入 'help' 查看指令说明，输入 'status' 查看关节状态，输入 'exit' 安全回零退出。\n"
    )
    prompt = "rebot-arm> "

    def __init__(self, arm: ReBotRSMITController, default_speed: float | None = None):
        super().__init__()
        self.arm = arm
        self.default_speed = default_speed if default_speed is not None else arm.config.default_speed_deg_s
        self.motor_count = self.arm.motor_count
        if default_speed is not None:
            self.arm.set_max_speeds([default_speed] * self.motor_count)

    def emptyline(self) -> bool:
        # Do nothing on empty enter
        return False

    def do_status(self, _arg: str) -> bool:
        """status / st: 显示当前所有关节的目标角度、发送角度与 MOS 温度。"""
        targets = self.arm.get_target_angles()
        commands = self.arm.get_command_angles()
        temps = self.arm.last_temperatures
        speeds = [math.degrees(s) for s in self.arm.max_speeds_rad_s]

        print("\n" + "-" * 72)
        print(f"{'关节':<6} {'CAN ID':<8} {'电机型号':<10} {'目标(°)':<10} {'当前/发送(°)':<14} {'限速(°/s)':<10} {'MOS温度':<8}")
        print("-" * 72)

        for i in range(self.motor_count):
            m_cfg = self.arm.config.motors[i]
            t_deg = f"{targets[i]:.2f}"
            c_deg = f"{commands[i]:.2f}"
            s_deg = f"{speeds[i]:.1f}"
            temp_str = f"{temps[i]:.1f}°C" if (i < len(temps) and temps[i] is not None) else "--"
            print(f"J{i+1:<5} {m_cfg.motor_id:<8} {m_cfg.model:<10} {t_deg:<10} {c_deg:<14} {s_deg:<10} {temp_str:<8}")

        print("-" * 72 + "\n")
        return False

    def do_st(self, arg: str) -> bool:
        """status 的快捷别名。"""
        return self.do_status(arg)

    def do_set(self, arg: str) -> bool:
        """set <joint_id> <angle_deg>: 设置指定关节的目标角度（绝对角度，单位：度）。
        示例:
          set 1 25.0    # 将 1 号关节转到 +25.0°
          set 4 -15.0   # 将 4 号关节转到 -15.0°
        """
        parts = arg.strip().split()
        if len(parts) != 2:
            print("[错误] 用法: set <joint_id (1~6)> <angle_deg>")
            return False

        try:
            joint_id = int(parts[0])
            angle_deg = float(parts[1])
        except ValueError:
            print("[错误] 参数必须为数值: set <joint_id> <angle_deg>")
            return False

        if not 1 <= joint_id <= self.motor_count:
            print(f"[错误] 关节编号无效: 必须在 1 到 {self.motor_count} 之间。")
            return False

        current = self.arm.get_target_angles()[joint_id - 1]
        self.arm.set_joint_angle(joint_id, angle_deg)
        print(f"✓ 关节 J{joint_id} 目标已设置: {current:.2f}° -> {angle_deg:.2f}° (平滑限速移动中...)")
        return False

    def do_step(self, arg: str) -> bool:
        """step <joint_id> <delta_deg>: 对指定关节进行相对步进点动微调（单位：度）。
        示例:
          step 1 5.0    # 1 号关节在当前目标基础上增加 +5.0°
          step 1 -5.0   # 1 号关节在当前目标基础上减小 -5.0°
        """
        parts = arg.strip().split()
        if len(parts) != 2:
            print("[错误] 用法: step <joint_id (1~6)> <delta_deg>")
            return False

        try:
            joint_id = int(parts[0])
            delta_deg = float(parts[1])
        except ValueError:
            print("[错误] 参数必须为数值: step <joint_id> <delta_deg>")
            return False

        if not 1 <= joint_id <= self.motor_count:
            print(f"[错误] 关节编号无效: 必须在 1 到 {self.motor_count} 之间。")
            return False

        current = self.arm.get_target_angles()[joint_id - 1]
        new_angle = current + delta_deg
        self.arm.set_joint_angle(joint_id, new_angle)
        print(f"✓ 关节 J{joint_id} 相对微调 {delta_deg:+.2f}°: {current:.2f}° -> {new_angle:.2f}°")
        return False

    def do_all(self, arg: str) -> bool:
        """all <a1> <a2> <a3> <a4> <a5> <a6>: 同时设置全部关节目标角度（度）。
        示例:
          all 20 0 0 0 0 0
        """
        parts = arg.strip().split()
        if len(parts) != self.motor_count:
            print(f"[错误] 必须提供 {self.motor_count} 个角度值，当前输入 {len(parts)} 个。")
            return False

        try:
            angles = [float(x) for x in parts]
        except ValueError:
            print("[错误] 角度必须为有效数字。")
            return False

        self.arm.set_joint_angles(angles)
        print(f"✓ 已下发全部关节新目标: {[round(a, 2) for a in angles]}°")
        return False

    def do_speed(self, arg: str) -> bool:
        """speed <speed_deg_s> 或 speed <joint_id> <speed_deg_s>: 设置移动限速（度/秒）。
        示例:
          speed 20.0       # 将所有关节速度上限设为 20°/s
          speed 1 30.0     # 仅将 J1 关节速度上限设为 30°/s
        """
        parts = arg.strip().split()
        if len(parts) == 1:
            try:
                val = float(parts[0])
                if val <= 0:
                    raise ValueError
            except ValueError:
                print("[错误] 速度必须是大于 0 的正数。")
                return False

            self.arm.set_max_speeds([val] * self.motor_count)
            print(f"✓ 全部关节速度限制已更新为: {val:.1f}°/s")

        elif len(parts) == 2:
            try:
                joint_id = int(parts[0])
                val = float(parts[1])
                if val <= 0:
                    raise ValueError
            except ValueError:
                print("[错误] 用法: speed <joint_id> <speed_deg_s>")
                return False

            if not 1 <= joint_id <= self.motor_count:
                print(f"[错误] 关节编号无效: 1~{self.motor_count}")
                return False

            speeds = [math.degrees(s) for s in self.arm.max_speeds_rad_s]
            speeds[joint_id - 1] = val
            self.arm.set_max_speeds(speeds)
            print(f"✓ 关节 J{joint_id} 速度限制已更新为: {val:.1f}°/s")
        else:
            print("[错误] 用法: speed <speed_deg_s> 或 speed <joint_id> <speed_deg_s>")

        return False

    def do_zero(self, _arg: str) -> bool:
        """zero: 将全部关节目标设置为 0°（按限速平滑移动）。"""
        self.arm.set_joint_angles([0.0] * self.motor_count)
        print("✓ 全部关节目标已重置为 0.0°，平滑回归中...")
        return False

    def do_wait(self, arg: str) -> bool:
        """wait [tolerance_deg]: 等待当前所有关节平滑到达目标角度（默认误差阈值 0.5°）。"""
        try:
            tol = float(arg.strip()) if arg.strip() else 0.5
        except ValueError:
            tol = 0.5

        print(f"等待关节运动到位中 (阈值: {tol}°)... 按 Ctrl+C 可取消等待。")
        try:
            while not self.arm.is_stopped:
                targets = self.arm.get_target_angles()
                commands = self.arm.get_command_angles()
                diffs = [abs(t - c) for t, c in zip(targets, commands)]
                max_diff = max(diffs)

                print(f"\r最大滞后误差: {max_diff:.2f}° | 发送位置: {[round(c, 1) for c in commands]}", end="", flush=True)

                if max_diff <= tol:
                    print("\n✓ 所有关节均已平滑到位！")
                    break
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n[中断] 用户中止等待。")

        return False

    def do_disable(self, _arg: str) -> bool:
        """disable: 失能所有电机（断开扭矩，可手动拖动机械臂，但不关闭通信）。"""
        print("⚠️ 正在失能所有电机，机械臂可能因重力下垂，请注意防护！")
        self.arm.disable_motors()
        print("✓ 所有电机已失能（可轻推示教）。如需重新受控，请输入 'enable'。")
        return False

    def do_enable(self, _arg: str) -> bool:
        """enable: 重新读取当前实际位置并重新使能锁定所有电机。"""
        try:
            self.arm.enable_motors(sync_positions=True)
            actual = self.arm.get_target_angles()
            print(f"✓ 所有电机已重新使能锁定！当前保持角度: {[round(a, 2) for a in actual]}°")
        except Exception as e:
            print(f"[错误] 重新使能失败: {e}")
        return False

    def do_watch(self, arg: str) -> bool:
        """watch [joint_id]: 手动拖动示教模式，失能电机并实时高频刷新打印实际机械角度。
        用法:
          watch        # 实时监控全部 6 个关节
          watch 2      # 仅聚焦监控 2 号关节（显示当前绝对角度与变化量）
        快捷别名: drag, teach, read
        按 Ctrl+C 即可退出监视，并自动将当前停留姿态同步为新基准。
        """
        target_joint = None
        if arg.strip():
            try:
                target_joint = int(arg.strip())
                if not 1 <= target_joint <= self.motor_count:
                    print(f"[错误] 关节编号无效: 必须在 1 到 {self.motor_count} 之间。")
                    return False
            except ValueError:
                print("[错误] 用法: watch [joint_id (1~6)]")
                return False

        print("\n" + "=" * 65)
        print("   【手动拖动示教 / 实时机械角度监控】")
        print("=" * 65)
        print("⚠️  正在失能电机以卸除刚度... 请用手托扶住机械臂防止自重下坠！")
        self.arm.disable_motors()
        print("✓ 电机已失能，您可以直接用手转动关节。")
        if target_joint is not None:
            print(f"▶ 正在以 ~20Hz 刷新 J{target_joint} 关节角度，按 Ctrl+C 退出...\n")
        else:
            print("▶ 正在以 ~20Hz 刷新所有关节角度，按 Ctrl+C 退出...\n")

        init_angles = None
        last_actual = None
        try:
            while not self.arm.is_stopped:
                t0 = time.monotonic()
                actual = self.arm.read_joint_angles()
                last_actual = actual
                if init_angles is None:
                    init_angles = actual

                if target_joint is not None:
                    idx = target_joint - 1
                    cur = actual[idx]
                    delta = cur - init_angles[idx]
                    print(
                        f"\r▶ J{target_joint} 机械角度: {cur:>7.2f}°  (较初值变化: {delta:>+6.2f}°)    ",
                        end="",
                        flush=True,
                    )
                else:
                    parts = [f"J{i+1}:{actual[i]:>+6.1f}°" for i in range(self.motor_count)]
                    print("\r" + " | ".join(parts) + "    ", end="", flush=True)

                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, 0.05 - elapsed))
        except KeyboardInterrupt:
            pass

        print("\n\n✓ 已退出拖动监视模式。")
        if last_actual is not None:
            self.arm.set_joint_angles(last_actual)
            with self.arm.target_lock:
                self.arm.command_positions[:] = [math.radians(x) for x in last_actual]
            print(f"当前停留位置已记录为基准目标: {[round(x, 2) for x in last_actual]}°")
        print("提示: 当前电机处于失能状态，如需在此姿态重新锁定使能，请输入 'enable'；如需平滑回零退出，请输入 'exit'。\n")
        return False

    def do_drag(self, arg: str) -> bool:
        """watch 的快捷别名（手动拖动示教）。"""
        return self.do_watch(arg)

    def do_teach(self, arg: str) -> bool:
        """watch 的快捷别名（示教读取）。"""
        return self.do_watch(arg)

    def do_read(self, arg: str) -> bool:
        """watch 的快捷别名（连续读取实际角度）。"""
        return self.do_watch(arg)

    def do_exit(self, _arg: str) -> bool:
        """exit / quit / q: 平滑回零后失能，并退出程序。"""
        print("\n正在执行安全回零退出程序...")
        self.arm.stop(return_to_zero=True, wait=True)
        print("✓ 已安全回零并关闭通信，再见！")
        return True

    def do_quit(self, arg: str) -> bool:
        """exit 的快捷别名。"""
        return self.do_exit(arg)

    def do_q(self, arg: str) -> bool:
        """exit 的快捷别名。"""
        return self.do_exit(arg)


def run_interactive(config_path: str | None, default_speed: float | None = None):
    cfg = load_config(config_path) if config_path else load_config()
    arm = ReBotRSMITController(cfg)
    print(f"[CAN] 连接 CAN 通道 '{cfg.channel}'，检测到 {arm.motor_count} 个电机配置...")

    try:
        arm.start(enable_esc=True, install_signal_handlers=False)
        if default_speed is not None:
            arm.set_max_speeds([default_speed] * arm.motor_count)
    except Exception as e:
        print(f"[错误] 初始化机械臂失败: {e}")
        print("请检查:")
        print("  1. 是否执行了 'sudo ip link set can0 up type can bitrate 1000000'")
        print("  2. 电源 48V 是否开启，CAN 线路与终端电阻是否正常")
        return 1

    shell = ReBotShell(arm, default_speed=default_speed)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print("\n\n[中断] 接收到 Ctrl+C，启动紧急回零退出...")
        arm.stop(return_to_zero=True, wait=True)
    return 0


def run_status_oneshot(config_path: str | None):
    """单次非运动读取当前实际角度与 MOS 温度"""
    cfg = load_config(config_path) if config_path else load_config()
    arm = ReBotRSMITController(cfg)
    print(f"[CAN] 读取通道: {cfg.channel} | 电机数: {arm.motor_count}")

    try:
        arm.connect()
        # 立即失能，仅读状态
        arm.disable_motors()
        time.sleep(0.05)
        actual_angles = arm.read_joint_angles()

        print("\n" + "=" * 60)
        print(f"{'关节':<6} {'CAN ID':<8} {'型号':<10} {'当前机械位置 (度)':<18}")
        print("=" * 60)
        for i, angle in enumerate(actual_angles):
            m = cfg.motors[i]
            print(f"J{i+1:<5} {m.motor_id:<8} {m.model:<10} {angle:>8.2f}°")
        print("=" * 60 + "\n")

    except Exception as e:
        print(f"[错误] 读取机械臂状态失败: {e}")
        return 1
    finally:
        arm.stop(return_to_zero=False, wait=True)
    return 0


def run_watch_oneshot(config_path: str | None, joint: int | None = None, hz: float = 20.0):
    """单次连接进入手动拖动并高频实时打印关节角度"""
    cfg = load_config(config_path) if config_path else load_config()
    arm = ReBotRSMITController(cfg)
    print(f"[CAN] 连接通道: {cfg.channel} | 电机数: {arm.motor_count}")

    interval = 1.0 / max(1.0, hz)

    try:
        arm.connect()
        arm.disable_motors()
        print("\n" + "=" * 65)
        print("   【手动拖动示教 / 实时机械角度监控】")
        print("=" * 65)
        print("⚠️  电机已失能（无阻抗力矩），请用手托住机械臂防止自重下坠！")
        if joint is not None:
            print(f"▶ 正在以 ~{hz:.0f}Hz 刷新 J{joint} 关节角度，按 Ctrl+C 退出...\n")
        else:
            print(f"正在以 ~{hz:.0f}Hz 刷新所有关节角度，按 Ctrl+C 退出...\n")

        init_angles = None
        while True:
            t0 = time.monotonic()
            actual = arm.read_joint_angles()
            if init_angles is None:
                init_angles = actual

            if joint is not None:
                idx = joint - 1
                cur = actual[idx]
                delta = cur - init_angles[idx]
                print(
                    f"\r▶ J{joint} 机械角度: {cur:>7.2f}°  (较初值变化: {delta:>+6.2f}°)    ",
                    end="",
                    flush=True,
                )
            else:
                parts = [f"J{i+1}:{actual[i]:>+6.1f}°" for i in range(arm.motor_count)]
                print("\r" + " | ".join(parts) + "    ", end="", flush=True)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))

    except KeyboardInterrupt:
        print("\n\n✓ 用户退出监视。")
    except Exception as e:
        print(f"\n[错误] 读取失败: {e}")
        return 1
    finally:
        arm.stop(return_to_zero=False, wait=True)
    return 0


def run_move_oneshot(config_path: str | None, joint: int | None, angle: float | None, angles_list: list[float] | None, speed: float | None, wait: bool, zero_on_exit: bool):
    """单次下发运动指令"""
    cfg = load_config(config_path) if config_path else load_config()
    arm = ReBotRSMITController(cfg)
    actual_speed = speed if speed is not None else cfg.default_speed_deg_s

    try:
        arm.start(enable_esc=True, install_signal_handlers=True)
        if speed is not None:
            arm.set_max_speeds([speed] * arm.motor_count)

        if angles_list is not None:
            if len(angles_list) != arm.motor_count:
                print(f"[错误] 必须提供 {arm.motor_count} 个角度，收到了 {len(angles_list)} 个。")
                arm.stop(return_to_zero=False, wait=True)
                return 1
            arm.set_joint_angles(angles_list)
            print(f"✓ 已下发关节目标: {[round(x, 2) for x in angles_list]}° (限速 {actual_speed}°/s)")
        elif joint is not None and angle is not None:
            if not 1 <= joint <= arm.motor_count:
                print(f"[错误] 关节编号无效: 1~{arm.motor_count}")
                arm.stop(return_to_zero=False, wait=True)
                return 1
            arm.set_joint_angle(joint, angle)
            print(f"✓ 关节 J{joint} 目标设置为 {angle:.2f}° (限速 {actual_speed}°/s)")
        else:
            print("[错误] 必须指定 --joint 与 --angle，或指定 --angles。")
            arm.stop(return_to_zero=False, wait=True)
            return 1

        if wait:
            print("等待运动到位中 (Ctrl+C 中断)...")
            while not arm.is_stopped:
                targets = arm.get_target_angles()
                commands = arm.get_command_angles()
                if max(abs(t - c) for t, c in zip(targets, commands)) <= 0.5:
                    print("✓ 运动到位！")
                    break
                time.sleep(0.05)
        else:
            # 至少等待一个控制周期更新
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n用户中断。")
    finally:
        arm.stop(return_to_zero=zero_on_exit, wait=True)
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="reBot Arm B601-RS 机械臂逐关节控制 CLI 工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  rebot-cli                            # 进入交互式控制台 (REPL 逐关节微调与控制)
  rebot-cli watch                      # 进入手动拖动示教模式，实时打印所有关节角度 (Ctrl+C 退出)
  rebot-cli watch --joint 1            # 聚焦监控 1 号关节手转角度与变化量
  rebot-cli status                     # 快速读取并打印当前 6 轴机械角度 (不移动)
  rebot-cli move --joint 1 --angle 25  # 将 J1 移动到 25°
  rebot-cli move --angles 15 0 0 0 0 0 # 同时下发 6 轴目标
  rebot-cli zero                       # 安全平滑回归零位
        """,
    )
    parser.add_argument("--config", "-c", default=None, help="指定 YAML 配置文件 (默认: config/rebotarm_rs.yaml)")
    parser.add_argument("--speed", "-s", type=float, default=None, help="默认运动速度上限 (°/s，未指定时读取配置文件)")

    subparsers = parser.add_subparsers(dest="subcommand", help="子命令")

    # interactive
    subparsers.add_parser("interactive", help="启动交互式 REPL 控制台 (默认)")

    # status
    subparsers.add_parser("status", help="读取当前所有关节角度与状态并退出 (不移动机械臂)")

    # watch / drag
    watch_parser = subparsers.add_parser("watch", aliases=["drag", "read"], help="手动拖动实时监控模式 (失能电机并实时高频输出关节角度)")
    watch_parser.add_argument("--joint", "-j", type=int, default=None, help="仅监视特定关节编号 (1~6)")
    watch_parser.add_argument("--hz", type=float, default=20.0, help="刷新频率 (Hz，默认: 20)")

    # zero
    subparsers.add_parser("zero", help="使能机械臂并平滑回到全 0° 零点")

    # move
    move_parser = subparsers.add_parser("move", help="单次命令模式移动关节")
    move_parser.add_argument("--joint", "-j", type=int, help="目标关节序号 (1~6)")
    move_parser.add_argument("--angle", "-a", type=float, help="目标角度 (度)")
    move_parser.add_argument("--angles", nargs="+", type=float, help="同时设置所有关节目标角度列表 (例如 --angles 20 0 0 0 0 0)")
    move_parser.add_argument("--wait", action="store_true", help="阻塞等待平滑运动到位后再退出")
    move_parser.add_argument("--hold", action="store_true", help="运动后不回零，保持在目标角度并失能 (默认退出时会自动回零)")

    args = parser.parse_args()

    if args.subcommand is None or args.subcommand == "interactive":
        sys.exit(run_interactive(args.config, default_speed=args.speed))

    elif args.subcommand == "status":
        sys.exit(run_status_oneshot(args.config))

    elif args.subcommand in ("watch", "drag", "read"):
        sys.exit(run_watch_oneshot(args.config, joint=args.joint, hz=args.hz))

    elif args.subcommand == "zero":
        sys.exit(run_move_oneshot(args.config, None, None, [0.0] * 6, args.speed, wait=True, zero_on_exit=True))

    elif args.subcommand == "move":
        sys.exit(run_move_oneshot(
            config_path=args.config,
            joint=args.joint,
            angle=args.angle,
            angles_list=args.angles,
            speed=args.speed,
            wait=args.wait,
            zero_on_exit=(not args.hold),
        ))


if __name__ == "__main__":
    main()
