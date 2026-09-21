"""leap_hand_node.py — LEAP Hand 16-DOF ROS2 运行时主控节点.

架构 (Phase 2 裁决: 普通 Node + 显式 FSM):
  1. 50Hz 定时读硬件并发布 /hand/joint_states;
  2. 订阅 /hand/joint_command, 经 align_command 严格校验后缓存;
  3. WatchdogStateMachine 裁决: 300ms 指令断流 -> 驱动层 Relax-to-OPEN;
  4. 全链路 SI 弧度, 统一节点时钟计时 (禁 wall-clock)。
"""
from __future__ import annotations

import time
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from tuinadex_core.command_processing import align_command
from tuinadex_core.names import CANONICAL_JOINT_NAMES
from tuinadex_core.watchdog import WatchdogAction, WatchdogStateMachine
from tuinadex_hand.driver.leap_driver import LeapHandDriver


class LeapHandNode(Node):
    """16-DOF LEAP Hand ROS2 运行时主节点."""

    def __init__(self, parameter_overrides: Optional[dict] = None):
        super().__init__("leap_hand_node")
        overrides = parameter_overrides or {}

        def declare(name: str, value):
            self.declare_parameter(name, overrides.get(name, value))

        declare("port", "/dev/ttyUSB0")
        declare("baudrate", 4000000)
        declare("kP", 300)
        declare("kI", 0)
        declare("kD", 100)
        declare("curr_lim", 150)
        declare("control_rate_hz", 50.0)
        declare("watchdog_timeout_sec", 0.30)
        declare("watchdog_hold_sec", 0.30)
        declare("watchdog_ramp_sec", 0.60)
        declare("shutdown_dwell_sec", 0.5)
        declare("mock_mode", False)

        gp = lambda name: self.get_parameter(name).value
        self.rate_hz = float(gp("control_rate_hz"))
        self.hold_sec = float(gp("watchdog_hold_sec"))
        self.ramp_sec = float(gp("watchdog_ramp_sec"))
        self.dwell_sec = float(gp("shutdown_dwell_sec"))

        self.driver = LeapHandDriver(
            port=str(gp("port")), baudrate=int(gp("baudrate")),
            kP=int(gp("kP")), kI=int(gp("kI")), kD=int(gp("kD")),
            curr_lim=int(gp("curr_lim")), mock_mode=bool(gp("mock_mode")),
        )
        if not self.driver.connect():
            self.get_logger().error(f"无法连接 LEAP Hand (port={gp('port')})，节点退出。")
            raise SystemExit("LEAP Hand connection failed")

        self.state_pub = self.create_publisher(JointState, "/hand/joint_states", 10)
        self.cmd_sub = self.create_subscription(
            JointState, "/hand/joint_command", self._on_joint_command, 10)

        self._latest_command: Optional[np.ndarray] = None
        self._fsm = WatchdogStateMachine(timeout_sec=float(gp("watchdog_timeout_sec")))
        self._relax_logged = False
        self.timer = self.create_timer(1.0 / self.rate_hz, self._control_loop)

    def _now(self) -> float:
        """节点时钟秒值 (统一时间源, 禁 wall-clock)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joint_command(self, msg: JointState) -> None:
        names = list(msg.name) if msg.name else None
        q, reason = align_command(msg.position, names)
        if q is None:
            self.get_logger().warning(f"指令被拒绝 ({reason})，已丢弃。")
            return
        was_relaxing = self._fsm.state.name == "RELAXING"
        self._fsm.on_command(self._now())
        if was_relaxing:
            self.driver.reset_loss_state()
            self._relax_logged = False
            self.get_logger().info("收到新指令，看门狗已解除，恢复跟随。")
        self._latest_command = q

    def _control_loop(self) -> None:
        t = self._now()
        action = self._fsm.tick(t)

        if action is WatchdogAction.WRITE:
            try:
                self.driver.write_command(self._latest_command)
            except Exception as e:
                self.get_logger().error(f"写入指令异常: {e}")
        elif action is WatchdogAction.RELAX_STEP:
            if not self._relax_logged:
                self.get_logger().warning(
                    f"指令断流 > {self._fsm.timeout_sec * 1000:.0f}ms，"
                    "触发 Relax-to-OPEN 安全回退！")
                self._relax_logged = True
            try:
                self.driver.relax_to_open_step(t, hold_time=self.hold_sec,
                                               ramp_time=self.ramp_sec)
            except Exception as e:
                self.get_logger().error(f"Relax 步进异常: {e}")

        try:
            pos, vel, eff = self.driver.read_states()
        except Exception as e:
            self.get_logger().error(f"读取物理状态失败: {e}")
            return

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(CANONICAL_JOINT_NAMES)
        state_msg.position = pos.tolist()
        state_msg.velocity = vel.tolist()
        state_msg.effort = eff.tolist()
        self.state_pub.publish(state_msg)

    def destroy_node(self) -> None:
        """安全关停序列: 停定时器 -> 全开指令 -> dwell 等待到位 -> 断开."""
        self.get_logger().info("安全退出: 停止控制循环, 回全开位, 等待到位后断开...")
        if self.timer is not None:
            self.timer.cancel()
        try:
            self.driver.set_open()
            time.sleep(self.dwell_sec)   # 让电机实际走到全开位
            self.driver.disconnect()
        except Exception as e:
            self.get_logger().warning(f"关停清理异常: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LeapHandNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
