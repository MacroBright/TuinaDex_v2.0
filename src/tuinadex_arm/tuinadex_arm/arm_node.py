"""arm_node.py — reBot B601-RS 6-DOF 机械臂 ROS2 运行时主控节点.

遵循 TuinaDex 系统分层与设计模式 (与 leap_hand_node 严格对称):
  1. 50Hz 定时读取硬件并发布 /arm/joint_states (sensor_msgs/JointState);
  2. 订阅 /arm/joint_command, 经 align_arm_command 严格校验限位后缓存;
  3. WatchdogStateMachine 裁决: 300ms 指令断流 -> 驱动层 Relax-to-HOME;
  4. 全链路 SI 国际标准单位 (rad), 统一节点时钟计时;
  5. 提供 /arm/enable, /arm/disable, /arm/safe_home 标准触发服务.
"""
from __future__ import annotations

import time
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from tuinadex_core.command_processing import align_arm_command
from tuinadex_core.names import CANONICAL_ARM_JOINT_NAMES
from tuinadex_core.watchdog import WatchdogAction, WatchdogStateMachine
from tuinadex_arm.driver.arm_driver import ArmDriver


class ArmNode(Node):
    """6-DOF reBot B601-RS 机械臂 ROS2 运行时主节点."""

    def __init__(self, parameter_overrides: Optional[dict] = None) -> None:
        super().__init__("arm_node")
        overrides = parameter_overrides or {}

        def declare(name: str, value):
            self.declare_parameter(name, overrides.get(name, value))

        declare("channel", "can0")
        declare("bitrate", 1000000)
        declare("host_id", 253)
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
        self.mock_mode = bool(gp("mock_mode"))

        self.driver = ArmDriver(
            channel=str(gp("channel")),
            bitrate=int(gp("bitrate")),
            host_id=int(gp("host_id")),
            control_rate_hz=self.rate_hz,
            mock_mode=self.mock_mode,
        )

        if not self.driver.connect():
            self.get_logger().error(f"无法连接机械臂 (channel={gp('channel')})，节点退出。")
            raise SystemExit("Arm connection failed")

        # 话题契约：与 /hand/joint_* 保持绝对对称
        self.state_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.cmd_sub = self.create_subscription(
            JointState, "/arm/joint_command", self._on_joint_command, 10
        )

        # 服务接口
        self.enable_srv = self.create_service(Trigger, "/arm/enable", self._on_enable)
        self.disable_srv = self.create_service(Trigger, "/arm/disable", self._on_disable)
        self.home_srv = self.create_service(Trigger, "/arm/safe_home", self._on_safe_home)

        self._latest_command: Optional[np.ndarray] = None
        self._fsm = WatchdogStateMachine(timeout_sec=float(gp("watchdog_timeout_sec")))
        self._relax_logged = False
        self.timer = self.create_timer(1.0 / self.rate_hz, self._control_loop)

        self.get_logger().info(
            f"TuinaDex 机械臂主控节点启动完成 [mock_mode={self.mock_mode}, rate={self.rate_hz}Hz]"
        )

    def _now(self) -> float:
        """统一时钟源 (秒)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joint_command(self, msg: JointState) -> None:
        names = list(msg.name) if msg.name else None
        q, reason = align_arm_command(msg.position, names)
        if q is None:
            self.get_logger().warning(f"机械臂指令被拒绝 ({reason})，已丢弃。")
            return
        was_relaxing = self._fsm.state.name == "RELAXING"
        self._fsm.on_command(self._now())
        if was_relaxing:
            self.driver.reset_loss_state()
            self._relax_logged = False
            self.get_logger().info("收到新机械臂指令，看门狗已解除，恢复跟随。")
        self._latest_command = q

    def _on_enable(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = self.driver.connect()
        resp.message = "Arm enabled" if resp.success else "Failed to enable arm"
        return resp

    def _on_disable(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self.driver.disconnect()
        resp.success = True
        resp.message = "Arm disabled"
        return resp

    def _on_safe_home(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self.driver.set_home()
        resp.success = True
        resp.message = "Arm commanded to safe home"
        return resp

    def _control_loop(self) -> None:
        t = self._now()
        action = self._fsm.tick(t)

        if action is WatchdogAction.WRITE:
            try:
                self.driver.write_command(self._latest_command)
            except Exception as e:
                self.get_logger().error(f"写入机械臂指令异常: {e}")
        elif action is WatchdogAction.RELAX_STEP:
            if not self._relax_logged:
                self.get_logger().warning(
                    f"机械臂指令断流 > {self._fsm.timeout_sec * 1000:.0f}ms，"
                    "触发 Relax-to-HOME 安全回退！"
                )
                self._relax_logged = True
            try:
                self.driver.relax_to_open_step(
                    t, hold_time=self.hold_sec, ramp_time=self.ramp_sec
                )
            except Exception as e:
                self.get_logger().error(f"机械臂 Relax 步进异常: {e}")

        try:
            pos, vel, eff = self.driver.read_states()
        except Exception as e:
            self.get_logger().error(f"读取机械臂物理状态失败: {e}")
            return

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(CANONICAL_ARM_JOINT_NAMES)
        state_msg.position = pos.tolist()
        state_msg.velocity = vel.tolist()
        state_msg.effort = eff.tolist()
        self.state_pub.publish(state_msg)

    def destroy_node(self) -> None:
        self.get_logger().info("安全退出: 停止机械臂控制循环, 回零位, 等待到位后断开...")
        if self.timer is not None:
            self.timer.cancel()
        try:
            self.driver.set_home()
            time.sleep(self.dwell_sec)
            self.driver.disconnect()
        except Exception as e:
            self.get_logger().warning(f"机械臂关停清理异常: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmNode()
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

