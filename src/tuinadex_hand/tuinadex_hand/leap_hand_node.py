"""leap_hand_node.py — LEAP Hand 16-DOF ROS2 运行时主控节点.

遵循架构不变量:
1. 定时以 50Hz 读取硬件物理状态并发布 /hand/joint_states (sensor_msgs/JointState)；
2. 订阅 /hand/joint_command (sensor_msgs/JointState)，解耦指令缓存；
3. 严格执行 300ms 通信看门狗: 当指令中断或丢失时，自动触发平滑 Relax-to-OPEN；
4. 所有接口数值强制 SI 弧度制 (rad)，以标定全开位为 0 rad。
"""
from __future__ import annotations

import time
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from tuinadex_hand.driver.leap_driver import (
    LeapHandDriver,
    CANONICAL_JOINT_NAMES,
)


class LeapHandNode(Node):
    """16-DOF LEAP Hand ROS2 运行时主节点."""

    def __init__(self):
        super().__init__("leap_hand_node")

        # 声明节点参数
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 4000000)
        self.declare_parameter("kP", 300)
        self.declare_parameter("kI", 0)
        self.declare_parameter("kD", 100)
        self.declare_parameter("curr_lim", 150)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("watchdog_timeout_sec", 0.30)
        self.declare_parameter("mock_mode", False)

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        kP = self.get_parameter("kP").get_parameter_value().integer_value
        kI = self.get_parameter("kI").get_parameter_value().integer_value
        kD = self.get_parameter("kD").get_parameter_value().integer_value
        curr_lim = self.get_parameter("curr_lim").get_parameter_value().integer_value
        self.rate_hz = self.get_parameter("control_rate_hz").get_parameter_value().double_value
        self.watchdog_timeout = self.get_parameter("watchdog_timeout_sec").get_parameter_value().double_value
        mock_mode = self.get_parameter("mock_mode").get_parameter_value().bool_value

        # 初始化底层驱动
        self.driver = LeapHandDriver(
            port=port,
            baudrate=baudrate,
            kP=kP,
            kI=kI,
            kD=kD,
            curr_lim=curr_lim,
            mock_mode=mock_mode,
        )

        if not self.driver.connect():
            self.get_logger().error(f"无法连接 LEAP Hand (port={port})，节点退出。")
            raise SystemExit("LEAP Hand connection failed")

        self.get_logger().info(
            f"LEAP Hand 节点启动成功! [port={port}, baud={baudrate}, rate={self.rate_hz}Hz, mock={mock_mode}]"
        )

        # 话题发布与订阅
        self.state_pub = self.create_publisher(JointState, "/hand/joint_states", 10)
        self.cmd_sub = self.create_subscription(
            JointState, "/hand/joint_command", self._on_joint_command, 10
        )

        # 指令缓存与看门狗计时
        self._latest_command: Optional[np.ndarray] = None
        self._last_cmd_time: float = 0.0
        self._watchdog_active = False

        # 启动 50Hz 控制主循环定时器
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self._control_loop)

    def _on_joint_command(self, msg: JointState) -> None:
        """异步接收上层目标指令 (无阻塞缓存)."""
        now = time.time()
        pos = np.asarray(msg.position, dtype=np.float64)

        if len(pos) != 16:
            self.get_logger().warn(
                f"收到无效指令维度: 预期 16 轴，实际收到 {len(pos)} 轴，已丢弃。"
            )
            return

        # 如果提供了 joint name，按 Canonical Order 校验或对齐
        if msg.name and len(msg.name) == 16:
            if list(msg.name) != CANONICAL_JOINT_NAMES:
                # 按名称重排为标准 Canonical 顺序
                name_to_idx = {n: i for i, n in enumerate(CANONICAL_JOINT_NAMES)}
                ordered_pos = np.zeros(16, dtype=np.float64)
                for src_idx, name in enumerate(msg.name):
                    if name in name_to_idx:
                        ordered_pos[name_to_idx[name]] = pos[src_idx]
                pos = ordered_pos

        self._latest_command = pos
        self._last_cmd_time = now
        if self._watchdog_active:
            self._watchdog_active = False
            self.driver.reset_loss_state()
            self.get_logger().info("收到新指令，看门狗已解除，恢复跟随。")

    def _control_loop(self) -> None:
        """50Hz 定时核心控制环路."""
        now = time.time()

        # 1. 检查指令超时看门狗
        if self._latest_command is not None:
            if (now - self._last_cmd_time) > self.watchdog_timeout:
                if not self._watchdog_active:
                    self._watchdog_active = True
                    self.get_logger().warn(
                        f"指令超时 (> {self.watchdog_timeout * 1000:.0f}ms)，触发 Relax-to-OPEN 安全回退！"
                    )
                self.driver.relax_to_open_step(now)
            else:
                # 正常下发最新目标位置
                try:
                    self.driver.write_command(self._latest_command)
                except Exception as e:
                    self.get_logger().error(f"写入指令异常: {e}")

        # 2. 读取实测物理状态
        try:
            pos, vel, eff = self.driver.read_states()
        except Exception as e:
            self.get_logger().error(f"读取物理状态失败: {e}")
            return

        # 3. 发布标准 /hand/joint_states
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(CANONICAL_JOINT_NAMES)
        state_msg.position = pos.tolist()
        state_msg.velocity = vel.tolist()
        state_msg.effort = eff.tolist()

        self.state_pub.publish(state_msg)

    def destroy_node(self) -> None:
        """安全释放与断开."""
        self.get_logger().info("正在安全退出 LEAP Hand 节点并回全开位...")
        try:
            self.driver.set_open()
            self.driver.disconnect()
        except Exception:
            pass
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
