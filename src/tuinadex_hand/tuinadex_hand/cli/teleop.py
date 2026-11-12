#!/usr/bin/env python3
"""leap-teleop — 16-DOF LEAP Hand 视觉遥操作系统 (硬件驱动复用 + 仿真算法重定向).

核心特性:
1. 复用原项目底层驱动与控制层 (LeapNode / DynamixelClient, 150mA 硬限流保护);
2. 采用 TuinaDex v2.0 仿真验证的关键点检测与遥操重定向 (MediaPipe HandLandmarker + JointMapper + 1€ 滤波);
3. 严格遵循黄金不变量契约: SI 弧度 (rad), 全开位为 0 rad 基准, 屈曲弯曲为正 (q > 0);
4. 交互式 HUD 仪表盘: 实时展示骨骼追踪、16 自由度目标与实测弧度、离合状态、手势锁定;
5. 优雅支持全流程 Mock 仿真模式 (支持脱机自测与 CI 验证).
"""
from __future__ import annotations

import argparse
import logging
import os
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# 源码工作区路径自适应导入
_repo_root = Path(__file__).resolve().parents[4]
for _p in ("src/tuinadex_core", "src/tuinadex_hand", "src/tuinadex_teleop"):
    _full = str(_repo_root / _p)
    if _full not in sys.path:
        sys.path.insert(0, _full)

from tuinadex_core.names import CANONICAL_JOINT_NAMES
from tuinadex_hand.core.hand_controller import LeapNode, JOINT_DIR
from tuinadex_hand.core.kinematics_limits import angle_safety_clip
from tuinadex_hand.core.pose_manager import load_poses, is_valid_pose

from tuinadex_teleop.tracking import HandTracker, JointMapper, OneEuroFilter
from tuinadex_teleop.tracking.teleop_logic import handedness_ok, regained_after_gap

logger = logging.getLogger("leap-teleop")

_FINGER_GROUPS = [
    ("食指 Index", 0),
    ("中指 Middle", 4),
    ("无名指 Ring", 8),
    ("拇指 Thumb", 12),
]
_JOINT_NAMES = ["侧摆", "MCP", "PIP", "DIP"]
_THUMB_NAMES = ["MCP", "侧摆", "PIP", "DIP"]


def draw_teleop_hud(
    frame: np.ndarray,
    target_angles: Optional[np.ndarray],
    actual_pos: Optional[np.ndarray],
    open_pose: np.ndarray,
    is_paused: bool,
    is_locked: bool,
    hand_seen: bool,
    fps: float,
    mock_mode: bool,
) -> np.ndarray:
    """在视频帧上绘制高信息密度、半透明的遥操调试 HUD 仪表盘."""
    h, w = frame.shape[:2]
    canvas = frame.copy()

    # 右侧半透明信息面板 (宽 380px)
    panel_w = min(420, w - 10)
    panel_h = min(460, h - 20)
    x1, y1 = w - panel_w - 10, 10
    x2, y2 = w - 10, 10 + panel_h

    overlay = canvas.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (20, 24, 28), -1)
    cv2.addWeighted(overlay, 0.72, canvas, 0.28, 0, canvas)

    # 1. 标题与状态栏
    status_text = "PAUSED ⏸" if is_paused else ("LOCKED 🔒" if is_locked else ("TRACKING 🟢" if hand_seen else "NO HAND ⚪"))
    status_color = (0, 220, 255) if is_paused else ((0, 165, 255) if is_locked else ((0, 240, 0) if hand_seen else (160, 160, 160)))
    
    cv2.putText(canvas, "TuinaDex LEAP Teleop", (x1 + 15, y1 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(canvas, f"STATUS: {status_text}", (x1 + 15, y1 + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.52, status_color, 2)
    
    mode_str = "MOCK SIMULATION" if mock_mode else "HARDWARE CONNECTED"
    mode_color = (180, 220, 120) if mock_mode else (0, 255, 128)
    cv2.putText(canvas, f"Mode: {mode_str} | {fps:.1f} FPS", (x1 + 15, y1 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.40, mode_color, 1)

    # 2. 16 关节相对弧度与实测状态表
    cv2.putText(canvas, f"{'手指':<8} {'侧摆':>6} {'MCP':>6} {'PIP':>6} {'DIP':>6}", (x1 + 15, y1 + 105), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)
    cv2.line(canvas, (x1 + 15, y1 + 112), (x2 - 15, y1 + 112), (70, 70, 70), 1)

    row_y = y1 + 130
    for gname, s_idx in _FINGER_GROUPS:
        if target_angles is not None:
            t = target_angles[s_idx : s_idx + 4]
            t_str = f"{t[0]:+5.2f} {t[1]:+5.2f} {t[2]:+5.2f} {t[3]:+5.2f}"
        else:
            t_str = "  --    --    --    --"
        cv2.putText(canvas, f"{gname:<10} {t_str}", (x1 + 15, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1)
        row_y += 24

    # 3. 舵机绝对位置与全开基准对比
    cv2.line(canvas, (x1 + 15, row_y), (x2 - 15, row_y), (70, 70, 70), 1)
    row_y += 18
    cv2.putText(canvas, "Motor Actual Offset from Open (rad):", (x1 + 15, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 200, 240), 1)
    row_y += 18

    if actual_pos is not None:
        delta = (actual_pos - open_pose) * JOINT_DIR
        for fi, (gname, s_idx) in enumerate(_FINGER_GROUPS):
            d = delta[s_idx : s_idx + 4]
            d_str = f"{d[0]:+5.2f} {d[1]:+5.2f} {d[2]:+5.2f} {d[3]:+5.2f}"
            cv2.putText(canvas, f"{gname[:2]:<4} act: {d_str}", (x1 + 15, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 220, 140), 1)
            row_y += 20
    else:
        cv2.putText(canvas, "  (Motor feedback waiting...)", (x1 + 15, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 120), 1)
        row_y += 20

    # 4. 底部快捷键指南
    cv2.line(canvas, (x1 + 15, y2 - 65), (x2 - 15, y2 - 65), (70, 70, 70), 1)
    cv2.putText(canvas, "Hotkeys:", (x1 + 15, y2 - 48), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 200, 0), 1)
    cv2.putText(canvas, "SPACE: Clutch  |  L: Lock Gesture", (x1 + 15, y2 - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1)
    cv2.putText(canvas, "O: Relax-to-OPEN  |  Q/ESC: Quit", (x1 + 15, y2 - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1)

    return canvas


def main(argv=None):
    parser = argparse.ArgumentParser(description="TuinaDex LEAP Hand Real-Time Teleoperation CLI (leap-teleop)")
    parser.add_argument("--port", type=str, default=None, help="Serial port for LEAP Hand (e.g. /dev/ttyUSB0)")
    parser.add_argument("--camera-id", type=int, default=0, help="Camera device index (default 0)")
    parser.add_argument("--rate", type=float, default=50.0, help="Control loop rate in Hz (default 50.0)")
    parser.add_argument("--filter-cutoff", type=float, default=1.0, help="1€ filter min cutoff (default 1.0)")
    parser.add_argument("--filter-beta", type=float, default=0.05, help="1€ filter beta (default 0.05)")
    parser.add_argument("--handedness", choices=["any", "Right", "Left"], default="Right", help="Filter handedness (default Right)")
    parser.add_argument("--no-preview", action="store_true", help="Headless mode (no OpenCV GUI)")
    parser.add_argument("--mock", action="store_true", help="Mock hardware LEAP Hand")
    parser.add_argument("--mock-camera", action="store_true", help="Mock camera input (generates synthetic frames)")
    parser.add_argument("--publish-ros2", action="store_true", help="Also publish /hand/joint_command via ROS2 if rclpy available")
    parser.add_argument("--max-frames", type=int, default=0, help="Exit automatically after N frames (useful for testing)")
    args = parser.parse_args(argv)

    print("\n" + "=" * 65)
    print("      TuinaDex v2.0 — 16-DOF LEAP Hand 视觉遥操作系统")
    print("      (底层驱动与控制层复用 + 仿真算法重定向)")
    print("=" * 65)

    # 1. 初始化 LEAP Hand 控制层 (LeapNode)
    try:
        leap = LeapNode(port=args.port, mock=args.mock)
        print(f"[LEAP] 灵巧手控制层已就绪 (Mock={args.mock}, Port={args.port or 'auto'}, CurrLimit={leap.curr_lim}mA)")
    except Exception as e:
        print(f"[错误] 无法初始化 LEAP Hand 硬件: {e}")
        if not args.mock:
            print("[提示] 若当前尚未连接实体手，可追加 --mock 参数进行脱机模拟调试。")
        return 1

    # 2. 初始化视觉追踪 (MediaPipe HandTracker) 与重定向映射器 (JointMapper)
    try:
        tracker = HandTracker(max_num_hands=1)
        mapper = JointMapper()
        one_euro_filter = OneEuroFilter(n_joints=16, min_cutoff=args.filter_cutoff, beta=args.filter_beta)
        print("[Vision] 关键点检测器 (HandLandmarker) 与 16-DOF 重定向映射器初始化成功。")
    except Exception as e:
        print(f"[错误] 无法初始化视觉追踪组件: {e}")
        leap.disconnect()
        return 1

    # 3. 可选初始化 ROS2 发布器
    ros2_node = None
    ros2_pub = None
    if args.publish_ros2:
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            if not rclpy.ok():
                rclpy.init()
            ros2_node = rclpy.create_node("leap_teleop_publisher")
            ros2_pub = ros2_node.create_publisher(JointState, "/hand/joint_command", 10)
            print("[ROS2] 已绑定并广播 /hand/joint_command 话题。")
        except Exception as e:
            print(f"[警告] ROS2 初始化失败，将仅使用直接硬件控制: {e}")

    # 4. 初始化视频捕获设备
    cap = None
    if not args.mock_camera:
        cap = cv2.VideoCapture(args.camera_id)
        if not cap.isOpened():
            print(f"[警告] 无法打开物理摄像头 ID={args.camera_id}，自动回退到模拟相机模式。")
            args.mock_camera = True
        else:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)

    # 状态量 (初始默认暂停，给操作者就绪准备时间)
    is_paused = True
    is_locked = False
    locked_angles = np.zeros(16, dtype=np.float64)
    target_angles: Optional[np.ndarray] = None
    hand_seen = False
    frame_count = 0
    t_start = time.time()
    last_fps_calc = time.time()
    fps = 0.0

    print("\n[系统就绪] 开始遥操主循环 (控制频率: {:.1f}Hz)...".format(args.rate))
    print("快捷键说明: [SPACE] 离合暂停/继续 | [L] 锁定当前手势 | [O] 全开复位 | [Q/ESC] 退出\n")

    try:
        while True:
            loop_t0 = time.time()

            # 4.1 获取图像帧
            if args.mock_camera:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "MOCK CAMERA (Synthetic Frame)", (40, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 2)
                # 绘制模拟动态小手圆点
                cx = int(320 + 80 * np.sin(frame_count * 0.05))
                cy = int(240 + 40 * np.cos(frame_count * 0.05))
                cv2.circle(frame, (cx, cy), 15, (0, 200, 255), -1)
                results = []
            else:
                ret, frame = cap.read()
                if not ret or frame is None:
                    time.sleep(0.01)
                    continue
                # MediaPipe 视觉推理
                results = tracker.detect(frame)

            frame_count += 1

            # 4.2 计算手部检测与重定向
            seen = bool(results) and handedness_ok(results[0].handedness, args.handedness)
            if regained_after_gap(hand_seen, seen):
                one_euro_filter.reset()
            hand_seen = seen

            if seen:
                hand = results[0]
                raw_angles = mapper.map_keypoints_to_leap(hand)
                filtered_angles = one_euro_filter(raw_angles)
            else:
                filtered_angles = None

            # 4.3 离合与手势锁定决策
            if is_locked:
                target_angles = locked_angles.copy()
            elif is_paused:
                target_angles = None
            else:
                target_angles = filtered_angles

            # 4.4 驱动灵巧手物理舵机 (或 Mock Client)
            if target_angles is not None:
                # 转换到舵机绝对硬件弧度: open_pose + joint_dir * target_angles
                target_pos = leap.open_pose + JOINT_DIR * target_angles
                clipped_pos = angle_safety_clip(target_pos)
                leap.set_leap(clipped_pos)

                # 可选同步发布 ROS2 指令
                if ros2_pub is not None and ros2_node is not None:
                    try:
                        from sensor_msgs.msg import JointState
                        msg = JointState()
                        msg.header.stamp = ros2_node.get_clock().now().to_msg()
                        msg.name = list(CANONICAL_JOINT_NAMES)
                        msg.position = target_angles.tolist()
                        ros2_pub.publish(msg)
                    except Exception:
                        pass

            # 4.5 读取物理舵机当前状态 (用于反馈显示)
            try:
                actual_pos = leap.read_pos()
            except Exception:
                actual_pos = None

            # 4.6 帧率统计
            now = time.time()
            if now - last_fps_calc >= 0.5:
                fps = frame_count / (now - t_start)
                last_fps_calc = now

            # 4.7 绘制 HUD 仪表盘与窗口交互
            if not args.no_preview:
                annotated = tracker.draw_landmarks(frame, results) if results else frame
                annotated = draw_teleop_hud(
                    annotated,
                    target_angles=target_angles,
                    actual_pos=actual_pos,
                    open_pose=leap.open_pose,
                    is_paused=is_paused,
                    is_locked=is_locked,
                    hand_seen=hand_seen,
                    fps=fps,
                    mock_mode=args.mock,
                )
                cv2.imshow("TuinaDex LEAP Teleop", annotated)

                key = cv2.waitKey(1) & 0xFF
                if key in (32, ord('c'), ord('C')):  # SPACE / C 离合暂停/继续
                    is_paused = not is_paused
                    print(f"[离合] {'暂停输出 ⏸' if is_paused else '恢复遥操 ▶'}")
                elif key in (ord('l'), ord('L')):  # L 锁定当前手势
                    is_locked = not is_locked
                    if is_locked and target_angles is not None:
                        locked_angles = target_angles.copy()
                    print(f"[锁定] {'当前手势已锁定 🔒' if is_locked else '手势锁定已解除 🔓'}")
                elif key in (ord('o'), ord('O')):  # O 回到全开基准位
                    leap.set_open()
                    print("[复位] 灵巧手平滑回退至全开平伸位姿 (OPEN_POSE).")
                elif key in (ord('q'), ord('Q'), 27):  # Q / ESC 退出
                    print("\n[退出] 收到退出指令，安全停止中...")
                    break

            # 自动化测试最大帧数控制
            if args.max_frames > 0 and frame_count >= args.max_frames:
                print(f"[测试] 已达到指定运行帧数 ({args.max_frames} 帧)，安全退出。")
                break

            # 控制循环速率自适应睡眠
            elapsed = time.time() - loop_t0
            period = 1.0 / args.rate
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print("\n[中断] 用户按下 Ctrl+C，安全退出中...")
    finally:
        # 安全退出收尾: 回到全开位 -> 释放舵机扭矩 -> 断开连接
        print("[安全收尾] 舵机平滑复位并断开通信...")
        try:
            leap.set_open()
            time.sleep(0.2)
            leap.set_torque(False)
            leap.disconnect()
        except Exception:
            pass

        if cap is not None and cap.isOpened():
            cap.release()
        try:
            tracker.close()
        except Exception:
            pass
        if not args.no_preview:
            cv2.destroyAllWindows()
        if ros2_node is not None:
            try:
                import rclpy
                ros2_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

    print("[系统结束] leap-teleop 已安全退出。\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

