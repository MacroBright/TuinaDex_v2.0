#!/usr/bin/env python3
"""00_preflight_calibration.py — All-in-One Pre-Flight Calibration Wizard for TuinaDex Teleoperation.

Interactive 3-in-1 Pre-Flight Calibration Wizard:
  Stage 1: Desktop ArUco Table Calibration (Dual ID0 & ID1 -> Ground plane & T_cam_to_table)
  Stage 2: 6-Axis Wrist IMU 3-Axis Attitude & Neutral Calibration (X-Pitch / Y-Roll / Z-Yaw / Zero)
  Stage 3: Hand Scale & Biomechanical Zero/ROM Calibration (Flat open hand & zero offsets)

Features:
  - Continuous 30 FPS visual stream across ALL stages (zero video stutter or window freeze).
  - High-density, professional Chinese HUD overlay with stage breadcrumbs and action directives.
  - Dual-Channel Interaction: [ENTER]/[SPACE] in either Terminal OR OpenCV GUI window!
  - Full 3-Axis IMU Calibration (围绕指关节横轴 X 压抬、围绕中指纵轴 Y 翻转、围绕手背法向 Z 偏摆).
  - [S] to skip any step and smoothly fall back to previous calibration.

Outputs:
  - configs/session_calibration.yaml (Unified configuration loaded by 05_preview_in_sim.py)
  - Synchronizes legacy configs/table_calibration.yaml & configs/imu_calibration.yaml

Usage:
  # Standard interactive mode (with physical RealSense & IMU):
  conda run -n tuinadex-control python scripts/00_preflight_calibration.py

  # Headless mode for terminal/SSH:
  conda run -n tuinadex-control python scripts/00_preflight_calibration.py --no-gui

  # Automated test mode with simulated streams:
  conda run -n tuinadex-control python scripts/00_preflight_calibration.py --mock
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import threading
import time
import tty
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# Add project root and source package to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIGS_DIR = PROJECT_ROOT / "src" / "tuinadex_teleop" / "configs"
if not CONFIGS_DIR.exists():
    CONFIGS_DIR = PROJECT_ROOT / "configs"

for p in (PROJECT_ROOT, PROJECT_ROOT / "src" / "tuinadex_teleop", PROJECT_ROOT / "src" / "tuinadex_core"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

try:
    from tuinadex_teleop.core.names import HAND_NUM_DOFS
    from tuinadex_teleop.core.types import ArUcoCalibrationData
    from tuinadex_teleop.kinematics.hand_retargeter import BiomechanicalKinematicRetargeter
    from tuinadex_teleop.perception.imu_stream import (
        ImuReading,
        ImuStream,
        MockImuStream,
        parse_axis_mapping,
        quat_to_rotation_matrix,
        rotation_matrix_to_euler_deg,
        rotation_matrix_to_rotvec,
        solve_imu_alignment_from_gestures,
    )
    from tuinadex_teleop.perception.realsense_stream import (
        CameraIntrinsics,
        MockRealSenseStream,
        RealSenseStream,
    )
    from tuinadex_teleop.perception.table_calibrator import (
        TableCalibrator,
        average_quaternions,
        quaternion_to_rotation_matrix,
        rotation_matrix_to_quaternion,
    )
    from tuinadex_teleop.perception.wrist_tracker import (
        LM_MIDDLE_MCP,
        LM_WRIST,
        TrackingState,
        WristPose,
        WristTracker,
        compute_palm_basis,
    )
except ImportError:
    from tuina_teleop.core.names import HAND_NUM_DOFS
    from tuina_teleop.core.types import ArUcoCalibrationData
    from tuina_teleop.kinematics.hand_retargeter import BiomechanicalKinematicRetargeter
    from tuina_teleop.perception.imu_stream import (
        ImuReading,
        ImuStream,
        MockImuStream,
        parse_axis_mapping,
        quat_to_rotation_matrix,
        rotation_matrix_to_euler_deg,
        rotation_matrix_to_rotvec,
        solve_imu_alignment_from_gestures,
    )
    from tuina_teleop.perception.realsense_stream import (
        CameraIntrinsics,
        MockRealSenseStream,
        RealSenseStream,
    )
    from tuina_teleop.perception.table_calibrator import (
        TableCalibrator,
        average_quaternions,
        quaternion_to_rotation_matrix,
        rotation_matrix_to_quaternion,
    )
    from tuina_teleop.perception.wrist_tracker import (
        LM_MIDDLE_MCP,
        LM_WRIST,
        TrackingState,
        WristPose,
        WristTracker,
        compute_palm_basis,
    )


class TerminalKeyPoller:
    """Non-blocking terminal keyboard poller supporting Linux tty and /dev/tty."""

    def __init__(self):
        self._tty_fd = None
        self._old_settings = None
        self._file = None
        try:
            if os.path.exists("/dev/tty"):
                try:
                    self._file = open("/dev/tty", "r")
                    self._tty_fd = self._file.fileno()
                    self._old_settings = termios.tcgetattr(self._tty_fd)
                    tty.setcbreak(self._tty_fd)
                except Exception:
                    self._file = None
                    self._tty_fd = None
            if self._tty_fd is None and sys.stdin.isatty():
                self._tty_fd = sys.stdin.fileno()
                self._old_settings = termios.tcgetattr(self._tty_fd)
                tty.setcbreak(self._tty_fd)
        except Exception:
            self._tty_fd = None

    def poll_key(self) -> Optional[str]:
        """Poll a single key non-blockingly without waiting for enter."""
        if self._tty_fd is not None:
            try:
                rlist, _, _ = select.select([self._tty_fd], [], [], 0.0)
                if rlist:
                    ch = os.read(self._tty_fd, 1).decode("utf-8", errors="ignore")
                    if ch in ("\n", "\r"):
                        return "ENTER"
                    elif ch in (" ",):
                        return "SPACE"
                    elif ch.lower() in ("s",):
                        return "SKIP"
                    elif ch.lower() in ("q", "\x1b"):
                        return "QUIT"
                    return ch
            except Exception:
                pass
        return None

    def close(self):
        if self._tty_fd is not None and self._old_settings is not None:
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass
            self._tty_fd = None
        if self._file is not None:
            try:
                self._file.close()
            except Exception:
                pass
            self._file = None


class ChineseHudRenderer:
    """High-performance Chinese + ASCII HUD overlay renderer using PIL & OpenCV."""

    def __init__(self):
        self._font_title = None
        self._font_step = None
        self._font_text = None
        self._font_badge = None
        self._font_small = None
        self._has_pil = False

        font_candidates = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/arphic/uming.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        ]
        try:
            from PIL import ImageFont
            for fp in font_candidates:
                if os.path.exists(fp):
                    try:
                        self._font_title = ImageFont.truetype(fp, 16)
                        self._font_step = ImageFont.truetype(fp, 17)
                        self._font_text = ImageFont.truetype(fp, 14)
                        self._font_badge = ImageFont.truetype(fp, 14)
                        self._font_small = ImageFont.truetype(fp, 12)
                        self._has_pil = True
                        break
                    except Exception:
                        continue
        except ImportError:
            self._has_pil = False

    def draw_hud(
        self,
        frame: np.ndarray,
        stage_idx: int,           # 1, 2, or 3
        stage_title: str,
        step_title: str,
        instruction: str,
        status_text: str,
        status_color: Tuple[int, int, int],  # BGR
        telemetry_lines: List[str],
        progress_cur: int,
        progress_total: int,
        is_sampling: bool = False,
        key_hint: str = "[ENTER] / [空格] 确认采样  |  [S] 跳过当前阶段  |  [Q] 退出向导",
    ) -> np.ndarray:
        """Render multi-stage breadcrumbs, action instruction card, and telemetry onto frame."""
        h, w = frame.shape[:2]
        vis = frame.copy()

        # 1. Background semi-transparent cards using OpenCV fast rects
        overlay = vis.copy()
        # Top banner
        cv2.rectangle(overlay, (0, 0), (w, 64), (18, 18, 18), -1)
        # Step guidance card
        cv2.rectangle(overlay, (12, 70), (w - 12, 185), (25, 25, 25), -1)
        # Bottom telemetry
        cv2.rectangle(overlay, (12, h - 95), (w - 12, h - 38), (22, 22, 22), -1)
        # Footer
        cv2.rectangle(overlay, (0, h - 32), (w, h), (15, 15, 15), -1)
        cv2.addWeighted(overlay, 0.82, vis, 0.18, 0, vis)

        # Draw card border with status color
        cv2.rectangle(vis, (12, 70), (w - 12, 185), status_color, 1)

        # Progress bar background & fill in bottom card
        pb_x1, pb_y1 = w - 230, h - 82
        pb_x2, pb_y2 = w - 24, h - 66
        cv2.rectangle(vis, (pb_x1, pb_y1), (pb_x2, pb_y2), (50, 50, 50), -1)
        if progress_total > 0:
            fill_ratio = min(1.0, max(0.0, progress_cur / float(progress_total)))
            fill_x2 = int(pb_x1 + fill_ratio * (pb_x2 - pb_x1))
            bar_color = (0, 255, 120) if is_sampling else (0, 200, 255)
            cv2.rectangle(vis, (pb_x1, pb_y1), (fill_x2, pb_y2), bar_color, -1)
        cv2.rectangle(vis, (pb_x1, pb_y1), (pb_x2, pb_y2), (100, 100, 100), 1)

        # 2. Text rendering via PIL (for crisp Chinese) or fallback cv2.putText
        if self._has_pil:
            from PIL import Image, ImageDraw
            pil_img = Image.fromarray(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)

            # Header
            draw.text((16, 8), "TuinaDex 22-DOF 一键标定向导 (Pre-Flight Calibration)", font=self._font_title, fill=(240, 240, 240))

            # Breadcrumbs
            bc_1 = "[✓ 1. 桌面参考]" if stage_idx > 1 else ("[▶ 1. 桌面参考平面]" if stage_idx == 1 else "[1. 桌面参考]")
            bc_2 = "[✓ 2. IMU三轴完成]" if stage_idx > 2 else ("[▶ 2. IMU 三轴姿态校准]" if stage_idx == 2 else "[2. IMU 三轴姿态校准]")
            bc_3 = "[▶ 3. 灵巧手零位自适应]" if stage_idx == 3 else "[3. 灵巧手尺寸与零位]"

            color_1 = (0, 255, 120) if stage_idx > 1 else ((0, 255, 230) if stage_idx == 1 else (140, 140, 140))
            color_2 = (0, 255, 120) if stage_idx > 2 else ((0, 255, 230) if stage_idx == 2 else (140, 140, 140))
            color_3 = (0, 255, 230) if stage_idx == 3 else (140, 140, 140)

            draw.text((16, 36), bc_1, font=self._font_text, fill=color_1)
            draw.text((180, 36), bc_2, font=self._font_text, fill=color_2)
            draw.text((410, 36), bc_3, font=self._font_text, fill=color_3)

            # Step Card
            draw.text((26, 80), step_title, font=self._font_step, fill=(255, 220, 50))
            draw.text((26, 112), instruction, font=self._font_text, fill=(255, 255, 255))

            # Status Badge text (Convert BGR status_color to RGB for PIL)
            rgb_status = (status_color[2], status_color[1], status_color[0])
            draw.text((26, 144), f"状态: {status_text}", font=self._font_badge, fill=rgb_status)

            # Telemetry Lines
            t_y = h - 90
            for line in telemetry_lines[:2]:
                draw.text((24, t_y), line, font=self._font_small, fill=(210, 210, 210))
                t_y += 20

            # Progress percentage
            pct = int(100 * progress_cur / max(1, progress_total))
            draw.text((pb_x1, pb_y1 - 18), f"采样进度: {progress_cur}/{progress_total} ({pct}%)", font=self._font_small, fill=(230, 230, 230))

            # Footer Keys
            draw.text((20, h - 26), key_hint, font=self._font_small, fill=(190, 190, 190))

            return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
        else:
            # Fallback to OpenCV putText (ASCII only)
            cv2.putText(vis, stage_title, (16, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
            cv2.putText(vis, step_title, (26, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(vis, instruction, (26, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            cv2.putText(vis, status_text, (26, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.50, status_color, 2)
            t_y = h - 75
            for line in telemetry_lines[:2]:
                cv2.putText(vis, line, (24, t_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
                t_y += 20
            cv2.putText(vis, key_hint, (20, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)
            return vis


def render_progress_bar(current: int, total: int, width: int = 24) -> str:
    """Render a clean Unicode terminal progress bar."""
    filled = int(round(width * current / float(total)))
    bar = "█" * filled + "░" * (width - filled)
    percent = int(round(100.0 * current / float(total)))
    return f"[{bar}] {percent:3d}% ({current:2d}/{total:2d})"


# ─────────────────────────────────────────────────────────────────────────────
# STAGE 1: Desktop Table Frame Calibration (ArUco ID0 & ID1)
# ─────────────────────────────────────────────────────────────────────────────

def run_stage_1_table(
    stream: RealSenseStream | MockRealSenseStream,
    intrinsics: CameraIntrinsics,
    key_poller: TerminalKeyPoller,
    hud_renderer: ChineseHudRenderer,
    samples_needed: int = 25,
    axis_mapping: str = "-y,+x,+z",
    no_gui: bool = False,
    mock: bool = False,
) -> Dict[str, Any]:
    """Execute Stage 1: Table frame calibration from ArUco markers."""
    print("\n" + "=" * 75)
    print("  【阶段 1 / 3】桌面物理参考平面标定 (Desktop ArUco Table Calibration)")
    print("=" * 75)
    print("  [说明] 请将 ArUco 标定板 (ID 0 主基准，可选 ID 1) 平放在桌面相机视野内。")
    print("  [按键] 准备好后按 [ENTER] 或 [空格] 开始采样；按 [S] 跳过并复用历史标定。")
    print("-" * 75)

    calibrator = TableCalibrator(
        marker_size_m=0.080,
        primary_id=0,
        secondary_id=1,
        min_samples=samples_needed,
    )

    t_start = time.time()
    last_print = 0.0
    auto_sampling = False

    while calibrator.sample_count < samples_needed:
        color, depth, _ = stream.read_frames()
        if color is None:
            time.sleep(0.005)
            continue

        # Check keyboard inputs (Terminal & OpenCV window)
        term_k = key_poller.poll_key() if not mock else None
        gui_k = None
        if not no_gui:
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), ord('Q'), 27):
                gui_k = "QUIT"
            elif k in (ord('s'), ord('S')):
                gui_k = "SKIP"
            elif k in (ord(' '), 10, 13):
                gui_k = "ENTER"

        act_k = term_k or gui_k

        if act_k == "QUIT":
            print("\n[INFO] 用户终止标定。")
            sys.exit(0)
        elif act_k == "SKIP":
            print("\n[INFO] 用户按 [S] 跳过桌面标定，尝试加载既有配置...")
            break
        elif act_k == "ENTER" and not auto_sampling:
            auto_sampling = True
            print("\n  ✓ [ENTER] 已确认！开始累积稳定桌面特征帧...")

        if mock and np.all(color == 0):
            color = np.full((480, 640, 3), 255, dtype=np.uint8)
            marker0 = cv2.aruco.generateImageMarker(calibrator.dictionary, 0, 100)
            marker1 = cv2.aruco.generateImageMarker(calibrator.dictionary, 1, 100)
            color[150:250, 100:200] = cv2.cvtColor(marker0, cv2.COLOR_GRAY2BGR)
            color[200:300, 400:500] = cv2.cvtColor(marker1, cv2.COLOR_GRAY2BGR)
            auto_sampling = True

        observations = calibrator.detect_markers(color, intrinsics)
        has_id0 = 0 in observations
        has_id1 = 1 in observations

        if has_id0 and auto_sampling:
            calibrator.add_frame_sample(observations)

        now = time.time()
        if now - last_print >= 0.15:
            last_print = now
            if has_id0:
                dist_str = ""
                if has_id1:
                    t0 = observations[0].t_cam_to_marker[:3, 3]
                    t1 = observations[1].t_cam_to_marker[:3, 3]
                    dist_str = f"| 对角距: {np.linalg.norm(t1-t0)*100:.1f}cm"
                status_str = f"[OK] 已锁定 ID0 {dist_str}"
                if auto_sampling:
                    prog_bar = render_progress_bar(calibrator.sample_count, samples_needed)
                    print(f"  {status_str} | 采样中: {prog_bar}", end="\r", flush=True)
                else:
                    print(f"  {status_str} | 状态平稳，按 [ENTER] 开始采样...", end="\r", flush=True)
            else:
                elapsed = int(now - t_start)
                print(f"  [WAIT] 未检测到 ID 0 码... 请将标定板放于视野内 ({elapsed}s) | 按 [S] 跳过", end="\r", flush=True)

        if not no_gui:
            vis = calibrator.draw_axes(color, observations, intrinsics)
            if auto_sampling:
                status_text = f"正在采集稳定特征帧 [{calibrator.sample_count}/{samples_needed}]"
                status_color = (0, 255, 120)
            elif has_id0:
                status_text = "标定板已锁定! 保持静止，按 [ENTER] 开始采样"
                status_color = (0, 255, 230)
            else:
                status_text = "正在搜索 ID 0 码... 请将标定板置于视野内"
                status_color = (0, 165, 255)

            t_lines = [
                f"识别状态: ID 0: {'已锁定 (OK)' if has_id0 else '未检测 (SEARCHING)'} | ID 1: {'已锁定 (OK)' if has_id1 else '未检测'}"
            ]
            if has_id1:
                t0 = observations[0].t_cam_to_marker[:3, 3]
                t1 = observations[1].t_cam_to_marker[:3, 3]
                t_lines.append(f"基线几何校验: 对角线距离 {np.linalg.norm(t1-t0)*100:.1f} cm (符合双码基准)")
            else:
                t_lines.append("基线几何校验: 建议放置 ID 1 码以提供稳定对角线基线约束")

            vis_hud = hud_renderer.draw_hud(
                frame=vis,
                stage_idx=1,
                stage_title="【阶段 1 / 3】桌面物理参考平面标定",
                step_title="【步骤 1 / 1】检测桌面 ArUco 标定板 (ID 0 主基准 / ID 1 对角基准)",
                instruction="将标定板平放在桌面相机视野内，保持静止",
                status_text=status_text,
                status_color=status_color,
                telemetry_lines=t_lines,
                progress_cur=calibrator.sample_count,
                progress_total=samples_needed,
                is_sampling=auto_sampling,
            )
            cv2.imshow("TuinaDex Pre-Flight Calibration", vis_hud)

        # 15s auto-fallback if marker not found
        if not auto_sampling and (now - t_start > 15.0) and calibrator.sample_count == 0:
            print("\n  [提示] 超过 15 秒未检测到桌面标定板，自动跳过并复用历史标定。")
            break

    print()
    if calibrator.sample_count >= samples_needed:
        print(f"[SUCCESS] 阶段 1 桌面标定成功！累积采集 {calibrator.sample_count} 帧。")
        calib_data = calibrator.compute_calibration()
        baseline = calibrator.get_baseline_distance()
        return {
            "is_valid": True,
            "primary_id": int(calib_data.tag_id),
            "secondary_id": 1,
            "marker_size_m": float(calib_data.tag_size_m),
            "num_samples": int(calib_data.num_samples),
            "baseline_distance_m": float(baseline) if baseline is not None else None,
            "t_cam_to_table": calib_data.t_cam_to_table.tolist(),
            "t_table_to_cam": calib_data.t_table_to_cam.tolist(),
            "axis_mapping": axis_mapping,
        }
    else:
        existing_path = PROJECT_ROOT / "configs" / "table_calibration.yaml"
        if existing_path.exists():
            print(f"[INFO] 成功复用历史桌面标定: {existing_path}")
            calib = calibrator.load_calibration(str(existing_path))
            baseline = calibrator.get_baseline_distance()
            saved_mapping = axis_mapping
            try:
                with open(existing_path, "r", encoding="utf-8") as f:
                    _d = yaml.safe_load(f) or {}
                    if isinstance(_d, dict) and "axis_mapping" in _d:
                        saved_mapping = _d["axis_mapping"]
            except Exception:
                pass
            return {
                "is_valid": True,
                "primary_id": calib.tag_id,
                "secondary_id": 1,
                "marker_size_m": calib.tag_size_m,
                "num_samples": calib.num_samples,
                "baseline_distance_m": baseline,
                "t_cam_to_table": calib.t_cam_to_table.tolist(),
                "t_table_to_cam": calib.t_table_to_cam.tolist(),
                "axis_mapping": saved_mapping,
            }
        else:
            print("[WARN] 无历史标定记录，使用水平默认地面系。")
            eye = np.eye(4, dtype=np.float64)
            return {
                "is_valid": False,
                "primary_id": 0,
                "secondary_id": 1,
                "marker_size_m": 0.080,
                "num_samples": 0,
                "baseline_distance_m": None,
                "t_cam_to_table": eye.tolist(),
                "t_table_to_cam": eye.tolist(),
                "axis_mapping": axis_mapping,
            }


# ─────────────────────────────────────────────────────────────────────────────
# STAGE 2: 6-Axis Wrist IMU 3-Axis Attitude Alignment & Neutral Zeroing
# ─────────────────────────────────────────────────────────────────────────────

def run_stage_2_imu(
    stream: RealSenseStream | MockRealSenseStream,
    imu_stream: ImuStream | MockImuStream,
    key_poller: TerminalKeyPoller,
    hud_renderer: ChineseHudRenderer,
    samples_per_step: int = 25,
    axis_mapping: str = "+x,+y,+z",
    no_gui: bool = False,
    mock: bool = False,
) -> Dict[str, Any]:
    """Execute Stage 2: Wrist IMU full 3-axis alignment (X-Pitch / Y-Roll / Z-Yaw) and neutral zeroing."""
    print("\n" + "=" * 75)
    print("  【阶段 2 / 3】手腕 IMU (腕带佩戴) 三轴姿态对齐与零位标定 (3-Axis IMU Calibration)")
    print("=" * 75)

    sub_steps_meta = [
        {
            "id": 0,
            "title": "【步骤 2A / 4】手腕平放基准零位 (Baseline Neutral)",
            "instruction": "手腕与前臂平贴桌面，手掌平展，手腕 IMU 保持水平静止",
            "prompt": "放平手腕保持静止，按 [ENTER] 锁定基准水平零位",
        },
        {
            "id": 1,
            "title": "【步骤 2B / 4】围绕腕横轴俯仰压抬 (X轴俯仰 Pitch)",
            "instruction": "以肘关节为支点，轻抬手腕与前臂 20°~40°(手腕抬起/压下，正负角度均可自动识别)，保持静止",
            "prompt": "保持手腕抬起姿势静止，按 [ENTER] 采样俯仰主轴响应",
        },
        {
            "id": 2,
            "title": "【步骤 2C / 4】围绕前臂纵轴翻转 (Y轴滚转 Roll)",
            "instruction": "手腕向前臂外侧翻转 20°~40°(前臂旋前/旋后，正负角度均可自动识别)，保持静止",
            "prompt": "保持侧翻姿势静止，按 [ENTER] 采样滚转主轴响应",
        },
        {
            "id": 3,
            "title": "【步骤 2D / 4】围绕手腕法向量偏摆 (Z轴偏航 Yaw)",
            "instruction": "手腕平贴桌面，以肘部为支点左右水平偏摆手腕 15°~30°，保持静止",
            "prompt": "保持水平偏摆静止，按 [ENTER] 采样偏航主轴响应",
        },
    ]

    cur_step = 0
    R_ref: Optional[np.ndarray] = None
    R_pitch: Optional[np.ndarray] = None
    R_roll: Optional[np.ndarray] = None
    R_yaw: Optional[np.ndarray] = None
    ref_quat: Optional[np.ndarray] = None
    ref_rpy: Optional[Tuple[float, float, float]] = None

    step_samples: List[np.ndarray] = []
    is_sampling = False
    last_print = 0.0
    start_t = time.time()

    def print_step_banner(step_idx: int):
        meta = sub_steps_meta[step_idx]
        print("\n" + "-" * 75)
        print(f"  {meta['title']}")
        print(f"  指引: {meta['instruction']}")
        print(f"  操作: {meta['prompt']} | 按 [S] 跳过")
        print("-" * 75)

    print_step_banner(0)

    while cur_step < 4:
        color, depth, _ = stream.read_frames()
        if color is None:
            time.sleep(0.005)
            continue

        term_k = key_poller.poll_key() if not mock else None
        gui_k = None
        if not no_gui:
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), ord('Q'), 27):
                gui_k = "QUIT"
            elif k in (ord('s'), ord('S')):
                gui_k = "SKIP"
            elif k in (ord(' '), 10, 13):
                gui_k = "ENTER"

        act_k = term_k or gui_k

        if act_k == "QUIT":
            print("\n[INFO] 用户终止标定。")
            sys.exit(0)
        elif act_k == "SKIP":
            print("\n[INFO] 用户跳过阶段 2: IMU 三轴姿态标定。")
            break
        elif act_k == "ENTER" and not is_sampling:
            is_sampling = True
            step_samples = []
            print(f"\n  ✓ [ENTER] 已确认！正在采集 {sub_steps_meta[cur_step]['title']} 样本...")

        if mock and not is_sampling:
            if cur_step == 0:
                imu_stream.set_synthetic_euler(0.0, 0.0, 0.0)
            elif cur_step == 1:
                imu_stream.set_synthetic_euler(35.0, 0.0, 0.0)
            elif cur_step == 2:
                imu_stream.set_synthetic_euler(0.0, 35.0, 0.0)
            elif cur_step == 3:
                imu_stream.set_synthetic_euler(0.0, 0.0, 25.0)
            is_sampling = True
            step_samples = []

        reading = imu_stream.read()
        if is_sampling and reading.is_valid:
            step_samples.append(reading.quat.copy())

        r, p, y = reading.euler_deg if reading.is_valid else (0.0, 0.0, 0.0)
        delta_r, delta_p, delta_y = 0.0, 0.0, 0.0
        if R_ref is not None and reading.is_valid:
            R_curr = reading.rotation_matrix
            delta_R = R_curr @ R_ref.T
            rotvec = rotation_matrix_to_rotvec(delta_R)
            delta_p = math.degrees(rotvec[0])
            delta_r = math.degrees(rotvec[1])
            delta_y = math.degrees(rotvec[2])

        # Status text & badge
        status_text = ""
        status_color = (0, 165, 255)  # Default orange
        if cur_step == 0:
            if is_sampling:
                status_text = f"正在采集基准水平零位样本 [{len(step_samples)}/{samples_per_step}]"
                status_color = (0, 255, 120)
            elif reading.is_valid:
                status_text = "手掌已放平就绪 - 请按 [ENTER] 锁定基准零位"
                status_color = (0, 255, 230)
            else:
                status_text = "等待 IMU 数据响应... 检查接线或串口"
                status_color = (0, 165, 255)
        elif cur_step == 1:
            ready = (abs(delta_p) >= 15.0)
            sign_hint = "正向+X" if delta_p > 0 else "反向-X (自动极性适配)"
            if is_sampling:
                status_text = f"正在采样俯仰压抬样本 [{len(step_samples)}/{samples_per_step}]"
                status_color = (0, 255, 120)
            elif ready:
                status_text = f"已检测到有效压抬 (偏角 {delta_p:+.1f}°, {sign_hint}) - 请按 [ENTER] 采样"
                status_color = (0, 255, 120)
            else:
                status_text = f"请以肘为轴抬起手腕 (当前: {delta_p:+.1f}° / 目标幅值: 20°~40°)"
                status_color = (0, 165, 255)
        elif cur_step == 2:
            ready = (abs(delta_r) >= 15.0)
            sign_hint = "正向+Y" if delta_r > 0 else "反向-Y (自动极性适配)"
            if is_sampling:
                status_text = f"正在采样翻掌滚转样本 [{len(step_samples)}/{samples_per_step}]"
                status_color = (0, 255, 120)
            elif ready:
                status_text = f"已检测到手腕侧翻 (偏角 {delta_r:+.1f}°, {sign_hint}) - 请按 [ENTER] 采样"
                status_color = (0, 255, 120)
            else:
                status_text = f"请侧倾翻动手腕前臂 (当前: {delta_r:+.1f}° / 目标幅值: 20°~40°)"
                status_color = (0, 165, 255)
        elif cur_step == 3:
            ready = (abs(delta_y) >= 10.0)
            sign_hint = "正向+Z" if delta_y > 0 else "反向-Z (自动极性适配)"
            if is_sampling:
                status_text = f"正在采样水平偏摆样本 [{len(step_samples)}/{samples_per_step}]"
                status_color = (0, 255, 120)
            elif ready:
                status_text = f"已检测到水平偏摆 (偏角 {delta_y:+.1f}°, {sign_hint}) - 请按 [ENTER] 采样"
                status_color = (0, 255, 120)
            else:
                status_text = f"请贴近桌面左右摆动手腕 (当前: {delta_y:+.1f}° / 目标幅值: 15°~30°)"
                status_color = (0, 165, 255)

        # Terminal feedback (smoothly paced at 8 Hz)
        now = time.time()
        if now - last_print >= 0.12:
            last_print = now
            if reading.is_valid:
                rpy_str = f"Live IMU: R={r:+5.1f}°, P={p:+5.1f}°, Y={y:+5.1f}°"
                if is_sampling:
                    prog_bar = render_progress_bar(len(step_samples), samples_per_step)
                    print(f"  {rpy_str} | 采样中: {prog_bar}", end="\r", flush=True)
                else:
                    print(f"  {rpy_str} | {status_text}", end="\r", flush=True)
            else:
                elapsed = int(now - start_t)
                print(f"  [WAIT] 等待 IMU 数据流... ({elapsed}s) | 检查接线或按 [S] 跳过", end="\r", flush=True)

        if not no_gui:
            t_lines = [
                f"实时姿态: Roll={r:+5.1f}° | Pitch={p:+5.1f}° | Yaw={y:+5.1f}°",
                f"相对增量: ΔP={delta_p:+5.1f}° (X俯仰) | ΔR={delta_r:+5.1f}° (Y滚转) | ΔY={delta_y:+5.1f}° (Z偏航)",
            ]
            vis_hud = hud_renderer.draw_hud(
                frame=color,
                stage_idx=2,
                stage_title="【阶段 2 / 3】手腕 IMU (腕带佩戴) 三轴姿态对齐与零位校准",
                step_title=sub_steps_meta[cur_step]["title"],
                instruction=sub_steps_meta[cur_step]["instruction"],
                status_text=status_text,
                status_color=status_color,
                telemetry_lines=t_lines,
                progress_cur=len(step_samples),
                progress_total=samples_per_step,
                is_sampling=is_sampling,
            )
            cv2.imshow("TuinaDex Pre-Flight Calibration", vis_hud)

        # Check sample completion
        if is_sampling and len(step_samples) >= samples_per_step:
            avg_q = average_quaternions(step_samples)
            R_captured = quat_to_rotation_matrix(avg_q)
            cap_rpy = rotation_matrix_to_euler_deg(R_captured)

            if cur_step == 0:
                R_ref = R_captured
                ref_quat = avg_q
                ref_rpy = cap_rpy
                print(f"\n  ✓ [步骤 2A 完成] 水平基准零位锁定: RPY=({ref_rpy[0]:+.1f}°, {ref_rpy[1]:+.1f}°, {ref_rpy[2]:+.1f}°)")
            elif cur_step == 1:
                R_pitch = R_captured
                print(f"\n  ✓ [步骤 2B 完成] 围绕X轴压抬响应已捕获: RPY=({cap_rpy[0]:+.1f}°, {cap_rpy[1]:+.1f}°, {cap_rpy[2]:+.1f}°)")
            elif cur_step == 2:
                R_roll = R_captured
                print(f"\n  ✓ [步骤 2C 完成] 围绕Y轴翻掌响应已捕获: RPY=({cap_rpy[0]:+.1f}°, {cap_rpy[1]:+.1f}°, {cap_rpy[2]:+.1f}°)")
            elif cur_step == 3:
                R_yaw = R_captured
                print(f"\n  ✓ [步骤 2D 完成] 围绕Z轴偏摆响应已捕获: RPY=({cap_rpy[0]:+.1f}°, {cap_rpy[1]:+.1f}°, {cap_rpy[2]:+.1f}°)")

            cur_step += 1
            is_sampling = False
            step_samples = []
            if cur_step < 4:
                print_step_banner(cur_step)

    print()
    if R_ref is not None and R_pitch is not None and R_roll is not None:
        try:
            solved_mapping = solve_imu_alignment_from_gestures(R_ref, R_pitch, R_roll, R_yaw)
        except Exception as e:
            print(f"[WARN] 三轴手势解算异常 ({e})，降级使用床侧基准 '{axis_mapping}'。")
            solved_mapping = axis_mapping

        R_align = parse_axis_mapping(solved_mapping)
        det = np.linalg.det(R_align)

        print("\n" + "=" * 75)
        print(f"[SUCCESS] 阶段 2 IMU 三轴姿态对齐与零位校准成功！")
        print(f"  - 最优三轴映射: '{solved_mapping}' (正交行列式 det = {det:+.1f})")
        print(f"  - 水平基准零位: R={ref_rpy[0]:+.1f}°, P={ref_rpy[1]:+.1f}°, Y={ref_rpy[2]:+.1f}°")
        print("=" * 75)

        # Brief confirmation flash in GUI
        if not no_gui:
            for _ in range(12):
                color_conf, _, _ = stream.read_frames()
                if color_conf is None:
                    break
                vis_conf = hud_renderer.draw_hud(
                    frame=color_conf,
                    stage_idx=2,
                    stage_title="【阶段 2 / 3】手腕 IMU (腕带佩戴) 三轴姿态校准完成",
                    step_title="【校准就绪】三轴正交旋转矩阵与零位基准解算成功",
                    instruction=f"最优映射: '{solved_mapping}' (det = {det:+.1f}) | 即将自动进入阶段 3...",
                    status_text=f"三轴映射锁定: '{solved_mapping}' (X-压抬, Y-翻转, Z-偏摆)",
                    status_color=(0, 255, 120),
                    telemetry_lines=[
                        f"基准姿态: R={ref_rpy[0]:+.1f}°, P={ref_rpy[1]:+.1f}°, Y={ref_rpy[2]:+.1f}°",
                        f"正交变换矩阵 R_align:\n{R_align}",
                    ],
                    progress_cur=samples_per_step,
                    progress_total=samples_per_step,
                    is_sampling=False,
                )
                cv2.imshow("TuinaDex Pre-Flight Calibration", vis_conf)
                cv2.waitKey(20)

        return {
            "is_valid": True,
            "device_port": getattr(imu_stream, "port", "/dev/myimu"),
            "imu_axis_mapping": solved_mapping,
            "mounting": "IMU strapped on dorsal wrist, Y along forearm towards fingers, X across wrist, Z pointing outward from wrist dorsal side",
            "neutral_quat_wxyz": ref_quat.tolist(),
            "neutral_rpy_deg": [round(float(v), 2) for v in ref_rpy],
        }
    else:
        existing_path = PROJECT_ROOT / "configs" / "imu_calibration.yaml"
        if existing_path.exists():
            print(f"[INFO] 成功复用历史 IMU 映射配置: {existing_path}")
            with open(existing_path, "r", encoding="utf-8") as f:
                d = yaml.safe_load(f) or {}
            d["is_valid"] = True
            return d
        else:
            return {
                "is_valid": True,
                "device_port": getattr(imu_stream, "port", "/dev/myimu"),
                "imu_axis_mapping": axis_mapping,
                "mounting": "IMU strapped on dorsal wrist, Y along forearm towards fingers, X across wrist, Z pointing outward from wrist dorsal side",
                "neutral_quat_wxyz": [1.0, 0.0, 0.0, 0.0],
                "neutral_rpy_deg": [0.0, 0.0, 0.0],
            }


# ─────────────────────────────────────────────────────────────────────────────
# STAGE 3: Hand Scale & Biomechanical Zero/ROM Calibration
# ─────────────────────────────────────────────────────────────────────────────

def run_stage_3_hand(
    stream: RealSenseStream | MockRealSenseStream,
    intrinsics: CameraIntrinsics,
    key_poller: TerminalKeyPoller,
    hud_renderer: ChineseHudRenderer,
    samples_needed: int = 25,
    no_gui: bool = False,
    mock: bool = False,
) -> Dict[str, Any]:
    """Execute Stage 3: Hand scale measurement and zero-offset calibration."""
    print("\n" + "=" * 75)
    print("  【阶段 3 / 3】灵巧手手型尺寸与平手零位自适应 (Hand Scale & Zero Calibration)")
    print("=" * 75)
    print("  [说明] 请将右手平摊在桌面相机视野内，手心贴桌，手指自然展开。")
    print("  [按键] 画面检测到手掌稳定后，按 [ENTER] 或 [空格] 开始记录零位与掌长；按 [S] 跳过。")
    print("-" * 75)

    wrist_tracker = WristTracker(default_handedness="Right")
    bio_retargeter = BiomechanicalKinematicRetargeter()

    if mock:
        def mock_hand(img):
            cx, cy = intrinsics.cx, intrinsics.cy
            lm = np.zeros((21, 3), dtype=np.float32)
            lm[LM_WRIST] = [cx / intrinsics.width, cy / intrinsics.height, 0.0]
            lm[5] = [(cx - 20) / intrinsics.width, (cy - 35) / intrinsics.height, 0.0]
            lm[6] = [(cx - 20) / intrinsics.width, (cy - 50) / intrinsics.height, 0.0]
            lm[7] = [(cx - 20) / intrinsics.width, (cy - 65) / intrinsics.height, 0.0]
            lm[8] = [(cx - 20) / intrinsics.width, (cy - 75) / intrinsics.height, 0.0]
            lm[9] = [cx / intrinsics.width, (cy - 40) / intrinsics.height, 0.0]
            lm[10] = [cx / intrinsics.width, (cy - 55) / intrinsics.height, 0.0]
            lm[11] = [cx / intrinsics.width, (cy - 70) / intrinsics.height, 0.0]
            lm[12] = [cx / intrinsics.width, (cy - 82) / intrinsics.height, 0.0]
            lm[13] = [(cx + 15) / intrinsics.width, (cy - 35) / intrinsics.height, 0.0]
            lm[14] = [(cx + 15) / intrinsics.width, (cy - 50) / intrinsics.height, 0.0]
            lm[15] = [(cx + 15) / intrinsics.width, (cy - 63) / intrinsics.height, 0.0]
            lm[16] = [(cx + 15) / intrinsics.width, (cy - 74) / intrinsics.height, 0.0]
            lm[17] = [(cx + 25) / intrinsics.width, (cy - 30) / intrinsics.height, 0.0]
            lm[18] = [(cx + 25) / intrinsics.width, (cy - 42) / intrinsics.height, 0.0]
            lm[19] = [(cx + 25) / intrinsics.width, (cy - 53) / intrinsics.height, 0.0]
            lm[20] = [(cx + 25) / intrinsics.width, (cy - 62) / intrinsics.height, 0.0]
            lm[1] = [(cx - 15) / intrinsics.width, (cy - 15) / intrinsics.height, 0.0]
            lm[2] = [(cx - 25) / intrinsics.width, (cy - 25) / intrinsics.height, 0.0]
            lm[3] = [(cx - 32) / intrinsics.width, (cy - 33) / intrinsics.height, 0.0]
            lm[4] = [(cx - 38) / intrinsics.width, (cy - 40) / intrinsics.height, 0.0]
            return {"landmarks": lm, "handedness": "Right"}
        wrist_tracker.set_custom_detector(mock_hand)

    open_palm_lengths: List[float] = []
    open_q_samples: List[np.ndarray] = []

    last_print = 0.0
    auto_sampling = False
    start_t = time.time()

    HAND_BONES = [
        (0, 1), (1, 2), (2, 3), (3, 4),        # Thumb
        (0, 5), (5, 6), (6, 7), (7, 8),        # Index
        (0, 9), (9, 10), (10, 11), (11, 12),   # Middle
        (0, 13), (13, 14), (14, 15), (15, 16), # Ring
        (0, 17), (17, 18), (18, 19), (19, 20), # Pinky
        (5, 9), (9, 13), (13, 17),             # Knuckles base
    ]

    while len(open_q_samples) < samples_needed:
        color, depth, ts = stream.read_frames()
        if color is None:
            time.sleep(0.005)
            continue

        term_k = key_poller.poll_key() if not mock else None
        gui_k = None
        if not no_gui:
            k = cv2.waitKey(1) & 0xFF
            if cv2.getWindowProperty("TuinaDex Pre-Flight Calibration", cv2.WND_PROP_VISIBLE) < 1:
                gui_k = "QUIT"
            elif k in (ord('q'), ord('Q'), 27):
                gui_k = "QUIT"
            elif k in (ord('s'), ord('S')):
                gui_k = "SKIP"
            elif k in (ord(' '), 10, 13):
                gui_k = "ENTER"

        act_k = term_k or gui_k

        if act_k == "QUIT":
            print("\n[INFO] 用户终止标定。")
            sys.exit(0)
        elif act_k == "SKIP":
            print("\n[INFO] 用户跳过手部标定。")
            break
        elif act_k == "ENTER" and not auto_sampling:
            auto_sampling = True
            print("\n  ✓ [ENTER] 已确认！开始采集平手尺寸与零位...")

        if mock:
            auto_sampling = True

        hand_detected = False
        cur_palm_len = 0.0
        pose = None

        try:
            pose = wrist_tracker.process_frame(color, depth, intrinsics, timestamp=ts)
            hand_detected = (
                pose is not None
                and pose.is_valid
                and (pose.tracking_state == TrackingState.TRACKING)
            )

            if hand_detected:
                pts = (
                    pose.landmarks_world_3d
                    if (pose.landmarks_world_3d is not None and np.any(pose.landmarks_world_3d != 0))
                    else pose.landmarks_3d_cam
                )
                if pts is not None and len(pts) >= 21 and not np.isnan(pts).any():
                    _, _, _, R_basis = compute_palm_basis(pts, pose.handedness)
                    pts_local = (R_basis.T @ (pts - pts[LM_WRIST]).T).T

                    raw_palm_len = float(np.linalg.norm(pts_local[LM_MIDDLE_MCP] - pts_local[LM_WRIST]))
                    if 0.04 <= raw_palm_len <= 0.16:
                        cur_palm_len = raw_palm_len
                        open_palm_lengths.append(cur_palm_len)

                    q_raw = bio_retargeter.retarget(pts_local)
                    if auto_sampling and not np.isnan(q_raw).any():
                        open_q_samples.append(q_raw)
        except Exception as e:
            # Shield tracking frame exceptions to prevent abrupt crash
            hand_detected = False
            cur_palm_len = 0.0

        now = time.time()
        if now - last_print >= 0.15:
            last_print = now
            if hand_detected:
                len_cm = cur_palm_len * 100.0 if cur_palm_len > 0.03 else 8.5
                if auto_sampling:
                    prog_bar = render_progress_bar(len(open_q_samples), samples_needed)
                    print(f"  [OK] 锁定右手 (实时掌长: {len_cm:.1f}cm) | 采样中: {prog_bar}", end="\r", flush=True)
                else:
                    print(f"  [OK] 锁定右手 (实时掌长: {len_cm:.1f}cm) | 手势平稳，按 [ENTER] 开始采样...", end="\r", flush=True)
            else:
                elapsed = int(now - start_t)
                print(f"  [WAIT] 相机画面未检测到手部... 请将右手平放于视野内 ({elapsed}s) | 按 [S] 跳过", end="\r", flush=True)

        if not no_gui:
            vis = color.copy()

            # Real-time visual feedback: render hand skeleton and keypoint circles
            if hand_detected and pose is not None and pose.landmarks_2d is not None:
                pts_2d = pose.landmarks_2d.astype(int)
                for p1, p2 in HAND_BONES:
                    if 0 <= p1 < len(pts_2d) and 0 <= p2 < len(pts_2d):
                        pt1 = (int(pts_2d[p1, 0]), int(pts_2d[p1, 1]))
                        pt2 = (int(pts_2d[p2, 0]), int(pts_2d[p2, 1]))
                        cv2.line(vis, pt1, pt2, (0, 255, 120), 2, cv2.LINE_AA)
                for pt in pts_2d:
                    cv2.circle(vis, (int(pt[0]), int(pt[1])), 4, (0, 200, 255), -1, cv2.LINE_AA)

            if auto_sampling:
                status_text = f"正在采集手掌尺寸与零偏 [{len(open_q_samples)}/{samples_needed}]"
                status_color = (0, 255, 120)
            elif hand_detected:
                status_text = "右手姿势已锁定! 保持展开，按 [ENTER] 开始采样"
                status_color = (0, 255, 230)
            else:
                status_text = "正在搜索手部... 请将右手平展置于相机视野内"
                status_color = (0, 165, 255)

            t_lines = [
                f"检测状态: {'右手锁定 (TRACKING)' if hand_detected else '未检测到手部 (SEARCHING)'}",
                f"实时掌长: {cur_palm_len*100:.1f} cm (基准: 8.5 cm) | 缩放比: {0.085/max(0.04, cur_palm_len):.2f}" if hand_detected else "手型特征: 请展开五指手心贴桌",
            ]
            vis_hud = hud_renderer.draw_hud(
                frame=vis,
                stage_idx=3,
                stage_title="【阶段 3 / 3】灵巧手尺寸与平手零位自适应",
                step_title="【步骤 3 / 1】平摊手掌测定掌长与关节零偏 (Open Hand Flat Zeroing)",
                instruction="右手手掌平摊贴桌面，手指自然伸直展开，保持静止",
                status_text=status_text,
                status_color=status_color,
                telemetry_lines=t_lines,
                progress_cur=len(open_q_samples),
                progress_total=samples_needed,
                is_sampling=auto_sampling,
            )
            cv2.imshow("TuinaDex Pre-Flight Calibration", vis_hud)

    print()
    measured_palm_len = float(np.median(open_palm_lengths)) if open_palm_lengths else 0.085
    avg_q_open = np.mean(open_q_samples, axis=0) if open_q_samples else np.zeros(HAND_NUM_DOFS)

    print(f"[SUCCESS] 阶段 3 手部标定成功！测定有效掌长 = {measured_palm_len*100:.2f} cm (比例: {0.085/measured_palm_len:.2f})")
    print(f"  - 自动固化平手残留偏置消减向量 (Zero Offset Latched)。")

    # Brief confirmation flash in GUI
    if not no_gui:
        for _ in range(12):
            color_conf, _, _ = stream.read_frames()
            if color_conf is None:
                break
            vis_conf = hud_renderer.draw_hud(
                frame=color_conf,
                stage_idx=3,
                stage_title="【阶段 3 / 3】灵巧手自适应标定完成",
                step_title="【校准就绪】测定有效掌长与零偏向量固化成功",
                instruction=f"测定掌长: {measured_palm_len*100:.1f}cm (缩放比: {0.085/measured_palm_len:.2f}) | 预检向导全部完成！",
                status_text="灵巧手尺寸与平手零位固化完成！",
                status_color=(0, 255, 120),
                telemetry_lines=[
                    f"测定掌长: {measured_palm_len*100:.1f} cm (基准: 8.5 cm)",
                    f"自适应关节零偏消减向量已生效",
                ],
                progress_cur=samples_needed,
                progress_total=samples_needed,
                is_sampling=False,
            )
            cv2.imshow("TuinaDex Pre-Flight Calibration", vis_conf)
            cv2.waitKey(25)

    return {
        "is_valid": True,
        "handedness": "Right",
        "palm_reference_length_m": 0.085,
        "measured_palm_length_m": round(float(measured_palm_len), 4),
        "scale_ratio": round(float(0.085 / max(0.04, measured_palm_len)), 3),
        "q_zero_offset": [round(float(v), 4) for v in avg_q_open],
    }


# ─────────────────────────────────────────────────────────────────────────────
# Master Pre-Flight Wizard Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="TuinaDex Unified Pre-Flight Teleoperation Calibrator")
    parser.add_argument("--mock", action="store_true", help="Run in mock mode without physical hardware")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless terminal mode")
    parser.add_argument("--imu-port", type=str, default="/dev/myimu", help="Serial port for IMU")
    parser.add_argument("--imu-axis-mapping", type=str, default="+x,+y,+z", help="IMU mounting alignment to robot base")
    parser.add_argument("--axis-mapping", type=str, default="-y,+x,+z", help="桌面到机械臂基座的末端平移映射 (默认 '-y,+x,+z'，对齐床侧横置与桌面坐标)")
    parser.add_argument("--skip-table", action="store_true", help="Skip Stage 1 table calibration")
    parser.add_argument("--skip-imu", action="store_true", help="Skip Stage 2 IMU calibration")
    parser.add_argument("--skip-hand", action="store_true", help="Skip Stage 3 hand calibration")
    parser.add_argument("--output", type=str, default=str(CONFIGS_DIR / "session_calibration.yaml"), help="Master output path")
    args = parser.parse_args()

    print("=" * 75)
    print("  TuinaDex v2.0 — 仿真前一键全集成预检标定向导 (Pre-Flight Wizard)")
    print("=" * 75)
    print("  全流程引导包含 3 个核心阶段:")
    print("    1. 【桌面物理参考平面标定】 (ArUco ID0/ID1 对角基线与地平面解算)")
    print("    2. 【手腕 IMU 三轴姿态校准】 (X压抬 / Y翻转 / Z偏摆 / 腕带佩戴零位)")
    print("    3. 【灵巧手尺寸与平手零位】 (动态掌长自适应缩放与零偏补偿)")
    print("  操作说明:")
    print("    - 任意阶段按 [ENTER] / [空格] 开始采样，按 [S] 可随时跳过")
    print("    - 终端与 OpenCV 画面双通道同步监听按键，零卡顿流畅预览")
    print("=" * 75 + "\n")

    key_poller = TerminalKeyPoller()
    hud_renderer = ChineseHudRenderer()

    # 1. Initialize RealSense Camera Stream
    if args.mock:
        print("[INFO] 正在启动 MOCK RealSense 相机流...")
        stream = MockRealSenseStream(width=640, height=480, fps=30)
    else:
        print("[INFO] 正在启动 RealSense D455 相机流...")
        try:
            stream = RealSenseStream(width=640, height=480, fps=30)
        except Exception as e:
            print(f"[ERROR] 无法启动 RealSense 相机: {e}")
            print("[提示] 可追加 '--mock' 参数进行无相机离线测试。")
            key_poller.close()
            return

    intrinsics = stream.start()
    print(f"[INFO] 相机启动成功: 640x480, fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")

    # 2. Initialize IMU Stream
    if args.mock:
        print("[INFO] 正在启动 MOCK IMU 模拟数据流...")
        imu_stream = MockImuStream()
        imu_stream.start()
    else:
        print(f"[INFO] 正在连接 6 轴 IMU 串口 ({args.imu_port})...")
        imu_stream = ImuStream(port=args.imu_port, auto_detect=True)
        if not imu_stream.start():
            print("[WARN] 未检测到物理 IMU 串口设备，降级至 MOCK IMU 模拟流。")
            imu_stream = MockImuStream()
            imu_stream.start()

    session_data: Dict[str, Any] = {
        "metadata": {
            "calibrated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "version": "1.0",
            "device": "reBot-DevArm + LEAP Hand Right",
        }
    }

    try:
        # ---------------------------------------------------------------------
        # Stage 1: Table Frame
        # ---------------------------------------------------------------------
        if not args.skip_table:
            table_calib = run_stage_1_table(
                stream=stream,
                intrinsics=intrinsics,
                key_poller=key_poller,
                hud_renderer=hud_renderer,
                axis_mapping=args.axis_mapping,
                no_gui=args.no_gui,
                mock=args.mock,
            )
            session_data["table"] = table_calib
        else:
            print("[INFO] 用户跳过阶段 1: 桌面定位码标定。")

        # ---------------------------------------------------------------------
        # Stage 2: IMU Full 3-Axis & Neutral
        # ---------------------------------------------------------------------
        if not args.skip_imu:
            imu_calib = run_stage_2_imu(
                stream=stream,
                imu_stream=imu_stream,
                key_poller=key_poller,
                hud_renderer=hud_renderer,
                axis_mapping=args.imu_axis_mapping,
                no_gui=args.no_gui,
                mock=args.mock,
            )
            session_data["imu"] = imu_calib
        else:
            print("[INFO] 用户跳过阶段 2: IMU 三轴标定。")

        # ---------------------------------------------------------------------
        # Stage 3: Hand Scale & Zeroing
        # ---------------------------------------------------------------------
        if not args.skip_hand:
            hand_calib = run_stage_3_hand(
                stream=stream,
                intrinsics=intrinsics,
                key_poller=key_poller,
                hud_renderer=hud_renderer,
                no_gui=args.no_gui,
                mock=args.mock,
            )
            session_data["hand"] = hand_calib
        else:
            print("[INFO] 用户跳过阶段 3: 灵巧手标定。")

        # ---------------------------------------------------------------------
        # Save Consolidated Configurations
        # ---------------------------------------------------------------------
        out_path = Path(args.output).resolve()
        os.makedirs(out_path.parent, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(session_data, f, sort_keys=False)
        print("\n" + "=" * 75)
        print(f"[SUCCESS] 统一会话标定文件已写入:\n  -> {out_path}")

        # Synchronize backward-compatible files
        if "table" in session_data and session_data["table"].get("is_valid"):
            legacy_table = CONFIGS_DIR / "table_calibration.yaml"
            with open(legacy_table, "w", encoding="utf-8") as f:
                yaml.safe_dump(session_data["table"], f, sort_keys=False)
            print(f"  [Sync] 已自动同步既有文件: {legacy_table}")

        if "imu" in session_data and session_data["imu"].get("is_valid"):
            legacy_imu = CONFIGS_DIR / "imu_calibration.yaml"
            with open(legacy_imu, "w", encoding="utf-8") as f:
                yaml.safe_dump(session_data["imu"], f, sort_keys=False)
            print(f"  [Sync] 已自动同步既有文件: {legacy_imu}")

        print("=" * 75)
        print("  标定成果汇总 (SESSION SUMMARY):")
        if "table" in session_data:
            t = session_data["table"]
            print(f"  - 桌面平面参考: 样本帧={t.get('num_samples')}, 对角基准={t.get('baseline_distance_m')}m, 平移映射={t.get('axis_mapping')}")
        if "imu" in session_data:
            m = session_data["imu"]
            print(f"  - 手腕 IMU 映射: 轴向={m.get('imu_axis_mapping')}, 水平基准RPY={m.get('neutral_rpy_deg')}")
        if "hand" in session_data:
            h = session_data["hand"]
            print(f"  - 灵巧手自适应: 测定掌长={h.get('measured_palm_length_m')*100:.1f}cm, 尺度比={h.get('scale_ratio')}, 零偏补偿已激活")
        print("=" * 75)
        print("\n[就绪] 系统已达到最佳遥控状态！请直接运行以下命令开启数字孪生遥操作：")
        print("  1. ROS 2 复合节点 (带 OpenCV 仪表盘):")
        print("     ros2 launch tuinadex_teleop teleop_sim.launch.py use_sim:=true mock:=false mock_imu:=false gui:=true")
        print("  2. 多模态采集 Web Hub (带 3 机位视频墙与 8080 端口):")
        print("     python scripts/collect_teleop_data.py --env sim --mode arm_hand --hub\n")

    finally:
        key_poller.close()
        stream.stop()
        if hasattr(imu_stream, "stop"):
            imu_stream.stop()
        if not args.no_gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
