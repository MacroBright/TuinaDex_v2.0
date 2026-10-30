#!/usr/bin/env python3
"""05_preview_in_sim.py — Real-time 22-DOF Composite Robot Teleoperation in MuJoCo Digital Twin.

Fuses:
1. RealSense D455 Hardware-Aligned 3D Tracking (Scheme C: wrist_tracker.py).
2. Spatial Incremental Clutch Controller (clutch.py with keyboard/pedal toggling).
3. Pinocchio / Pink QP-IK Solver for 6-DOF reBot-DevArm (arm_ik_solver.py).
4. Kinematic Optimization Retargeting for 16-DOF LEAP Hand (hand_retargeter.py).
5. MuJoCo Physics Simulation Twin with Tuina Bed & Patient Dummy (MujocoSimAdapter).
6. Live dual-pane HUD + 3D interactive viewer with contact force telemetry.

Usage:
    # Run interactive teleoperation with physical D455 camera and 3D MuJoCo viewer:
    python scripts/05_preview_in_sim.py

    # Run in headless mode (e.g. SSH / automated CI):
    python scripts/05_preview_in_sim.py --no-gui

    # Run offline simulation with mock camera:
    python scripts/05_preview_in_sim.py --mock --max-frames 60 --no-gui
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

# Add workspace root and tuinadex_teleop to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
teleop_src = PROJECT_ROOT / 'src' / 'tuinadex_teleop'
if str(teleop_src) not in sys.path:
    sys.path.insert(0, str(teleop_src))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Resolve configs directory
CONFIGS_DIR = teleop_src / 'configs' if (teleop_src / 'configs').is_dir() else (PROJECT_ROOT / 'configs')

# Dynamic environment loader for tuinadex-control if running under other python
_cmeel_lib = os.path.expanduser(
    "~/win_office/conda/envs/tuinadex-control/lib/python3.10/site-packages/cmeel.prefix/lib"
)
_site_pkgs = os.path.expanduser(
    "~/win_office/conda/envs/tuinadex-control/lib/python3.10/site-packages"
)
if os.path.isdir(_cmeel_lib) and _cmeel_lib not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = f"{_cmeel_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"
if os.path.isdir(_site_pkgs) and _site_pkgs not in sys.path:
    sys.path.append(_site_pkgs)

from tuinadex_teleop.adapters import MujocoSimAdapter, DEFAULT_READY_ARM_Q
from tuinadex_teleop.core.clutch import ClutchController
from tuinadex_teleop.core.names import (
    ARM_NUM_DOFS,
    CANONICAL_ARM_JOINT_NAMES,
    CANONICAL_COMPOSITE_JOINT_NAMES,
    CANONICAL_HAND_JOINT_NAMES,
    COMPOSITE_NUM_DOFS,
    HAND_NUM_DOFS,
)
from tuinadex_teleop.core.safety import (
    SafetyConfig,
    SafetyLevel,
    SafetyReport,
    SafetySupervisor,
    SafetyViolation,
)
from tuinadex_teleop.core.types import (
    ArUcoCalibrationData,
    ClutchState,
    RobotObservation,
    TeleopArmCommand,
    TeleopCommand,
    TeleopHandCommand,
)
from tuinadex_teleop.kinematics.arm_ik_solver import (
    ArmIKMetrics,
    ArmIKSolver,
    ArmOrientationMode,
    R_DOWNWARD_DEFAULT,
)
from tuinadex_teleop.kinematics.hand_retargeter import (
    HandRetargeter,
    HandRetargetingMetrics,
)
from tuinadex_teleop.perception.realsense_stream import (
    CameraIntrinsics,
    MockRealSenseStream,
    RealSenseStream,
)
from tuinadex_teleop.perception.table_calibrator import TableCalibrator
from tuinadex_teleop.perception.wrist_tracker import (
    TrackingState,
    WristPose,
    WristTracker,
    draw_wrist_hud,
)
from tuinadex_teleop.perception.imu_stream import (
    ImuReading,
    ImuStream,
    MockImuStream,
    WristImuTracker,
)


def load_session_calibration(path: str | Path = "configs/session_calibration.yaml") -> Optional[dict]:
    """Load unified session calibration (table + IMU + hand) if available."""
    p_canonical = (CONFIGS_DIR / Path(path).name).resolve()
    p_cwd = Path(path).resolve()
    target_p = p_canonical if p_canonical.is_file() else (p_cwd if p_cwd.is_file() else None)
    if target_p is None:
        return None
    try:
        import yaml
        with open(target_p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            if isinstance(data, dict):
                print(f"[INFO] Loaded unified session calibration from {target_p}")
                return data
    except Exception as e:
        print(f"[WARN] Failed to load session calibration from {target_p} ({e}).")
    return None


def parse_table_from_session(session_data: dict) -> Optional[ArUcoCalibrationData]:
    """Extract ArUcoCalibrationData from session calibration dictionary."""
    if not isinstance(session_data, dict) or "table" not in session_data:
        return None
    t_info = session_data["table"]
    if not t_info.get("is_valid", False):
        return None
    try:
        t_cam = np.array(t_info["t_cam_to_table"], dtype=np.float64)
        t_tab = np.array(t_info["t_table_to_cam"], dtype=np.float64)
        return ArUcoCalibrationData(
            t_cam_to_table=t_cam,
            t_table_to_cam=t_tab,
            tag_id=int(t_info.get("primary_id", 0)),
            tag_size_m=float(t_info.get("marker_size_m", 0.080)),
            num_samples=int(t_info.get("num_samples", 0)),
            is_valid=True,
        )
    except Exception as e:
        print(f"[WARN] Failed to parse table data from session calibration: {e}")
        return None


def load_table_calibration(path: str | Path) -> Optional[ArUcoCalibrationData]:
    """Load table calibration if available; return None otherwise."""
    p_canonical = (CONFIGS_DIR / Path(path).name).resolve()
    p_cwd = Path(path).resolve()
    target_p = p_canonical if p_canonical.is_file() else (p_cwd if p_cwd.is_file() else None)
    if target_p is None:
        return None
    try:
        calibrator = TableCalibrator(primary_id=0, secondary_id=1)
        data = calibrator.load_calibration(str(target_p))
        print(f"[INFO] Loaded valid table calibration from {target_p}")
        return data
    except Exception as e:
        print(f"[WARN] Failed to load table calibration from {target_p} ({e}). Using identity frame.")
        return None


def load_imu_calibration(path: str | Path = "configs/imu_calibration.yaml") -> Optional[str]:
    """Load IMU attitude alignment mapping string from yaml if available."""
    p_canonical = (CONFIGS_DIR / Path(path).name).resolve()
    p_cwd = Path(path).resolve()
    target_p = p_canonical if p_canonical.is_file() else (p_cwd if p_cwd.is_file() else None)
    if target_p is None:
        return None
    try:
        import yaml
        with open(target_p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            if isinstance(data, dict) and "imu_axis_mapping" in data:
                mapping = str(data["imu_axis_mapping"]).strip()
                print(f"[INFO] Loaded IMU attitude alignment from {target_p}: '{mapping}'")
                return mapping
    except Exception as e:
        print(f"[WARN] Failed to load IMU calibration from {target_p} ({e}).")
    return None


AXIS_MAPPING_DESCRIPTIONS: dict[str, str] = {
    "-y,+x,+z": "人手向右->机械臂向右, 人手向前->机械臂向前 (当前默认/床侧横置对齐)",
    "+y,-x,+z": "人手向右->机械臂向左, 人手向前->机械臂向后 (反向对调/90°安装基座备选)",
    "+y,+x,+z": "人手向右->机械臂向左, 人手向前->机械臂向前",
    "-y,-x,+z": "人手向右->机械臂向右, 人手向前->机械臂向后",
    "-x,-y,+z": "未对调: 人手向右->机械臂向后, 人手向前->机械臂向左",
    "+x,+y,+z": "未对调: 人手向右->机械臂向前, 人手向前->机械臂向右",
    "+x,-y,+z": "未对调: 人手向右->机械臂向前, 人手向前->机械臂向左",
    "-x,+y,+z": "未对调: 人手向右->机械臂向后, 人手向前->机械臂向右",
}

IMU_MAPPING_DESCRIPTIONS: dict[str, str] = {
    "+x,+y,+z": "手腕抬起(+X)->前进轴俯仰, 旋前旋后(+Y)->横向轴滚转, 手腕偏摆(+Z)->竖直轴偏航 [当前实测最优解/标准推荐]",
    "-x,-y,+z": "手腕抬起(-X)->前进轴俯仰, 旋前旋后(-Y)->横向轴滚转, 手腕偏摆(+Z)->竖直轴偏航 [反向极性备选]",
    "+x,-y,-z": "前进轴俯仰同向(+X), 横向轴滚转反向(-Y), 竖直轴偏航反向(-Z)",
    "-x,+y,-z": "前进轴俯仰反向(-X), 横向轴滚转同向(+Y), 竖直轴偏航反向(-Z)",
    "+y,-x,+z": "轴向交换备选 (Roll->Robot X, Pitch->Robot Y)",
    "+y,+x,-z": "轴向交换备选 (Roll->Robot X, Pitch反向->Robot Y)",
}


def draw_simulation_hud(
    camera_view: np.ndarray,
    clutch: ClutchController,
    orientation_mode: ArmOrientationMode,
    arm_cmd: TeleopArmCommand,
    hand_cmd: TeleopHandCommand,
    obs: RobotObservation,
    forces: dict[str, float],
    arm_metrics: ArmIKMetrics,
    hand_metrics: HandRetargetingMetrics,
    fps: float,
    imu_reading: Optional[ImuReading] = None,
    imu_tracker: Optional[WristImuTracker] = None,
    safety_report: Optional[SafetyReport] = None,
) -> np.ndarray:
    """Create a high-density, polished telemetry dashboard for the 22-DOF MuJoCo twin."""
    cam_h, cam_w = camera_view.shape[:2]
    panel_w = 640
    panel = np.zeros((cam_h, panel_w, 3), dtype=np.uint8)

    # 1. Header
    cv2.putText(panel, "TuinaDex Digital Twin Teleop (MuJoCo)", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2, cv2.LINE_AA)

    # 2. Clutch & Status Badges
    state_color = (0, 220, 0) if clutch.state == ClutchState.ENGAGED else (0, 200, 255)
    if clutch.state == ClutchState.ESTOP:
        state_color = (0, 0, 255)
    elif clutch.state == ClutchState.SAFE_HOLD:
        state_color = (255, 0, 255)
    elif clutch.state == ClutchState.SAFE_RETRACTING:
        state_color = (0, 140, 255)
    cv2.rectangle(panel, (20, 42), (200, 74), state_color, -1)
    cv2.putText(panel, f"CLUTCH: [{clutch.state.name}]", (26, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 2, cv2.LINE_AA)

    # Safety status badge
    s_level = safety_report.level if safety_report is not None else SafetyLevel.NORMAL
    if s_level == SafetyLevel.BYPASS:
        s_bg = (40, 45, 55)
        s_fg = (200, 210, 225)
        s_msg = "SAFETY: [BYPASS/FREE]"
    elif s_level == SafetyLevel.NORMAL:
        s_bg = (20, 60, 20)
        s_fg = (80, 255, 80)
        s_msg = "SAFETY: GUARDED"
    elif s_level == SafetyLevel.HOLD:
        s_bg = (30, 65, 85)
        s_fg = (0, 220, 255)
        s_msg = "SAFETY: HOLD"
    elif s_level == SafetyLevel.RETRACTING:
        s_bg = (0, 50, 130)
        s_fg = (0, 160, 255)
        v_name = safety_report.violation.name if safety_report else "RETRACT"
        s_msg = f"SAFETY: RETRACT ({v_name[:12]})"
    elif s_level == SafetyLevel.FROZEN:
        s_bg = (0, 0, 120)
        s_fg = (0, 100, 255)
        v_name = safety_report.violation.name if safety_report else "LOCK"
        s_msg = f"SAFETY: FROZEN ({v_name[:12]})"
    else:
        s_bg = (0, 0, 140)
        s_fg = (0, 0, 255)
        s_msg = "SAFETY: ESTOP"

    cv2.rectangle(panel, (208, 42), (panel_w - 95, 74), s_bg, -1)
    cv2.rectangle(panel, (208, 42), (panel_w - 95, 74), s_fg, 1)
    cv2.putText(panel, s_msg, (214, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, s_fg, 1, cv2.LINE_AA)

    fps_text = f"{fps:.1f} FPS"
    cv2.putText(panel, fps_text, (panel_w - 85, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 128), 1, cv2.LINE_AA)

    # 3. Contact Forces Badge (Tuina Pressure)
    y_pos = 98
    f_tot = forces.get("total", 0.0)
    contact_active = f_tot > 0.5
    f_color = (0, 140, 255) if contact_active else (80, 80, 90)
    cv2.rectangle(panel, (20, y_pos - 18), (panel_w - 20, y_pos + 12), f_color, -1)
    f_summary = (
        f"TUINA CONTACT: {f_tot:5.1f} N  |  "
        f"Idx: {forces.get('index', 0.0):.1f}N | "
        f"Mid: {forces.get('middle', 0.0):.1f}N | "
        f"Rng: {forces.get('ring', 0.0):.1f}N | "
        f"Thb: {forces.get('thumb', 0.0):.1f}N"
    )
    cv2.putText(panel, f_summary, (30, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255) if contact_active else (200, 200, 200), 1, cv2.LINE_AA)

    # 4. Cartesian Positions & Latencies
    y_pos += 28
    tgt_p = arm_cmd.target_pose[:3, 3]
    cv2.putText(panel, f"Arm Target (m):  [{tgt_p[0]:+.3f}, {tgt_p[1]:+.3f}, {tgt_p[2]:+.3f}]",
                (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
    y_pos += 17

    # IMU Telemetry
    if imu_reading is not None and imu_reading.is_valid:
        r, p, y = imu_reading.euler_deg
        dr, dp, dy = imu_tracker.delta_euler_deg if imu_tracker else (0.0, 0.0, 0.0)
        imu_str = f"IMU 6-DOF: R:{r:+5.1f} P:{p:+5.1f} Y:{y:+5.1f} | dRPY:[{dr:+4.1f},{dp:+4.1f},{dy:+4.1f}]"
        imu_col = (0, 255, 220)
    else:
        imu_str = "IMU: DISCONNECTED / NONE (Visual or Downward Only)"
        imu_col = (130, 130, 140)
    cv2.putText(panel, imu_str, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.40, imu_col, 1, cv2.LINE_AA)
    y_pos += 17

    imu_map_str = imu_tracker.axis_mapping_str if imu_tracker else "none"
    metric_str = (
        f"IK:{arm_metrics.computation_time_ms:.1f}ms | "
        f"PosAxis:{clutch.axis_mapping_str} | "
        f"IMUAxis:{imu_map_str}"
    )
    cv2.putText(panel, metric_str, (20, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1, cv2.LINE_AA)

    # 5. 6-DOF Arm Joints
    y_pos += 26
    cv2.line(panel, (20, y_pos - 8), (panel_w - 20, y_pos - 8), (60, 60, 70), 1)
    cv2.putText(panel, "reBot-DevArm 6-DOF Joints (Target vs Simulated):", (20, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)
    y_pos += 14

    bar_origin_x = 110
    bar_max_w = 220
    for i in range(ARM_NUM_DOFS):
        jname = CANONICAL_ARM_JOINT_NAMES[i]
        q_tgt = arm_cmd.target_q[i]
        q_act = obs.arm_actual_q[i]

        cv2.putText(panel, f"{jname:6s}", (20, y_pos + 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

        # Baseline & center tick
        cv2.rectangle(panel, (bar_origin_x, y_pos + 2), (bar_origin_x + bar_max_w, y_pos + 10), (45, 45, 50), -1)
        cx = bar_origin_x + bar_max_w // 2
        cv2.line(panel, (cx, y_pos), (cx, y_pos + 12), (100, 100, 100), 1)

        # Draw actual bar
        ratio = max(-1.0, min(1.0, q_act / 3.14))
        w = int(ratio * (bar_max_w // 2))
        if w >= 0:
            cv2.rectangle(panel, (cx, y_pos + 3), (cx + w, y_pos + 9), (0, 220, 120), -1)
        else:
            cv2.rectangle(panel, (cx + w, y_pos + 3), (cx, y_pos + 9), (0, 180, 255), -1)

        # Target tick
        t_ratio = max(-1.0, min(1.0, q_tgt / 3.14))
        tx = int(cx + t_ratio * (bar_max_w // 2))
        cv2.line(panel, (tx, y_pos), (tx, y_pos + 12), (0, 255, 255), 2)

        # Values
        cv2.putText(panel, f"T:{q_tgt:+.2f} A:{q_act:+.2f}", (bar_origin_x + bar_max_w + 10, y_pos + 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)
        y_pos += 15

    # 6. 16-DOF LEAP Hand Joints (Compact Grid)
    y_pos += 12
    cv2.line(panel, (20, y_pos - 6), (panel_w - 20, y_pos - 6), (60, 60, 70), 1)
    cv2.putText(panel, "LEAP Hand 16-DOF Actual Positions (rad):", (20, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)
    y_pos += 15

    col_w = 145
    for finger_idx, finger_name in enumerate(["Index", "Middle", "Ring", "Thumb"]):
        x_base = 20 + finger_idx * col_w
        cv2.putText(panel, f"[{finger_name}]", (x_base, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1, cv2.LINE_AA)
        for j_offset in range(4):
            hand_idx = finger_idx * 4 + j_offset
            q_val = obs.hand_actual_q[hand_idx]
            sub_name = CANONICAL_HAND_JOINT_NAMES[hand_idx].split("_")[-1]
            cv2.putText(panel, f"{sub_name[:3]}:{q_val:+.2f}", (x_base, y_pos + 13 * (j_offset + 1)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (190, 190, 190), 1, cv2.LINE_AA)

    # 7. Hotkey footer
    footer_y = cam_h - 15
    cv2.line(panel, (20, footer_y - 15), (panel_w - 20, footer_y - 15), (60, 60, 70), 1)
    footer_text = "[C] Clutch | [S] Safety | [Z] Zero Hand | [I] Zero IMU | [M] Mode | [A] Pos | [O] Orient"
    cv2.putText(panel, footer_text, (10, footer_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 150), 1, cv2.LINE_AA)

    return np.hstack([camera_view, panel])


def main():
    parser = argparse.ArgumentParser(description="22-DOF Composite Robot Teleoperation in MuJoCo Digital Twin")
    parser.add_argument("--mock", action="store_true", help="Run with mock camera stream")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless terminal mode (no OpenCV/Viewer)")
    parser.add_argument("--max-frames", type=int, default=0, help="Exit after N frames (for automated tests)")
    parser.add_argument("--session-config", type=str, default=str(CONFIGS_DIR / "session_calibration.yaml"), help="Unified session calibration YAML")
    parser.add_argument("--config", type=str, default=str(CONFIGS_DIR / "table_calibration.yaml"), help="Table calibration yaml")
    parser.add_argument("--model", type=str, default=str(CONFIGS_DIR / "mjcf" / "tuina_sim_scene.xml"), help="MuJoCo scene XML")
    parser.add_argument(
        "--solver-mode",
        type=str,
        default="hybrid",
        choices=["hybrid", "geometric", "dex_retargeting"],
        help="LEAP Hand retargeting solver mode (hybrid, geometric, dex_retargeting)",
    )
    parser.add_argument(
        "--axis-mapping",
        type=str,
        default=None,
        help="Direction alignment mapping from table to robot base (e.g. '-y,+x,+z', '+y,-x,+z', default: auto-load from session config or '-y,+x,+z')",
    )
    parser.add_argument(
        "--imu-port",
        type=str,
        default="/dev/myimu",
        help="Serial port for 6-axis IMU attitude sensor (default: /dev/myimu)",
    )
    parser.add_argument(
        "--imu-axis-mapping",
        type=str,
        default=None,
        help="Attitude alignment mapping from IMU to robot base (e.g. '+x,+y,+z', '-x,-y,+z', default: auto-load configs/session_calibration.yaml or '+x,+y,+z')",
    )
    parser.add_argument(
        "--mock-imu",
        action="store_true",
        help="Run with simulated mock IMU stream (even if physical sensor is plugged in)",
    )
    parser.add_argument(
        "--no-imu",
        action="store_true",
        help="Disable IMU tracking and run in vision/fixed downward orientation mode",
    )
    parser.add_argument(
        "--auto-engage",
        action="store_true",
        help="Automatically engage clutch on first valid hand detection (useful for unattended mock CI)",
    )
    parser.add_argument(
        "--no-safety",
        action="store_true",
        help="Start with safety guardrails bypassed (Unrestricted mode, like baseline before safety additions)",
    )
    args = parser.parse_args()

    print("=" * 75)
    print("  TuinaDex 22-DOF Composite Digital Twin Teleoperation (MuJoCo)")
    print(f"  Robot: reBot-DevArm (6-DOF) + LEAP Hand (16-DOF)")
    print(f"  Scene: {args.model}")
    print(f"  Mode:  {'Headless Terminal' if args.no_gui else 'Interactive MuJoCo + OpenCV GUI'}")
    print("=" * 75)

    # 1. Perception setup
    session_calib = load_session_calibration(args.session_config)

    # Table calibration: prioritize session calibration, fallback to legacy args.config
    calib_data = None
    if session_calib is not None:
        calib_data = parse_table_from_session(session_calib)
    if calib_data is None:
        calib_data = load_table_calibration(args.config)
    t_cam_to_table = calib_data.t_cam_to_table if calib_data else None

    if args.mock:
        print("[INFO] Starting MOCK camera stream...")
        stream = MockRealSenseStream()
    else:
        print("[INFO] Connecting to Intel RealSense D455...")
        try:
            stream = RealSenseStream(width=640, height=480, fps=30)
        except Exception as e:
            print(f"[ERROR] Could not initialize physical RealSense camera: {e}")
            print("[INFO] Fallback: launch with '--mock'")
            return

    intrinsics = stream.start()
    print(f"[INFO] Camera started: 640x480, fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")

    # Setup 6-axis IMU Stream
    if args.no_imu:
        imu_stream = None
        print("[INFO] IMU tracking disabled by user (--no-imu).")
    elif args.mock_imu:
        print("[INFO] Starting MOCK IMU stream...")
        imu_stream = MockImuStream()
        imu_stream.start()
    else:
        print(f"[INFO] Connecting to 6-axis IMU on {args.imu_port}...")
        imu_stream = ImuStream(port=args.imu_port, auto_detect=True)
        if not imu_stream.start():
            print("[INFO] Physical IMU not found. Using MOCK IMU stream for simulation.")
            imu_stream = MockImuStream()
            imu_stream.start()

    # 2. Kinematics & Controllers
    wrist_tracker = WristTracker(default_handedness="Right")
    if args.mock:
        cx, cy = intrinsics.cx, intrinsics.cy
        def synthetic_mock_hand(img):
            t = time.monotonic()
            dx = 60.0 * np.sin(t * 1.5)
            dy = 40.0 * np.cos(t * 1.5)
            lm = np.zeros((21, 3), dtype=np.float32)
            lm[0] = [(cx + dx) / intrinsics.width, (cy + dy) / intrinsics.height, 0.0]
            lm[5] = [(cx + dx - 20) / intrinsics.width, (cy + dy - 35) / intrinsics.height, 0.0]
            lm[9] = [(cx + dx) / intrinsics.width, (cy + dy - 40) / intrinsics.height, 0.0]
            lm[17] = [(cx + dx + 20) / intrinsics.width, (cy + dy - 35) / intrinsics.height, 0.0]
            return {"landmarks": lm, "handedness": "Right"}
        wrist_tracker.set_custom_detector(synthetic_mock_hand)

    arm_ik = ArmIKSolver(
        urdf_path=CONFIGS_DIR / "urdf" / "rebot_arm_rs.urdf",
        ee_frame_name="link6",
        home_q=DEFAULT_READY_ARM_Q,
    )
    home_pos, home_R = arm_ik.forward_kinematics(arm_ik.home_q)

    # Position alignment: CLI arg -> session config -> legacy table config -> fallback default "+y,-x,+z"
    session_pos_map = None
    if session_calib is not None and "table" in session_calib:
        session_pos_map = session_calib["table"].get("axis_mapping")
    if session_pos_map is None and Path(args.config).is_file():
        try:
            with open(args.config, "r", encoding="utf-8") as f:
                _t_yaml = yaml.safe_load(f) or {}
                if isinstance(_t_yaml, dict) and "axis_mapping" in _t_yaml:
                    session_pos_map = _t_yaml["axis_mapping"]
        except Exception:
            pass

    initial_pos_map = (
        args.axis_mapping
        if args.axis_mapping is not None
        else (session_pos_map if session_pos_map is not None else "-y,+x,+z")
    )
    print(f"[INFO] Initial position direction mapping: '{initial_pos_map}' ({AXIS_MAPPING_DESCRIPTIONS.get(initial_pos_map, 'custom')})")

    clutch = ClutchController(
        scale=1.0,
        workspace_x=arm_ik.workspace_x,
        workspace_y=arm_ik.workspace_y,
        workspace_z=arm_ik.workspace_z,
        axis_mapping=initial_pos_map,
    )

    # IMU alignment: check session config -> legacy config -> fallback default
    session_imu_map = None
    session_imu_quat = None
    if session_calib is not None and "imu" in session_calib:
        session_imu_map = session_calib["imu"].get("imu_axis_mapping")
        session_imu_quat = session_calib["imu"].get("neutral_quat_wxyz")
    loaded_imu_map = session_imu_map if session_imu_map else load_imu_calibration()
    initial_imu_map = (
        args.imu_axis_mapping
        if args.imu_axis_mapping is not None
        else (loaded_imu_map if loaded_imu_map is not None else "+x,+y,+z")
    )
    print(f"[INFO] Initial IMU attitude alignment mapping: '{initial_imu_map}'")

    wrist_imu_tracker = WristImuTracker(
        axis_mapping=initial_imu_map,
        neutral_robot_rot=home_R,
    )
    if session_imu_quat is not None and len(session_imu_quat) == 4:
        wrist_imu_tracker.set_reference_quat(session_imu_quat)
        neu_rpy = session_calib["imu"].get("neutral_rpy_deg", [0, 0, 0])
        print(f"[INFO] Applied pre-calibrated IMU zero reference from session (RPY={neu_rpy} deg).")

    hand_retargeter = HandRetargeter(
        urdf_path=str(CONFIGS_DIR / "urdf" / "leap_hand_right.urdf"),
        solver_mode=args.solver_mode,
        scaling_factor=1.60,
    )
    if session_calib is not None and "hand" in session_calib:
        h_info = session_calib["hand"]
        q_zero = h_info.get("q_zero_offset")
        palm_l = h_info.get("measured_palm_length_m")
        hand_retargeter.set_calibration(q_zero_offset=q_zero, palm_reference_length_m=palm_l)
        print(f"[INFO] Applied hand calibration from session: palm={palm_l*100:.1f}cm, zero-offset active.")

    # 3. MuJoCo Simulation Adapter
    sim_adapter = MujocoSimAdapter(
        model_path=CONFIGS_DIR / "mjcf" / "tuina_sim_scene.xml" if args.model == "configs/mjcf/tuina_sim_scene.xml" else Path(args.model),
        headless=args.no_gui,
    )
    if not sim_adapter.connect():
        print("[ERROR] Failed to connect MujocoSimAdapter.")
        stream.stop()
        if imu_stream is not None:
            imu_stream.stop()
        return
    print("[INFO] MujocoSimAdapter connected successfully!")

    # Initial state reading
    obs = sim_adapter.read_observation()
    forces = sim_adapter.get_contact_forces()

    # 4. Multi-tier Safety Supervisor
    safety_cfg = SafetyConfig()
    safety_enabled = not args.no_safety
    teleop_cfg_p = CONFIGS_DIR / "teleop_config.yaml"
    if teleop_cfg_p.is_file():
        try:
            with open(teleop_cfg_p, "r", encoding="utf-8") as f:
                _t_cfg = yaml.safe_load(f) or {}
                if "safety" in _t_cfg and isinstance(_t_cfg["safety"], dict):
                    s_dict = _t_cfg["safety"]
                    if not args.no_safety:
                        safety_enabled = bool(s_dict.get("enabled", True))
                    safety_cfg = SafetyConfig(
                        hand_loss_timeout_s=float(s_dict.get("hand_loss_timeout_s", 0.80)),
                        max_latency_s=float(s_dict.get("max_latency_s", 0.25)),
                        max_pos_jump_m=float(s_dict.get("max_pos_jump_m", 0.18)),
                        max_rot_jump_deg=float(s_dict.get("max_rot_jump_deg", 60.0)),
                        max_hand_q_jump_rad=float(s_dict.get("max_hand_q_jump_rad", 1.50)),
                        min_singular_value=float(s_dict.get("min_singular_value", 0.010)),
                        min_manipulability=float(s_dict.get("min_manipulability", 0.0005)),
                        joint_limit_margin_rad=float(s_dict.get("joint_limit_margin_rad", 0.02)),
                        max_total_force_n=float(s_dict.get("max_total_force_n", 120.0)),
                        max_finger_force_n=float(s_dict.get("max_finger_force_n", 70.0)),
                        max_force_rate_n_s=float(s_dict.get("max_force_rate_n_s", 400.0)),
                        min_force_rate_trigger_n=float(s_dict.get("min_force_rate_trigger_n", 10.0)),
                        lift_off_height_m=float(s_dict.get("lift_off_height_m", 0.06)),
                        lift_off_duration_s=float(s_dict.get("lift_off_duration_s", 0.6)),
                        retract_duration_s=float(s_dict.get("retract_duration_s", 1.8)),
                    )
        except Exception as e:
            print(f"[WARN] Failed to load safety config from teleop_config.yaml: {e}")
    safety_supervisor = SafetySupervisor(safety_cfg, enabled=safety_enabled)
    safety_report = SafetyReport(
        level=SafetyLevel.NORMAL if safety_enabled else SafetyLevel.BYPASS,
        violation=SafetyViolation.NONE,
        message="Safety supervisor initialized" if safety_enabled else "Safety guardrails BYPASS",
        timestamp=time.monotonic(),
    )
    status_label = f"ACTIVE (Force ceiling: {safety_cfg.max_total_force_n}N, Jump: {safety_cfg.max_pos_jump_m*100:.0f}cm)" if safety_enabled else "BYPASS (无安全门限自由模式)"
    print(f"[INFO] Safety Supervisor: {status_label}")

    orientation_modes = [
        ArmOrientationMode.TRACK_IMU,
        ArmOrientationMode.FIXED_DOWNWARD,
        ArmOrientationMode.TRACK_WRIST,
        ArmOrientationMode.POSITION_ONLY,
    ]
    mode_idx = 0 if imu_stream is not None else 1
    current_mode = orientation_modes[mode_idx]

    frame_count = 0
    fps = 30.0
    dt = 1.0 / 30.0
    t_last = time.time()
    t_print = time.time()

    axis_presets = [
        "-y,+x,+z",
        "+y,-x,+z",
        "+y,+x,+z",
        "-y,-x,+z",
        "-x,-y,+z",
        "+x,+y,+z",
        "+x,-y,+z",
        "-x,+y,+z",
    ]
    if initial_pos_map.lower() in axis_presets:
        axis_preset_idx = axis_presets.index(initial_pos_map.lower())
    else:
        axis_presets.insert(0, initial_pos_map.lower())
        axis_preset_idx = 0

    imu_presets = [
        "+x,+y,+z",
        "-x,-y,+z",
        "+x,-y,-z",
        "-x,+y,-z",
        "+y,-x,+z",
        "+y,+x,-z",
    ]
    if initial_imu_map.lower() in imu_presets:
        imu_preset_idx = imu_presets.index(initial_imu_map.lower())
    else:
        imu_presets.insert(0, initial_imu_map.lower())
        imu_preset_idx = 0

    print("\n===========================================================================")
    print(" TuinaDex 22-DOF Teleoperation Live — Controls:")
    print("   'c' / SPACE : Toggle Clutch (Engage / Disengage & Zero IMU)")
    print("   's' / 'S'   : Toggle Safety Guardrails (ACTIVE 有门限 / BYPASS 自由模式)")
    print("   'i'         : Zero/Calibrate IMU reference posture")
    print("   'z'         : Zero Hand open posture (人手自然张开归零复位，关节完全展开)")
    print("   'e'         : Emergency Stop (ESTOP)")
    print("   'r'         : Reset / Re-zero all filters and simulator")
    print("   'm'         : Cycle orientation mode (TRACK_IMU, FIXED_DOWNWARD, TRACK_WRIST, POSITION_ONLY)")
    print("   'a'         : Cycle position direction mapping (-y,+x,+z -> +y,-x,+z -> ...)")
    print("   'o'         : Cycle IMU orientation mapping (+x,+y,+z -> -x,-y,+z -> ...)")
    print("   '1' / '2' / '3' : Switch hand solver (1:Hybrid, 2:Geometric, 3:DexPilot)")
    print("   'q' or ESC  : Exit")
    print("===========================================================================\n")

    try:
        while True:
            t_loop_start = time.time()
            color, depth, ts = stream.read_frames()
            if color is None:
                time.sleep(0.005)
                continue

            frame_count += 1
            t_now = ts if ts is not None else time.monotonic()

            if args.mock and depth is None:
                depth = np.ones((intrinsics.height, intrinsics.width), dtype=np.float32) * 0.65

            # 1. Perception: Wrist tracking & hand landmarks
            wrist_pose = wrist_tracker.process_frame(
                color_bgr=color,
                depth_m=depth,
                intrinsics=intrinsics,
                table_calib=calib_data,
                timestamp=t_now,
            )

            # Read IMU 6-axis attitude
            imu_reading = imu_stream.read() if imu_stream is not None else ImuReading()

            # 2. Hand Retargeting (16-DOF LEAP Hand)
            hand_cmd = hand_retargeter.retarget_pose(wrist_pose, timestamp=t_now)
            hand_metrics = hand_retargeter.metrics

            # Current actual robot end-effector Cartesian pose
            cur_arm_p, cur_arm_R = arm_ik.forward_kinematics(obs.arm_actual_q)

            # 3. Multi-tier Safety Evaluation across 4 Gates
            safety_report = safety_supervisor.evaluate(
                t_now=t_now,
                wrist_tracking_valid=(wrist_pose.tracking_state == TrackingState.TRACKING),
                wrist_pos=wrist_pose.position_table,
                wrist_rot=wrist_pose.orientation_table_matrix,
                wrist_timestamp=wrist_pose.timestamp,
                current_arm_q=obs.arm_actual_q,
                current_arm_pos=cur_arm_p,
                current_arm_rot=cur_arm_R,
                current_hand_q=obs.hand_actual_q,
                home_arm_q=arm_ik.home_q,
                contact_forces=forces,
                clutch_state=clutch.state,
                arm_ik_solver=arm_ik,
            )

            # 4. Target Arbitration & Safety Guardrail Actions
            if safety_report.level == SafetyLevel.RETRACTING and safety_supervisor.retract_controller is not None:
                clutch.set_safe_retracting()
                retract_pos, retract_rot, retract_arm_q, retract_hand_q, finished = (
                    safety_supervisor.step_retract(t_now, obs.arm_actual_q)
                )
                hand_cmd.target_q = retract_hand_q.copy()
                if retract_arm_q is not None:
                    # Stage 2: Direct joint-space homing
                    fk_p, fk_R = arm_ik.forward_kinematics(retract_arm_q)
                    target_pose = np.eye(4, dtype=np.float64)
                    target_pose[:3, :3] = fk_R
                    target_pose[:3, 3] = fk_p
                    arm_cmd = TeleopArmCommand(
                        timestamp=t_now,
                        target_pose=target_pose,
                        target_q=retract_arm_q,
                        target_dq=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
                        clutch_state=clutch.state,
                    )
                    arm_ik.current_q = retract_arm_q.copy()
                else:
                    # Stage 1: Cartesian vertical lift-off (+Z)
                    arm_cmd = arm_ik.solve(
                        target_pos=retract_pos,
                        target_rot=retract_rot,
                        orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
                        dt=1.0 / max(1.0, fps),
                        clutch_state=clutch.state,
                        timestamp=t_now,
                    )
                if finished:
                    clutch.disengage()

            elif safety_report.level == SafetyLevel.FROZEN:
                clutch.set_safe_hold()
                target_pose = np.eye(4, dtype=np.float64)
                target_pose[:3, :3] = safety_supervisor.frozen_arm_rot
                target_pose[:3, 3] = safety_supervisor.frozen_arm_pos
                arm_cmd = TeleopArmCommand(
                    timestamp=t_now,
                    target_pose=target_pose,
                    target_q=safety_supervisor.frozen_arm_q.copy(),
                    target_dq=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
                    clutch_state=clutch.state,
                )
                hand_cmd.target_q = safety_supervisor.frozen_hand_q.copy()

            else:
                # Normal or Transient Hold Teleoperation
                if wrist_pose.tracking_state == TrackingState.TRACKING and wrist_pose.position_table is not None:
                    if args.auto_engage and clutch.state == ClutchState.DISENGAGED and clutch._human_anchor is None:
                        clutch.engage(
                            current_human_pos=wrist_pose.position_table,
                            current_robot_pos=home_pos,
                            current_human_rot=wrist_pose.orientation_table_matrix,
                            current_robot_rot=home_R,
                        )
                        if imu_reading.is_valid and not wrist_imu_tracker.is_calibrated:
                            wrist_imu_tracker.zero(imu_reading)
                    target_pos = clutch.compute_target(wrist_pose.position_table, fallback_robot_pos=home_pos)
                else:
                    target_pos = clutch.compute_target(np.zeros(3), fallback_robot_pos=home_pos)

                # Target orientation
                if current_mode == ArmOrientationMode.TRACK_IMU and imu_reading.is_valid:
                    R_target = wrist_imu_tracker.compute_target_rotation(
                        reading=imu_reading,
                        fallback_rot=home_R,
                        anchor_robot_rot=home_R,
                    )
                elif current_mode == ArmOrientationMode.FIXED_DOWNWARD:
                    R_target = R_DOWNWARD_DEFAULT.copy()
                elif current_mode == ArmOrientationMode.TRACK_WRIST:
                    R_target = wrist_pose.orientation_table_matrix if wrist_pose.orientation_table_matrix is not None else R_DOWNWARD_DEFAULT.copy()
                else:
                    R_target = None

                # 4. Arm QP-IK Solve (6-DOF reBot-DevArm)
                arm_cmd = arm_ik.solve(
                    target_pos=target_pos,
                    target_rot=R_target,
                    orientation_mode=current_mode,
                    dt=1.0 / max(1.0, fps),
                    clutch_state=clutch.state,
                    timestamp=t_now,
                )
            arm_metrics = arm_ik.get_metrics()

            # 5. Composite 22-DOF Command
            composite_cmd = TeleopCommand(
                timestamp=t_now,
                arm_command=arm_cmd,
                hand_command=hand_cmd,
                clutch_state=clutch.state,
            )

            # 6. MuJoCo Physics Step: Synchronize physical advancement to real-time frame duration
            dt_sim = max(0.002, min(0.080, dt)) if dt > 0.0 else (1.0 / 30.0)
            sim_dt = sim_adapter.model.opt.timestep if sim_adapter.model is not None else 0.002
            n_substeps = max(1, min(30, int(round(dt_sim / sim_dt))))
            sim_adapter.send_command(composite_cmd)
            sim_adapter.step(n_steps=n_substeps)

            # 7. Read Real-time State & Contact Telemetry
            obs = sim_adapter.read_observation()
            forces = sim_adapter.get_contact_forces()

            # FPS calculation
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now
            if dt > 0.0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            tgt_p = arm_cmd.target_pose[:3, 3]

            # Terminal log in headless mode (every 0.5s)
            if args.no_gui:
                if t_now - t_print >= 0.5:
                    t_print = t_now
                    tot_f = forces.get("total", 0.0)
                    dr, dp, dy = wrist_imu_tracker.delta_euler_deg
                    print(
                        f"[{frame_count:04d}] {wrist_pose.tracking_state.name:10s} | "
                        f"Safety:{safety_report.level.name:10s} | "
                        f"Clutch:{clutch.state.name:10s} | "
                        f"Mode:{current_mode.name:14s} | "
                        f"ArmTgt:[{tgt_p[0]:+.2f},{tgt_p[1]:+.2f},{tgt_p[2]:+.2f}] | "
                        f"dRPY:[{dr:+3.0f},{dp:+3.0f},{dy:+3.0f}] | "
                        f"Forces:{tot_f:4.1f}N | "
                        f"IK:{arm_metrics.computation_time_ms:4.1f}ms | "
                        f"FPS:{fps:4.1f}"
                    )
            else:
                # GUI Dashboard
                hud_cam = draw_wrist_hud(color, wrist_pose, intrinsics)
                dashboard = draw_simulation_hud(
                    hud_cam,
                    clutch,
                    current_mode,
                    arm_cmd,
                    hand_cmd,
                    obs,
                    forces,
                    arm_metrics,
                    hand_metrics,
                    fps,
                    imu_reading=imu_reading,
                    imu_tracker=wrist_imu_tracker,
                    safety_report=safety_report,
                )
                cv2.imshow("TuinaDex 22-DOF Digital Twin Teleoperation", dashboard)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    print("\n[INFO] User requested exit.")
                    break
                elif key in (32, ord("c")):
                    tot_f = forces.get("total", 0.0)
                    if clutch.state != ClutchState.ENGAGED and tot_f > 40.0:
                        print(f"\n[CLUTCH WARN] 无法接管: 当前接触力 ({tot_f:.1f}N) 过大，请先抬起机械臂脱离接触！")
                    else:
                        cur_ee_p, cur_ee_R = arm_ik.forward_kinematics(obs.arm_actual_q)
                        clutch.toggle(
                            current_human_pos=wrist_pose.position_table if wrist_pose.position_table is not None else None,
                            current_robot_pos=cur_ee_p,
                            current_human_rot=wrist_pose.orientation_table_matrix,
                            current_robot_rot=cur_ee_R,
                        )
                        if clutch.state == ClutchState.ENGAGED and imu_reading.is_valid:
                            wrist_imu_tracker.zero(imu_reading)
                        print(f"\n[CLUTCH] State changed to: {clutch.state.name} (IMU Zero-Referenced)")
                elif key in (ord("s"), ord("S")):
                    is_on = safety_supervisor.toggle_enabled()
                    mode_str = "ACTIVE (有安全门限模式)" if is_on else "BYPASS (无安全门限/原始自由模式)"
                    print(f"\n[SAFETY] 安全门限模式已切换为: {mode_str}")
                elif key == ord("i"):
                    if imu_reading.is_valid:
                        wrist_imu_tracker.zero(imu_reading)
                        rpy = imu_reading.euler_deg
                        print(f"\n[IMU] Zero-reference posture re-latched at RPY=({rpy[0]:+.1f}, {rpy[1]:+.1f}, {rpy[2]:+.1f}) deg.")
                    else:
                        print("\n[IMU] Warning: IMU reading invalid or offline, cannot latch zero reference.")
                elif key in (ord("z"), ord("Z")):
                    if wrist_pose.is_valid and wrist_pose.tracking_state == TrackingState.TRACKING:
                        zero_q = hand_retargeter.zero(wrist_pose)
                        hand_cmd.target_q = zero_q.copy()
                        composite_cmd.hand_command = hand_cmd
                        sim_adapter.send_command(composite_cmd)
                        sim_adapter.step(n_steps=1)
                        print("\n[HAND] 灵巧手自然张开归零成功：关节全部复位展开 (所有 16 关节对齐归零姿态 Q=0)")
                        try:
                            for session_p in [
                                CONFIGS_DIR / "session_calibration.yaml",
                                PROJECT_ROOT.parent / "configs" / "session_calibration.yaml",
                            ]:
                                if session_p.is_file():
                                    with open(session_p, "r", encoding="utf-8") as f:
                                        s_data = yaml.safe_load(f) or {}
                                    if "hand" in s_data and hand_retargeter.q_zero_offset is not None:
                                        s_data["hand"]["q_zero_offset"] = [round(float(v), 4) for v in hand_retargeter.q_zero_offset]
                                        with open(session_p, "w", encoding="utf-8") as f:
                                            yaml.dump(s_data, f, default_flow_style=False)
                                        print(f"[HAND] 已同步保存最新平手归零向量至: {session_p}")
                        except Exception:
                            pass
                    else:
                        zero_q = hand_retargeter.reset_to_zero_pose()
                        hand_cmd.target_q = zero_q.copy()
                        composite_cmd.hand_command = hand_cmd
                        sim_adapter.send_command(composite_cmd)
                        sim_adapter.step(n_steps=1)
                        print("\n[HAND] 提示: 未检测到人手，已强制将灵巧手关节复位归零 (Open Zero Posture)")
                elif key == ord("e"):
                    clutch.estop()
                    safety_supervisor.level = SafetyLevel.ESTOP
                    print(f"\n[ESTOP] Emergency Stop activated!")
                elif key == ord("r"):
                    wrist_tracker.reset()
                    wrist_imu_tracker.reset()
                    safety_supervisor.reset()
                    clutch.reset(safe_robot_pos=home_pos, safe_robot_rot=home_R)
                    arm_ik.reset()
                    sim_adapter.reset()
                    print("\n[RESET] Filters, Safety Supervisor, IMU zero, and simulator reset to ready pose.")
                    print("\n[RESET] Filters, IMU zero, and simulator reset to ready pose.")
                elif key == ord("m"):
                    mode_idx = (mode_idx + 1) % len(orientation_modes)
                    current_mode = orientation_modes[mode_idx]
                    print(f"\n[MODE] Switched to: {current_mode.name}")
                elif key == ord("a"):
                    axis_preset_idx = (axis_preset_idx + 1) % len(axis_presets)
                    new_map = axis_presets[axis_preset_idx]
                    clutch.set_axis_mapping(new_map)
                    desc = AXIS_MAPPING_DESCRIPTIONS.get(new_map, "自定义矩阵")
                    print(f"\n[POS AXIS] 切换末端平移映射为: '{new_map}' | {desc}")
                elif key == ord("o"):
                    imu_preset_idx = (imu_preset_idx + 1) % len(imu_presets)
                    new_imu_map = imu_presets[imu_preset_idx]
                    wrist_imu_tracker.set_axis_mapping(new_imu_map)
                    if imu_reading.is_valid:
                        wrist_imu_tracker.zero(imu_reading)
                    desc = IMU_MAPPING_DESCRIPTIONS.get(new_imu_map, "")
                    print(f"\n[IMU ORIENTATION] 切换姿态映射为: '{new_imu_map}' ({desc}) (已重新归零)")
                elif key == ord("1"):
                    hand_retargeter.solver_mode = "hybrid"
                    print("\n[HAND SOLVER] Switched to: HYBRID (Biomechanical + Pinch Optimization)")
                elif key == ord("2"):
                    hand_retargeter.solver_mode = "geometric"
                    print("\n[HAND SOLVER] Switched to: GEOMETRIC (Direct Bone Flexion with Ulnar Synergy)")
                elif key == ord("3"):
                    hand_retargeter.solver_mode = "dex_retargeting"
                    print("\n[HAND SOLVER] Switched to: DEXPILOT (Pinocchio Kinematic Optimization)")

            if args.max_frames > 0 and frame_count >= args.max_frames:
                print(f"\n[INFO] Reached max frames ({args.max_frames}). Exiting.")
                break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by keyboard.")
    finally:
        stream.stop()
        if imu_stream is not None:
            imu_stream.stop()
        sim_adapter.disconnect()
        if not args.no_gui:
            cv2.destroyAllWindows()
        print(f"[INFO] Closed simulation pipeline. Total frames processed: {frame_count}.")


if __name__ == "__main__":
    main()
