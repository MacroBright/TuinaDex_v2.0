"""teleop_hud.py — High-density, polished telemetry dashboard for 22-DOF Composite Teleoperation."""
from __future__ import annotations

from typing import Optional
import cv2
import numpy as np

from ..core.clutch import ClutchController
from ..core.names import (
    ARM_NUM_DOFS,
    CANONICAL_ARM_JOINT_NAMES,
    CANONICAL_HAND_JOINT_NAMES,
)
from ..core.safety import SafetyLevel, SafetyReport
from ..core.types import (
    ClutchState,
    RobotObservation,
    TeleopArmCommand,
    TeleopHandCommand,
)
from ..kinematics.arm_ik_solver import ArmIKMetrics, ArmOrientationMode
from ..kinematics.hand_retargeter import HandRetargetingMetrics
from ..perception.imu_stream import (
    AXIS_MAPPING_CYCLE,
    HotkeyState,
    ImuReading,
    WristImuTracker,
    cycle_axis_mapping,
    cycle_orientation_mode,
    hotkey_allowed,
    swap_axis_mapping as swap_xy_axes,
)



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
    toast: Optional[str] = None,
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
    elif orientation_mode == ArmOrientationMode.FIXED_DOWNWARD and (imu_reading is None or not imu_reading.is_valid):
        imu_str = "IMU: DISABLED (no_imu=true | Fixed Downward Mode)"
        imu_col = (130, 130, 140)
    else:
        imu_str = "IMU: DISCONNECTED (Check /dev/myimu or /dev/ttyUSB*)"
        imu_col = (0, 120, 255)
    cv2.putText(panel, imu_str, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.40, imu_col, 1, cv2.LINE_AA)
    y_pos += 17

    imu_map_str = imu_tracker.axis_mapping_str if imu_tracker else "none"
    metric_str = (
        f"IK:{arm_metrics.computation_time_ms:.1f}ms | "
        f"PosAxis:{clutch.axis_mapping_str} | "
        f"IMUAxis:{imu_map_str} | "
        f"Mode:{orientation_mode.name}"
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
    footer_text = "[C] Clutch | [S] Safety | [Z] Zero Hand | [I] Zero IMU | [M] Mode | [A] Pos | [X] IMU XY"
    cv2.putText(panel, footer_text, (10, footer_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 150), 1, cv2.LINE_AA)

    if toast:
        (text_w, text_h), _ = cv2.getTextSize(
            toast, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
        )
        x0 = max(12, (panel_w - text_w) // 2)
        y0 = cam_h - 52
        cv2.rectangle(
            panel, (x0 - 12, y0 - text_h - 8), (x0 + text_w + 12, y0 + 8),
            (30, 30, 120), -1,
        )
        cv2.putText(panel, toast, (x0, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

    return np.hstack([camera_view, panel])
