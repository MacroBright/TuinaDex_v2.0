#!/usr/bin/env python3
"""calibrate_imu.py — Interactive 3-Gesture IMU Wrist Attitude Alignment and Zero Calibration Tool.

This tool solves the coordinate alignment mapping between a wrist-worn 6-axis IMU (e.g. Yahboom CH340)
and the 6-DOF robot arm end-effector, guaranteeing strict right-handedness SO(3) det(R) = +1.0.

3-Step Procedure:
  1. Neutral Baseline (平放桌面零位): Hand flat on desk, palm down, pointing straight forward.
  2. Pitch Up (手腕上翘俯仰): Tilt fingers upward ~25°–45° (dorsiflexion).
  3. Roll Right (手腕向右翻滚): Roll wrist to the right ~25°–45° (pronation/supination).

Results are saved to:
  - src/tuinadex_teleop/configs/session_calibration.yaml
  - src/tuinadex_teleop/configs/imu_calibration.yaml
"""
from __future__ import annotations

import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

# Add workspace source directories to path
_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "src" / "tuinadex_teleop"))

from tuinadex_teleop.perception.imu_stream import (
    ImuStream,
    MockImuStream,
    ImuReading,
    solve_imu_alignment_from_gestures,
    parse_axis_mapping,
    rotation_matrix_to_quat,
    rotation_matrix_to_euler_deg,
)


def average_quaternions(quats: List[np.ndarray]) -> np.ndarray:
    """Compute average unit quaternion using Markley's eigenvector method.
    
    Args:
        quats: List of (4,) quaternions in [w, x, y, z] order.
    Returns:
        (4,) normalized average quaternion in [w, x, y, z] order.
    """
    if not quats:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    A = np.zeros((4, 4), dtype=np.float64)
    for q in quats:
        q_norm = q / np.linalg.norm(q)
        A += np.outer(q_norm, q_norm)
    A /= len(quats)
    eigenvalues, eigenvectors = np.linalg.eigh(A)
    avg_q = eigenvectors[:, np.argmax(eigenvalues)]
    if avg_q[0] < 0.0:
        avg_q = -avg_q
    return avg_q / np.linalg.norm(avg_q)


def sample_stable_reading(
    stream: Any,
    num_samples: int = 15,
    sample_interval_s: float = 0.03,
) -> ImuReading:
    """Sample multiple IMU readings and compute smooth filtered attitude."""
    samples: List[ImuReading] = []
    quats: List[np.ndarray] = []
    start_time = time.time()

    for _ in range(num_samples):
        reading = stream.read()
        if reading.is_valid:
            samples.append(reading)
            quats.append(reading.quat)
        time.sleep(sample_interval_s)

    if not samples:
        # Fallback single read if burst was empty
        reading = stream.read()
        if not reading.is_valid:
            raise RuntimeError("Failed to collect valid IMU samples from stream.")
        return reading

    avg_quat = average_quaternions(quats)
    from tuinadex_teleop.perception.imu_stream import quat_to_rotation_matrix
    avg_R = quat_to_rotation_matrix(avg_quat)
    avg_euler = rotation_matrix_to_euler_deg(avg_R)
    avg_rad = tuple(math.radians(a) for a in avg_euler)

    return ImuReading(
        timestamp=time.time(),
        quat=avg_quat,
        rotation_matrix=avg_R,
        euler_deg=avg_euler,
        euler_rad=avg_rad,
        is_valid=True,
    )


def save_calibration_results(
    mapping_str: str,
    neutral_quat: np.ndarray,
    neutral_rpy_deg: Tuple[float, float, float],
    device_port: str,
    session_yaml_path: Path,
    imu_yaml_path: Path,
) -> None:
    """Persist calibration parameters to standard yaml config files."""
    imu_payload = {
        "is_valid": True,
        "device_port": str(device_port),
        "imu_axis_mapping": str(mapping_str),
        "mounting": "Calibrated via interactive calibrate_imu.py 3-gesture routine",
        "neutral_quat_wxyz": [float(x) for x in neutral_quat],
        "neutral_rpy_deg": [float(round(x, 2)) for x in neutral_rpy_deg],
    }

    # 1. Update session_calibration.yaml (preserve other keys like hand, table, metadata)
    session_data: Dict[str, Any] = {}
    if session_yaml_path.exists():
        try:
            with open(session_yaml_path, "r", encoding="utf-8") as f:
                loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    session_data = loaded
        except Exception as e:
            print(f"[WARN] Failed to read existing session config ({e}), generating fresh dict.")

    session_data["imu"] = imu_payload
    if "metadata" not in session_data:
        session_data["metadata"] = {}
    session_data["metadata"]["calibrated_at"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    session_data["metadata"]["device"] = "reBot-DevArm + LEAP Hand Right"

    session_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with open(session_yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(session_data, f, sort_keys=False, allow_unicode=True)
    print(f"  [OK] Saved to: {session_yaml_path}")

    # 2. Update standalone imu_calibration.yaml
    imu_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with open(imu_yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(imu_payload, f, sort_keys=False, allow_unicode=True)
    print(f"  [OK] Saved to: {imu_yaml_path}")


def run_calibration(
    port: str = "/dev/myimu",
    baudrate: int = 115200,
    mock: bool = False,
    auto: bool = False,
    session_yaml_path: Optional[Path] = None,
    imu_yaml_path: Optional[Path] = None,
) -> Tuple[str, np.ndarray]:
    """Execute 3-gesture calibration flow."""
    session_path = session_yaml_path or (_REPO_ROOT / "src/tuinadex_teleop/configs/session_calibration.yaml")
    imu_path = imu_yaml_path or (_REPO_ROOT / "src/tuinadex_teleop/configs/imu_calibration.yaml")

    print("\n" + "=" * 68)
    print("      TuinaDex v2.0 — 6-DOF 手腕 IMU 空间对齐与零位交互校准工具")
    print("=" * 68)
    print("本工具通过 3 个标准自然手势，自动解算手腕 IMU 到机械臂末端旋转矩阵：")
    print("  1. 【基准平放】: 手掌平放桌面，手指自然指向前方（机械臂工作方向）")
    print("  2. 【手腕俯仰】: 手指向上翘起 25°~45°（手腕掌屈背伸），解算机械臂前后俯仰轴")
    print("  3. 【手腕翻滚】: 手腕向右侧翻转 25°~45°（旋前旋后），解算机械臂左右翻转轴")
    print("  -> 自动通过外积封闭垂直偏摆轴，严格确保 det(R) = +1.0 (右手刚体旋转)")
    print("-" * 68)

    # 1. Initialize IMU stream
    if mock:
        print("[INIT] 启动 MOCK 模拟 IMU 数据流...")
        stream: Any = MockImuStream()
        stream.start()
        active_port = "MOCK_IMU"
    else:
        print(f"[INIT] 尝试连接物理 IMU (候选端口: {port}, /dev/myimu, /dev/ttyUSB0...)...")
        stream = ImuStream(port=port, baudrate=baudrate, auto_detect=True)
        if not stream.start():
            print(f"\n[ERROR] 无法打开 IMU 串口！请确认：")
            print("  1. USB 数据线已连接；")
            print("  2. 串口权限已开启: sudo chmod 666 /dev/ttyUSB0 或 /dev/myimu；")
            print("  3. 若需在脱机 CI 环境测试，请添加 --mock 参数。")
            sys.exit(1)
        active_port = stream.port

    print(f"[OK] IMU 数据流已连接 ({active_port})。\n")

    try:
        # Step 1: Neutral baseline
        print("-" * 68)
        print("【步骤 1/3: 零位基准姿态 (Neutral Baseline)】")
        print("请将手部自然平放在桌面上，手心朝下，手指自然指向正前方（对齐机械臂正前方）。")
        print("保持手腕和手臂静止不动。")
        if not auto:
            input(">> 姿态就位后，按 [Enter] 键采集零位基准...")
        else:
            time.sleep(0.2)
        if mock:
            stream.set_synthetic_euler(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)

        reading_ref = sample_stable_reading(stream)
        R_ref = reading_ref.rotation_matrix
        r, p, y = reading_ref.euler_deg
        print(f"  -> 采集成功: 基准姿态欧拉角 [Roll: {r:+6.1f}°, Pitch: {p:+6.1f}°, Yaw: {y:+6.1f}°]")

        # Step 2: Pitch Up gesture
        print("\n" + "-" * 68)
        print("【步骤 2/3: 俯仰姿态 (Pitch Up)】")
        print("请将手指向上翘起约 25°~45°（手腕上抬，手掌根部保持在桌面上）。")
        print("保持该上翘姿态不要晃动。")
        if not auto:
            input(">> 姿态就位后，按 [Enter] 键采集俯仰轴...")
        else:
            time.sleep(0.2)
        if mock:
            # Default dorsal mount: pitching up is rotation around X
            stream.set_synthetic_euler(roll_deg=0.0, pitch_deg=35.0, yaw_deg=0.0)

        reading_pitch = sample_stable_reading(stream)
        R_pitch = reading_pitch.rotation_matrix
        delta_p = R_pitch @ R_ref.T
        from tuinadex_teleop.perception.imu_stream import rotation_matrix_to_rotvec
        rotvec_p = rotation_matrix_to_rotvec(delta_p)
        deg_p = math.degrees(float(np.linalg.norm(rotvec_p)))
        print(f"  -> 采集成功: 检测到俯仰角变化: {deg_p:.1f}°")

        # Step 3: Roll Right gesture
        print("\n" + "-" * 68)
        print("【步骤 3/3: 翻滚姿态 (Roll Right)】")
        print("请将手腕向右侧翻转约 25°~45°（向小拇指一侧旋转，类似侧掌姿势）。")
        print("保持该侧翻姿态不要晃动。")
        if not auto:
            input(">> 姿态就位后，按 [Enter] 键采集翻滚轴...")
        else:
            time.sleep(0.2)
        if mock:
            # Default dorsal mount: rolling right is rotation around Y
            stream.set_synthetic_euler(roll_deg=35.0, pitch_deg=0.0, yaw_deg=0.0)

        reading_roll = sample_stable_reading(stream)
        R_roll = reading_roll.rotation_matrix
        delta_r = R_roll @ R_ref.T
        rotvec_r = rotation_matrix_to_rotvec(delta_r)
        deg_r = math.degrees(float(np.linalg.norm(rotvec_r)))
        print(f"  -> 采集成功: 检测到翻滚角变化: {deg_r:.1f}°")

        # 4. Solve alignment
        print("\n" + "=" * 68)
        print("【解算校准矩阵与合法性校验】")
        mapping_str = solve_imu_alignment_from_gestures(R_ref, R_pitch, R_roll)
        R_align = parse_axis_mapping(mapping_str)
        det_val = float(np.linalg.det(R_align))

        if not np.isclose(det_val, 1.0, atol=1e-4):
            raise RuntimeError(f"解算失败: 行列式异常 det(R) = {det_val} != 1.0 (非合法人体右手旋转群)")

        neutral_quat = rotation_matrix_to_quat(R_ref)
        neutral_rpy = rotation_matrix_to_euler_deg(R_ref)

        print(f"  [OK] 解算最佳映射规则: {mapping_str}")
        print(f"  [OK] 刚体旋转合法性: det(R_align) = {det_val:+.4f} (严格保持右手系 SO(3))")
        print(f"  [OK] 基准参考四元数: [w:{neutral_quat[0]:.4f}, x:{neutral_quat[1]:.4f}, y:{neutral_quat[2]:.4f}, z:{neutral_quat[3]:.4f}]")
        print(f"  [OK] 静止零位欧拉角: [Roll:{neutral_rpy[0]:+.1f}°, Pitch:{neutral_rpy[1]:+.1f}°, Yaw:{neutral_rpy[2]:+.1f}°]")

        # 5. Persist to disk
        print("\n【写入配置文件】")
        save_calibration_results(
            mapping_str=mapping_str,
            neutral_quat=neutral_quat,
            neutral_rpy_deg=neutral_rpy,
            device_port=active_port,
            session_yaml_path=session_path,
            imu_yaml_path=imu_path,
        )

        print("=" * 68)
        print("  >>> 校准成功完成！ <<<")
        print("再次启动 teleop_sim 节点时，系统将自动载入该最优映射与零位姿态，无需手动调整。")
        print("=" * 68 + "\n")

        return mapping_str, neutral_quat

    finally:
        stream.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="TuinaDex IMU Wrist Attitude Alignment and Zero Calibration")
    parser.add_argument("--port", type=str, default="/dev/myimu", help="IMU serial port (default: /dev/myimu)")
    parser.add_argument("--baudrate", type=int, default=115200, help="IMU baud rate (default: 115200)")
    parser.add_argument("--mock", action="store_true", help="Run with mock simulated IMU")
    parser.add_argument("--auto", action="store_true", help="Non-interactive automatic mode for CI/test")
    parser.add_argument("--output-session", type=str, default=None, help="Custom path to session_calibration.yaml")
    parser.add_argument("--output-imu", type=str, default=None, help="Custom path to imu_calibration.yaml")
    args = parser.parse_args()

    session_path = Path(args.output_session) if args.output_session else None
    imu_path = Path(args.output_imu) if args.output_imu else None

    run_calibration(
        port=args.port,
        baudrate=args.baudrate,
        mock=args.mock,
        auto=args.auto,
        session_yaml_path=session_path,
        imu_yaml_path=imu_path,
    )


if __name__ == "__main__":
    main()
