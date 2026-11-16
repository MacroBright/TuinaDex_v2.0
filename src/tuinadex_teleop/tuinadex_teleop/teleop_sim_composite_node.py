"""teleop_sim_composite_node.py — 22-DOF Composite Robot Simulation Teleoperation ROS 2 Node.

Integrates:
1. RealSense hardware-aligned 3D Tracking & Table Calibration (WristTracker, TableCalibrator).
2. 6-axis IMU attitude streaming & orientation alignment (ImuStream, WristImuTracker).
3. Spatial Incremental Clutch Controller (ClutchController).
4. Pinocchio / Pink QP-IK Solver for 6-DOF reBot-DevArm (ArmIKSolver).
5. Kinematic Optimization Retargeting for 16-DOF LEAP Hand (HandRetargeter).
6. Multi-tier 4-Gate Safety Supervisor (SafetySupervisor).
7. MuJoCo Physics Digital Twin with Tuina Bed & Patient Dummy (MujocoSimAdapter).
8. Unified ROS 2 command & state publishing (/arm/joint_command, /hand/joint_command,
   /arm/joint_states, /hand/joint_states).
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np
import yaml

if sys.version_info[:2] == (3, 10):
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

from tuinadex_core.names import CANONICAL_ARM_JOINT_NAMES, CANONICAL_JOINT_NAMES

from tuinadex_teleop.adapters import MujocoSimAdapter, DEFAULT_READY_ARM_Q
from tuinadex_teleop.core.clutch import ClutchController
from tuinadex_teleop.core.names import ARM_NUM_DOFS, HAND_NUM_DOFS
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
)
from tuinadex_teleop.kinematics.hand_retargeter import (
    DEFAULT_OPEN_ZERO_OFFSET,
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
)
from tuinadex_teleop.perception.imu_stream import (
    ImuReading,
    ImuStream,
    MockImuStream,
    WristImuTracker,
)


def _parse_bool(val: Any) -> bool:
    """Safely parse boolean values, converting string representations robustly."""
    if isinstance(val, str):
        return val.strip().lower() in ("true", "1", "yes", "on")
    return bool(val)


def _resolve_config_path(rel_path: str) -> Path:
    """Resolve configuration or model path across package share and source trees."""
    p = Path(rel_path)
    if p.is_absolute() and p.exists():
        return p

    # 1. Search in ament package share if available
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("tuinadex_teleop"))
        candidate = (share_dir / rel_path).resolve()
        if candidate.exists():
            return candidate
    except Exception:
        pass

    # 2. Search in current workspace source tree
    current_dir = Path(__file__).resolve().parent
    for parent in [current_dir, current_dir.parent, current_dir.parent.parent, current_dir.parent.parent.parent]:
        candidate = (parent / rel_path).resolve()
        if candidate.exists():
            return candidate
        candidate_src = (parent / "src" / "tuinadex_teleop" / rel_path).resolve()
        if candidate_src.exists():
            return candidate_src

    # 3. Fallback to CWD
    return Path(rel_path).resolve()


class TeleopSimCompositeNode(Node):
    """ROS 2 Node for 22-DOF Composite Robot Simulation Teleoperation."""

    def __init__(self, **kwargs):
        super().__init__("teleop_sim_composite_node", **kwargs)

        # Declare parameters
        self.declare_parameter("model_path", "configs/mjcf/tuina_sim_scene.xml")
        self.declare_parameter("session_config", "configs/session_calibration.yaml")
        self.declare_parameter("table_config", "configs/table_calibration.yaml")
        self.declare_parameter("urdf_arm_path", "configs/urdf/rebot_arm_rs.urdf")
        self.declare_parameter("urdf_hand_path", "configs/urdf/leap_hand_right.urdf")
        self.declare_parameter("use_sim", True)
        self.declare_parameter("mock", False)
        self.declare_parameter("mock_imu", False)
        self.declare_parameter("no_imu", False)
        self.declare_parameter("no_safety", False)
        self.declare_parameter("auto_engage", False)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("camera_fps", 0)
        self.declare_parameter("solver_mode", "hybrid")
        self.declare_parameter("axis_mapping", "+y,-x,+z")
        self.declare_parameter("imu_axis_mapping", "+x,+y,+z")
        self.declare_parameter("gui", False)
        self.declare_parameter("teleop_config", "")

        self.model_path = _resolve_config_path(str(self.get_parameter("model_path").value))
        self.session_config_p = _resolve_config_path(str(self.get_parameter("session_config").value))
        self.table_config_p = _resolve_config_path(str(self.get_parameter("table_config").value))
        self.urdf_arm_p = _resolve_config_path(str(self.get_parameter("urdf_arm_path").value))
        self.urdf_hand_p = _resolve_config_path(str(self.get_parameter("urdf_hand_path").value))
        self.use_sim = _parse_bool(self.get_parameter("use_sim").value)
        self.mock = _parse_bool(self.get_parameter("mock").value)
        self.mock_imu = _parse_bool(self.get_parameter("mock_imu").value)
        self.no_imu = _parse_bool(self.get_parameter("no_imu").value)
        self.no_safety = _parse_bool(self.get_parameter("no_safety").value)
        self.auto_engage = _parse_bool(self.get_parameter("auto_engage").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.camera_fps = int(self.get_parameter("camera_fps").value)
        self.solver_mode = str(self.get_parameter("solver_mode").value)
        self.axis_mapping = str(self.get_parameter("axis_mapping").value)
        self.imu_axis_mapping = str(self.get_parameter("imu_axis_mapping").value)
        self.gui = _parse_bool(self.get_parameter("gui").value)

        self.get_logger().info(
            f"Initializing TeleopSimCompositeNode: use_sim={self.use_sim}, mock={self.mock}, "
            f"no_imu={self.no_imu}, mock_imu={self.mock_imu}, rate={self.publish_rate_hz}Hz, axis_mapping='{self.axis_mapping}'"
        )

        # 1. Perception setup
        self.session_calib = self._load_session_calibration(self.session_config_p)
        self.calib_data = None
        if self.session_calib is not None:
            self.calib_data = self._parse_table_from_session(self.session_calib)
        if self.calib_data is None and self.table_config_p.is_file():
            self.calib_data = self._load_table_calibration(self.table_config_p)

        if self.mock:
            self.get_logger().info("Starting MOCK RealSense camera stream...")
            self.stream = MockRealSenseStream()
        else:
            self.get_logger().info("Connecting to physical Intel RealSense D455...")
            try:
                self.stream = RealSenseStream(
                    width=640,
                    height=480,
                    fps=(self.camera_fps if self.camera_fps > 0 else None),
                )
            except Exception as e:
                self.get_logger().warning(f"Physical RealSense init failed ({e}), falling back to MOCK camera stream.")
                self.stream = MockRealSenseStream()

        self.intrinsics = self.stream.start()

        # IMU stream
        if self.no_imu:
            self.imu_stream = None
            self.get_logger().info("[IMU] IMU tracking disabled (no_imu=True).")
        elif self.mock_imu:
            self.get_logger().info("[IMU] Starting MOCK IMU stream (mock_imu=True)...")
            self.imu_stream = MockImuStream()
            self.imu_stream.start()
        else:
            self.get_logger().info("[IMU] Connecting to physical 6-axis IMU (/dev/myimu or /dev/ttyUSB*)...")
            self.imu_stream = ImuStream(auto_detect=True)
            if self.imu_stream.start():
                self.get_logger().info(f"[IMU] Physical IMU connected on {self.imu_stream.port} @ 115200 baud.")
            else:
                self.get_logger().warning("[IMU] Physical IMU not found or cannot be opened, falling back to MOCK IMU.")
                self.imu_stream = MockImuStream()
                self.imu_stream.start()

        # 2. Kinematics & Controllers (固定锁定右手，彻底杜绝手背微侧倾时的左右手突变反转)
        self.wrist_tracker = WristTracker(default_handedness="Right", lock_handedness=True)
        if self.mock:
            cx, cy = self.intrinsics.cx, self.intrinsics.cy
            def synthetic_mock_hand(img):
                t = time.monotonic()
                dx = 60.0 * np.sin(t * 1.5)
                dy = 40.0 * np.cos(t * 1.5)
                lm = np.zeros((21, 3), dtype=np.float32)
                lm[0] = [(cx + dx) / self.intrinsics.width, (cy + dy) / self.intrinsics.height, 0.0]
                lm[5] = [(cx + dx - 20) / self.intrinsics.width, (cy + dy - 35) / self.intrinsics.height, 0.0]
                lm[9] = [(cx + dx) / self.intrinsics.width, (cy + dy - 40) / self.intrinsics.height, 0.0]
                lm[17] = [(cx + dx + 20) / self.intrinsics.width, (cy + dy - 35) / self.intrinsics.height, 0.0]
                return {"landmarks": lm, "handedness": "Right"}
            self.wrist_tracker.set_custom_detector(synthetic_mock_hand)

        # 准备姿态与运动配置加载 (可由 configs/teleop_real_posture.yaml 自定义覆盖)
        self.ready_arm_q = DEFAULT_READY_ARM_Q.copy()
        self.ready_hand_q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._homing_duration = 3.0  # 默认 3.0 秒五次多项式平滑插值过渡
        self._load_teleop_posture_config()

        self.arm_ik = ArmIKSolver(
            urdf_path=self.urdf_arm_p,
            ee_frame_name="link6",
            home_q=self.ready_arm_q,
        )
        self.home_pos, self.home_R = self.arm_ik.forward_kinematics(self.arm_ik.home_q)

        # Clutch
        self.clutch = ClutchController(
            scale=1.0,
            workspace_x=self.arm_ik.workspace_x,
            workspace_y=self.arm_ik.workspace_y,
            workspace_z=self.arm_ik.workspace_z,
            axis_mapping=self.axis_mapping,
        )

        # Wrist IMU tracker
        self.wrist_imu_tracker = WristImuTracker(
            axis_mapping=self.imu_axis_mapping,
            neutral_robot_rot=self.home_R,
        )
        if self.session_calib is not None and "imu" in self.session_calib:
            session_imu_quat = self.session_calib["imu"].get("neutral_quat_wxyz")
            if session_imu_quat is not None and len(session_imu_quat) == 4:
                self.wrist_imu_tracker.set_reference_quat(session_imu_quat)
                self.get_logger().info("Applied IMU zero reference from session calibration.")

        # Hand Retargeter
        self.hand_retargeter = HandRetargeter(
            urdf_path=str(self.urdf_hand_p),
            solver_mode=self.solver_mode,
            scaling_factor=1.60,
        )
        if self.session_calib is not None and "hand" in self.session_calib:
            h_info = self.session_calib["hand"]
            q_zero = h_info.get("q_zero_offset")
            palm_l = h_info.get("measured_palm_length_m")
            self.hand_retargeter.set_calibration(q_zero_offset=q_zero, palm_reference_length_m=palm_l)
        else:
            self.hand_retargeter.set_calibration(q_zero_offset=DEFAULT_OPEN_ZERO_OFFSET)

        # 3. Safety Supervisor
        safety_cfg = SafetyConfig()
        safety_enabled = not self.no_safety
        self.safety_supervisor = SafetySupervisor(safety_cfg, enabled=safety_enabled)
        self.safety_report = SafetyReport(
            level=SafetyLevel.NORMAL if safety_enabled else SafetyLevel.BYPASS,
            violation=SafetyViolation.NONE,
            message="Safety supervisor initialized",
            timestamp=time.monotonic(),
        )

        # 4. Simulation Adapter
        self.sim_adapter: Optional[MujocoSimAdapter] = None
        self.latest_obs = RobotObservation(
            timestamp=time.monotonic(),
            arm_actual_q=self.arm_ik.home_q.copy(),
            arm_actual_dq=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
            arm_effort=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
            hand_actual_q=np.zeros(HAND_NUM_DOFS, dtype=np.float64),
            is_connected=True,
        )
        self.contact_forces: dict[str, float] = {}
        self._latched_hand_q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._hand_hold_lock = threading.RLock()
        self._hand_hold_enabled = False
        self._hand_hold_q: Optional[np.ndarray] = None
        self._last_published_hand_q = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._last_loop_t_now: Optional[float] = None
        self._destroyed = False

        # 平滑回准备姿态状态机 (Ramp-to-Ready Stance)
        self._is_homing = False
        self._homing_start_time = 0.0
        self._homing_arm_start = self.ready_arm_q.copy()
        self._homing_hand_start = self.ready_hand_q.copy()

        # 安全分步归零退出状态机 (Ramp-to-Zero Staged Exit: J4-J6 优先回正 -> J1-J3 随后折叠)
        self._is_exiting_zero = False
        self._exit_zero_stage = 0
        self._exit_zero_start_time = 0.0
        self._exit_zero_stage1_duration = 1.8
        self._exit_zero_stage2_duration = 2.5
        self._exit_zero_start_q = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
        self._exit_zero_hand_start = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        self._exit_zero_stage1_end_q = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
        self._exit_zero_complete = False

        if self.use_sim:
            self.get_logger().info(f"Connecting MujocoSimAdapter with model: {self.model_path}")
            self.sim_adapter = MujocoSimAdapter(model_path=self.model_path, headless=True)
            if not self.sim_adapter.connect():
                self.get_logger().error("Failed to connect MujocoSimAdapter!")
                raise RuntimeError("MujocoSimAdapter connection failed")
            self.latest_obs = self.sim_adapter.read_observation()
            self.contact_forces = self.sim_adapter.get_contact_forces()

        # 5. Publishers & Subscribers
        self.arm_cmd_pub = self.create_publisher(JointState, "/arm/joint_command", 10)
        self.hand_cmd_pub = self.create_publisher(JointState, "/hand/joint_command", 10)
        self.home_sub = self.create_subscription(
            Empty, "/teleop/home", lambda _: self._handle_home_pose(), 10
        )
        self.hand_hold_sub = self.create_subscription(
            Bool, "/teleop/hand_hold", lambda msg: self._set_hand_hold(bool(msg.data), source="ROS2"), 10
        )
        self.clutch_sub = self.create_subscription(
            Bool, "/teleop/clutch", lambda msg: self._set_pending_clutch("engage" if msg.data else "disengage"), 10
        )
        self.clutch_toggle_sub = self.create_subscription(
            Empty, "/teleop/clutch_toggle", lambda _: self._set_pending_clutch("toggle"), 10
        )
        self.clutch_state_pub = self.create_publisher(Bool, "/teleop/clutch_state", 10)

        if self.use_sim:
            self.arm_state_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
            self.hand_state_pub = self.create_publisher(JointState, "/hand/joint_states", 10)
            self.arm_state_sub = None
            self.hand_state_sub = None
        else:
            self.arm_state_pub = None
            self.hand_state_pub = None
            self.arm_state_sub = self.create_subscription(
                JointState, "/arm/joint_states", self._on_arm_state, 10
            )
            self.hand_state_sub = self.create_subscription(
                JointState, "/hand/joint_states", self._on_hand_state, 10
            )

        # Mode setup
        self.current_orientation_mode = (
            ArmOrientationMode.TRACK_IMU if self.imu_stream is not None else ArmOrientationMode.FIXED_DOWNWARD
        )
        from .perception.imu_stream import HotkeyState
        self.hotkeys = HotkeyState(
            axis_mapping=self.clutch.axis_mapping_str,
            orientation_mode=self.current_orientation_mode,
        )

        self.last_step_time = time.time()
        self.frame_count = 0
        self._last_stream_seq = -1
        self._last_wrist_pose = None
        self._pending_clutch_cmd: str | None = None

        # Direct UDP Broadcast socket for cross-environment IPC
        import socket
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_dest = ("127.0.0.1", 9876)

        # Listen for UDP HOME / HAND_HOLD / CLUTCH triggers from Web Hub
        self._udp_home_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._udp_home_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._udp_home_sock.bind(("127.0.0.1", 9877))
            self._udp_home_sock.settimeout(0.2)
            def _udp_home_worker():
                while not getattr(self, "_destroyed", False) and rclpy.ok():
                    try:
                        data, _ = self._udp_home_sock.recvfrom(64)
                        if data == b"HOME":
                            self._handle_home_pose()
                        elif data == b"HAND_HOLD":
                            self._set_hand_hold(not self._hand_hold_enabled, source="HUB_UDP")
                        elif data == b"HAND_HOLD:1":
                            self._set_hand_hold(True, source="HUB_UDP")
                        elif data == b"HAND_HOLD:0":
                            self._set_hand_hold(False, source="HUB_UDP")
                        elif data in (b"CLUTCH", b"CLUTCH:TOGGLE"):
                            self._set_pending_clutch("toggle")
                        elif data == b"CLUTCH:1":
                            self._set_pending_clutch("engage")
                        elif data == b"CLUTCH:0":
                            self._set_pending_clutch("disengage")
                        elif data == b"AXIS_CYCLE":
                            self._handle_axis_cycle()
                        elif data == b"IMU_AXES_CYCLE":
                            self._handle_cycle_imu_axes(forward=True)
                        elif data == b"IMU_AXES_CYCLE_REV":
                            self._handle_cycle_imu_axes(forward=False)
                    except socket.timeout:
                        continue
                    except Exception:
                        break
            threading.Thread(target=_udp_home_worker, daemon=True).start()
        except Exception as exc:
            self.get_logger().debug(f"UDP home listener inactive: {exc}")

        # Periodic timer
        timer_period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(timer_period, self._loop_step)
        self.get_logger().info(f"TeleopSimCompositeNode started successfully (rate={self.publish_rate_hz}Hz).")

    def _loop_step(self) -> None:
        """Main teleoperation update and ROS 2 publish loop."""
        color, depth, ts = self.stream.read_frames()
        if color is None:
            return

        self.frame_count += 1
        t_now = ts if ts is not None else time.monotonic()
        self._last_loop_t_now = t_now

        if self.mock and depth is None:
            depth = np.ones((self.intrinsics.height, self.intrinsics.width), dtype=np.float32) * 0.65

        # 1. Perception (once per captured frame; the 60Hz loop reuses the pose, I38)
        stream_seq = (
            self.stream.latest_seq()
            if hasattr(self.stream, "latest_seq")
            else self.frame_count
        )
        if stream_seq == self._last_stream_seq and self._last_wrist_pose is not None:
            wrist_pose = self._last_wrist_pose
        else:
            self._last_stream_seq = stream_seq
            wrist_pose = self.wrist_tracker.process_frame(
                color_bgr=color,
                depth_m=depth,
                intrinsics=self.intrinsics,
                table_calib=self.calib_data,
                timestamp=t_now,
            )
            self._last_wrist_pose = wrist_pose
        imu_reading = self.imu_stream.read() if self.imu_stream is not None else ImuReading()

        # 2. Hand Retargeting
        hand_cmd = self.hand_retargeter.retarget_pose(wrist_pose, timestamp=t_now)
        with self._hand_hold_lock:
            hand_hold_enabled = bool(self._hand_hold_enabled and self._hand_hold_q is not None)
            hand_hold_q = self._hand_hold_q.copy() if hand_hold_enabled and self._hand_hold_q is not None else None
        if hand_hold_enabled and hand_hold_q is not None:
            hand_cmd.target_q = hand_hold_q.copy()
            hand_cmd.is_clutched = True
            self.hand_retargeter.sync_held_state(hand_hold_q, timestamp=t_now)

        # Check pending clutch toggle/engage/disengage from UDP or ROS 2
        if self._pending_clutch_cmd is not None:
            cmd = self._pending_clutch_cmd
            self._pending_clutch_cmd = None
            if cmd == "toggle":
                self._trigger_clutch_toggle(wrist_pose, hand_cmd, imu_reading)
            elif cmd == "engage":
                self._trigger_clutch_engage(wrist_pose, hand_cmd, imu_reading)
            elif cmd == "disengage":
                self._trigger_clutch_disengage()

        # Actual robot end-effector Cartesian pose
        cur_arm_p, cur_arm_R = self.arm_ik.forward_kinematics(self.latest_obs.arm_actual_q)

        # 3. Safety Evaluation
        eval_target_hand_q = (
            None
            if (self._is_homing or self._is_exiting_zero)
            else (
                hand_cmd.target_q
                if (hand_hold_enabled or self.clutch.state == ClutchState.ENGAGED)
                else self._latched_hand_q
            )
        )
        self.safety_report = self.safety_supervisor.evaluate(
            t_now=t_now,
            wrist_tracking_valid=(wrist_pose.tracking_state == TrackingState.TRACKING),
            wrist_pos=wrist_pose.position_table,
            wrist_rot=wrist_pose.orientation_table_matrix,
            wrist_timestamp=wrist_pose.timestamp,
            current_arm_q=self.latest_obs.arm_actual_q,
            current_arm_pos=cur_arm_p,
            current_arm_rot=cur_arm_R,
            current_hand_q=self.latest_obs.hand_actual_q,
            home_arm_q=self.arm_ik.home_q,
            contact_forces=self.contact_forces,
            clutch_state=self.clutch.state,
            arm_ik_solver=self.arm_ik,
            target_hand_q=eval_target_hand_q,
            hand_current=self.latest_obs.hand_current,
            hand_joint_limits=self.hand_retargeter.joint_limits,
        )

        # 4. Target Arbitration & Safety Actions
        if self.safety_report.level == SafetyLevel.RETRACTING and self.safety_supervisor.retract_controller is not None:
            with self._hand_hold_lock:
                if self._hand_hold_enabled:
                    self._hand_hold_enabled = False
                    self._hand_hold_q = None
                    self._notify_hotkey("安全撤离: 已自动解除灵巧手保持")
            self.clutch.set_safe_retracting()
            retract_pos, retract_rot, retract_arm_q, retract_hand_q, finished = (
                self.safety_supervisor.step_retract(t_now, self.latest_obs.arm_actual_q)
            )
            hand_cmd.target_q = retract_hand_q.copy()
            if retract_arm_q is not None:
                fk_p, fk_R = self.arm_ik.forward_kinematics(retract_arm_q)
                target_pose = np.eye(4, dtype=np.float64)
                target_pose[:3, :3] = fk_R
                target_pose[:3, 3] = fk_p
                arm_cmd = TeleopArmCommand(
                    timestamp=t_now,
                    target_pose=target_pose,
                    target_q=retract_arm_q,
                    target_dq=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
                    clutch_state=self.clutch.state,
                )
                self.arm_ik.current_q = retract_arm_q.copy()
            else:
                arm_cmd = self.arm_ik.solve(
                    target_pos=retract_pos,
                    target_rot=retract_rot,
                    orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
                    dt=1.0 / max(1.0, self.publish_rate_hz),
                    clutch_state=self.clutch.state,
                    timestamp=t_now,
                )
            if finished:
                self.clutch.disengage()

        elif self.safety_report.level == SafetyLevel.FROZEN:
            self.clutch.set_safe_hold()
            target_pose = np.eye(4, dtype=np.float64)
            target_pose[:3, :3] = self.safety_supervisor.frozen_arm_rot
            target_pose[:3, 3] = self.safety_supervisor.frozen_arm_pos
            arm_cmd = TeleopArmCommand(
                timestamp=t_now,
                target_pose=target_pose,
                target_q=self.safety_supervisor.frozen_arm_q.copy(),
                target_dq=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
                clutch_state=self.clutch.state,
            )
            hand_cmd.target_q = self.safety_supervisor.frozen_hand_q.copy()

        elif self.safety_report.level == SafetyLevel.HOLD:
            hold_arm_q = self.arm_ik.current_q.copy()
            hold_pos, hold_R = self.arm_ik.forward_kinematics(hold_arm_q)
            hold_pose = np.eye(4, dtype=np.float64)
            hold_pose[:3, :3] = hold_R
            hold_pose[:3, 3] = hold_pos
            arm_cmd = TeleopArmCommand(
                timestamp=t_now,
                target_pose=hold_pose,
                target_q=hold_arm_q,
                target_dq=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
                clutch_state=ClutchState.SAFE_HOLD,
            )
            if hand_hold_enabled and hand_hold_q is not None:
                hand_cmd.target_q = hand_hold_q.copy()
                self._latched_hand_q = hand_hold_q.copy()
            elif self._latched_hand_q is not None:
                hand_cmd.target_q = self._latched_hand_q.copy()

        else:
            # Normal teleoperation
            if wrist_pose.tracking_state == TrackingState.TRACKING and wrist_pose.position_table is not None:
                if (
                    self.auto_engage
                    and self.clutch.state in (
                        ClutchState.DISENGAGED,
                        ClutchState.SAFE_HOLD,
                        ClutchState.SAFE_RETRACTING,
                    )
                    and getattr(self.clutch, "_human_anchor", None) is None
                ):
                    self._engage_clutch(
                        wrist_pose=wrist_pose,
                        hand_cmd=hand_cmd,
                        imu_reading=imu_reading,
                        zero_imu_always=False,
                    )
                    if self.safety_report.level in (SafetyLevel.FROZEN, SafetyLevel.RETRACTING):
                        self.safety_supervisor.reset()
                    self._notify_hotkey("自动接管 [AUTO-ENGAGED]")
                target_pos = self.clutch.compute_target(wrist_pose.position_table, fallback_robot_pos=self.home_pos)
            else:
                target_pos = self.clutch.compute_target(np.zeros(3), fallback_robot_pos=self.home_pos)

            # Hold the latched engage-time orientation so --no-imu / FIXED_DOWNWARD
            # teleoperates directly from the active initial posture without snapping
            # to R_DOWNWARD_DEFAULT.
            anchor_rot = (
                self.clutch._robot_rot_anchor.copy()
                if self.clutch._robot_rot_anchor is not None
                else self.home_R.copy()
            )

            # Target orientation
            if self.current_orientation_mode == ArmOrientationMode.TRACK_IMU:
                if imu_reading.is_valid:
                    R_target = self.wrist_imu_tracker.compute_target_rotation(
                        reading=imu_reading,
                        fallback_rot=anchor_rot,
                        anchor_robot_rot=anchor_rot,
                    )
                else:
                    R_target = anchor_rot
            elif self.current_orientation_mode == ArmOrientationMode.FIXED_DOWNWARD:
                R_target = anchor_rot
            elif self.current_orientation_mode == ArmOrientationMode.TRACK_WRIST:
                R_target = (
                    self.clutch.compute_target_orientation(
                        wrist_pose.orientation_table_matrix,
                        fallback_robot_rot=anchor_rot,
                    )
                    if wrist_pose.orientation_table_matrix is not None
                    else anchor_rot
                )
            else:
                R_target = None

            # Arm QP-IK Solve
            arm_cmd = self.arm_ik.solve(
                target_pos=target_pos,
                target_rot=R_target,
                orientation_mode=self.current_orientation_mode,
                dt=1.0 / max(1.0, self.publish_rate_hz),
                clutch_state=self.clutch.state,
                timestamp=t_now,
            )

            # 灵巧手保持模式优先级高于离合状态：保持锁定动作；未锁定时离线锁手，接通跟随
            if hand_hold_enabled and hand_hold_q is not None:
                hand_cmd.target_q = hand_hold_q.copy()
                self._latched_hand_q = hand_hold_q.copy()
            elif self.clutch.state != ClutchState.ENGAGED:
                if self._latched_hand_q is not None:
                    hand_cmd.target_q = self._latched_hand_q.copy()
                else:
                    hand_cmd.target_q = np.zeros(HAND_NUM_DOFS, dtype=np.float32)
            else:
                self._latched_hand_q = hand_cmd.target_q.copy()

        # 4.5 平滑回准备姿态状态机 (Ramp-to-Ready) 覆盖
        if self._is_homing:
            now = time.monotonic()
            elapsed = now - self._homing_start_time
            alpha = min(1.0, max(0.0, elapsed / self._homing_duration))
            # 5次多项式平滑 (Smoothstep Quintic): 6*t^5 - 15*t^4 + 10*t^3
            s = alpha * alpha * alpha * (alpha * (alpha * 6.0 - 15.0) + 10.0)

            homing_arm_q = (1.0 - s) * self._homing_arm_start + s * self.ready_arm_q
            homing_hand_q = (1.0 - s) * self._homing_hand_start + s * self.ready_hand_q

            arm_cmd.target_q = homing_arm_q.copy()
            arm_cmd.target_dq = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
            hand_cmd.target_q = homing_hand_q.copy()
            self._latched_hand_q = homing_hand_q.copy()

            if alpha >= 1.0:
                self._is_homing = False
                self.arm_ik.reset()
                self.arm_ik.current_q = self.ready_arm_q.copy()
                self.clutch.reset(safe_robot_pos=self.home_pos, safe_robot_rot=self.home_R)
                self._notify_hotkey("一键回位完成: 机械臂与灵巧手已就绪")
                self.get_logger().info("一键回位完成: 臂手已平滑到达推拿准备姿态。")

        # 4.6 安全分步归零退出状态机 (Ramp-to-Zero Staged Exit) 覆盖 (J4-J6 优先回正 -> J1-J3 随后折叠)
        if self._is_exiting_zero:
            now = time.monotonic()
            elapsed = now - self._exit_zero_start_time

            if self._exit_zero_stage == 1:
                # 阶段 1: 手腕与末端 (J4-J6) 优先平滑归零回正展开，J1-J3 保持当前高位不变
                alpha = min(1.0, max(0.0, elapsed / self._exit_zero_stage1_duration))
                s = alpha * alpha * alpha * (alpha * (alpha * 6.0 - 15.0) + 10.0)

                target_arm_q = self._exit_zero_start_q.copy()
                target_arm_q[3:6] = (1.0 - s) * self._exit_zero_start_q[3:6]
                target_hand_q = (1.0 - s) * self._exit_zero_hand_start

                arm_cmd.target_q = target_arm_q.copy()
                arm_cmd.target_dq = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
                hand_cmd.target_q = target_hand_q.copy()
                self._latched_hand_q = target_hand_q.copy()

                self.hotkeys.toast_now("【安全退出 1/2】手腕 (J4-J6) 优先平滑回正展开...", time.time(), 1.0)

                if alpha >= 1.0:
                    self._exit_zero_stage = 2
                    self._exit_zero_start_time = time.monotonic()
                    self._exit_zero_stage1_end_q = target_arm_q.copy()
                    self._exit_zero_stage1_end_q[3:6] = 0.0
                    self._notify_hotkey("安全退出: 阶段 2/2 机械臂主体 (J1-J3) 随后平滑折叠归零...")
                    self.get_logger().info("安全退出 阶段 1 完成！开始阶段 2/2: 机械臂主体 (J1-J3) 随后平滑折叠归零 (历时 2.5s)...")

            elif self._exit_zero_stage == 2:
                # 阶段 2: J4-J6 锁死在 0.0 rad，J1-J3 平滑折叠归零
                alpha = min(1.0, max(0.0, elapsed / self._exit_zero_stage2_duration))
                s = alpha * alpha * alpha * (alpha * (alpha * 6.0 - 15.0) + 10.0)

                target_arm_q = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
                target_arm_q[0:3] = (1.0 - s) * self._exit_zero_stage1_end_q[0:3]
                target_hand_q = np.zeros(HAND_NUM_DOFS, dtype=np.float32)

                arm_cmd.target_q = target_arm_q.copy()
                arm_cmd.target_dq = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
                hand_cmd.target_q = target_hand_q.copy()
                self._latched_hand_q = target_hand_q.copy()

                self.hotkeys.toast_now("【安全退出 2/2】机械臂主体 (J1-J3) 平滑折叠归零...", time.time(), 1.0)

                if alpha >= 1.0:
                    self._exit_zero_stage = 3
                    self._exit_zero_complete = True
                    self.get_logger().info("分步安全归零全部完成！臂手已平稳收回至全 0 姿态。正在安全退出视窗与节点...")

        # 5. Publish target commands to ROS 2
        self._last_published_hand_q = np.asarray(hand_cmd.target_q, dtype=np.float64).copy()
        ros_stamp = self.get_clock().now().to_msg()

        # Arm command
        arm_msg = JointState()
        arm_msg.header.stamp = ros_stamp
        arm_msg.name = list(CANONICAL_ARM_JOINT_NAMES)
        arm_msg.position = arm_cmd.target_q.tolist()
        arm_msg.velocity = arm_cmd.target_dq.tolist()
        self.arm_cmd_pub.publish(arm_msg)

        # Hand command
        hand_msg = JointState()
        hand_msg.header.stamp = ros_stamp
        hand_msg.name = list(CANONICAL_JOINT_NAMES)
        hand_msg.position = hand_cmd.target_q.tolist()
        self.hand_cmd_pub.publish(hand_msg)

        # Broadcast 22-DOF command to local UDP bus (bridges Conda/ROS environments)
        try:
            import struct
            q_22 = np.zeros(22, dtype=np.float32)
            q_22[:6] = arm_cmd.target_q[:6]
            q_22[6:22] = hand_cmd.target_q[:16]
            pkt = struct.pack("<4s22f", b"TD22", *q_22)
            self._udp_sock.sendto(pkt, self._udp_dest)
        except Exception:
            pass

        # 6. Step physics simulation if enabled
        if self.use_sim and self.sim_adapter is not None:
            composite_cmd = TeleopCommand(
                timestamp=t_now,
                arm_command=arm_cmd,
                hand_command=hand_cmd,
                clutch_state=self.clutch.state,
            )
            dt_step = 1.0 / max(1.0, self.publish_rate_hz)
            sim_dt = self.sim_adapter.model.opt.timestep if self.sim_adapter.model is not None else 0.002
            n_substeps = max(1, min(30, int(round(dt_step / sim_dt))))
            self.sim_adapter.send_command(composite_cmd)
            self.sim_adapter.step(n_steps=n_substeps)

            # Read new observation & telemetry
            self.latest_obs = self.sim_adapter.read_observation()
            self.contact_forces = self.sim_adapter.get_contact_forces()

            # Publish simulation joint states
            if self.arm_state_pub is not None:
                s_arm = JointState()
                s_arm.header.stamp = ros_stamp
                s_arm.name = list(CANONICAL_ARM_JOINT_NAMES)
                s_arm.position = self.latest_obs.arm_actual_q.tolist()
                s_arm.velocity = self.latest_obs.arm_actual_dq.tolist()
                s_arm.effort = self.latest_obs.arm_effort.tolist()
                self.arm_state_pub.publish(s_arm)

            if self.hand_state_pub is not None:
                s_hand = JointState()
                s_hand.header.stamp = ros_stamp
                s_hand.name = list(CANONICAL_JOINT_NAMES)
                s_hand.position = self.latest_obs.hand_actual_q.tolist()
                self.hand_state_pub.publish(s_hand)

        # 7. Render OpenCV Telemetry HUD if gui is enabled
        if self.gui:
            import cv2
            from .perception.wrist_tracker import draw_wrist_hud
            from .ui.teleop_hud import draw_simulation_hud

            t_step = time.time()
            fps = max(1.0, 1.0 / (t_step - self.last_step_time)) if (t_step > self.last_step_time) else self.publish_rate_hz
            self.last_step_time = t_step

            hud_cam = draw_wrist_hud(color, wrist_pose, self.intrinsics)
            dashboard = draw_simulation_hud(
                camera_view=hud_cam,
                clutch=self.clutch,
                orientation_mode=self.current_orientation_mode,
                arm_cmd=arm_cmd,
                hand_cmd=hand_cmd,
                obs=self.latest_obs,
                forces=self.contact_forces,
                arm_metrics=self.arm_ik.get_metrics(),
                hand_metrics=self.hand_retargeter.metrics,
                fps=fps,
                imu_reading=imu_reading,
                imu_tracker=self.wrist_imu_tracker,
                safety_report=self.safety_report,
                toast=self.hotkeys.active_toast(time.time()),
                hand_hold=self._hand_hold_enabled,
                is_sim=self.use_sim,
            )
            win_title = "TuinaDex 22-DOF Teleoperation HUD [仿真 (MuJoCo)]" if self.use_sim else "TuinaDex 22-DOF Teleoperation HUD [真机 (Real)]"
            cv2.imshow(win_title, dashboard)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                if not self._is_exiting_zero:
                    self.get_logger().info("收到退出键 (Q/ESC)，启动两阶段分步安全归零退出流程 (J4-J6 优先回正 -> J1-J3 随后归零)...")
                    self._start_safe_exit_to_zero()
            elif key in (32, ord("c"), ord("C")):
                self._trigger_clutch_toggle(wrist_pose, hand_cmd, imu_reading)
            elif key in (ord("s"), ord("S")):
                self.safety_supervisor.toggle_enabled()
            elif key in (ord("a"), ord("A")):
                self._handle_axis_cycle()
            elif key in (ord("x"),):
                self._handle_cycle_imu_axes(forward=True)
            elif key in (ord("X"),):
                self._handle_cycle_imu_axes(forward=False)
            elif key in (ord("h"), ord("H"), ord("r"), ord("R")):
                self._handle_home_pose()
            elif key in (ord("m"), ord("M")):
                self._handle_mode_cycle()
            elif key in (ord("i"), ord("I")):
                self._handle_imu_zero(imu_reading)
            elif key in (ord("z"), ord("Z")):
                self._handle_hand_zero(wrist_pose)
            elif key in (ord("l"), ord("L")):
                self._set_hand_hold(not self._hand_hold_enabled, source="HUD")

        if getattr(self, "_exit_zero_complete", False):
            self.get_logger().info("安全分步归零完成，正在安全退出视窗与节点...")
            if getattr(self, "gui", False):
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
            if rclpy.ok():
                rclpy.shutdown()
            return

    # ---------------- W9 hotkey handlers ----------------
    def _hotkey_guard(self) -> tuple[bool, str]:
        """I40: no abrupt mapping/zero changes during engaged high-speed motion."""
        from .perception.imu_stream import hotkey_allowed

        speed = 0.0
        if self.latest_obs is not None:
            speed = float(np.linalg.norm(self.latest_obs.arm_actual_dq))
        return hotkey_allowed(self.clutch.state, speed)

    def _notify_hotkey(self, text: str) -> None:
        self.get_logger().info(f"[HOTKEY] {text}")
        self.hotkeys.toast_now(text, time.time(), 1.0)

    def _handle_axis_cycle(self) -> None:
        from .perception.imu_stream import cycle_axis_mapping

        allowed, reason = self._hotkey_guard()
        if not allowed:
            self._notify_hotkey(reason)
            return
        mapping = cycle_axis_mapping(self.clutch.axis_mapping_str)
        self.clutch.set_axis_mapping(mapping)
        self.axis_mapping = mapping
        self.hotkeys.axis_mapping = mapping
        self._notify_hotkey(f"PosAxis -> {mapping}")

    def _handle_cycle_imu_axes(self, forward: bool = True) -> None:
        allowed, reason = self._hotkey_guard()
        if not allowed:
            self._notify_hotkey(reason)
            return
        mapping, idx, total = self.wrist_imu_tracker.cycle_axis_mapping(forward=forward)
        self.imu_axis_mapping = mapping
        self._notify_hotkey(f"IMUAxis [{idx}/{total}] -> {mapping}")

    def _handle_swap_imu_axes(self) -> None:
        self._handle_cycle_imu_axes(forward=True)

    def _on_arm_state(self, msg: JointState) -> None:
        if len(msg.position) >= ARM_NUM_DOFS:
            self.latest_obs.arm_actual_q = np.array(msg.position[:ARM_NUM_DOFS], dtype=np.float64)
        if len(msg.velocity) >= ARM_NUM_DOFS:
            self.latest_obs.arm_actual_dq = np.array(msg.velocity[:ARM_NUM_DOFS], dtype=np.float64)
        if len(msg.effort) >= ARM_NUM_DOFS:
            self.latest_obs.arm_effort = np.array(msg.effort[:ARM_NUM_DOFS], dtype=np.float64)

    def _on_hand_state(self, msg: JointState) -> None:
        if len(msg.position) >= HAND_NUM_DOFS:
            self.latest_obs.hand_actual_q = np.array(msg.position[:HAND_NUM_DOFS], dtype=np.float64)
        if len(msg.effort) >= HAND_NUM_DOFS:
            self.latest_obs.hand_current = np.array(msg.effort[:HAND_NUM_DOFS], dtype=np.float64)

    def _set_hand_hold(self, enabled: bool, source: str = "HOTKEY") -> bool:
        """Toggle or explicitly set LEAP Hand action-hold mode."""
        with self._hand_hold_lock:
            if enabled:
                if getattr(self, "_is_exiting_zero", False):
                    self._notify_hotkey("正在执行安全退出，禁止保持手部动作")
                    return self._hand_hold_enabled
                if self._hand_hold_enabled and self._hand_hold_q is not None:
                    return True
                hold_src = None
                for candidate in (
                    getattr(self, "_last_published_hand_q", None),
                    self._latched_hand_q,
                    self.latest_obs.hand_actual_q if self.latest_obs is not None else None,
                ):
                    if candidate is not None and len(candidate) >= HAND_NUM_DOFS and np.all(np.isfinite(candidate)):
                        hold_src = np.asarray(candidate[:HAND_NUM_DOFS], dtype=np.float64)
                        break
                if hold_src is None:
                    hold_src = np.zeros(HAND_NUM_DOFS, dtype=np.float64)
                held = np.clip(
                    hold_src,
                    self.hand_retargeter.joint_limits[:, 0],
                    self.hand_retargeter.joint_limits[:, 1],
                )
                self._hand_hold_enabled = True
                self._hand_hold_q = held.copy()
                self._latched_hand_q = held.copy()
                self.hand_retargeter.sync_held_state(held, timestamp=self._last_loop_t_now)
                self._notify_hotkey(f"灵巧手动作保持开启 [HAND: HOLD] ({source}) - 按 [L] 恢复")
                return True

            if not self._hand_hold_enabled:
                return False
            resume_q = (
                self._hand_hold_q.copy()
                if self._hand_hold_q is not None
                else (
                    self._latched_hand_q.copy()
                    if self._latched_hand_q is not None
                    else np.zeros(HAND_NUM_DOFS, dtype=np.float64)
                )
            )
            self._hand_hold_enabled = False
            self._hand_hold_q = None
            self._latched_hand_q = resume_q.copy()
            self.hand_retargeter.sync_held_state(resume_q, timestamp=self._last_loop_t_now)
            self._notify_hotkey(f"灵巧手恢复遥操跟随 [HAND: FOLLOW] ({source})")
            return False

    def _select_engage_arm_q(self) -> np.ndarray:
        """Select continuous arm joint angles to anchor the clutch at engage time.

        Prefer the currently commanded IK configuration when the physical/simulated arm
        is tracking closely (avoids ratcheting down by steady-state gravity sag), while
        snapping to actual joint feedback if an external move created > 0.15 rad offset.
        """
        cmd_q = np.asarray(self.arm_ik.current_q, dtype=np.float64)
        if self.latest_obs is not None and self.latest_obs.arm_actual_q is not None:
            actual_q = np.asarray(self.latest_obs.arm_actual_q, dtype=np.float64)
            if np.all(np.isfinite(actual_q)) and np.max(np.abs(actual_q - cmd_q)) > 0.15:
                return actual_q.copy()
        return cmd_q.copy()

    def _set_pending_clutch(self, cmd: str) -> None:
        self._pending_clutch_cmd = str(cmd)

    def _broadcast_clutch_state(self, is_engaged: bool) -> None:
        if hasattr(self, "clutch_state_pub") and self.clutch_state_pub is not None:
            try:
                self.clutch_state_pub.publish(Bool(data=bool(is_engaged)))
            except Exception:
                pass
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(b"CLUTCH_STATE:1" if is_engaged else b"CLUTCH_STATE:0", ("127.0.0.1", 9878))
            sock.close()
        except Exception:
            pass

    def _trigger_clutch_disengage(self) -> None:
        if self.clutch.state == ClutchState.ENGAGED:
            self.clutch.disengage()
            if self._hand_hold_enabled and self._hand_hold_q is not None:
                self._latched_hand_q = self._hand_hold_q.copy()
                self._notify_hotkey("离合暂停 [DISENGAGED]: 机械臂锁定 (灵巧手继续保持)")
            else:
                self._latched_hand_q = self._last_published_hand_q.copy()
                self._notify_hotkey("离合暂停 [DISENGAGED]: 动作安全锁定")
            self._broadcast_clutch_state(False)

    def _trigger_clutch_engage(self, wrist_pose, hand_cmd, imu_reading) -> bool:
        tot_f = self.contact_forces.get("total", 0.0)
        if tot_f > 40.0:
            self.get_logger().warning(f"无法接管: 接触力({tot_f:.1f}N)过大，请先脱离接触！")
            self._notify_hotkey("无法接管: 接触力过大")
            return False
        elif wrist_pose is None or wrist_pose.tracking_state != TrackingState.TRACKING or wrist_pose.position_table is None:
            self.get_logger().warning("无法接管: 未检测到人手位置，请将手放入相机视野内！")
            self._notify_hotkey("无法接管: 未检测到人手")
            return False
        else:
            self._engage_clutch(
                wrist_pose=wrist_pose,
                hand_cmd=hand_cmd,
                imu_reading=imu_reading,
                zero_imu_always=True,
            )
            self._notify_hotkey("离合接管 [ENGAGED]: 动作实时跟随")
            self._broadcast_clutch_state(True)
            return True

    def _trigger_clutch_toggle(self, wrist_pose, hand_cmd, imu_reading) -> None:
        if self.clutch.state == ClutchState.ENGAGED:
            self._trigger_clutch_disengage()
        else:
            self._trigger_clutch_engage(wrist_pose, hand_cmd, imu_reading)

    def _engage_clutch(
        self,
        wrist_pose: WristPose,
        hand_cmd: TeleopHandCommand,
        imu_reading: ImuReading,
        zero_imu_always: bool = True,
    ) -> None:
        """Anchor teleoperation at the active arm posture with zero jump on engage."""
        self._is_homing = False
        engage_q = self._select_engage_arm_q()
        cur_ee_p, cur_ee_R = self.arm_ik.forward_kinematics(engage_q)
        self.arm_ik.reset(q=engage_q)
        self.clutch.engage(
            current_human_pos=wrist_pose.position_table,
            current_robot_pos=cur_ee_p,
            current_human_rot=wrist_pose.orientation_table_matrix,
            current_robot_rot=cur_ee_R,
        )
        with self._hand_hold_lock:
            if self._hand_hold_enabled and self._hand_hold_q is not None:
                self._latched_hand_q = self._hand_hold_q.copy()
            else:
                prev_hand_q = (
                    self._last_published_hand_q
                    if getattr(self, "_last_published_hand_q", None) is not None
                    else self._latched_hand_q
                )
                self._latched_hand_q = np.asarray(prev_hand_q, dtype=np.float64).copy()
                self.hand_retargeter.sync_held_state(
                    self._latched_hand_q,
                    timestamp=getattr(self, "_last_loop_t_now", None),
                )
        if imu_reading.is_valid and (zero_imu_always or not self.wrist_imu_tracker.is_calibrated):
            self.wrist_imu_tracker.zero(imu_reading)

    def _handle_home_pose(self) -> None:
        with self._hand_hold_lock:
            self._hand_hold_enabled = False
            self._hand_hold_q = None
        self.clutch.disengage()
        self._broadcast_clutch_state(False)

        # 每次按 [R]/[H] 键或收到回位信号时，动态同步重载最新的 teleop_real_posture.yaml 准备姿态配置 (arm.ready_pose_deg)
        self._load_teleop_posture_config()

        self.arm_ik.reset()
        self.hand_retargeter.reset()
        # 清除任何历史冻结或避险状态，允许一键平滑回位
        self.safety_supervisor.reset()

        # 捕获回位起点 (优先使用实测物理位置，若无则使用当前求解目标)
        if (
            self.latest_obs is not None
            and self.latest_obs.arm_actual_q is not None
            and np.any(np.abs(self.latest_obs.arm_actual_q) > 1e-4)
        ):
            self._homing_arm_start = self.latest_obs.arm_actual_q.copy()
        elif hasattr(self.arm_ik, "current_q") and self.arm_ik.current_q is not None:
            self._homing_arm_start = self.arm_ik.current_q.copy()
        else:
            self._homing_arm_start = self.ready_arm_q.copy()

        if (
            self.latest_obs is not None
            and self.latest_obs.hand_actual_q is not None
            and np.any(np.abs(self.latest_obs.hand_actual_q) > 1e-4)
        ):
            self._homing_hand_start = self.latest_obs.hand_actual_q.copy()
        elif self._latched_hand_q is not None:
            self._homing_hand_start = self._latched_hand_q.copy()
        else:
            self._homing_hand_start = self.ready_hand_q.copy()

        self._homing_start_time = time.monotonic()
        self._is_homing = True

        deg_str = ", ".join(f"{round(float(math.degrees(x)), 1)}°" for x in self.ready_arm_q)
        self._notify_hotkey(f"一键回位: 机械臂与灵巧手正平滑移动至准备姿态 [{deg_str}]...")
        self.get_logger().info(
            f"一键回位启动: 臂手开始平滑插值过渡至推拿准备姿态 [{deg_str}] (历时 {self._homing_duration}s)..."
        )

    def _start_safe_exit_to_zero(self) -> None:
        """启动两阶段安全归零退出流程:
        - 阶段 1: 手腕与末端 (J4-J6) 优先平滑回正展开，J1-J3 保持当前高位姿态不变
        - 阶段 2: 机械臂主体 (J1-J3) 随后平滑折叠归零
        """
        with self._hand_hold_lock:
            self._hand_hold_enabled = False
            self._hand_hold_q = None
        self.clutch.disengage()
        self.safety_supervisor.reset()
        self._is_homing = False
        self._is_exiting_zero = True
        self._exit_zero_stage = 1
        self._exit_zero_start_time = time.monotonic()

        # 优先读取机械臂实际物理位置，确保插值起点连续平滑
        if (
            self.latest_obs is not None
            and self.latest_obs.arm_actual_q is not None
            and np.any(np.abs(self.latest_obs.arm_actual_q) > 1e-4)
        ):
            self._exit_zero_start_q = self.latest_obs.arm_actual_q.copy()
        elif hasattr(self.arm_ik, "current_q") and self.arm_ik.current_q is not None:
            self._exit_zero_start_q = self.arm_ik.current_q.copy()
        else:
            self._exit_zero_start_q = self.ready_arm_q.copy()

        if (
            self.latest_obs is not None
            and self.latest_obs.hand_actual_q is not None
            and np.any(np.abs(self.latest_obs.hand_actual_q) > 1e-4)
        ):
            self._exit_zero_hand_start = self.latest_obs.hand_actual_q.copy()
        elif self._latched_hand_q is not None:
            self._exit_zero_hand_start = self._latched_hand_q.copy()
        else:
            self._exit_zero_hand_start = np.zeros(HAND_NUM_DOFS, dtype=np.float64)

        self._notify_hotkey("安全退出: 阶段 1/2 手腕 (J4-J6) 优先平滑回正展开...")
        self.get_logger().info(
            "安全退出启动: 阶段 1/2 手腕与末端 (J4-J6) 优先平滑回正展开 (历时 1.8s)..."
        )

    def _handle_mode_cycle(self) -> None:
        from .perception.imu_stream import cycle_orientation_mode

        allowed, reason = self._hotkey_guard()
        if not allowed:
            self._notify_hotkey(reason)
            return
        self.current_orientation_mode = cycle_orientation_mode(
            self.current_orientation_mode
        )
        self.hotkeys.orientation_mode = self.current_orientation_mode
        self._notify_hotkey(f"Mode -> {self.current_orientation_mode.name}")

    def _handle_imu_zero(self, imu_reading) -> None:
        allowed, reason = self._hotkey_guard()
        if not allowed:
            self._notify_hotkey(reason)
            return
        if self.no_imu:
            self._notify_hotkey("IMU功能未启用(no_imu=true)，如需启用请去除参数启动")
            return
        if imu_reading is not None and imu_reading.is_valid:
            self.wrist_imu_tracker.zero(imu_reading)
            self._notify_hotkey("IMU 零位已重置")
        else:
            self.wrist_imu_tracker.zero(None)
            self._notify_hotkey("IMU 无有效读数 (串口数据流未就绪)")

    def _handle_hand_zero(self, wrist_pose) -> None:
        if getattr(self, "_hand_hold_enabled", False):
            self._notify_hotkey("请先按 [L] 解除灵巧手保持后再校准零位")
            return
        allowed, reason = self._hotkey_guard()
        if not allowed:
            self._notify_hotkey(reason)
            return
        self.hand_retargeter.zero(wrist_pose)
        self._notify_hotkey("手部零位已校准")

    def _load_session_calibration(self, path: Path) -> Optional[dict]:
        """Load session calibration YAML."""
        if not path.is_file():
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
                if isinstance(data, dict):
                    self.get_logger().info(f"Loaded session calibration from {path}")
                    return data
        except Exception as e:
            self.get_logger().warning(f"Failed to load session calibration from {path} ({e})")
        return None

    def _parse_table_from_session(self, session_data: dict) -> Optional[ArUcoCalibrationData]:
        """Parse ArUcoCalibrationData from session dict."""
        if not isinstance(session_data, dict) or "table" not in session_data:
            return None
        t_info = session_data["table"]
        if not t_info.get("is_valid", False):
            return None
        try:
            return ArUcoCalibrationData(
                t_cam_to_table=np.array(t_info["t_cam_to_table"], dtype=np.float64),
                t_table_to_cam=np.array(t_info["t_table_to_cam"], dtype=np.float64),
                tag_id=int(t_info.get("primary_id", 0)),
                tag_size_m=float(t_info.get("marker_size_m", 0.080)),
                num_samples=int(t_info.get("num_samples", 0)),
                is_valid=True,
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to parse table calibration: {e}")
            return None

    def _load_table_calibration(self, path: Path) -> Optional[ArUcoCalibrationData]:
        """Load legacy table calibration YAML."""
        if not path.is_file():
            return None
        try:
            calibrator = TableCalibrator(primary_id=0, secondary_id=1)
            return calibrator.load_calibration(str(path))
        except Exception as e:
            self.get_logger().warning(f"Failed to load table calibration ({e})")
            return None

    def _load_teleop_posture_config(self) -> None:
        """加载自定义真机臂手准备姿态与运动配置文件 (configs/teleop_real_posture.yaml)."""
        cfg_val = str(self.get_parameter("teleop_config").value or "").strip()
        cfg_path = None
        if cfg_val:
            cfg_path = _resolve_config_path(cfg_val)
        else:
            for cand in ("configs/teleop_real_posture.yaml", "src/tuinadex_teleop/configs/teleop_real_posture.yaml"):
                default_p = _resolve_config_path(cand)
                if default_p.is_file():
                    cfg_path = default_p
                    break

        if cfg_path is None or not cfg_path.is_file():
            return

        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            # 1. 机械臂准备姿态
            arm_data = data.get("arm", {})
            if "ready_pose_deg" in arm_data:
                deg_info = arm_data["ready_pose_deg"]
                if isinstance(deg_info, dict):
                    deg_list = [float(deg_info.get(f"joint{i+1}", 0.0)) for i in range(ARM_NUM_DOFS)]
                elif isinstance(deg_info, (list, tuple)):
                    deg_list = [float(x) for x in deg_info[:ARM_NUM_DOFS]]
                else:
                    deg_list = None
                if deg_list and len(deg_list) == ARM_NUM_DOFS:
                    self.ready_arm_q = np.radians(deg_list).astype(np.float64)
            elif "ready_pose_rad" in arm_data:
                rad_info = arm_data["ready_pose_rad"]
                if isinstance(rad_info, dict):
                    rad_list = [float(rad_info.get(f"joint{i+1}", 0.0)) for i in range(ARM_NUM_DOFS)]
                elif isinstance(rad_info, (list, tuple)):
                    rad_list = [float(x) for x in rad_info[:ARM_NUM_DOFS]]
                else:
                    rad_list = None
                if rad_list and len(rad_list) == ARM_NUM_DOFS:
                    self.ready_arm_q = np.array(rad_list, dtype=np.float64)

            # 同步更新运动学求解器基准姿态与笛卡尔正运动学零位参考
            if hasattr(self, "arm_ik") and self.arm_ik is not None:
                self.arm_ik.home_q = self.ready_arm_q.copy()
                self.home_pos, self.home_R = self.arm_ik.forward_kinematics(self.ready_arm_q)
            if hasattr(self, "clutch") and self.clutch is not None:
                if hasattr(self, "home_pos") and self.home_pos is not None:
                    self.clutch.home_pos = self.home_pos.copy()
                if hasattr(self, "home_R") and self.home_R is not None:
                    self.clutch.home_rot = self.home_R.copy()
            if hasattr(self, "wrist_imu_tracker") and self.wrist_imu_tracker is not None:
                if hasattr(self, "home_R") and self.home_R is not None:
                    self.wrist_imu_tracker.neutral_robot_rot = self.home_R.copy()

            # 2. 平滑回位耗时
            motion_data = arm_data.get("motion", {})
            if "homing_duration_sec" in motion_data:
                self._homing_duration = max(0.5, float(motion_data["homing_duration_sec"]))

            # 3. 灵巧手准备姿态
            hand_data = data.get("hand", {})
            if "ready_pose_rad" in hand_data:
                h_info = hand_data["ready_pose_rad"]
                if isinstance(h_info, dict):
                    h_list = [float(h_info.get(f"joint_{i}", 0.0)) for i in range(HAND_NUM_DOFS)]
                elif isinstance(h_info, (list, tuple)):
                    h_list = [float(x) for x in h_info[:HAND_NUM_DOFS]]
                else:
                    h_list = None
                if h_list and len(h_list) == HAND_NUM_DOFS:
                    self.ready_hand_q = np.array(h_list, dtype=np.float64)

            # 动态更新灵巧手滤波平滑参数 (1€ filter: cutoff, beta, deadband)
            if hasattr(self, "hand_retargeter") and self.hand_retargeter is not None:
                if hasattr(self.hand_retargeter, "_filter"):
                    f = self.hand_retargeter._filter
                    if "filter_cutoff" in hand_data:
                        f.min_cutoff = max(0.05, float(hand_data["filter_cutoff"]))
                    if "filter_beta" in hand_data:
                        f.beta = max(0.001, float(hand_data["filter_beta"]))
                    if "filter_deadband" in hand_data:
                        f.deadband = max(0.0, float(hand_data["filter_deadband"]))

            # 4. 遥控交互与轴对齐配置
            teleop_data = data.get("teleop", {})
            if "axis_mapping" in teleop_data:
                m_str = str(teleop_data["axis_mapping"]).strip()
                if m_str:
                    self.axis_mapping = m_str
                    if hasattr(self, "clutch") and self.clutch is not None:
                        self.clutch.set_axis_mapping(m_str)
                    if hasattr(self, "hotkeys") and self.hotkeys is not None:
                        self.hotkeys.axis_mapping = m_str

            imu_map_str = None
            for key in ("IMUaxis", "imu_axis_mapping", "imu_axis", "imuaxis"):
                if key in teleop_data:
                    imu_map_str = str(teleop_data[key]).strip()
                    break
            if not imu_map_str and "imu" in data and isinstance(data["imu"], dict):
                for key in ("IMUaxis", "imu_axis_mapping", "imu_axis", "imuaxis"):
                    if key in data["imu"]:
                        imu_map_str = str(data["imu"][key]).strip()
                        break
            if imu_map_str:
                self.imu_axis_mapping = imu_map_str
                if hasattr(self, "wrist_imu_tracker") and self.wrist_imu_tracker is not None:
                    self.wrist_imu_tracker.set_axis_mapping(imu_map_str)

            self.get_logger().info(
                f"[CONFIG] 成功从 {cfg_path.name} 加载真机准备姿态配置: "
                f"机械臂就绪姿态={[round(float(math.degrees(x)), 2) for x in self.ready_arm_q]}°, "
                f"回位时长={self._homing_duration}s, 轴映射={self.axis_mapping}, IMU轴映射={self.imu_axis_mapping}"
            )
        except Exception as e:
            self.get_logger().warning(f"读取真机姿态配置 {cfg_path} 异常: {e}，回退使用系统默认值。")

    def destroy_node(self) -> None:
        """Clean shutdown hook."""
        self.get_logger().info("Shutting down TeleopSimCompositeNode...")
        self._destroyed = True
        for sock_attr in ("_udp_home_sock", "_udp_sock"):
            sock = getattr(self, sock_attr, None)
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        if hasattr(self, "stream") and self.stream is not None:
            self.stream.stop()
        if hasattr(self, "imu_stream") and self.imu_stream is not None:
            self.imu_stream.stop()
        if hasattr(self, "wrist_tracker") and self.wrist_tracker is not None:
            try:
                self.wrist_tracker.close()
            except Exception:
                pass
        if hasattr(self, "sim_adapter") and self.sim_adapter is not None:
            self.sim_adapter.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSimCompositeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
