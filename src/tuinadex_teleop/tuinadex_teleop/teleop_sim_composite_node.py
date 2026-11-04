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

import os
from pathlib import Path
import sys
import time
from typing import Optional

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
from std_msgs.msg import Empty

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
    R_DOWNWARD_DEFAULT,
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
        self.declare_parameter("auto_engage", True)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("camera_fps", 0)
        self.declare_parameter("solver_mode", "hybrid")
        self.declare_parameter("axis_mapping", "-y,+x,+z")
        self.declare_parameter("imu_axis_mapping", "+x,+y,+z")
        self.declare_parameter("gui", False)

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

        # 2. Kinematics & Controllers
        self.wrist_tracker = WristTracker(default_handedness="Right")
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

        self.arm_ik = ArmIKSolver(
            urdf_path=self.urdf_arm_p,
            ee_frame_name="link6",
            home_q=DEFAULT_READY_ARM_Q,
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

        if self.use_sim:
            self.arm_state_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
            self.hand_state_pub = self.create_publisher(JointState, "/hand/joint_states", 10)
        else:
            self.arm_state_pub = None
            self.hand_state_pub = None

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

        # Direct UDP Broadcast socket for cross-environment IPC
        import socket
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_dest = ("127.0.0.1", 9876)

        # Listen for UDP HOME trigger from Web Hub
        self._udp_home_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._udp_home_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._udp_home_sock.bind(("127.0.0.1", 9877))
            self._udp_home_sock.settimeout(0.2)
            def _udp_home_worker():
                import rclpy
                while rclpy.ok():
                    try:
                        data, _ = self._udp_home_sock.recvfrom(32)
                        if data == b"HOME":
                            self._handle_home_pose()
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

        # Actual robot end-effector Cartesian pose
        cur_arm_p, cur_arm_R = self.arm_ik.forward_kinematics(self.latest_obs.arm_actual_q)

        # 3. Safety Evaluation
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
        )

        # 4. Target Arbitration & Safety Actions
        if self.safety_report.level == SafetyLevel.RETRACTING and self.safety_supervisor.retract_controller is not None:
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
                ):
                    self.clutch.engage(
                        current_human_pos=wrist_pose.position_table,
                        current_robot_pos=cur_arm_p,
                        current_human_rot=wrist_pose.orientation_table_matrix,
                        current_robot_rot=cur_arm_R,
                    )
                    if self.safety_report.level in (SafetyLevel.FROZEN, SafetyLevel.RETRACTING):
                        self.safety_supervisor.reset()
                    if imu_reading.is_valid and not self.wrist_imu_tracker.is_calibrated:
                        self.wrist_imu_tracker.zero(imu_reading)
                target_pos = self.clutch.compute_target(wrist_pose.position_table, fallback_robot_pos=self.home_pos)
            else:
                target_pos = self.clutch.compute_target(np.zeros(3), fallback_robot_pos=self.home_pos)

            # Target orientation
            if self.current_orientation_mode == ArmOrientationMode.TRACK_IMU and imu_reading.is_valid:
                R_target = self.wrist_imu_tracker.compute_target_rotation(
                    reading=imu_reading,
                    fallback_rot=self.home_R,
                    anchor_robot_rot=self.home_R,
                )
            elif self.current_orientation_mode == ArmOrientationMode.FIXED_DOWNWARD:
                R_target = R_DOWNWARD_DEFAULT.copy()
            elif self.current_orientation_mode == ArmOrientationMode.TRACK_WRIST:
                R_target = (
                    wrist_pose.orientation_table_matrix
                    if wrist_pose.orientation_table_matrix is not None
                    else R_DOWNWARD_DEFAULT.copy()
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

        # 5. Publish target commands to ROS 2
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
            )
            cv2.imshow("TuinaDex 22-DOF Teleoperation HUD", dashboard)
            key = cv2.waitKey(1) & 0xFF
            if key in (32, ord("c")):
                tot_f = self.contact_forces.get("total", 0.0)
                if self.clutch.state != ClutchState.ENGAGED and tot_f > 40.0:
                    self.get_logger().warn(f"无法接管: 接触力({tot_f:.1f}N)过大，请先脱离接触！")
                else:
                    cur_ee_p, cur_ee_R = self.arm_ik.forward_kinematics(self.latest_obs.arm_actual_q)
                    self.clutch.toggle(
                        current_human_pos=wrist_pose.position_table if wrist_pose.position_table is not None else None,
                        current_robot_pos=cur_ee_p,
                        current_human_rot=wrist_pose.orientation_table_matrix,
                        current_robot_rot=cur_ee_R,
                    )
                    if self.clutch.state == ClutchState.ENGAGED and imu_reading.is_valid:
                        self.wrist_imu_tracker.zero(imu_reading)
            elif key in (ord("s"), ord("S")):
                self.safety_supervisor.toggle_enabled()
            elif key in (ord("a"), ord("A")):
                self._handle_axis_cycle()
            elif key in (ord("x"),):
                self._handle_cycle_imu_axes(forward=True)
            elif key in (ord("X"),):
                self._handle_cycle_imu_axes(forward=False)
            elif key in (ord("h"), ord("H")):
                self._handle_home_pose()
            elif key in (ord("m"), ord("M")):
                self._handle_mode_cycle()
            elif key in (ord("i"), ord("I")):
                self._handle_imu_zero(imu_reading)
            elif key in (ord("z"), ord("Z")):
                self._handle_hand_zero(wrist_pose)

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

    def _handle_home_pose(self) -> None:
        self.clutch.disengage()
        self.arm_ik.reset()
        self.hand_retargeter.reset()
        if self.use_sim and self.sim_adapter is not None:
            now = time.monotonic()
            home_arm = TeleopArmCommand(
                timestamp=now,
                target_pose=np.eye(4),
                target_q=DEFAULT_READY_ARM_Q.copy(),
            )
            home_hand = TeleopHandCommand(
                timestamp=now,
                target_q=np.zeros(16, dtype=np.float32),
            )
            cmd = TeleopCommand(
                timestamp=now,
                arm_command=home_arm,
                hand_command=home_hand,
                clutch_state=ClutchState.DISENGAGED,
            )
            self.sim_adapter.send_command(cmd)
        self._notify_hotkey("一键回初始姿态: 臂手已复位至就绪位姿")

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

    def destroy_node(self) -> None:
        """Clean shutdown hook."""
        self.get_logger().info("Shutting down TeleopSimCompositeNode...")
        if hasattr(self, "stream") and self.stream is not None:
            self.stream.stop()
        if hasattr(self, "imu_stream") and self.imu_stream is not None:
            self.imu_stream.stop()
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
