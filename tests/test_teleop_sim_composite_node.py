"""test_teleop_sim_composite_node.py — Integration test for TeleopSimCompositeNode.

Verifies:
1. Node initializes properly with mock camera, mock IMU, and headless simulation.
2. Node publishes target commands on /arm/joint_command (6-DOF) and /hand/joint_command (16-DOF).
3. In sim mode, node publishes simulation feedback on /arm/joint_states and /hand/joint_states.
4. Message payloads strictly conform to TuinaDex canonical names and valid radian arrays.
"""
from __future__ import annotations

import time
import numpy as np
import pytest

import rclpy
from sensor_msgs.msg import JointState

from tuinadex_core.names import CANONICAL_ARM_JOINT_NAMES, CANONICAL_JOINT_NAMES
from tuinadex_teleop.teleop_sim_composite_node import TeleopSimCompositeNode


@pytest.fixture(scope="function")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_wrist_pose(
    *,
    timestamp: float,
    is_valid: bool,
    tracking_state,
    position_table: np.ndarray | None = None,
):
    from tuinadex_teleop.perception.wrist_tracker import WristPose

    pos_t = np.zeros(3, dtype=np.float64) if position_table is None else position_table
    return WristPose(
        timestamp=timestamp,
        is_valid=is_valid,
        tracking_state=tracking_state,
        position_cam=np.zeros(3, dtype=np.float64),
        position_table=pos_t,
        position_table_raw=pos_t.copy(),
        orientation_table_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        orientation_table_matrix=np.eye(3, dtype=np.float64),
        euler_deg=(0.0, 0.0, 0.0),
        wrist_pixel_uv=(0.0, 0.0),
        landmarks_2d=np.zeros((21, 2), dtype=np.float64),
        landmarks_3d_cam=np.zeros((21, 3), dtype=np.float64),
        depth_m=0.0,
    )


def test_teleop_sim_composite_node_mock_lifecycle(rclpy_init):
    """Test full loopback execution of TeleopSimCompositeNode in mock mode."""
    from rclpy.parameter import Parameter
    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("mock_imu", Parameter.Type.BOOL, True),
        Parameter("auto_engage", Parameter.Type.BOOL, True),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 50.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)

    arm_cmd_received = []
    hand_cmd_received = []
    arm_state_received = []
    hand_state_received = []

    sub_node = rclpy.create_node("teleop_sim_test_sub")

    def _arm_cmd_cb(msg: JointState):
        arm_cmd_received.append(msg)

    def _hand_cmd_cb(msg: JointState):
        hand_cmd_received.append(msg)

    def _arm_state_cb(msg: JointState):
        arm_state_received.append(msg)

    def _hand_state_cb(msg: JointState):
        hand_state_received.append(msg)

    sub_node.create_subscription(JointState, "/arm/joint_command", _arm_cmd_cb, 10)
    sub_node.create_subscription(JointState, "/hand/joint_command", _hand_cmd_cb, 10)
    sub_node.create_subscription(JointState, "/arm/joint_states", _arm_state_cb, 10)
    sub_node.create_subscription(JointState, "/hand/joint_states", _hand_state_cb, 10)

    try:
        # Step node for 15 cycles
        for _ in range(15):
            node._loop_step()
            rclpy.spin_once(sub_node, timeout_sec=0.01)
            time.sleep(0.01)

        # Verify command publication
        assert len(arm_cmd_received) > 0, "No /arm/joint_command received"
        assert len(hand_cmd_received) > 0, "No /hand/joint_command received"

        latest_arm_cmd = arm_cmd_received[-1]
        assert list(latest_arm_cmd.name) == list(CANONICAL_ARM_JOINT_NAMES)
        assert len(latest_arm_cmd.position) == 6
        assert np.all(np.isfinite(latest_arm_cmd.position))

        latest_hand_cmd = hand_cmd_received[-1]
        assert list(latest_hand_cmd.name) == list(CANONICAL_JOINT_NAMES)
        assert len(latest_hand_cmd.position) == 16
        assert np.all(np.isfinite(latest_hand_cmd.position))

        # Verify simulation state publication
        if node.use_sim:
            assert len(arm_state_received) > 0, "No /arm/joint_states received"
            assert len(hand_state_received) > 0, "No /hand/joint_states received"

            latest_arm_state = arm_state_received[-1]
            assert list(latest_arm_state.name) == list(CANONICAL_ARM_JOINT_NAMES)
            assert len(latest_arm_state.position) == 6
            assert np.all(np.isfinite(latest_arm_state.position))

            latest_hand_state = hand_state_received[-1]
            assert list(latest_hand_state.name) == list(CANONICAL_JOINT_NAMES)
            assert len(latest_hand_state.position) == 16
            assert np.all(np.isfinite(latest_hand_state.position))

    finally:
        node.destroy_node()
        sub_node.destroy_node()


def test_teleop_sim_composite_node_homing_trajectory(rclpy_init):
    """Test smooth Quintic homing transition to DEFAULT_READY_ARM_Q."""
    from rclpy.parameter import Parameter
    from tuinadex_teleop.adapters import DEFAULT_READY_ARM_Q

    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("mock_imu", Parameter.Type.BOOL, True),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 50.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)

    # 人为设置一个偏离就绪位的起点
    node.latest_obs.arm_actual_q = np.array([0.5, 0.2, 0.1, -0.3, 0.2, 0.1], dtype=np.float64)
    node.latest_obs.hand_actual_q = np.ones(16, dtype=np.float64) * 0.8

    try:
        # 触发一键回准备姿态
        node._handle_home_pose()
        assert node._is_homing is True
        assert np.allclose(node._homing_arm_start, [0.5, 0.2, 0.1, -0.3, 0.2, 0.1])

        # 步进一次循环，验证中间插值输出
        node._loop_step()
        assert node._is_homing is True

        # 人为把时间快进到回位完成之后
        node._homing_start_time = time.monotonic() - (node._homing_duration + 0.1)
        node._loop_step()

        # 验证回位已结束并精确对齐
        assert node._is_homing is False
        assert np.allclose(node._latched_hand_q, np.zeros(16), atol=1e-4)
    finally:
        node.destroy_node()


def test_teleop_sim_composite_node_staged_exit_trajectory(rclpy_init):
    """Test two-stage safe exit trajectory (Stage 1: J4-J6 zeroed -> Stage 2: J1-J3 zeroed)."""
    from rclpy.parameter import Parameter

    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("mock_imu", Parameter.Type.BOOL, True),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 50.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)

    # 设置起点：J1-J3 在工作位，J4-J6 (手腕) 在弯折位
    start_arm_q = np.array([0.2, 0.5, 0.6, -0.4, 0.3, -0.2], dtype=np.float64)
    node.latest_obs.arm_actual_q = start_arm_q.copy()
    node.latest_obs.hand_actual_q = np.ones(16, dtype=np.float64) * 0.5

    try:
        # 触发分步安全归零退出
        node._start_safe_exit_to_zero()
        assert node._is_exiting_zero is True
        assert node._exit_zero_stage == 1

        # 阶段 1 进行中：J1-J3 必须保持不变，J4-J6 正在回零
        node._exit_zero_start_time = time.monotonic() - (node._exit_zero_stage1_duration * 0.5)
        node._loop_step()
        assert np.allclose(node._exit_zero_start_q[:3], start_arm_q[:3])
        assert node._exit_zero_stage == 1

        # 快进至阶段 1 完成
        node._exit_zero_start_time = time.monotonic() - (node._exit_zero_stage1_duration + 0.1)
        node._loop_step()
        assert node._exit_zero_stage == 2
        # 验证阶段 1 终点：J4-J6 已精确为 0，J1-J3 依然维持高位起点不变
        assert np.allclose(node._exit_zero_stage1_end_q[3:], [0.0, 0.0, 0.0])
        assert np.allclose(node._exit_zero_stage1_end_q[:3], start_arm_q[:3])

        # 快进至阶段 2 完成
        node._exit_zero_start_time = time.monotonic() - (node._exit_zero_stage2_duration + 0.1)
        node._loop_step()
        assert node._exit_zero_complete is True
    finally:
        node.destroy_node()


def test_teleop_sim_composite_node_no_imu_engage_preserves_initial_posture(rclpy_init, tmp_path):
    """Engaging clutch in --no-imu mode must preserve configured initial posture without snapping."""
    import yaml
    from rclpy.parameter import Parameter

    custom_deg = [0.0, 30.0, 30.0, -30.0, 0.0, 0.0]
    custom_rad = np.deg2rad(custom_deg)
    posture_cfg = tmp_path / "custom_posture.yaml"
    posture_cfg.write_text(
        yaml.safe_dump({
            "arm": {
                "ready_pose_deg": {f"joint{i + 1}": v for i, v in enumerate(custom_deg)},
            }
        }),
        encoding="utf-8",
    )

    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("no_imu", Parameter.Type.BOOL, True),
        Parameter("use_sim", Parameter.Type.BOOL, False),
        Parameter("auto_engage", Parameter.Type.BOOL, True),
        Parameter("teleop_config", Parameter.Type.STRING, str(posture_cfg)),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 60.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)
    sub_node = rclpy.create_node("teleop_no_imu_engage_test_sub")
    arm_cmd_received = []
    sub_node.create_subscription(JointState, "/arm/joint_command", arm_cmd_received.append, 10)

    try:
        # Hold physical feedback synced near configured initial posture;
        # auto_engage engages on first tracked frame and runs 60 cycles with a stationary hand.
        w, h = node.intrinsics.width, node.intrinsics.height
        cx, cy = node.intrinsics.cx, node.intrinsics.cy
        fixed_pts = np.zeros((21, 3), dtype=np.float32)
        fixed_pts[0] = [cx / w, cy / h, 0.0]
        fixed_pts[5] = [(cx - 20.0) / w, (cy - 35.0) / h, 0.0]
        fixed_pts[9] = [cx / w, (cy - 40.0) / h, 0.0]
        fixed_pts[17] = [(cx + 20.0) / w, (cy - 35.0) / h, 0.0]
        node.wrist_tracker.set_custom_detector(
            lambda _bgr: {
                "landmarks": fixed_pts.copy(),
                "world_landmarks": None,
                "handedness": "Right",
                "score": 1.0,
            }
        )
        node.latest_obs.arm_actual_q = custom_rad.copy()
        for _ in range(60):
            node._loop_step()
            rclpy.spin_once(sub_node, timeout_sec=0.005)

        assert len(arm_cmd_received) >= 30
        final_q = np.array(arm_cmd_received[-1].position, dtype=np.float64)
        np.testing.assert_allclose(final_q, custom_rad, atol=1e-3)
        assert node.clutch._robot_rot_anchor is not None
        _, expected_R = node.arm_ik.forward_kinematics(custom_rad)
        np.testing.assert_allclose(node.clutch._robot_rot_anchor, expected_R, atol=1e-5)
    finally:
        node.destroy_node()
        sub_node.destroy_node()


def test_teleop_sim_composite_node_engage_snaps_to_large_actual_offset(rclpy_init, tmp_path):
    """If physical arm sits >0.15 rad away from ready_arm_q at engage, anchor directly there."""
    from rclpy.parameter import Parameter
    from tuinadex_teleop.core.types import TeleopHandCommand
    from tuinadex_teleop.perception.imu_stream import ImuReading
    from tuinadex_teleop.perception.wrist_tracker import TrackingState, WristPose

    alt_rad = np.deg2rad([0.0, 40.0, 35.0, -45.0, 0.0, 0.0])
    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("no_imu", Parameter.Type.BOOL, True),
        Parameter("use_sim", Parameter.Type.BOOL, False),
        Parameter("auto_engage", Parameter.Type.BOOL, False),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 60.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)
    sub_node = rclpy.create_node("teleop_large_offset_engage_test_sub")
    arm_cmd_received = []
    sub_node.create_subscription(JointState, "/arm/joint_command", arm_cmd_received.append, 10)

    try:
        node.latest_obs.arm_actual_q = alt_rad.copy()
        wrist_pose = _make_wrist_pose(
            timestamp=1.0,
            is_valid=True,
            tracking_state=TrackingState.TRACKING,
            position_table=np.array([0.0, 0.0, 0.25], dtype=np.float64),
        )
        hand_cmd = TeleopHandCommand(timestamp=1.0, target_q=np.zeros(16, dtype=np.float32))
        node._engage_clutch(wrist_pose=wrist_pose, hand_cmd=hand_cmd, imu_reading=ImuReading())

        node.wrist_tracker.process_frame = lambda **kwargs: wrist_pose
        node._loop_step()
        rclpy.spin_once(sub_node, timeout_sec=0.005)

        assert len(arm_cmd_received) >= 1
        cmd_q = np.array(arm_cmd_received[-1].position, dtype=np.float64)
        np.testing.assert_allclose(cmd_q, alt_rad, atol=1e-3)
    finally:
        node.destroy_node()
        sub_node.destroy_node()


def test_teleop_sim_composite_node_hand_hold_lifecycle_and_safety_override(rclpy_init):
    """Verify Hand Hold locks hand command across clutch toggles, resumes smoothly, and yields to safety retract."""
    from rclpy.parameter import Parameter
    from tuinadex_teleop.core.safety import SafetyLevel, SafetyViolation
    from tuinadex_teleop.core.types import ClutchState

    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("mock_imu", Parameter.Type.BOOL, True),
        Parameter("use_sim", Parameter.Type.BOOL, False),
        Parameter("auto_engage", Parameter.Type.BOOL, True),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 60.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)
    sub_node = rclpy.create_node("teleop_hand_hold_test_sub")
    hand_cmd_received = []
    sub_node.create_subscription(JointState, "/hand/joint_command", hand_cmd_received.append, 10)

    try:
        held_pose = np.linspace(0.2, 0.9, 16, dtype=np.float64)
        node._last_published_hand_q = held_pose.copy()
        node._last_loop_t_now = time.monotonic()

        # Enable hand hold -> must lock held_pose continuously even when clutch disengages
        assert node._set_hand_hold(True, source="TEST") is True
        node.clutch.disengage()
        for _ in range(5):
            node._loop_step()
            rclpy.spin_once(sub_node, timeout_sec=0.005)
        assert len(hand_cmd_received) >= 5
        np.testing.assert_allclose(np.array(hand_cmd_received[-1].position), held_pose, atol=1e-4)

        # Resume -> must stay slew-limited (no single-frame teleport away from held_pose)
        assert node._set_hand_hold(False, source="TEST") is False
        prev_q = np.array(hand_cmd_received[-1].position, dtype=np.float64)
        node._loop_step()
        rclpy.spin_once(sub_node, timeout_sec=0.005)
        next_q = np.array(hand_cmd_received[-1].position, dtype=np.float64)
        assert np.max(np.abs(next_q - prev_q)) <= 0.35

        # Safety RETRACTING override: while HAND_HOLD is ON and engaged, sustained hand over-current forces release
        node._last_published_hand_q = held_pose.copy()
        assert node._set_hand_hold(True, source="TEST") is True
        node._loop_step()
        assert node.clutch.state == ClutchState.ENGAGED
        node.safety_supervisor.config.max_hand_current_ma = 200.0
        node.safety_supervisor.config.hand_overcurrent_timeout_s = 0.05
        node.latest_obs.hand_current = np.full(16, 300.0, dtype=np.float64)
        node._loop_step()
        time.sleep(0.06)
        node._loop_step()
        assert node.safety_report.level == SafetyLevel.RETRACTING
        assert node.safety_report.violation == SafetyViolation.HAND_CURRENT_OVERLOAD
        assert node._hand_hold_enabled is False
    finally:
        node.destroy_node()
        sub_node.destroy_node()


def test_teleop_sim_composite_node_safety_hold_freezes_arm_and_hand(rclpy_init):
    """During transient Gate-1 SafetyLevel.HOLD, arm must not drift toward zeros(3) and hand stays steady."""
    from rclpy.parameter import Parameter
    from tuinadex_teleop.core.safety import SafetyLevel
    from tuinadex_teleop.perception.wrist_tracker import TrackingState, WristPose

    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("no_imu", Parameter.Type.BOOL, True),
        Parameter("use_sim", Parameter.Type.BOOL, False),
        Parameter("auto_engage", Parameter.Type.BOOL, True),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 60.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)
    sub_node = rclpy.create_node("teleop_safety_hold_sub")
    arm_cmd_received = []
    sub_node.create_subscription(JointState, "/arm/joint_command", arm_cmd_received.append, 10)

    try:
        # First valid step engages clutch
        node._loop_step()
        rclpy.spin_once(sub_node, timeout_sec=0.005)
        pre_hold_arm = np.array(arm_cmd_received[-1].position, dtype=np.float64)

        # Simulate transient hand loss (< 0.80s) -> SafetyLevel.HOLD
        lost_pose = _make_wrist_pose(
            timestamp=time.monotonic(),
            is_valid=False,
            tracking_state=TrackingState.NO_HAND,
        )
        node._last_stream_seq = -999
        node.wrist_tracker.process_frame = lambda **kwargs: lost_pose
        node._loop_step()
        rclpy.spin_once(sub_node, timeout_sec=0.005)

        assert node.safety_report.level == SafetyLevel.HOLD
        hold_arm = np.array(arm_cmd_received[-1].position, dtype=np.float64)
        np.testing.assert_allclose(hold_arm, pre_hold_arm, atol=1e-5)
    finally:
        node.destroy_node()
        sub_node.destroy_node()


def test_teleop_sim_composite_node_home_pose_reloads_config_dynamically(rclpy_init, tmp_path):
    """Verify pressing [R] / calling _handle_home_pose() dynamically reloads ready_pose_deg from file."""
    import yaml
    from rclpy.parameter import Parameter

    cfg_file = tmp_path / "teleop_real_posture.yaml"
    init_deg = [0.0, 70.0, 55.0, -45.0, 0.0, 0.0]
    updated_deg = [15.0, 45.0, 30.0, -25.0, 5.0, 10.0]

    cfg_file.write_text(
        yaml.safe_dump({
            "arm": {
                "ready_pose_deg": {f"joint{i+1}": v for i, v in enumerate(init_deg)},
                "motion": {"homing_duration_sec": 1.0},
            },
            "teleop": {
                "IMUaxis": "+x,+y,+z",
            },
        }),
        encoding="utf-8",
    )

    overrides = [
        Parameter("mock", Parameter.Type.BOOL, True),
        Parameter("no_imu", Parameter.Type.BOOL, True),
        Parameter("use_sim", Parameter.Type.BOOL, False),
        Parameter("teleop_config", Parameter.Type.STRING, str(cfg_file)),
        Parameter("publish_rate_hz", Parameter.Type.DOUBLE, 60.0),
    ]
    node = TeleopSimCompositeNode(parameter_overrides=overrides)
    sub_node = rclpy.create_node("teleop_home_reload_test_sub")
    arm_cmd_received = []
    sub_node.create_subscription(JointState, "/arm/joint_command", arm_cmd_received.append, 10)

    try:
        # Initially loaded init_deg & IMUaxis
        np.testing.assert_allclose(node.ready_arm_q, np.radians(init_deg), atol=1e-5)
        assert node.imu_axis_mapping == "+x,+y,+z"

        # User modifies teleop_real_posture.yaml while system is running
        cfg_file.write_text(
            yaml.safe_dump({
                "arm": {
                    "ready_pose_deg": {f"joint{i+1}": v for i, v in enumerate(updated_deg)},
                    "motion": {"homing_duration_sec": 0.5},
                },
                "teleop": {
                    "IMUaxis": "+y,+x,-z",
                },
            }),
            encoding="utf-8",
        )

        # Trigger return to ready pose (e.g. user presses [R])
        node.latest_obs.arm_actual_q = np.radians(init_deg)
        node._handle_home_pose()

        # Verify ready_arm_q, IK solver home_q and IMUaxis are dynamically synchronized
        np.testing.assert_allclose(node.ready_arm_q, np.radians(updated_deg), atol=1e-5)
        np.testing.assert_allclose(node.arm_ik.home_q, np.radians(updated_deg), atol=1e-5)
        assert node.imu_axis_mapping == "+y,+x,-z"
        assert node.wrist_imu_tracker.axis_mapping_str == "+y,+x,-z"
        np.testing.assert_allclose(node.arm_ik.home_q, np.radians(updated_deg), atol=1e-5)
        assert node._is_homing is True

        # Fast forward time to finish homing
        node._homing_start_time = time.monotonic() - 1.0
        node._loop_step()
        rclpy.spin_once(sub_node, timeout_sec=0.005)

        assert node._is_homing is False
        assert len(arm_cmd_received) >= 1
        final_cmd = np.array(arm_cmd_received[-1].position, dtype=np.float64)
        np.testing.assert_allclose(final_cmd, np.radians(updated_deg), atol=1e-4)
    finally:
        node.destroy_node()
        sub_node.destroy_node()


def test_draw_simulation_hud_titles_distinguish_sim_and_real():
    from tuinadex_teleop.ui.teleop_hud import draw_simulation_hud
    from tuinadex_teleop.core.clutch import ClutchController
    from tuinadex_teleop.core.types import TeleopArmCommand, TeleopHandCommand, RobotObservation
    from tuinadex_teleop.kinematics.arm_ik_solver import ArmOrientationMode, ArmIKMetrics
    from tuinadex_teleop.kinematics.hand_retargeter import HandRetargetingMetrics

    cam = np.zeros((480, 640, 3), dtype=np.uint8)
    clutch = ClutchController()
    arm_cmd = TeleopArmCommand(timestamp=0.0, target_pose=np.eye(4), target_q=np.zeros(6))
    hand_cmd = TeleopHandCommand(timestamp=0.0, target_q=np.zeros(16))
    obs = RobotObservation(
        timestamp=0.0,
        arm_actual_q=np.zeros(6),
        arm_actual_dq=np.zeros(6),
        arm_effort=np.zeros(6),
        hand_actual_q=np.zeros(16),
    )
    arm_metrics = ArmIKMetrics()
    hand_metrics = HandRetargetingMetrics()

    hud_sim = draw_simulation_hud(
        camera_view=cam,
        clutch=clutch,
        orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
        arm_cmd=arm_cmd,
        hand_cmd=hand_cmd,
        obs=obs,
        forces={},
        arm_metrics=arm_metrics,
        hand_metrics=hand_metrics,
        fps=60.0,
        is_sim=True,
    )
    assert hud_sim.shape == (480, 640 + 640, 3)

    hud_real = draw_simulation_hud(
        camera_view=cam,
        clutch=clutch,
        orientation_mode=ArmOrientationMode.FIXED_DOWNWARD,
        arm_cmd=arm_cmd,
        hand_cmd=hand_cmd,
        obs=obs,
        forces={},
        arm_metrics=arm_metrics,
        hand_metrics=hand_metrics,
        fps=60.0,
        is_sim=False,
    )
    assert hud_real.shape == (480, 640 + 640, 3)

