"""teleop_composite_real.launch.py — 22-DOF 臂手协同真实硬件视觉遥操作一键拉起 Launch.

同时加载:
1. tuinadex_arm / arm_node (6-DOF 机械臂 SocketCAN 底层驱动，订阅 /arm/joint_command, 发布 /arm/joint_states)
2. tuinadex_hand / leap_hand_node (16-DOF 灵巧手 Dynamixel 驱动，订阅 /hand/joint_command, 发布 /hand/joint_states)
3. tuinadex_teleop / teleop_sim_composite_node (22-DOF 视觉遥操解算内核，use_sim:=false 模式，发布 commands 并监听 states)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_pkg = get_package_share_directory("tuinadex_teleop")
    hand_pkg = get_package_share_directory("tuinadex_hand")
    arm_pkg = get_package_share_directory("tuinadex_arm")

    default_model = os.path.join(teleop_pkg, "configs", "mjcf", "tuina_sim_scene.xml")
    default_session_cfg = os.path.join(teleop_pkg, "configs", "session_calibration.yaml")
    default_table_cfg = os.path.join(teleop_pkg, "configs", "table_calibration.yaml")
    default_urdf_arm = os.path.join(teleop_pkg, "configs", "urdf", "rebot_arm_rs.urdf")
    default_urdf_hand = os.path.join(teleop_pkg, "configs", "urdf", "leap_hand_right.urdf")
    default_teleop_posture_cfg = os.path.join(teleop_pkg, "configs", "teleop_real_posture.yaml")

    default_hand_config = os.path.join(hand_pkg, "config", "leap_hand_params.yaml")
    default_arm_config = os.path.join(arm_pkg, "config", "arm_params.yaml")

    arm_channel_arg = DeclareLaunchArgument(
        "arm_channel", default_value="can0", description="SocketCAN channel for 6-DOF reBot arm"
    )
    hand_port_arg = DeclareLaunchArgument(
        "hand_port", default_value="/dev/ttyUSB0", description="Serial port for 16-DOF LEAP hand"
    )
    camera_id_arg = DeclareLaunchArgument(
        "camera_id", default_value="4", description="Camera device index (default: 4 for RealSense RGB)"
    )
    teleop_config_arg = DeclareLaunchArgument(
        "teleop_config", default_value=default_teleop_posture_cfg, description="Path to teleop_real_posture.yaml"
    )
    mock_arg = DeclareLaunchArgument(
        "mock", default_value="false", description="Run hardware drivers and camera in mock mode"
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true", description="Show OpenCV teleop HUD tracking window"
    )
    rate_hz_arg = DeclareLaunchArgument(
        "rate_hz", default_value="60.0", description="Publish and control loop rate in Hz"
    )
    no_imu_arg = DeclareLaunchArgument(
        "no_imu", default_value="false", description="Disable IMU attitude tracking"
    )
    axis_mapping_arg = DeclareLaunchArgument(
        "axis_mapping", default_value="+y,-x,+z", description="Human wrist to robot pos mapping"
    )
    mock_hand_default = "true" if not os.path.exists("/dev/ttyUSB0") else "false"
    mock_hand_arg = DeclareLaunchArgument(
        "mock_hand",
        default_value=mock_hand_default,
        description="Run LEAP hand in mock mode (auto-detected true if /dev/ttyUSB0 is absent)",
    )

    # 1. 灵巧手硬件驱动节点
    hand_node = Node(
        package="tuinadex_hand",
        executable="leap_hand_node",
        name="leap_hand_node",
        output="screen",
        parameters=[
            default_hand_config,
            {
                "port": LaunchConfiguration("hand_port"),
                "mock_mode": LaunchConfiguration("mock_hand"),
                "publish_rate_hz": LaunchConfiguration("rate_hz"),
            },
        ],
    )

    # 2. 机械臂硬件驱动节点
    arm_node = Node(
        package="tuinadex_arm",
        executable="arm_node",
        name="arm_node",
        output="screen",
        parameters=[
            default_arm_config,
            {
                "channel": LaunchConfiguration("arm_channel"),
                "mock_mode": LaunchConfiguration("mock"),
                "control_rate_hz": LaunchConfiguration("rate_hz"),
            },
        ],
    )

    # 3. 22-DOF 臂手联合视觉遥操作节点 (use_sim:=false)
    teleop_node = Node(
        package="tuinadex_teleop",
        executable="teleop_sim_composite_node",
        name="teleop_sim_composite_node",
        output="screen",
        parameters=[{
            "model_path": default_model,
            "session_config": default_session_cfg,
            "table_config": default_table_cfg,
            "urdf_arm_path": default_urdf_arm,
            "urdf_hand_path": default_urdf_hand,
            "use_sim": False,
            "mock": LaunchConfiguration("mock"),
            "no_imu": LaunchConfiguration("no_imu"),
            "publish_rate_hz": LaunchConfiguration("rate_hz"),
            "gui": LaunchConfiguration("gui"),
            "axis_mapping": LaunchConfiguration("axis_mapping"),
            "teleop_config": LaunchConfiguration("teleop_config"),
        }],
    )

    # 当 teleop 视窗完成分步安全归零退出后，触发 launch 全局关机
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=teleop_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        arm_channel_arg,
        hand_port_arg,
        camera_id_arg,
        teleop_config_arg,
        axis_mapping_arg,
        mock_arg,
        gui_arg,
        rate_hz_arg,
        no_imu_arg,
        mock_hand_arg,
        arm_node,
        hand_node,
        teleop_node,
        exit_handler,
    ])

