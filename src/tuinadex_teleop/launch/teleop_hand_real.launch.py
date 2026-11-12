"""teleop_hand_real.launch.py — 16-DOF LEAP Hand 真实硬件视觉遥操作一键拉起 Launch.

同时加载:
1. tuinadex_hand / leap_hand_node (硬件驱动底座，订阅 /hand/joint_command，发布 /hand/joint_states)
2. tuinadex_teleop / teleop_gesture_node (视觉手势感知，捕捉摄像头骨骼点，发布 /hand/joint_command)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hand_pkg = get_package_share_directory("tuinadex_hand")
    teleop_pkg = get_package_share_directory("tuinadex_teleop")

    hand_default_config = os.path.join(hand_pkg, "config", "leap_hand_params.yaml")
    teleop_default_config = os.path.join(teleop_pkg, "config", "teleop_params.yaml")

    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0", description="Serial port for LEAP Hand"
    )
    camera_id_arg = DeclareLaunchArgument(
        "camera_id", default_value="4", description="Camera device index (default: 4 for RealSense RGB)"
    )
    mock_arg = DeclareLaunchArgument(
        "mock", default_value="false", description="Run both driver and camera in mock mode"
    )
    show_preview_arg = DeclareLaunchArgument(
        "show_preview", default_value="true", description="Show OpenCV teleop HUD preview"
    )
    rate_hz_arg = DeclareLaunchArgument(
        "rate_hz", default_value="50.0", description="Control and telemetry loop rate in Hz"
    )

    # 1. 灵巧手硬件驱动节点
    hand_node = Node(
        package="tuinadex_hand",
        executable="leap_hand_node",
        name="leap_hand_node",
        output="screen",
        parameters=[
            hand_default_config,
            {
                "port": LaunchConfiguration("port"),
                "mock_mode": LaunchConfiguration("mock"),
                "publish_rate_hz": LaunchConfiguration("rate_hz"),
            },
        ],
    )

    # 2. 视觉手势感知与动作重定向节点
    teleop_node = Node(
        package="tuinadex_teleop",
        executable="teleop_gesture_node",
        name="teleop_gesture_node",
        output="screen",
        parameters=[
            teleop_default_config,
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "show_preview": LaunchConfiguration("show_preview"),
                "mock_camera": LaunchConfiguration("mock"),
                "publish_rate_hz": LaunchConfiguration("rate_hz"),
            },
        ],
    )

    return LaunchDescription([
        port_arg,
        camera_id_arg,
        mock_arg,
        show_preview_arg,
        rate_hz_arg,
        hand_node,
        teleop_node,
    ])

