import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("tuinadex_arm")
    default_config = os.path.join(pkg_dir, "config", "arm_params.yaml")

    channel_arg = DeclareLaunchArgument("channel", default_value="can0", description="CAN channel")
    mock_arg = DeclareLaunchArgument("mock", default_value="false", description="Run in mock mode")
    rate_arg = DeclareLaunchArgument("control_rate_hz", default_value="50.0", description="Control rate")
    timeout_arg = DeclareLaunchArgument("watchdog_timeout_sec", default_value="0.30")
    hold_arg = DeclareLaunchArgument("watchdog_hold_sec", default_value="0.30")
    ramp_arg = DeclareLaunchArgument("watchdog_ramp_sec", default_value="0.60")

    arm_node = Node(
        package="tuinadex_arm",
        executable="arm_node",
        name="arm_node",
        output="screen",
        parameters=[
            default_config,
            {
                "channel": LaunchConfiguration("channel"),
                "mock_mode": LaunchConfiguration("mock"),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "watchdog_timeout_sec": LaunchConfiguration("watchdog_timeout_sec"),
                "watchdog_hold_sec": LaunchConfiguration("watchdog_hold_sec"),
                "watchdog_ramp_sec": LaunchConfiguration("watchdog_ramp_sec"),
            },
        ],
    )

    return LaunchDescription([
        channel_arg,
        mock_arg,
        rate_arg,
        timeout_arg,
        hold_arg,
        ramp_arg,
        arm_node,
    ])

