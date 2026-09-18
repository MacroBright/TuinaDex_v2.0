import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("tuinadex_hand")
    default_config = os.path.join(pkg_dir, "config", "leap_hand_params.yaml")

    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyUSB0", description="Serial port")
    mock_arg = DeclareLaunchArgument("mock", default_value="false", description="Run in mock mode")

    hand_node = Node(
        package="tuinadex_hand",
        executable="leap_hand_node",
        name="leap_hand_node",
        output="screen",
        parameters=[
            default_config,
            {
                "port": LaunchConfiguration("port"),
                "mock_mode": LaunchConfiguration("mock"),
            },
        ],
    )

    return LaunchDescription([
        port_arg,
        mock_arg,
        hand_node,
    ])
