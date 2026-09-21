import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("tuinadex_teleop")
    default_config = os.path.join(pkg_dir, "config", "teleop_params.yaml")

    camera_id_arg = DeclareLaunchArgument(
        "camera_id", default_value="0", description="Camera device index (e.g. 0 for /dev/video0)"
    )
    show_preview_arg = DeclareLaunchArgument(
        "show_preview", default_value="true", description="Show OpenCV preview window"
    )
    handedness_arg = DeclareLaunchArgument(
        "handedness", default_value="any", description="Filter handedness: any, Right, Left"
    )

    teleop_node = Node(
        package="tuinadex_teleop",
        executable="teleop_gesture_node",
        name="teleop_gesture_node",
        output="screen",
        parameters=[
            default_config,
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "show_preview": LaunchConfiguration("show_preview"),
                "required_handedness": LaunchConfiguration("handedness"),
            },
        ],
    )

    return LaunchDescription([
        camera_id_arg,
        show_preview_arg,
        handedness_arg,
        teleop_node,
    ])
