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
    mock_camera_arg = DeclareLaunchArgument(
        "mock_camera", default_value="false", description="Use synthetic camera frames"
    )

    # 系统 Python 3.14 自带完整的 mediapipe/opencv/numpy，严禁注入跨版本 3.10 site-packages
    extra_env = {}

    teleop_node = Node(
        package="tuinadex_teleop",
        executable="teleop_gesture_node",
        name="teleop_gesture_node",
        output="screen",
        additional_env=extra_env,
        parameters=[
            default_config,
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "show_preview": LaunchConfiguration("show_preview"),
                "required_handedness": LaunchConfiguration("handedness"),
                "mock_camera": LaunchConfiguration("mock_camera"),
            },
        ],
    )

    return LaunchDescription([
        camera_id_arg,
        show_preview_arg,
        handedness_arg,
        mock_camera_arg,
        teleop_node,
    ])
