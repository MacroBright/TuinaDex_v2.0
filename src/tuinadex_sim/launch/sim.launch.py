import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("tuinadex_sim")
    default_config = os.path.join(pkg_dir, "config", "sim_params.yaml")

    rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="50.0",
        description="ROS2 joint state publish rate (Hz)",
    )
    scene_arg = DeclareLaunchArgument(
        "use_scene_world",
        default_value="true",
        description="Whether to load full Tuina scene with bed and patient dummy",
    )
    timeout_arg = DeclareLaunchArgument(
        "watchdog_timeout_sec", default_value="0.30"
    )
    hold_arg = DeclareLaunchArgument(
        "watchdog_hold_sec", default_value="0.30"
    )
    ramp_arg = DeclareLaunchArgument(
        "watchdog_ramp_sec", default_value="0.60"
    )
    substeps_arg = DeclareLaunchArgument(
        "physics_substeps", default_value="10"
    )

    sim_node = Node(
        package="tuinadex_sim",
        executable="leap_hand_sim_node",
        name="leap_hand_sim_node",
        output="screen",
        parameters=[
            default_config,
            {
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "use_scene_world": LaunchConfiguration("use_scene_world"),
                "watchdog_timeout_sec": LaunchConfiguration("watchdog_timeout_sec"),
                "watchdog_hold_sec": LaunchConfiguration("watchdog_hold_sec"),
                "watchdog_ramp_sec": LaunchConfiguration("watchdog_ramp_sec"),
                "physics_substeps": LaunchConfiguration("physics_substeps"),
            },
        ],
    )

    return LaunchDescription([
        rate_arg,
        scene_arg,
        timeout_arg,
        hold_arg,
        ramp_arg,
        substeps_arg,
        sim_node,
    ])
