import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("tuinadex_hand")
    sim_pkg_dir = get_package_share_directory("tuinadex_sim")
    default_config = os.path.join(pkg_dir, "config", "leap_hand_params.yaml")
    default_sim_config = os.path.join(sim_pkg_dir, "config", "sim_params.yaml")

    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyUSB0", description="Serial port")
    mock_arg = DeclareLaunchArgument("mock", default_value="false", description="Run in mock mode")
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Launch MuJoCo digital twin instead of hardware runtime",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz", default_value="50.0", description="ROS2 state rate"
    )
    use_scene_world_arg = DeclareLaunchArgument(
        "use_scene_world",
        default_value="true",
        description="Load full Tuina scene instead of hand-only world",
    )
    watchdog_timeout_arg = DeclareLaunchArgument(
        "watchdog_timeout_sec", default_value="0.30"
    )
    watchdog_hold_arg = DeclareLaunchArgument(
        "watchdog_hold_sec", default_value="0.30"
    )
    watchdog_ramp_arg = DeclareLaunchArgument(
        "watchdog_ramp_sec", default_value="0.60"
    )
    physics_substeps_arg = DeclareLaunchArgument(
        "physics_substeps", default_value="10"
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Launch MuJoCo 3D passive viewer window",
    )

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
        condition=UnlessCondition(LaunchConfiguration("use_sim")),
    )

    sim_node = Node(
        package="tuinadex_sim",
        executable="leap_hand_sim_node",
        name="leap_hand_sim_node",
        output="screen",
        parameters=[
            default_sim_config,
            {
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "use_scene_world": LaunchConfiguration("use_scene_world"),
                "watchdog_timeout_sec": LaunchConfiguration("watchdog_timeout_sec"),
                "watchdog_hold_sec": LaunchConfiguration("watchdog_hold_sec"),
                "watchdog_ramp_sec": LaunchConfiguration("watchdog_ramp_sec"),
                "physics_substeps": LaunchConfiguration("physics_substeps"),
                "gui": LaunchConfiguration("gui"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    return LaunchDescription([
        port_arg,
        mock_arg,
        use_sim_arg,
        gui_arg,
        publish_rate_arg,
        use_scene_world_arg,
        watchdog_timeout_arg,
        watchdog_hold_arg,
        watchdog_ramp_arg,
        physics_substeps_arg,
        hand_node,
        sim_node,
    ])
