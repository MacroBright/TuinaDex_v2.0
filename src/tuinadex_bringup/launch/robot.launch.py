"""robot.launch.py — TuinaDex v2.0 臂手协同整机联合启动 Launch 入口.

支持:
  1. 模式切换: mode:=arm_hand (默认 22D) | hand_only (16D) | arm_only (6D)
  2. 双模驱动: use_sim:=false (真机/Mock 运行时) | use_sim:=true (MuJoCo 动力学孪生仿真)
  3. 安全自检: mock:=true (脱机安全 Mock 回环测试，杜绝向总线非法写电)
  4. 采集录制: record:=true (一键拉起 rosbag2 录制节点)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("tuinadex_bringup")
    hand_pkg = get_package_share_directory("tuinadex_hand")
    arm_pkg = get_package_share_directory("tuinadex_arm")
    sim_pkg = get_package_share_directory("tuinadex_sim")

    default_robot_config = os.path.join(bringup_pkg, "config", "robot_params.yaml")
    default_hand_config = os.path.join(hand_pkg, "config", "leap_hand_params.yaml")
    default_arm_config = os.path.join(arm_pkg, "config", "arm_params.yaml")
    default_sim_config = os.path.join(sim_pkg, "config", "sim_params.yaml")

    # Launch 参数声明
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="arm_hand",
        description="Operation mode: 'arm_hand' (22D), 'hand_only' (16D), or 'arm_only' (6D)",
    )
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Launch MuJoCo digital twin simulation (true) or hardware runtime (false)",
    )
    mock_arg = DeclareLaunchArgument(
        "mock",
        default_value="false",
        description="Run hardware nodes in safe offline Mock mode",
    )
    arm_channel_arg = DeclareLaunchArgument(
        "arm_channel",
        default_value="can0",
        description="SocketCAN channel for 6-DOF reBot arm",
    )
    hand_port_arg = DeclareLaunchArgument(
        "hand_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for 16-DOF LEAP hand",
    )
    rate_arg = DeclareLaunchArgument(
        "rate_hz",
        default_value="50.0",
        description="Unified control and telemetry rate (Hz)",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Launch MuJoCo 3D passive viewer window when use_sim:=true",
    )
    record_arg = DeclareLaunchArgument(
        "record",
        default_value="false",
        description="Whether to launch tuinadex_recorder_node",
    )
    use_scene_world_arg = DeclareLaunchArgument(
        "use_scene_world",
        default_value="true",
        description="Load full Tuina scene with bed and patient dummy in simulation",
    )
    timeout_arg = DeclareLaunchArgument(
        "watchdog_timeout_sec",
        default_value="0.30",
        description="Watchdog communication loss timeout (s)",
    )
    hold_arg = DeclareLaunchArgument(
        "watchdog_hold_sec",
        default_value="0.30",
        description="Watchdog hold duration before ramp (s)",
    )
    ramp_arg = DeclareLaunchArgument(
        "watchdog_ramp_sec",
        default_value="0.60",
        description="Watchdog ramp transition to home duration (s)",
    )

    # 启动条件表达式
    is_real = PythonExpression(["'", LaunchConfiguration("use_sim"), "' == 'false'"])
    is_sim = PythonExpression(["'", LaunchConfiguration("use_sim"), "' == 'true'"])

    launch_hand_real = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("use_sim"), "' == 'false' and '",
            LaunchConfiguration("mode"), "' in ('arm_hand', 'hand_only')",
        ])
    )
    launch_arm_real = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("use_sim"), "' == 'false' and '",
            LaunchConfiguration("mode"), "' in ('arm_hand', 'arm_only')",
        ])
    )
    launch_hand_sim = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("use_sim"), "' == 'true' and '",
            LaunchConfiguration("mode"), "' in ('arm_hand', 'hand_only')",
        ])
    )
    launch_arm_sim = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("use_sim"), "' == 'true' and '",
            LaunchConfiguration("mode"), "' in ('arm_hand', 'arm_only')",
        ])
    )

    # 节点定义
    hand_real_node = Node(
        package="tuinadex_hand",
        executable="leap_hand_node",
        name="leap_hand_node",
        output="screen",
        parameters=[
            default_hand_config,
            {
                "port": LaunchConfiguration("hand_port"),
                "mock_mode": LaunchConfiguration("mock"),
                "control_rate_hz": LaunchConfiguration("rate_hz"),
                "watchdog_timeout_sec": LaunchConfiguration("watchdog_timeout_sec"),
                "watchdog_hold_sec": LaunchConfiguration("watchdog_hold_sec"),
                "watchdog_ramp_sec": LaunchConfiguration("watchdog_ramp_sec"),
            },
        ],
        condition=launch_hand_real,
    )

    arm_real_node = Node(
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
                "watchdog_timeout_sec": LaunchConfiguration("watchdog_timeout_sec"),
                "watchdog_hold_sec": LaunchConfiguration("watchdog_hold_sec"),
                "watchdog_ramp_sec": LaunchConfiguration("watchdog_ramp_sec"),
            },
        ],
        condition=launch_arm_real,
    )

    hand_sim_node = Node(
        package="tuinadex_sim",
        executable="leap_hand_sim_node",
        name="leap_hand_sim_node",
        output="screen",
        parameters=[
            default_sim_config,
            {
                "publish_rate_hz": LaunchConfiguration("rate_hz"),
                "use_scene_world": LaunchConfiguration("use_scene_world"),
                "watchdog_timeout_sec": LaunchConfiguration("watchdog_timeout_sec"),
                "watchdog_hold_sec": LaunchConfiguration("watchdog_hold_sec"),
                "watchdog_ramp_sec": LaunchConfiguration("watchdog_ramp_sec"),
                "gui": LaunchConfiguration("gui"),
            },
        ],
        condition=launch_hand_sim,
    )

    arm_sim_node = Node(
        package="tuinadex_arm",
        executable="arm_node",
        name="arm_node",
        output="screen",
        parameters=[
            default_arm_config,
            {
                "channel": LaunchConfiguration("arm_channel"),
                "mock_mode": True,  # 仿真模式下臂节点以 Mock 模式协同接入
                "control_rate_hz": LaunchConfiguration("rate_hz"),
                "watchdog_timeout_sec": LaunchConfiguration("watchdog_timeout_sec"),
                "watchdog_hold_sec": LaunchConfiguration("watchdog_hold_sec"),
                "watchdog_ramp_sec": LaunchConfiguration("watchdog_ramp_sec"),
            },
        ],
        condition=launch_arm_sim,
    )

    recorder_node = Node(
        package="tuinadex_recorder",
        executable="tuinadex_recorder_node",
        name="tuinadex_recorder_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("record")),
    )

    return LaunchDescription([
        mode_arg,
        use_sim_arg,
        mock_arg,
        arm_channel_arg,
        hand_port_arg,
        rate_arg,
        gui_arg,
        record_arg,
        use_scene_world_arg,
        timeout_arg,
        hold_arg,
        ramp_arg,
        hand_real_node,
        arm_real_node,
        hand_sim_node,
        arm_sim_node,
        recorder_node,
    ])

