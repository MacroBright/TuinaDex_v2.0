"""Launch file for TuinaDex 22-DOF Composite Simulation Teleoperation Node."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tuinadex_teleop')

    default_model = os.path.join(pkg_share, 'configs', 'mjcf', 'tuina_sim_scene.xml')
    default_session_cfg = os.path.join(pkg_share, 'configs', 'session_calibration.yaml')
    default_table_cfg = os.path.join(pkg_share, 'configs', 'table_calibration.yaml')
    default_urdf_arm = os.path.join(pkg_share, 'configs', 'urdf', 'rebot_arm_rs.urdf')
    default_urdf_hand = os.path.join(pkg_share, 'configs', 'urdf', 'leap_hand_right.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='Run physics simulation step'),
        DeclareLaunchArgument('mock', default_value='false', description='Use mock camera for testing/CI'),
        DeclareLaunchArgument('mock_imu', default_value='false', description='Use mock IMU'),
        DeclareLaunchArgument('no_imu', default_value='false', description='Disable IMU attitude tracking'),
        DeclareLaunchArgument('no_safety', default_value='false', description='Bypass safety guardrails'),
        DeclareLaunchArgument('auto_engage', default_value='true', description='Auto-engage clutch when hand is seen'),
        DeclareLaunchArgument('publish_rate_hz', default_value='60.0', description='Publishing frequency in Hz'),
        DeclareLaunchArgument('camera_fps', default_value='0', description='Camera FPS (0 = auto: 60 on USB3, 30 on USB2)'),
        DeclareLaunchArgument('axis_mapping', default_value='-y,+x,+z', description='Human wrist to robot pos mapping'),
        DeclareLaunchArgument('imu_axis_mapping', default_value='+x,+y,+z', description='IMU to robot attitude mapping'),
        DeclareLaunchArgument('solver_mode', default_value='hybrid', description='Hand retargeting solver mode'),
        DeclareLaunchArgument('gui', default_value='true', description='Open OpenCV tracking HUD window'),

        Node(
            package='tuinadex_teleop',
            executable='teleop_sim_composite_node',
            name='teleop_sim_composite_node',
            output='screen',
            parameters=[{
                'model_path': default_model,
                'session_config': default_session_cfg,
                'table_config': default_table_cfg,
                'urdf_arm_path': default_urdf_arm,
                'urdf_hand_path': default_urdf_hand,
                'use_sim': LaunchConfiguration('use_sim'),
                'mock': LaunchConfiguration('mock'),
                'mock_imu': LaunchConfiguration('mock_imu'),
                'no_imu': LaunchConfiguration('no_imu'),
                'no_safety': LaunchConfiguration('no_safety'),
                'auto_engage': LaunchConfiguration('auto_engage'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'camera_fps': LaunchConfiguration('camera_fps'),
                'axis_mapping': LaunchConfiguration('axis_mapping'),
                'imu_axis_mapping': LaunchConfiguration('imu_axis_mapping'),
                'solver_mode': LaunchConfiguration('solver_mode'),
                'gui': LaunchConfiguration('gui'),
            }],
        ),
    ])

