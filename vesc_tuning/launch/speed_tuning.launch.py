import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_vesc_yaml = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'vesc.yaml')

    args = [
        DeclareLaunchArgument('target_distance', default_value='1.0'),
        DeclareLaunchArgument('target_speed', default_value='0.5'),
        DeclareLaunchArgument('reverse', default_value='false'),
        DeclareLaunchArgument('speed_to_erpm_gain', default_value='4614.0'),
        DeclareLaunchArgument('speed_to_erpm_offset', default_value='0.0'),
        DeclareLaunchArgument('vesc_yaml_path', default_value=default_vesc_yaml),
        DeclareLaunchArgument('command_topic', default_value='/commands/motor/speed'),
        DeclareLaunchArgument('servo_topic', default_value='/commands/servo/position'),
        DeclareLaunchArgument('servo_center', default_value='0.5304'),
        DeclareLaunchArgument('drive_duration_safety_max', default_value='10.0'),
    ]

    warning = LogInfo(msg='TUNING MODE ACTIVE — ensure the car has at least 2 m of '
                          'clear space ahead.')

    # NOTE: this node is interactive (reads the keyboard). stdin is not reliably
    # forwarded to nodes started via `ros2 launch`; for the interactive session
    # prefer `ros2 run vesc_tuning speed_tuning_node --ros-args -p ...`.
    node = Node(
        package='vesc_tuning',
        executable='speed_tuning_node',
        name='speed_tuning_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'target_distance': LaunchConfiguration('target_distance'),
            'target_speed': LaunchConfiguration('target_speed'),
            'reverse': LaunchConfiguration('reverse'),
            'speed_to_erpm_gain': LaunchConfiguration('speed_to_erpm_gain'),
            'speed_to_erpm_offset': LaunchConfiguration('speed_to_erpm_offset'),
            'vesc_yaml_path': LaunchConfiguration('vesc_yaml_path'),
            'command_topic': LaunchConfiguration('command_topic'),
            'servo_topic': LaunchConfiguration('servo_topic'),
            'servo_center': LaunchConfiguration('servo_center'),
            'drive_duration_safety_max': LaunchConfiguration('drive_duration_safety_max'),
        }],
    )

    return LaunchDescription(args + [warning, node])
