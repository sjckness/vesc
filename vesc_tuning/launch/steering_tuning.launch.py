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
        DeclareLaunchArgument('steering_angle', default_value='0.26'),
        DeclareLaunchArgument('target_speed', default_value='0.5'),
        DeclareLaunchArgument('direction', default_value='left'),
        DeclareLaunchArgument('steering_to_servo_gain', default_value='-1.2135'),
        DeclareLaunchArgument('steering_to_servo_offset', default_value='0.5304'),
        DeclareLaunchArgument('wheelbase', default_value='0.25'),
        DeclareLaunchArgument('vesc_yaml_path', default_value=default_vesc_yaml),
        DeclareLaunchArgument('command_topic', default_value='/commands/motor/speed'),
        DeclareLaunchArgument('servo_topic', default_value='/commands/servo/position'),
        DeclareLaunchArgument('drive_duration_safety_max', default_value='15.0'),
        DeclareLaunchArgument('speed_to_erpm_gain', default_value='4614.0'),
        DeclareLaunchArgument('speed_to_erpm_offset', default_value='0.0'),
    ]

    warning = LogInfo(msg='TUNING MODE ACTIVE — ensure the car has at least 2 m of '
                          'clear space ahead.')

    # NOTE: interactive node — see speed_tuning.launch.py note about stdin and
    # `ros2 launch`. Prefer `ros2 run vesc_tuning steering_tuning_node` for the
    # interactive session.
    node = Node(
        package='vesc_tuning',
        executable='steering_tuning_node',
        name='steering_tuning_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'steering_angle': LaunchConfiguration('steering_angle'),
            'target_speed': LaunchConfiguration('target_speed'),
            'direction': LaunchConfiguration('direction'),
            'steering_to_servo_gain': LaunchConfiguration('steering_to_servo_gain'),
            'steering_to_servo_offset': LaunchConfiguration('steering_to_servo_offset'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'vesc_yaml_path': LaunchConfiguration('vesc_yaml_path'),
            'command_topic': LaunchConfiguration('command_topic'),
            'servo_topic': LaunchConfiguration('servo_topic'),
            'drive_duration_safety_max': LaunchConfiguration('drive_duration_safety_max'),
            'speed_to_erpm_gain': LaunchConfiguration('speed_to_erpm_gain'),
            'speed_to_erpm_offset': LaunchConfiguration('speed_to_erpm_offset'),
        }],
    )

    return LaunchDescription(args + [warning, node])
