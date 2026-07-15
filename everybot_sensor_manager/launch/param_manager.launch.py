from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='everybot_param_setter',
            package='everybot_sensor_manager',
            executable='param_setter',
            output='screen',
        )
    ])
