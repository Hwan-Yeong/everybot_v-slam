from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='everybot_log',
            executable='everybot_logging',
            name='everybot_logging',
            output='screen',
            respawn=True,
            respawn_delay=1.0
        ),
    ])