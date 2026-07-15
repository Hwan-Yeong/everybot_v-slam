import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('udp_interface'),
        'config',
        'udp_interface_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='udp_interface',
            executable='udp_communication',
            name='udp_interface',
            output='screen',
            parameters=[param_file_path],
            respawn=True,
            respawn_delay=1.0
        ),
    ])