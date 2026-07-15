
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('everybot_lidar'),
        'config',
        'everybot_lidar_params.yaml'
    )
    return LaunchDescription([
        
        launch_ros.actions.Node(
            package='everybot_lidar',
            executable='everybot_lidar_merger',
            parameters=[param_file],
            output='screen',
            respawn=True,
            respawn_delay=1.0,
        ),

        launch_ros.actions.Node(
            name='everybot_pointcloud_to_laserscan',
            package='everybot_lidar',
            executable='pointcloud_to_laserscan_node',
            parameters=[param_file],
            respawn=True,
            respawn_delay=1.0
        )
    ])
