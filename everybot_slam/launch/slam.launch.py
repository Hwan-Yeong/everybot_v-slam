"""RTAB-Map 기반 hybrid 3D SLAM (RK3588 측).

사용법:
  # 맵핑 (새 맵 시작: rtabmap_args:="-d" 로 기존 DB 삭제)
  ros2 launch everybot_slam slam.launch.py rtabmap_args:="-d"

  # 맵핑 (기존 DB 이어서)
  ros2 launch everybot_slam slam.launch.py

  # 운영(localization) 모드 - 기존 AMCL+map_server(localization_launch.py) 대체
  ros2 launch everybot_slam slam.launch.py localization:=true

전제:
  - everybot_bringup 실행 중 (odom->base_link TF, /scan)
  - Jetson 에서 jetson_rgbd_sync.launch.py 실행 중 (/rgbd_image)
  - Jetson-RK3588 시간 동기화 (chrony) 완료
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('everybot_slam')
    params_file = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')

    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    rgbd_topic = LaunchConfiguration('rgbd_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    remappings = [
        ('rgbd_image', rgbd_topic),
        ('scan', scan_topic),
        # Nav2 static layer 및 기존 코드가 /map 을 구독
        ('grid_map', '/map'),
        # uart_communication(MCU) 가 /amcl_pose 를 구독하므로 호환 유지
        ('localization_pose', '/amcl_pose'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='true: localization 모드 (맵 고정), false: mapping 모드'),
        DeclareLaunchArgument(
            'database_path', default_value='/home/airbot/app_rw/map/rtabmap.db',
            description='RTAB-Map DB 경로 (맵 저장/로드)'),
        DeclareLaunchArgument(
            'rtabmap_args', default_value='',
            description='rtabmap 노드 추가 인자. 새 맵 시작 시 "-d" (DB 삭제)'),
        DeclareLaunchArgument(
            'rgbd_topic', default_value='/rgbd_image',
            description='Jetson rgbd_sync 출력 토픽'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='전/후방 merge 된 2D LiDAR scan'),

        # ---------------- Mapping mode ----------------
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen',
            parameters=[
                params_file,
                {
                    'database_path': database_path,
                    'Mem/IncrementalMemory': 'true',
                    'Mem/InitWMWithAllNodes': 'false',
                },
            ],
            remappings=remappings,
            arguments=[rtabmap_args],
        ),

        # ---------------- Localization mode ----------------
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen',
            parameters=[
                params_file,
                {
                    'database_path': database_path,
                    # 맵 고정 (신규 노드를 맵에 추가하지 않음)
                    'Mem/IncrementalMemory': 'false',
                    # 시작 시 전체 맵 노드를 WM 에 로드 (전역 재위치 인식)
                    'Mem/InitWMWithAllNodes': 'true',
                },
            ],
            remappings=remappings,
        ),
    ])
