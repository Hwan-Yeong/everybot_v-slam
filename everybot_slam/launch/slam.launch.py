"""RTAB-Map 기반 hybrid 3D SLAM (RK3588/AMR 측).

사용법:
  # 맵핑 (새 맵 시작: rtabmap_args:="-d" 로 기존 DB 삭제)
  ros2 launch everybot_slam slam.launch.py rtabmap_args:="-d"

  # 맵핑 (기존 DB 이어서)
  ros2 launch everybot_slam slam.launch.py

  # 운영(localization) 모드 - 기존 AMCL+map_server(localization_launch.py) 대체
  ros2 launch everybot_slam slam.launch.py localization:=true

rgbd_sync (rgb+depth 짝맞춤) 은 기본적으로 이 launch 가 AMR 에서 함께 실행한다.
Jetson 은 driver 만 실행하면 됨. Jetson 쪽에서 sync 를 돌리고 싶으면
(네트워크 토픽 수 절감 목적) jetson_rgbd_sync.launch.py 를 Jetson 에서 실행하고
여기서는 rgbd_sync:=false 로 끈다.

전제:
  - everybot_bringup 실행 중 (odom->base_link TF, /scan)
  - Jetson 에서 depth camera driver 실행 중 (/camera/rgb/*, /camera/depth/*)
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
    rgbd_sync = LaunchConfiguration('rgbd_sync')
    rgbd_topic = LaunchConfiguration('rgbd_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

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
            'rgbd_sync', default_value='true',
            description='true: AMR 에서 rgbd_sync 실행 (기본). '
                        'false: Jetson 의 jetson_rgbd_sync.launch.py 사용'),
        DeclareLaunchArgument(
            'rgbd_topic', default_value='/rgbd_image',
            description='동기화된 RGBD 토픽'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='전/후방 merge 된 2D LiDAR scan'),
        DeclareLaunchArgument(
            'rgb_topic', default_value='/camera/rgb/image_raw',
            description='RGB 이미지 토픽 (M4.51s driver, rgbd_sync=true 시 사용)'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth/image_raw',
            description='Depth 이미지 토픽 (16UC1, rgbd_sync=true 시 사용)'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/rgb/camera_info',
            description='CameraInfo 토픽 (RGB 기준, rgbd_sync=true 시 사용)'),

        # ---------------- RGB-D 동기화 (AMR 측 실행) ----------------
        # Jetson driver 가 발행한 rgb/depth/info 를 stamp 기준으로 짝맞춰
        # 단일 rgbd_image 토픽 생성. stamp 는 driver 가 캡처 시점에 찍으므로
        # 어느 보드에서 실행해도 짝맞춤 결과는 동일 (chrony 동기화 전제).
        Node(
            condition=IfCondition(rgbd_sync),
            package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'sync_queue_size': 30,
                'topic_queue_size': 10,
                'qos': 2,  # sensor best_effort (네트워크 스트림)
                'depth_scale': 1.0,
            }],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('rgbd_image', rgbd_topic),
            ],
        ),

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
