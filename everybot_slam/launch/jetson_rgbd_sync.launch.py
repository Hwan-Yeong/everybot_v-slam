"""Jetson 측: RGB + Depth + CameraInfo -> 단일 RGBDImage 토픽 동기화.

이 launch 는 **Jetson 보드에서** 실행한다 (depth camera driver 와 같은 보드).
rgb/depth 를 같은 클럭에서 짝지어 하나의 토픽으로 묶은 뒤 네트워크로 전송해야
RK3588 쪽에서 클럭 차이/지터로 인한 sync 실패가 없다.

사용법 (토픽명은 M4.51s 드라이버에 맞게 인자로 조정):
  ros2 launch everybot_slam jetson_rgbd_sync.launch.py \
      rgb_topic:=/camera/color/image_raw \
      depth_topic:=/camera/depth/image_raw \
      camera_info_topic:=/camera/color/camera_info

전제:
  - ROS_DOMAIN_ID 일치 (기존 depth 전송과 동일)
  - chrony 로 RK3588 과 시간 동기화
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic', default_value='/camera/rgb/image_raw',
            description='RGB 이미지 토픽 (M4.51s driver)'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth/image_raw',
            description='Depth 이미지 토픽 (16UC1, frame_id: inusensor_depth)'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/rgb/camera_info',
            description='CameraInfo 토픽 (RGB 기준)'),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync',
            output='screen',
            parameters=[{
                # 드라이버가 rgb/depth stamp 를 정확히 일치시키지 못하는 경우 근사 동기화
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'sync_queue_size': 30,
                'topic_queue_size': 10,
                'qos': 2,  # sensor best_effort
                # depth 가 rgb 와 정렬(aligned)되어 있지 않다면 드라이버에서 정렬 출력 사용 필요
                'depth_scale': 1.0,
            }],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                # 출력: /rgbd_image (RK3588 의 slam.launch.py 가 구독)
            ],
        ),
    ])
