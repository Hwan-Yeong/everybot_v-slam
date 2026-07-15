# everybot_slam

RTAB-Map 기반 hybrid 3D SLAM (2D LiDAR + 전방 RGB-D camera).
기존 slam_toolbox(2D) / AMCL+map_server 를 대체한다.

## 아키텍처

```
[Jetson]
  M4.51s driver ──► rgbd_sync ──► /rgbd_image ──┐  (GigE, ROS2 DDS, best_effort)
                                                │
[RK3588]                                        ▼
  MCU(uart) ──► /odom + odom→base_link TF ──► rtabmap ◄── /scan (전/후방 merge)
                                                │
                     ┌──────────────────────────┼──────────────────┐
                     ▼                          ▼                  ▼
               map→odom TF                /map (2D grid)     cloud_map (3D)
                     │                          │
                     ▼                          ▼
                   Nav2          global costmap static layer
  (local costmap STVL ◄── /sensor_to_pointcloud/depth_camera/local)
```

역할 분담:
- **실시간 pose**: MCU odom(IMU yaw 융합) — visual odometry 미사용 (전방 단일 카메라
  FOV 로는 회전 시 tracking loss 필연 + RK3588 CPU 절약)
- **drift 보정 / loop closure**: rtabmap — visual(BoW) 후보 탐색 후 LiDAR ICP 정밀 보정
- **3D 장애물 회피**: sensor_manager 의 depth cloud → STVL (SLAM 과 독립 경로)

## 사전 조건

1. **패키지 설치** (RK3588, Jetson 양쪽):
   ```bash
   sudo apt install ros-humble-rtabmap-ros
   ```
2. **시간 동기화 (필수)** — depth camera driver stamp 가 ROS time 과 어긋나면
   rgbd/odom/scan 동기화가 깨진다. RK3588 을 chrony 서버로, Jetson 을 클라이언트로:
   ```bash
   # RK3588 (/etc/chrony/chrony.conf): allow <jetson subnet>, local stratum 10
   # Jetson  (/etc/chrony/chrony.conf): server <rk3588 ip> iburst minpoll 0 maxpoll 4
   chronyc tracking   # offset 수 ms 이내 확인
   ```
3. **카메라 TF**: Jetson 측 `m4_51s.urdf.xacro` 가 발행
   (`base_link → camera_link → camera_m_4_51s → inusensor → inusensor_depth/rgb`).
   RK3588 의 everybot.urdf 와는 `base_link` 를 공통 루트로 자연 병합됨.
   - depth frame_id = `inusensor_depth`, rgb frame = `inusensor_rgb`
   - extrinsics 는 xacro 인자 (camera_x=0.177, camera_z=0.26, camera_yaw=-0.05)
   - 주의: Jetson 이 죽으면 카메라 TF 도 사라짐 → rtabmap/sensor_manager 의
     TF lookup 경고 발생 (SLAM 은 LiDAR+odom 으로 degrade 동작)
4. **RGB 토픽**: `/camera/rgb/image_raw`, `/camera/rgb/camera_info` (기본값 반영됨).
   depth 와 RGB 는 정렬(aligned)된 스트림이어야 함 — 미정렬이면 driver 의
   aligned depth 출력 사용.

## 실행 순서

```bash
# [Jetson] depth camera driver 만 실행 (rgbd_sync 는 AMR 에서 실행됨)

# [RK3588] 플랫폼 bringup (odom/TF/scan/sensor_manager)
ros2 launch everybot_bringup bringup.launch.py

# [RK3588] 맵핑 (새 맵: -d 로 DB 초기화) — rgbd_sync 포함 실행됨
ros2 launch everybot_slam slam.launch.py rtabmap_args:="-d"
#   → 조이스틱/수동 주행으로 맵 작성. /map 과 cloud_map 을 rviz2 로 확인
#   → 시작점 부근 재방문(loop closure) 후 종료하면 DB 에 자동 저장

# [RK3588] 운영 (기존 localization_launch.py 대신)
ros2 launch everybot_slam slam.launch.py localization:=true
ros2 launch everybot_navigation navigation_launch.py   # Nav2
```

기존 `everybot_navigation/launch/localization_launch.py`(AMCL+map_server)는 사용하지
않는다. rtabmap 이 `map→odom` TF 와 `/map`(latched) 을 직접 발행하고,
`localization_pose` 는 `/amcl_pose` 로 리맵되어 MCU(uart_communication) 호환 유지.

## 검증 체크리스트

```bash
ros2 topic hz /rgbd_image          # ~15Hz
ros2 topic hz /scan                # ~10Hz
ros2 run tf2_tools view_frames     # map→odom→base_link→inusensor_depth/inusensor_rgb
ros2 topic echo /rtabmap/info --once   # loop closure 발생 여부 (Loop closure id != 0)
```

- 맵핑 중 rtabmap 로그에 `Rejected loop closure` 다발 → 정상 (보수적 검증)
- `Could not get transform ... inusensor_*` → driver frame_id 와 URDF 링크명 불일치
- rgbd sync 경고 (`Did not receive data since 5 seconds`) → chrony offset 확인

## 휠체어(depth cam 3개) 확장

1. Jetson 측: 카메라별 `rgbd_sync` 3개 실행 → `/rgbd_image0..2`
2. `rtabmap_params.yaml`: `rgbd_cameras: 3`
3. `slam.launch.py` remappings 에 `rgbd_image0..2` 추가
4. URDF 에 카메라 링크 3개 (RTAB-Map 은 multi-camera 를 같은 그래프에서 처리)

## 튜닝 포인트

| 증상 | 파라미터 |
|---|---|
| RK3588 CPU 과부하 | `Rtabmap/DetectionRate` ↓ (0.5), `Mem/ImagePreDecimation: "2"` |
| loop closure 안 잡힘 | `Vis/MinInliers` ↓ (10), 조명/텍스처 확인 |
| 잘못된 loop closure | `Vis/MinInliers` ↑ (20), `RGBD/OptimizeMaxError` ↓ |
| 맵 품질 (grid) | `Grid/RangeMax`, `Grid/CellSize` |
| 네트워크 부하 | rgbd_image/compressed + rtabmap_util republish 사용 검토 |
