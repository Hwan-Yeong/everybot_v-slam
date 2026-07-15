# vslam_ws

RK3588 기반 자율주행 AMR SW (3D visual SLAM + Nav2).
SLAM 상세 아키텍처/실행 방법은 `everybot_slam/README.md` 참고.

## 워크스페이스 셋업

이 레포에는 자사 패키지만 커밋한다. **rtabmap / rtabmap_ros 는 외부 오픈소스라
커밋하지 않고** (.gitignore 처리) 아래처럼 각자 받아서 빌드한다.

```bash
cd ~/vslam_ws/src   # 이 레포를 clone 한 위치

# 방법 A: vcstool (권장 - 버전이 deps.repos 파일로 관리됨)
sudo apt install python3-vcstool
vcs import < deps.repos

# 방법 B: 직접 clone
git clone https://github.com/introlab/rtabmap.git rtabmap
git clone --branch humble-devel https://github.com/introlab/rtabmap_ros.git rtabmap_ros
```

의존성 설치 및 빌드:

```bash
cd ~/vslam_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

export MAKEFLAGS="-j6"   # RK3588 에서는 -j2~4 권장 (메모리 부족 방지)
colcon build --symlink-install \
  --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release
```

- `-DRTABMAP_SYNC_MULTI_RGBD=ON`: depth camera 2대 이상(휠체어 확장) 필수 옵션

## 외부 소스 버전 관리 규칙

`deps.repos` 의 `version` 이 **개발/검증 완료 시점의 버전 기록**이다.
개발자는 항상 `vcs import < deps.repos` 로 받으면 검증된 버전을 그대로 얻는다.

버전 검증/고정 워크플로우:

```bash
# 1) (업데이트 필요 시) deps.repos 의 version 을 브랜치로 바꾸고 최신 pull
cd src && vcs import < deps.repos && vcs pull rtabmap rtabmap_ros

# 2) 빌드 + 실기 검증

# 3) 검증 통과한 정확한 commit hash 추출
vcs export --exact rtabmap rtabmap_ros
#   -> 출력의 version(hash) 을 deps.repos 에 반영하고 커밋
```

## 패키지 구성

| 패키지 | 역할 |
|---|---|
| everybot_bringup | MCU(UART) 통신, odom/IMU/TF, URDF |
| everybot_lidar | 전/후방 2D LiDAR driver + scan merge (/scan) |
| everybot_sensor_manager | depth/ToF/IR -> PointCloud 변환 (STVL 입력) |
| everybot_slam | RTAB-Map 기반 hybrid 3D SLAM (mapping/localization) |
| everybot_navigation | Nav2 (planner/controller/costmap/BT) |
| everybot_custom_msgs | 내부 커스텀 메시지 |
| everybot_log | 로깅 |
| udp_interface | 앱/외부 통신 (UDP) |
| serial | 시리얼 라이브러리 (외부 vendored) |
| rtabmap, rtabmap_ros | (커밋 안 함) 위 셋업 절차로 clone |

## 멀티 보드 구성 (Jetson)

Jetson 은 depth camera driver 만 실행한다 (rgb/depth 동기화는 AMR 의
slam.launch.py 가 수행). ROS_DOMAIN_ID 일치 + chrony 시간 동기화 필수
(everybot_slam/README.md 참고).
