# Depth Camera RANSAC 바닥 제거 필터

`DepthCameraCloudConverter` 의 filter pipeline 에 포함된 바닥(지면) 평면 제거 필터.
구현: `src/cloud_converter/sensors/depth_camera.cpp` 의 `RemoveGroundPlane()`.

## 1. RANSAC 이론 요약

RANSAC(RANdom SAmple Consensus)은 outlier 가 섞인 데이터에서 모델(여기서는 평면)을
강건하게 추정하는 반복 알고리즘이다.

1. 점 3개를 무작위로 뽑아 평면 후보를 만든다.
2. 전체 점 중 평면과의 거리가 `distance_threshold` 이하인 점(inlier)을 센다.
3. 1~2 를 `max_iterations` 회 반복하여 inlier 가 가장 많은 평면을 채택한다.
4. (`setOptimizeCoefficients`) 채택된 inlier 들로 least-squares 재적합하여 정밀화한다.

최소제곱법과 달리 장애물 점(outlier)이 평면 추정을 끌어당기지 않는 것이 핵심 장점.
바닥이 프레임의 지배적 평면인 실내 주행 환경에 잘 맞는다.

필요 반복 횟수 이론값: `N = log(1-p) / log(1-w^3)`
(p: 성공 확률, w: inlier 비율). 예: 바닥이 점의 50%, p=0.99 → N≈35.
바닥이 20%까지 떨어져도 N≈574 이지만, 본 구현은 z축 수직 제약 모델
(`SACMODEL_PERPENDICULAR_PLANE`)을 사용해 탐색 공간이 좁아 100회로 충분하다.

## 2. 본 구현의 구조

파이프라인 순서: **RANSAC 바닥 제거 → height crop**.
(min_z crop 이 바닥 노이즈 분포의 아래쪽만 자른 뒤 fit 하면 평면이 위로 편향되므로
RANSAC 을 먼저 수행)

단순 "가장 큰 평면 제거"가 아니라, 검출된 평면이 **바닥 검증** 3가지를 모두
통과할 때만 제거한다:

| 검증 | 파라미터 | 목적 |
|---|---|---|
| 기울기: 법선이 z축 대비 허용각 이내 | `eps_angle_deg` | 벽/가구 측면 오제거 방지 |
| 높이: 평면을 센서 xy 위치로 연장한 z ≈ 0 | `max_ground_offset` | 박스 윗면 등 저상 수평면 오제거 방지 (바닥은 항상 로봇 밑을 지남) |
| 지지도: inlier 비율 하한 | `min_inlier_ratio` | 노이즈에 우연히 맞은 평면 배제 |

검증 실패 평면은 fit 후보에서 제외하고 다음 평면을 재탐색한다 (최대 3회).
끝내 바닥을 못 찾으면 `fallback_min_z` 기반 단순 z-crop 으로 폴백한다.

제거 방식: inlier index 제거가 아니라 **평면 기준 부호 있는 거리**로 제거.
`signed_dist = a·x + b·y + c·z + d > distance_threshold` 인 점만 유지하므로
평면 근방뿐 아니라 평면 아래쪽(멀티패스 노이즈 등)도 함께 제거된다.

## 3. 파라미터 튜닝 가이드

```yaml
ransac_plane:
  enable: true
  distance_threshold: 0.02
  max_iterations: 100
  eps_angle_deg: 10.0
  max_ground_offset: 0.05
  min_inlier_ratio: 0.05
  fallback_min_z: 0.02
```

### distance_threshold (가장 중요)
평면 위쪽으로 이 두께 이하의 점이 모두 제거된다. 즉 **감지 가능한 최소 장애물
높이의 하한**이다.

- 높이 h 인 장애물은 대략 `h - distance_threshold` 두께의 점만 살아남는다.
  3cm 장애물 + threshold 0.02 → 상단 1cm 만 생존. **3cm 장애물을 감지하려면
  반드시 0.03 미만**이어야 한다.
- 반대로 depth 노이즈(스테레오 기준 2m 에서 ±1~2cm)보다 작으면 바닥 점이
  덜 지워져 오감지가 남는다.
- 권장 범위: **0.015 ~ 0.025**. 바닥 오감지가 남으면 올리고, 낮은 장애물을
  놓치면 내린다.

### eps_angle_deg
바닥 평면이 수평에서 벗어날 수 있는 허용각. 경사로, 카메라 extrinsic 오차,
localization 이 반영 못 하는 소폭의 로봇 pitch 를 흡수한다.

- 너무 작으면(≤3°): 문턱 승월 등 pitch 상황에서 바닥 검출 실패 → 폴백만 동작.
- 너무 크면(≥20°): 완만한 경사 형태의 장애물 더미를 바닥으로 오인할 여지.
- 권장: **8 ~ 12**.

### max_ground_offset
검출 평면을 센서 xy 위치로 연장했을 때의 높이 허용치. 바닥이면 로봇 밑을
지나므로 ≈0 이어야 한다는 성질을 이용한 검증.

- 감지해야 할 최소 장애물 높이(3cm)보다 크고, 바닥으로 봐줄 수 없는 높이
  (예: 10cm 단차)보다 작게. 권장: **0.04 ~ 0.07**.
- 로봇이 문턱 위에 올라가 있는 동안은 실제 바닥이 이 검증에 걸릴 수 있다
  (아래 IMU 항목 참고).

### min_inlier_ratio
바닥으로 인정할 최소 inlier 비율(전체 점 대비).

- 벽에 근접해 바닥이 거의 안 보이는 프레임에서 엉뚱한 평면을 채택하지 않게 한다.
- 너무 높으면(≥0.3) 장애물 많은 장면에서 바닥 검출이 자주 실패한다.
- 권장: **0.03 ~ 0.10**.

### max_iterations
z축 제약 모델이라 탐색이 빨리 수렴한다. 100이면 충분하며, CPU 여유가 없으면
50까지 낮춰도 무방.

### fallback_min_z
바닥 미검출 프레임에서의 안전망(단순 z-crop). `distance_threshold` 와 같은 값
권장. 0 이하로 두면 폴백 없이 원본 유지(=바닥 점이 그대로 나감).

## 4. 응용 / 확장

### 문턱 승월 시 pitch 보정 (TODO, IMU 연동)
2D localization 은 roll/pitch 를 map TF 에 반영하지 않으므로, 문턱 승월 중에는
cloud 전체가 기울어져 들어온다. `eps_angle_deg` 이내는 RANSAC 이 흡수하지만
그 이상은 IMU 가 필요하다. 연동 지점은 `RemoveGroundPlane()` 상단 주석 참고:

- (a) `seg.setAxis()` 의 기준축 (0,0,1) 을 IMU roll/pitch 로 회전시켜 전달
- (b) |pitch| > 임계값 동안 eps_angle 일시 완화, 또는 해당 프레임 장애물 출력 skip
  (camera 모듈의 `object_ignore_pitch_th_deg` 와 동일한 접근)

### 성능 최적화 여지 (현재 불필요)
현재 입력은 다운샘플 후 최대 ~7천 점 수준이라 1~2ms 이내로 동작한다.
포인트가 늘어나면: fit 은 서브샘플로 하고 제거만 전체에 적용, 또는 이전 프레임
평면 계수를 초기 검증에 재사용(temporal coherence)하는 방식이 있다.

### 다중 평면 환경
시야 대부분을 박스 윗면이 차지하는 경우 최대 평면이 바닥이 아닐 수 있다.
본 구현은 검증 실패 평면을 제외하고 최대 3회 재탐색하므로 이런 장면에서도
바닥을 찾는다.
