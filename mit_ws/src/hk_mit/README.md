# LiDAR 정지 장애물 회피

Jump Distance 군집화 → 측정 시각 TF로 `map` 변환 → Frenet 후보 생성 →
차체·곡률·설정 도로 폭 검사 → 경로 및 속도 제한 발행으로 연결되어 있습니다.
첫 구현의 범위는 전진 주행 중 정지 장애물 회피입니다.

## 실행

```bash
cd /mit_ws
colcon build --packages-select hk_mit
source /mit_ws/install/setup.bash
ros2 launch hk_mit bringup.launch.py target_speed:=5.0 rviz:=true
```

인지와 회피는 기본으로 켜집니다. 시뮬레이터에서 주행을 시작해야 START 이벤트를
받아 계획·제어가 활성화됩니다. 시작·복귀에는 이전 관측과 선택 경로를 초기화합니다.
기존에 실행 중인 bringup은 먼저 종료하여 제어 노드가 중복되지 않게 합니다.

CSV는 기본으로 기록하지 않습니다. 기록할 때는 `record_lidar_csv:=true`를
추가합니다. `lidar_csv_path`를 생략하면 `~/hk_mit_data/lidar_<UTC시각>.csv`를
만들며, 기존 파일은 덮어쓰지 않습니다. CSV를 꺼도 인지는 동작합니다.

기준 경로 추종만 비교하려면 `avoidance:=false perception:=false`로 실행합니다.
회피가 켜진 상태에서 인지를 끄면 센서 부재로 정지합니다.

## 파일과 인터페이스

| 파일 | 역할 |
|---|---|
| `hk_mit/lidar_perception.py` | 유효성 검사, 빔 순서 기반 군집화, TF 변환, 관측·마커 발행, 선택적 CSV |
| `hk_mit/avoidance.py` | ROS 독립 계산: 기준 경로 투영, quintic d(s), 차체 충돌·도로 폭·곡률 검사, 후보 평가 |
| `hk_mit/planner.py` | 관측 유효기간, 이벤트 초기화, 경로·속도 제한·디버그 발행 |
| `hk_mit/control.py` | 계획 속도 제한 반영, 빈 경로·0 속도·입력 중단 시 제동 |
| `config/bringup.yaml` | 도로 폭, 차체 크기, 여유 거리, 후보·속도·시간 제한 조정 |

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/hk/perception/obstacles` | `sensor_msgs/PointCloud2` | `map` 관측점: float32 x/y/z, uint32 cluster_id. stamp는 원래 스캔 시각 |
| `/hk/perception/markers` | `visualization_msgs/MarkerArray` | 군집 윤곽, 군집 번호와 측정점 수 |
| `/hk/plan/local_path` | `nav_msgs/Path` | 재계산한 경로 방향을 포함한 `map` 지역 경로 |
| `/hk/plan/speed_limit` | `std_msgs/Float32` | m/s. 0이면 정지 요청. 제어기는 target_speed와 이 값 중 작은 값 적용 |
| `/hk/plan/status` | `std_msgs/String` | JSON 상태, 선택 offset, 속도 제한, 후보별 결과 개수 |
| `/hk/plan/avoidance_markers` | `visualization_msgs/MarkerArray` | 기준 경로·가정한 경계, 후보, 탈락 이유, 선택 경로 차체 |

RViz는 녹색 선택 경로, 청록색 유효 후보, 빨간색 탈락 후보를 표시합니다.
`collision`, `road_boundary`, `curvature` 같은 문구가 탈락 사유입니다.
군집 번호는 스캔마다 새로 정해지며 추적용 객체 ID가 아닙니다.

```bash
ros2 topic echo /hk/plan/status
ros2 topic echo /hk/plan/speed_limit
```

## 계산과 실패 처리

- 라이다의 원래 빔 순서와 결측 구간을 유지하고, 거리·빔 간격에 따라 군집 분리
  거리를 조정합니다. 360도 스캔의 끝과 처음도 인접할 때만 병합합니다.
- 벽과 단일 점도 충돌 검사에 남깁니다. 가까운 물체는 관측점의 convex hull,
  8 m를 넘는 긴 벽은 관측 윤곽 선분으로 검사하여 굽은 벽의 안쪽을 통째로 막지 않습니다.
- `+Inf`는 미검출, 이 시뮬레이터의 `-Inf`는 너무 가까운 반환이므로 최소 거리의
  위험점으로 취급합니다. NaN·범위 밖 값이 너무 많으면 관측을 발행하지 않습니다.
- 측정 시각의 TF가 없으면 잠깐 재시도하며, 최신 TF를 대신 쓰거나 라이다 좌표를
  `map`으로 이름만 바꾸어 발행하지 않습니다. 최대 10개 스캔을 대기열에 두어
  다음 스캔이 와도 변환을 기다리는 이전 스캔을 덮어쓰지 않습니다. 만료된 스캔은 버립니다.
  빔별 motion deskew는 구현하지 않았습니다.
- 후보는 현재 위치와 진행방향에 연결된 quintic d(s)로 시작해 옆으로 이동하고,
  관측된 장애물을 지난 뒤 기준 경로로 돌아옵니다. 이전 경로를 이어가는 후보도
  재검사하여 매번 회피 완료 지점을 앞으로 미루는 문제를 줄입니다.
  이전 경로가 여전히 유효하면 우선 유지하고, 기준 경로에 충돌이 없으면 불필요한
  좌우 후보 평가를 생략합니다. 기존 경로가 막혔을 때 대안 후보들을 평가합니다.
- 뒤차축 기준 4.635 × 1.892 m 차체, 뒤 오버행 0.79 m를 사용합니다.
  최대 0.25 m 간격으로 검사하고 장애물 선분과 차체의 교차·포함 관계도 검사합니다.
  표면 불확실성과 샘플 사이 이동·회전 여유를 추가합니다.
- 조향 한계에서 허용 곡률을 계산합니다. 속도 상한은 곡률에 따른 횡가속도 제한과
  `avoidance_speed`를 반영합니다. 비용은 경로 이탈·곡률·관측물 여유 거리·이전 경로 변화입니다.
- 신선한 **빈 cloud**는 장애물 없음, **cloud 미수신/오래됨**은 센서 부재입니다.
  후자나 유효 후보 없음은 빈 Path와 속도 0을 발행합니다. 발행만 중단하지 않습니다.
- 관측·위치는 기본 0.35초, 제어의 계획 입력은 0.5초 제한입니다. 제어는 빈 경로와
  속도 0에서 양의 PI 적분값이 남아 있어도 가속하지 않고 제동합니다.
  노드별 `/clock` 도착 순서에 따른 작은 차이는 `clock_tolerance`(0.05초)까지 허용합니다.
  TF와 센서 수신은 최신 메시지를 우선하며, 인지 노드는 TF 수신과 스캔 처리를 분리한
  두 개의 실행 스레드를 사용합니다. 스캔·이벤트·CSV 처리는 같은 callback group에서 직렬 처리합니다.
- 라이다와 계획은 10 Hz, 제어는 50 Hz입니다. 노드 내부 파라미터는 시작할 때 읽습니다.

## 적용 범위와 튜닝 전제

공개 `course.json`에는 실제 도로 경계가 없습니다. `road_half_width: 3.75`는
기준 경로 좌우에 둔 **초기 가정**이며 검증된 도로 폭이 아닙니다. 이 설정과
라이다 관측을 기준으로 후보를 검사하므로 실제 도로와 맞춰 조정해야 합니다.
`max_obstacle_range`는 `ahead_m + 차체 앞 길이 + 여유 거리`보다 크게 유지합니다.

LiDAR는 보이는 면만 관측합니다. `surface_padding`은 그 불확실성에 대한
고정 여유이며, 가려진 물체의 실제 전체 부피를 복원하거나 가려진 공간을
관측했다고 보장하지 않습니다. 현재는 정지 장애물용이며 NPC 속도 추정·예측,
양보 판단, 후진 주차, 여러 장애물 사이의 복잡한 좌우 교대 회피는 별도 확장이 필요합니다.
목표 속도 프로파일·제동거리 최적화와 노면·구동 지연을 포함한 동역학 계획도 범위 밖입니다.
위치 추정은 기존 코드 그대로이므로 GPS 음영 구간의 위치 갱신 한계가 남아 있습니다.

## 주차 (평행·후면, `hk_mit/parking.py`)

전역 경로를 고쳐서 주차합니다. 켠 주차 미션(`/data/current/course.json` options)의 존에 다가가며
칸 쪽 벽 선의 입구(평행: 8 × 4 m 홈, 후면: 폭 5 m 통로 → 19 × 5 m 방)를 LiDAR 누적점으로 찾아 seed slide를 구하고
(입구 먼 쪽 끝 - 입구 길이/2, 동작 시작 4 m 전까지 계속 다시 찾음), 후보 동작(원호 반지름 ≥ 1.05 R_min)을
칸 좌표계 자유 공간 모형·관측점에 대해 차체 검사한 뒤 경로에 끼웁니다.

- 평행: 칸 5 m 앞에서 정지 → 후진 S자 → 3.2 s 유지 → (실제 선 자세에서 다시 만든) 전진 S자로 복귀
- 후면: 왼쪽 90°로 통로 → 방에서 오른쪽 90° → 정지 → 직진 후진(방 북쪽 벽을 스캔에서 재서 정지 지점 보정) → 유지 → 전진 복귀
- 경로 점마다 기어(+1/-1), 구간 끝 = 정지 지점. 지금 구간만 발행하고 가장 가까운 점 찾기도 구간 끝을 넘지 않음
- `/hk/plan/gear` (`std_msgs/Int8`): 0 일반 주행, 부호 = 방향(+ 전진, - 후진), 크기 2 = 발행한 경로의 끝이 정지 지점
- 제어: 기어는 0.05 m/s 아래에서만 바꾸고 그동안 제동하며 조향을 미리 맞춤. 후진은 뒤차축 되먹임(차 방향 + π),
  주차 전진은 Stanley + 곡률 피드포워드, 정지 지점까지 √(2 a 남은 거리)로 감속
- 성공 판정(규정·결과 JSON `missions`: pending → parked → done): 차체 네 꼭짓점 칸 안(0.05 m), 방향 ±20°, 0.05 m/s 이하 3 s

## 검증

```bash
cd /mit_ws/src/hk_mit
env ROS_DOMAIN_ID=187 python3 -m unittest discover -s test -v
```

합성 장면에서 단일 장애물 회피, 완전 차단 시 정지, 재계획 후 복귀, 회전한
차체와 벽 충돌, 곡선 경로, 군집 경계·무효값·360도 병합, TF 변환, 관측 유효기간,
CSV 독립 동작과 제동을 검사합니다. 이 검증은 실제 시뮬레이터 seed별 완주 성능과 구분됩니다.

## 설계 참고

알고리즘 구조를 참고하고 이 패키지 인터페이스에 맞춰 독립 구현했습니다.

- [laser_segmentation Humble Jump Distance](https://github.com/ajtudela/laser_segmentation/blob/humble/src/segmentation/jump_distance.cpp)
- [PythonRobotics Frenet Optimal Trajectory](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/FrenetOptimalTrajectory/frenet_optimal_trajectory.py)
