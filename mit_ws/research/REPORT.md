# HEVEN 2026 `hk_mit` 공개 자료 조사 보고서

작성일 2026-11-14 (조사 에이전트). 공개 알고리즘을 요약하고 우리 코드에 맞춰 새로 쓴 짧은 스케치만 담았습니다. 외부 코드는 복사하지 않았습니다.

같은 폴더의 스케치 (표준 라이브러리만 사용):

| 파일 | 내용 |
|---|---|
| `parking_geometry.py` | 평행·후진 주차 기하, 차체 외곽선 충돌 검사, `--course`로 course.json 실제 배치 계산, 작은 hybrid A* 대체 탐색 |
| `roundabout_gap.py` | 링 위 각도 칼만 추적, 합류 판단, IDM 추종 (자체 시험 추정 속도 4.28 m/s, 참값 4.0) |
| `path_smoothing.py` | 경로 곡률 제한 평활화 (route.csv 최대 곡률 0.39 → 0.208, 최대 이탈 0.66 m) |

## 0. `course.json`에서 확인한 사실

- **평행주차 `m_ppark`** (cp8→cp9)
  - 도로는 x=−13.1647을 따라 남쪽으로 진행하고, 칸은 진행 방향 오른쪽(서쪽)에 있습니다.
  - 칸 중심 (−18.1147, 120.7754), yaw −90°(진행 방향과 같음), 7.4×3.6 m. seed에 따라 y 방향 ±3 m 이동.
  - 칸 안쪽 가장자리는 도로 중심선에서 3.15 m로, 통로 가장자리(2.196 m)와 0.95 m 떨어져 있습니다.
  - 미션 존 x∈[−20.66, −15.66], y∈[113.28, 128.28].
- **후진주차 `m_rpark`** (cp9→cp10)
  - 칸 중심 (0.3353, 82.9754), 같은 남행 도로에서 동쪽(왼쪽)으로 13.5 m.
  - 5.4×3.6 m, yaw −90°, 긴 축이 도로와 평행.
  - 14×26 m 공터(x∈[−10.66, 3.34], y∈[63.28, 89.28]) 안, ±3 m 이동.
- **회전교차로**: 중심 (108.8353, 118.7754), NPC 반경 11 m.
- **route의 급한 꺾임**: 최소 회전반경(4.22 m)보다 급한 곳은 회전교차로 진입(s 325.5~328)과 이탈(s 343~345.5) 두 곳뿐, 곡률 최대 0.39. S자 구간은 곡률 0.18~0.19(R 5.2~5.5 m).
- **확인 필요**: 칸 경계에 물리적 물체가 있는지, 존 안 주행에 벌점이 있는지, yaw −90°의 의미.

## 1. 주차 (우선순위 1)

**출처**
- PythonRobotics ReedsShepp, HybridAStar (MIT): https://github.com/AtsushiSakai/PythonRobotics
- Autoware freespace_planner (Hybrid A*), behavior_path_planner geometric parallel parking (Apache-2.0): https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_freespace_planner
- rsplan (라이선스 확인 필요): https://github.com/builtrobotics/rsplan
- 평행주차 칸 길이 식: https://www.scitepress.org/Papers/2022/110456/110456.pdf , https://mdpi.com/2624-8921/4/4/1195 , https://phys.org/news/2009-12-scientist-formula.amp

**기하 (뒤차축 기준)**
- R_min = 3.0/tan35.4° = 4.221 m. 바깥 앞모서리 반경 6.441 m, 안쪽 측면 반경 3.275 m.
- 같은 반경 원호 2개 S자: cosθ = 1 − d/(2R), 종거리 Δx = 2R·sinθ, 원호 길이 Rθ.
- 칸이 통로에 바로 붙은 일반 모델(여유 0.15 m): R 4.22, θ 48.9°, 후진 7.21 m로 가능하나 최종 위치 창이 0.3 m뿐.

**`--course` 결과 (존 전체 주행 가능 가정)**

| 경우 | 결과 |
|---|---|
| 평행 slide 0, −3 | A: 후진 S자. R 4.22, 61°. 칸 중심 7.4 m 북쪽, 도로 중심 오른쪽 0.6 m에서 출발. 우조향 후진 4.5 m → 좌조향 후진 4.5 m |
| 평행 slide +3 | A는 존 남쪽 끝과 충돌. B: 전진 S자 진입만 가능. 후진 필수면 hybrid 탐색 필요 |
| 평행 strict | 해 없음 |
| 후진 slide 0, +3 | C: 전진 S자(R 6.72, 89°, 옆으로 13.2 m) → 정지 → 직진 후진 5.5 m(slide +3이면 2.5 m) → 전진으로 나감 |
| 후진 slide −3, strict | 해 없음. S자를 늦게 시작하거나 hybrid 탐색 |

**권장 구현: 새 `parking.py`**
1. 칸 검출: 존 안 라이다 점을 칸 좌표계로 옮겨 칸 길이 이상 비어 있는 구간 찾기. 허용 오차 ±1.38 m < seed 이동 ±3 m라서 검출 수단부터 확인.
2. 계획: 원호 후보 A/B/C 충돌 검사 → 가장 짧은 것, 실패 시 hybrid 탐색(느리므로 별도 스레드나 미리 계산한 표).
3. 상태 기계: APPROACH → ALIGN → STOP → SET_STEER → ARC1 → ARC2(→ STRAIGHT) → HOLD → EXIT.
   - 정지 시 EKF 자세 고정, 이후 바퀴 거리 + 자이로 yaw 상대 오도메트리.
   - 정지한 채 조향 ±1, 지연 + 0.15 s 대기. Δψ ≥ θ − |ω|·τ_steer에서 미리 반대 조향.
   - 후진 1.5~2 m/s, 기어 전환은 v≈0에서만, HOLD 동안 brake 1.
4. control.py: 후진 시 `GEAR_REVERSE`, 뒤차축 기준 pure pursuit(yaw+π).
5. 예상 시간: 평행 8~10 s, 후진주차 6~9 s.

함정: 원호 지나침, 존 끝 벌점, 라이다 최소 거리 0.5 m의 −Inf 위험점. 주차 중 회피 플래너 끄기.

## 2. 회전교차로 (우선순위 2)

**출처**: IDM (Treiber et al. 2000) https://www.emergentmind.com/topics/intelligent-driver-model-idm , RSS2022 paper 96 https://www.nrso.ntua.gr/rss2022/wp-content/uploads/2022/06/RSS2022_paper_96.pdf , HCM 6판(critical headway 4.1~4.6 s), KIT Gap-Approaching IDM https://publikationen.bibliothek.kit.edu/1000165122/155391787

**방법 (`roundabout_gap.py`)**
- 추적: 군집 중심 φ = atan2(y−cy, x−cx), |반경 − 11| < 1.5 m만, 상태 [φ, ω] 등각속도 칼만. 각도 최근접 연관(게이트 2.5/11 rad), 3회 관측 시 확정.
- 합류 판단 (반시계 진행, 위험 차는 φ<0 상류):
  - 상류 NPC 합류점 도착 시각 − 우리 차가 합류점을 빠져나가는 시각 > 2.5~3.5 s
  - 뒤차 안전 거리 v·T + (v_npc² − v_ego²)/(2b) + 차 길이
  - 우리 1/4 바퀴 구간 앞 NPC 거리 > s0 + 차 길이 + 0.5·v
- 정지선: 동쪽점 3~4 m 전. 합류 불가 시 speed_limit = √(2·a·거리). 접근 4~5 m/s.
- 링 위: IDM(v0 = min(6.5, √(a_lat·11)), T 1.0 s, s0 2.5 m, a 2, b 3) → speed_limit 상한.
- 함정: 회피 플래너가 NPC를 정지 장애물로 봄, 새 트랙 ω=0, 자차 점 섞임.

## 3. 빠른 정지 장애물 회피 (우선순위 3)

**출처**: PythonRobotics FrenetOptimalTrajectory (MIT), Apollo FEM_POS_DEVIATION / DP+QP (Apache-2.0) https://apolloauto.readthedocs.io/en/latest/specs/reference_line_smoother/ , Autoware path_optimizer (Apache-2.0)

**느린 원인**: `max_lateral_accel` 1.8, `avoidance_speed` 상한과 재시도 시 0.5배, 장애물 사이마다 중앙 복귀하는 후보 구조.

**권장**
1. 횡이동 길이를 속도에 맞춤: quintic 최대 곡률 ≈ 5.77·d/L² → L = v·√(5.77·d/a_lat). `max_lateral_accel` 3~3.5, `avoidance_speed`는 제동 가능 속도로만 제한. 예상 15~20 s 단축.
2. 장애물 3개 구간: 장애물 station마다 횡위치 {−2.1, −1.6, 0, 1.6, 2.1} 중 선택하는 DP(125조합), knot 사이 quintic.
3. 충돌 검사: 원 4개(r 1.109 m) KD-tree 1차 → 통과 후보만 다각형 확정.
4. 한쪽으로 결정한 뒤 반대편 후보에 큰 벌점.
5. offset ±2.3은 벽과 0.1 m 겹침 → 실질 최대 ±2.1.

## 4. 경로 추종 + 지연 (우선순위 4)

**출처**: PythonRobotics PathTracking (MIT), Autoware mpc_lateral_controller / pure_pursuit (Apache-2.0), ETH-PBL MAP-Controller (GPL-3.0, 아이디어만), F1TENTH 비교 https://www.matec-conferences.org/articles/matecconf/pdf/2024/18/matecconf_rapdasa2024_04013.pdf

**권장**
1. `path_smoothing.py`로 route 평활(R ≥ 4.8, 최대 이탈 0.66 m) 후 속도 프로파일 재계산.
2. 지연만큼 앞 지점 곡률 피드포워드: δ_ff = atan(WB·κ(s + v·τ)), `k_curvature` 0.6~1.0.
3. pure pursuit(Ld = clip(0.35·v + 2.5, 3, 10))는 실험 플래그로만.
4. 조향 변화율 제한(±2.5/s 정규화).
5. 여유가 있으면 조향 1차 지연을 넣은 LQR.

## 5. 저마찰 · 경사 · GNSS 음영

- 저마찰: 3.4 m/s, R 5.5 m → a_lat ≈ 2.1, μ ≈ 0.21. 존 전용 `corner_lateral_accel` 1.7, `profile_accel` 1.2, `profile_decel` 1.5, 슬립률 > 0.1이면 accel 0.5배.
- 경사: 내리막 존에서 `profile_decel` 0.5 낮춤.
- GNSS 음영: 진입 전 자이로 바이어스 수렴 확인, 음영 중 바이어스·바퀴 배율 고정, 경로 횡오차 의사측정, 벽이 보이면 라이다 횡위치 갱신, 속도 5~10% 감속.

## 6. 우선순위 실행 목록

| 순위 | 작업 | 영향 | 난이도 |
|---|---|---|---|
| 1 | 주차 상태 기계 + 원호 계획 + 후진 기어 | 0점 방지 | 중 |
| 2 | 칸 경계 물체·존 벌점·yaw 의미 확인 | 1의 전제 | 하 |
| 3 | 회전교차로 추적 + 정지선 + IDM | 충돌 0 + 2~4 s | 중 |
| 4 | route 평활화 + 속도 프로파일 재계산 | 안정 + 1~2 s | 하 |
| 5 | 회피 a_lat 상향 + 속도 연동 길이 + offset ±2.1 | 15~20 s | 하~중 |
| 6 | 앞선 곡률 피드포워드 + 조향 변화율 제한 | 고속 S자 안정 | 하 |
| 7 | 저마찰 존 전용 프로파일 + TCS | 실패 방지 + 2~3 s | 하 |
| 8 | 장애물 3개 다중 knot DP + 원 근사 KD-tree | 3~5 s | 중~상 |
| 9 | GNSS 음영 바이어스 고정 + 지도 정합 | 이탈 방지 | 중 |
| 10 | LQR 실험 | 불확실 | 상 |
