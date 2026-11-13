# speed 에이전트 최종 보고 (옵션 전부 켬, 장애물·주차 제외)

seed 0/1/2: 98.6 / 102.2 / 100.5 s, 감점 0 (이전 102.8 / 114.7(감점 1) / 104.6 s). 패치: `speed.diff` (739줄).

## 확인한 사실
- 벽: 좁은 통로 route ±2.20 m, 장애물 구간 ±3.75, 회전교차로·주차 구간 ±3.0(한쪽 열림), 폭 전이 구간이 미션 앞뒤 6~15 m.
- seed 1 감점(s≈627)은 위치 추정 횡오차 1.1 m가 원인(제어 문제 아님).
- lap_logger의 '통로 여유'는 차체 네 모서리만 봐서 부정확. 실제 여유는 LiDAR 점과 차체 사각형 거리로 잼.

## 코드 변경
- `wall_match.py` (새 파일): route ± corridor_width/2 벽 지도, 미션·회전교차로 주변 18 m 제외, 측별 최빈값 → ±0.2 m 게이트 → Huber IRLS, 관측 가능한 방향만 EKF 측정.
- `localization.py`: LiDAR 벽 측정(스캔 시각 자세 보간), NIS>16 기각(10회 연속이면 공분산 키워 수용), 별도 스레드(MultiThreadedExecutor + RLock, epoch로 오래된 결과 폐기), 순서 뒤바뀐 IMU 버림, 시작·복귀 즉시 publish.
- `control.py`: odom 지연(최대 0.25 s) 예측 보상, `offset_span`, `ready()` 분리, 출발 보조(`launch_assist` true, `launch_max_dist` 8, `launch_max_speed` 5; 미검증·미작동, 끄려면 false).
- `planner.py`, `course.py`: `files_signature()`로 START마다 경로·프로파일 재계산 생략, `preload()`.
- `all_options.yaml`: `speed_zones` [56,165,3.4, 185,272,6.0] (음영 8 m/s 상한 제거, 경사 S자 6.0), `corner_lateral_accel` 6.0, `profile_decel` 5.0, `roundabout_speed` 5.5, localization `wall_enabled` true, `corridor_width` 0, `wall_inflation` 4, `wall_zone_margin` 18. `k_offset` 0 유지.

## 결과 (여유 = LiDAR 기준)
| 주행 | seed | 시간 | 감점 | 최소 여유 | 음영 600~650 | 회전교차로 | 위치 오차 평균 |
|---|---|---|---|---|---|---|---|
| sp_base_s1 | 1 | 114.7 | 1 | 0.00 | 0.00 | 0.63 | 0.30 |
| sp_v1_s0 | 0 | 98.6 | 0 | 0.28 (s=255) | 0.72 | 0.86 | 0.14 |
| sp_v1_s1 | 1 | 102.2 | 0 | 0.28 (s=228) | 0.75 | 0.79 | 0.13 |
| sp_v1_s2 | 2 | 100.5 | 0 | 0.16 (s=254) | 0.65 | 0.90 | 0.14 |

벽이 보이는 구간 위치 오차 0.01~0.03 m. 출발 0.60~0.62 s, cp1까지 2.8 s.

## 남은 위험
- 경사 S자(s 228~255) 여유 0.16~0.28 m: 조향 지연으로 곡률 반전부에서 안쪽 0.8~0.9 m 파고듦. 후보: 상한 5.5(+0.7 s) 또는 corner_lateral_accel 5.5 (미시험).
- sp_w3_s0 위치 오차 144 m: 노드가 START보다 늦게 떠 지난 자세를 한 번 낸 것(차 정지 상태).
- 출발 지연 최대 5.7 s는 run_test.sh가 노드 기동 후 6 s만 기다리기 때문(부하 시 노드 기동 약 8 s).
- 부하가 크면 odom 0.06~0.17 s 끊김.
- 저마찰 3.4 m/s 상한(약 28 s) 그대로. 도로 폭 단계는 0단계만 검증.
