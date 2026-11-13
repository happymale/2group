#!/bin/bash
# 빌드(선택) → bringup 실행 → 시도 시작 → 기록 → 종료.  컨테이너 안에서 heven 사용자로 실행합니다.
#   bash /mit_ws/tools/run_test.sh <이름> [launch 인자...]
# 환경 변수
#   HEVEN_TOKEN  웹 화면 토큰. 주면 시도를 자동으로 시작·중지합니다(없으면 웹 화면에서 '연습 시작'을 누름).
#   START_CP     시작 체크포인트 (0 = 출발, 연습 모드 CP부터). 기본 0
#   OPTIONS      켤 옵션 이름을 쉼표로 (예: low_friction,slope 또는 all). 기본 없음
#   ROAD_LEVEL   도로 폭 단계 0~5. 기본 0
#   SEED         seed. 기본 0
#   STOP_S       경로 거리(m)가 이 값을 넘으면 기록을 끝내고 시도를 중지 (구간 시험용). 기본 끝까지
#   BUILD        0이면 빌드를 건너뜀 (여러 시뮬레이터가 같은 install을 쓸 때). 기본 1
#   WS           작업 공간 (src/hk_mit이 있는 곳). 기본 /mit_ws. 에이전트별 복사본이면 /mit_ws/agents/<이름>
set -e
NAME=${1:-test}; shift || true
WS=${WS:-/mit_ws}
OUT=$WS/runs; [ "$WS" = /mit_ws ] && OUT=/mit_ws/tools/runs; mkdir -p "$OUT"
source /etc/heven.bashrc
if [ "${BUILD:-1}" = 1 ]; then
  cd "$WS" && colcon build --base-paths src --packages-select hk_mit > "$OUT/$NAME.build.log" 2>&1 || { tail -20 "$OUT/$NAME.build.log"; exit 1; }
fi
source "$WS/install/setup.bash"
api() { curl -s -H 'Content-Type: application/json' -b "heven_token=$HEVEN_TOKEN" -X POST "http://localhost:8080/api/attempt/$1" -d "$2"; }
ros2 launch hk_mit bringup.launch.py rviz:=false "$@" > "$OUT/$NAME.launch.log" 2>&1 &
LAUNCH=$!
cleanup() {
  if [ -n "$HEVEN_TOKEN" ]; then api stop '{}' > /dev/null || true; fi
  kill -INT $LAUNCH 2>/dev/null; sleep 2; kill $LAUNCH 2>/dev/null
  pkill -f "$WS/install/hk_mit/lib/hk_mit/" 2>/dev/null || true
}
trap cleanup EXIT
echo "READY $NAME"
python3 /mit_ws/tools/lap_logger.py "$OUT/$NAME.csv" ${STOP_S:+--stop-s $STOP_S} > "$OUT/$NAME.summary.txt" 2>&1 &
LOGGER=$!
if [ -n "$HEVEN_TOKEN" ]; then
  sleep 6                                    # 노드·기록기가 이벤트를 받을 준비
  OPTS=$(python3 - "$OPTIONS" "${ROAD_LEVEL:-0}" <<'EOF'
import json, sys
names = ['rear_parking', 'parallel_parking', 'roundabout', 'obstacle1', 'obstacle3', 'low_friction', 'slope',
         'gnss_shadow', 'gnss_noise', 'imu_noise', 'lidar_noise', 'steer_latency', 'drive_latency']
on = names if sys.argv[1] == 'all' else [n for n in sys.argv[1].split(',') if n]
bad = [n for n in on if n not in names]
if bad:
    sys.exit(f'unknown options: {bad}')
opts = {n: n in on for n in names}
opts['road_level'] = int(sys.argv[2])
print(json.dumps(opts))
EOF
)
  echo "start: $(api start "{\"mode\":\"practice\",\"seed\":${SEED:-0},\"options\":$OPTS,\"start_cp\":${START_CP:-0}}")"
fi
wait $LOGGER || true
cat "$OUT/$NAME.summary.txt"
