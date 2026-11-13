#!/bin/bash
# 사용: run.sh 이름 [run_test.sh 인자...]   (START_CP, STOP_S, OPTIONS, SEED, BUILD 환경 변수 전달)
NAME=$1; shift
source /etc/heven.bashrc
mkdir -p /mit_ws/agents/obstacle/runs
(sleep 4; source /mit_ws/agents/obstacle/install/setup.bash; timeout 200 python3 /mit_ws/agents/obstacle/probe/probe.py /mit_ws/agents/obstacle/runs/$NAME.probe.pkl ${PROBE_S:-60}) &
bash /mit_ws/tools/run_test.sh $NAME extra_params:=${EXTRA:-/mit_ws/agents/obstacle/install/hk_mit/share/hk_mit/config/all_options.yaml} "$@"
wait
pkill -f "hk_mit_robot_state_publisher.*agents/obstacle" 2>/dev/null; pkill -f "__node:=hk_mit_robot_state_publisher" 2>/dev/null; true
