#!/bin/bash
# 컨테이너 안에서: bash run.sh 이름  (환경 변수 START_CP, STOP_S, OPTIONS, SEED, BUILD 등은 그대로 넘김)
NAME=$1; shift
source /etc/heven.bashrc
python3 /mit_ws/agents/roundabout/tools_rb/npc_log.py /mit_ws/agents/roundabout/runs/$NAME.npc.csv 170 > /mit_ws/agents/roundabout/runs/$NAME.npclog.out 2>&1 &
bash /mit_ws/tools/run_test.sh $NAME extra_params:=/mit_ws/agents/roundabout/install/hk_mit/share/hk_mit/config/all_options.yaml "$@"
pkill -f "npc_log.py /mit_ws/agents/roundabout/runs/$NAME" || true
