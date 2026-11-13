#!/bin/bash
# 사용: batch.sh 결과파일 "이름 seed cp stop options yaml(T|A) zlo zhi slo" ...
OUT=$1; shift
T=/mit_ws/agents/obstacle/probe/ob3_test.yaml
A=${WS:-/mit_ws/agents/obstacle}/install/hk_mit/share/hk_mit/config/all_options.yaml
B=1
: > $OUT
for spec in "$@"; do
  read name seed cp stop opts y zlo zhi slo <<< "$spec"
  [ "$y" = T ] && Y=$T || Y=$A
  SEED=$seed BUILD=$B EXTRA=$Y START_CP=$cp STOP_S=$stop OPTIONS=$opts PROBE_S=90 WS=${WS:-/mit_ws/agents/obstacle} \
    timeout 400 bash /mit_ws/agents/obstacle/probe/run.sh $name > /mit_ws/agents/obstacle/runs/$name.batch.txt 2>&1
  B=0
  source /etc/heven.bashrc
  { echo "== $name seed=$seed cp=$cp opts=$opts yaml=$y"; grep -E "ABORT|구간 끝|kind=2" /mit_ws/agents/obstacle/runs/$name.batch.txt;
    grep -E "우회 경로" /mit_ws/agents/obstacle/runs/$name.launch.log | cut -c60-260 | tail -2;
    python3 /mit_ws/agents/obstacle/probe/seg.py $name $slo $stop 2>&1 | tail -1;
    python3 /mit_ws/agents/obstacle/probe/clear.py $name $zlo $zhi 2>&1 | tail -1; } >> $OUT
  curl -s -H 'Content-Type: application/json' -b heven_token=$HEVEN_TOKEN -X POST http://localhost:8080/api/attempt/stop -d '{}' > /dev/null
  sleep 2
done
echo BATCH_DONE >> $OUT
