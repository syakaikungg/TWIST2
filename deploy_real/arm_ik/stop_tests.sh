#!/bin/bash
# Self-tests of every stop path: IK (synthetic inputs, writes Redis) + arm_sdk bridge in LOG-ONLY mode.
# The bridge reads rt/lowstate (read only). Nothing is published to rt/arm_sdk or rt/lowcmd.
cd ~/nextech/arm_ik || exit 1
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
PY=~/miniforge3/envs/gmr/bin/python
OUT=~/nextech_setup/logs/stop_tests_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
FAKE=""
python3 - <<'EOF' || FAKE="--fake-state"
import sys, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
ChannelFactoryInitialize(0, "eth0")
got = []
s = ChannelSubscriber("rt/lowstate", LowState_); s.Init(lambda m: got.append(1), 10)
t0 = time.time()
while not got and time.time() - t0 < 5: time.sleep(0.05)
print("lowstate available" if got else "no lowstate -> bridge uses --fake-state")
sys.exit(0 if got else 1)
EOF

run() {  # name ik_scenario ik_duration action action_time
  local name=$1 sc=$2 dur=$3 action=$4 at=$5
  echo "=== $name ==="
  python3 arm_sdk_bridge.py $FAKE --duration $((dur + 4)) > "$OUT/$name.bridge.txt" 2>&1 &
  local bpid=$!
  sleep 3   # bridge up (DDS discovery)
  $PY arm_ik_dryrun.py --selftest --selftest-redis --scenario "$sc" --duration "$dur" > "$OUT/$name.ik.txt" 2>&1 &
  local ipid=$!
  if [ -n "$action" ]; then
    sleep "$at"
    case $action in
      stopsh) ./stop.sh >> "$OUT/$name.ik.txt" ;;
      killik) kill -9 $ipid ;;
      termbridge) kill -TERM $bpid ;;
    esac
  fi
  wait $ipid 2>/dev/null
  wait $bpid 2>/dev/null
  python3 -c "import redis; r = redis.Redis(); print('drill_cmd after:', r.get('drill_cmd')); print('teleop_state after:', r.hgetall('teleop_state'))" > "$OUT/$name.redis.txt"
  grep -E 'EMERGENCY|RAMP|TRACK|STOPPED|released|lowstate ok|LOG ONLY|fake' "$OUT/$name.bridge.txt"
  grep -E 'EMERGENCY|grip|A ->|B ->' "$OUT/$name.ik.txt"
  cat "$OUT/$name.redis.txt"
}

run deadman     deadman    9  ""         0
run estop_x     estop_x    9  ""         0
run pico_stale  pico_stale 9  ""         0
run stop_sh     none      16  stopsh     5
run ik_killed   none      16  killik     5
run bridge_term none      16  termbridge 5
echo "logs in $OUT"
