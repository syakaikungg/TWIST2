#!/bin/bash
# Check the new bridge monitors in LOG-ONLY mode (real rt/lowstate, IK scenario "hold"). No robot commands.
# In log-only mode the robot does not follow, so |command - measured| grows with the commanded motion.
cd ~/nextech/arm_ik || exit 1
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
PY=~/miniforge3/envs/gmr/bin/python
run() {  # name bridge-args
  echo "=== $1 ($2) ==="
  python3 -u arm_sdk_bridge.py --duration 20 $2 > /tmp/mon_bridge.txt 2>&1 &
  local b=$!
  sleep 3
  $PY arm_ik_dryrun.py --selftest --selftest-redis --scenario hold > /tmp/mon_ik.txt 2>&1
  wait $b
  grep -E 'monitors|RAMP_UP|TRACK|EMERGENCY|STOPPED|released' /tmp/mon_bridge.txt
}
run defaults ""
run track_err_0.1 "--max-track-err 0.1"
run temp_30C "--max-temp 30"
