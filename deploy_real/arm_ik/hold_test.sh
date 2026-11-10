#!/bin/bash
# "Nothing moves until the grip" test: bridge LOG-ONLY (real rt/lowstate) + IK scenario "hold". No robot commands.
cd ~/nextech/arm_ik || exit 1
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
PY=~/miniforge3/envs/gmr/bin/python
python3 arm_sdk_bridge.py --duration 22 > /tmp/hold_bridge.txt 2>&1 &
BPID=$!
sleep 3
$PY arm_ik_dryrun.py --selftest --selftest-redis --scenario hold > /tmp/hold_ik.txt 2>&1
wait $BPID
grep -E 'lowstate ok|LOG ONLY|RAMP|TRACK|released|EMERGENCY|STOPPED' /tmp/hold_bridge.txt
grep -E 'A ->|A ignored|grip|B ->|start pose reached|EMERGENCY' /tmp/hold_ik.txt
python3 check_hold.py
