#!/bin/bash
# LIVE run: the bridge PUBLISHES rt/arm_sdk (--send). Nothing moves until right A (arms hold their measured
# pose) and the right grip (follow). Both programs stop by themselves: IK after 115 s, bridge after 120 s.
# Stop at any time: left X/Y on the PICO, or ~/nextech/arm_ik/stop.sh.
cd ~/nextech/arm_ik || exit 1
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
TS=$(date +%Y%m%d_%H%M%S)
L=~/nextech_setup/logs
nohup python3 -u arm_sdk_bridge.py --send --max-vel 0.5 --duration 120 > "$L/live_bridge_$TS.txt" 2>&1 < /dev/null &
echo "bridge pid $!"
sleep 4
nohup ~/miniforge3/envs/gmr/bin/python -u arm_ik_dryrun.py --arms both --duration 115 > "$L/live_ik_$TS.txt" 2>&1 < /dev/null &
echo "ik pid $!"
sleep 4
echo "--- bridge ---"; grep -E 'lowstate|SEND|LOG ONLY|kp=|monitors' "$L/live_bridge_$TS.txt"
echo "--- ik ---"; grep -E 'arm_ik_dryrun:|cleared' "$L/live_ik_$TS.txt"
echo "logs: $L/live_bridge_$TS.txt $L/live_ik_$TS.txt"
