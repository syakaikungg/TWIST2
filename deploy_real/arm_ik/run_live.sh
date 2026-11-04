#!/bin/bash
# LIVE run on the real G1 with the PICO.
#   run_live.sh dry  [max_vel=0.5] [seconds=60]    bridge LOG ONLY: reads rt/lowstate, publishes nothing
#   run_live.sh send [max_vel=0.5] [seconds=180]   bridge PUBLISHES rt/arm_sdk (--send)
# The mode has to be typed: no argument does nothing. IK and bridge use the same joint speed limit; the hand
# target speed is 0.6 m/s as in the simulator runs. Nothing moves until right A (arms hold their measured pose)
# and a grip (right grip = right arm, left grip = left arm, both = both arms). Both programs stop by themselves:
# IK after (seconds - 5) s, bridge after seconds s.
# Stop the arms at any time: left X/Y on the PICO or ~/nextech/arm_ik/stop.sh (arm_sdk weight -> 0 in 1 s: arms and
# waist go back to the robot's own controller; the legs are never commanded). The remote's L2+B is whole-body
# damping (the robot goes limp): last resort only, with the robot hanging on its rope.
MODE=${1:-}
case "$MODE" in
  send) SEND=--send; SECS=${3:-180} ;;
  dry)  SEND="";     SECS=${3:-60} ;;
  *) echo "usage: run_live.sh dry|send [max_vel] [seconds]"; exit 1 ;;
esac
VEL=${2:-0.5}
cd ~/nextech/arm_ik || exit 1
if ps -eo cmd | grep -E 'arm_sdk_bridge|arm_ik_dryrun|arm_sim\.py|sim_run\.sh' | grep -v grep; then
  echo "ABORT: a bridge / IK / simulator is already running (stop it first)"; exit 1
fi
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
TS=$(date +%Y%m%d_%H%M%S)
L=~/nextech_setup/logs
echo "mode $MODE, max-vel $VEL rad/s, $SECS s"
nohup python3 -u arm_sdk_bridge.py $SEND --max-vel "$VEL" --duration "$SECS" > "$L/live_bridge_${MODE}_$TS.txt" 2>&1 < /dev/null &
echo "bridge pid $!"
sleep 4
nohup ~/miniforge3/envs/gmr/bin/python -u arm_ik_dryrun.py --arms both --duration $((SECS - 5)) \
  --max-vel "$VEL" --hand-speed 0.6 > "$L/live_ik_${MODE}_$TS.txt" 2>&1 < /dev/null &
echo "ik pid $!"
sleep 4
echo "--- bridge ---"; grep -E 'lowstate|SEND|LOG ONLY|kp=|monitors' "$L/live_bridge_${MODE}_$TS.txt"
echo "--- ik ---"; grep -E 'arm_ik_dryrun:|cleared' "$L/live_ik_${MODE}_$TS.txt"
echo "logs: $L/live_bridge_${MODE}_$TS.txt $L/live_ik_${MODE}_$TS.txt"
