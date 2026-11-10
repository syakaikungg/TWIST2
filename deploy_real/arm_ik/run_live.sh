#!/bin/bash
# LIVE run on the real G1 with the PICO.
#   run_live.sh dry  [max_vel=0.5] [seconds=60]  [tool=hand]   bridge LOG ONLY: reads rt/lowstate, publishes nothing
#   run_live.sh send [max_vel=0.5] [seconds=180] [tool=hand]   bridge PUBLISHES rt/arm_sdk (--send)
# The mode has to be typed: no argument does nothing. IK and bridge use the same joint speed limit; the hand
# target speed is 0.6 m/s as in the simulator runs. Nothing moves until right A (arms hold their measured pose)
# and a grip (right grip = right arm, left grip = left arm, both = both arms). Both programs stop by themselves:
# IK after (seconds - 5) s, bridge after seconds s.
# tool: what the IK's collision avoidance puts on the right wrist: hand (the hand is still on, default until the
# drill adapter is mounted) or drill (the provisional drill shape of sim_drill.py).
# The waist is brought slowly to 0 (upright) in roll and pitch from right A on (bridge default --waist-level).
# The bridge's integral waist correction is OFF by default since 2026-11-10 (WAIST_CORRECT=roll,pitch turns it on as
# before): with the tool on the right wrist it wound up to its 0.15 rad limit while the waist did not move, the waist
# pitch torque went 4 -> 24 Nm and the motor 54 -> 66 C in 25 s. For the exhibition the robot hangs with the legs limp.
# Motor temperature limit of the bridge: MAX_TEMP (default 70 C since 2026-11-10, 65 C from 2026-11-04, 60 C before;
# e.g. MAX_TEMP=65 bash run_live.sh send; 50..70). Raised to 70 C at the operator's request on 2026-11-10: with the
# tool on the right wrist the right shoulder pitch holds 8-10 Nm and reached 65 C after about 30 s of following.
# A motor the bridge commands at MAX_TEMP - 5 shows a warning on the PICO; above MAX_TEMP the bridge stops (3 s release). A leg motor (the robot's own) at 70 C or more only shows a warning.
# Stop the arms at any time: left X/Y on the PICO or ~/nextech/arm_ik/stop.sh (arm_sdk weight -> 0 in 3 s: arms and
# waist go back to the robot's own controller; the legs are never commanded). The remote's L2+B is whole-body
# damping (the robot goes limp): last resort only, with the robot hanging on its rope.
MODE=${1:-}
case "$MODE" in
  send) SEND=--send; SECS=${3:-180} ;;
  dry)  SEND="";     SECS=${3:-60} ;;
  *) echo "usage: run_live.sh dry|send [max_vel] [seconds]"; exit 1 ;;
esac
VEL=${2:-0.5}
TOOL=${4:-hand}
MAX_TEMP=${MAX_TEMP:-70}
WAIST_CORRECT=${WAIST_CORRECT-}
case "$WAIST_CORRECT" in ""|roll|pitch|roll,pitch|pitch,roll) ;; *) echo "WAIST_CORRECT must be empty, roll, pitch or roll,pitch"; exit 1 ;; esac
case "$MAX_TEMP" in 5[0-9]|6[0-9]|70) ;; *) echo "MAX_TEMP must be 50..70"; exit 1 ;; esac
case "$TOOL" in hand|drill) ;; *) echo "tool must be hand or drill"; exit 1 ;; esac
cd ~/nextech/arm_ik || exit 1
if ps -eo cmd | grep -E 'arm_sdk_bridge|arm_ik_dryrun|arm_sim\.py|sim_run\.sh' | grep -v grep; then
  echo "ABORT: a bridge / IK / simulator is already running (stop it first)"; exit 1
fi
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
TS=$(date +%Y%m%d_%H%M%S)
L=~/nextech_setup/logs
echo "mode $MODE, max-vel $VEL rad/s, $SECS s, tool $TOOL, max-temp $MAX_TEMP C, waist correction ${WAIST_CORRECT:-off}"
nohup python3 -u arm_sdk_bridge.py $SEND --max-vel "$VEL" --duration "$SECS" --max-temp "$MAX_TEMP" \
  --waist-correct "$WAIST_CORRECT" > "$L/live_bridge_${MODE}_$TS.txt" 2>&1 < /dev/null &
echo "bridge pid $!"
sleep 4
nohup ~/miniforge3/envs/gmr/bin/python -u arm_ik_dryrun.py --arms both --duration $((SECS - 5)) \
  --max-vel "$VEL" --hand-speed 0.6 --tool "$TOOL" > "$L/live_ik_${MODE}_$TS.txt" 2>&1 < /dev/null &
echo "ik pid $!"
sleep 4
echo "--- bridge ---"; grep -E 'lowstate|SEND|LOG ONLY|kp=|monitors|waist target|waist correction' "$L/live_bridge_${MODE}_$TS.txt"
echo "--- ik ---"; grep -E 'arm_ik_dryrun:|cleared' "$L/live_ik_${MODE}_$TS.txt"
echo "logs: $L/live_bridge_${MODE}_$TS.txt $L/live_ik_${MODE}_$TS.txt"
