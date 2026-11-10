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
# MAX_TEMP=none (2026-11-10, operator's request, for one measurement run): the bridge does not stop on temperature
# (--max-temp 200); the PICO warning still shows from 65 C. Above 70 C nothing from Unitree says the motors are safe,
# and a protection inside the motor may cut the torque (the arm drops). Only with the robot hanging and someone at
# left X. Every other monitor (tracking error, lowstate, IK gap, left X) stays.
# Temperature pause TEMP_PAUSE (IK --temp-pause; off = 0 by default): when the hottest arm/waist motor reaches it,
# the arms glide back to the start pose (where they were at A, arms down) and pause (weight -> 0 in 2 s, no
# emergency stop); A is refused until it is at or below TEMP_RESUME (default 60 C). Why: with the tool, the right
# shoulder pitch rose about 1 C/s while following and a released arm still holds the tool where it stopped, so it
# kept heating to the stop (59 -> 71 C in 14 s on 2026-11-10); after the stop it fell 71 -> 60 C in about 1.5 min.
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
case "$MAX_TEMP" in 5[0-9]|6[0-9]|70) BR_TEMP=$MAX_TEMP; IK_WARN=0 ;; none) BR_TEMP=200; IK_WARN=65 ;;
  *) echo "MAX_TEMP must be 50..70 or none"; exit 1 ;; esac
case "$TOOL" in hand|drill) ;; *) echo "tool must be hand or drill"; exit 1 ;; esac
TEMP_PAUSE=${TEMP_PAUSE-0}
TEMP_RESUME=${TEMP_RESUME:-60}
case "$TEMP_PAUSE" in ""|0) TEMP_PAUSE=0 ;; [4-6][0-9]) ;; *) echo "TEMP_PAUSE must be 0 (off) or 40..69"; exit 1 ;; esac
case "$TEMP_RESUME" in [3-6][0-9]) ;; *) echo "TEMP_RESUME must be 30..69"; exit 1 ;; esac
if [ "$TEMP_PAUSE" != 0 ]; then
  [ "$TEMP_PAUSE" -lt "$BR_TEMP" ] || { echo "TEMP_PAUSE ($TEMP_PAUSE) must be below MAX_TEMP ($MAX_TEMP)"; exit 1; }
  [ "$TEMP_RESUME" -lt "$TEMP_PAUSE" ] || { echo "TEMP_RESUME ($TEMP_RESUME) must be below TEMP_PAUSE ($TEMP_PAUSE)"; exit 1; }
fi
cd ~/nextech/arm_ik || exit 1
if ps -eo cmd | grep -E 'arm_sdk_bridge|arm_ik_dryrun|arm_sim\.py|sim_run\.sh' | grep -v grep; then
  echo "ABORT: a bridge / IK / simulator is already running (stop it first)"; exit 1
fi
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
TS=$(date +%Y%m%d_%H%M%S)
L=~/nextech_setup/logs
tp="temp pause off"; [ "$TEMP_PAUSE" != 0 ] && tp="temp pause $TEMP_PAUSE C (A again at <= $TEMP_RESUME C)"
echo "mode $MODE, max-vel $VEL rad/s, $SECS s, tool $TOOL, max-temp $MAX_TEMP C, $tp, waist correction ${WAIST_CORRECT:-off}"
[ "$MAX_TEMP" = none ] && echo "WARNING: NO TEMPERATURE STOP in the bridge (measurement run); PICO warning from 65 C"
nohup python3 -u arm_sdk_bridge.py $SEND --max-vel "$VEL" --duration "$SECS" --max-temp "$BR_TEMP" \
  --waist-correct "$WAIST_CORRECT" > "$L/live_bridge_${MODE}_$TS.txt" 2>&1 < /dev/null &
echo "bridge pid $!"
sleep 4
nohup ~/miniforge3/envs/gmr/bin/python -u arm_ik_dryrun.py --arms both --duration $((SECS - 5)) \
  --max-vel "$VEL" --hand-speed 0.6 --tool "$TOOL" --temp-pause "$TEMP_PAUSE" --temp-resume "$TEMP_RESUME" --temp-warn "$IK_WARN" \
  > "$L/live_ik_${MODE}_$TS.txt" 2>&1 < /dev/null &
echo "ik pid $!"
sleep 4
echo "--- bridge ---"; grep -E 'lowstate|SEND|LOG ONLY|kp=|monitors|waist target|waist correction' "$L/live_bridge_${MODE}_$TS.txt"
echo "--- ik ---"; grep -E 'arm_ik_dryrun:|cleared' "$L/live_ik_${MODE}_$TS.txt"
echo "logs: $L/live_bridge_${MODE}_$TS.txt $L/live_ik_${MODE}_$TS.txt"
