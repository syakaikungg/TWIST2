#!/bin/bash
# AUTOMATIC PLAYBACK on the real G1: a fixed arm motion again and again with cool-down breaks (play_motion.py
# instead of the IK; the bridge and its monitors are the same as for teleop). Nobody needs to wear the headset.
#   run_play.sh dry  [seconds=60]  [motion=motions/drill_work.json]   bridge LOG ONLY (nothing is sent; the bridge
#                                                                      stops on tracking error once the glide starts,
#                                                                      since the arms do not move: only a start check)
#   run_play.sh send [seconds=600] [motion=motions/drill_work.json]   the arms MOVE (only after an explicit go, the
#                                                                      robot hanging in the lock stand, L2+up)
# Cycle: arms glide to the rest pose (arms down, driver down) -> WORK s of the motion's loop -> back to rest ->
# rest at least COOL s and until the hottest arm/waist motor is at or below TEMP_RESUME C -> WORK again ...
# WORK ends early at TEMP_PAUSE C. At the end the arms glide back to where they were at the start and the robot's
# own controller takes them (weight -> 0 in 2 s).
# Settings (environment): WORK (30 s), COOL (60 s), TEMP_PAUSE (65 C), TEMP_RESUME (58 C), MAX_TEMP (the bridge's
# stop, 75 C, 50..80), CYCLES (0 = until the seconds run out), SPEED (1.0: loop speed), VEL (0.5 rad/s: the bridge's
# joint speed limit), PICO (1: left X/Y = emergency stop, right A = pause at rest / resume; 0 = do not read the PICO).
# e.g. WORK=20 COOL=90 bash run_play.sh send 900
# Stop: left X/Y on the PICO (if connected), ~/nextech/arm_ik/stop.sh (g1.sh stop on the PC), or the remote's
# L2+B (whole-body damping, last resort). The bridge also stops by itself on MAX_TEMP, tracking error, lowstate gap.
MODE=${1:-}
case "$MODE" in
  send) SEND=--send; SECS=${2:-600} ;;
  dry)  SEND="";     SECS=${2:-60} ;;
  *) echo "usage: run_play.sh dry|send [seconds] [motion.json]"; exit 1 ;;
esac
MOTION=${3:-motions/drill_work.json}
WORK=${WORK:-30}; COOL=${COOL:-60}; TEMP_PAUSE=${TEMP_PAUSE:-65}; TEMP_RESUME=${TEMP_RESUME:-58}
MAX_TEMP=${MAX_TEMP:-75}; CYCLES=${CYCLES:-0}; SPEED=${SPEED:-1.0}; VEL=${VEL:-0.5}; PICO=${PICO:-1}
num() { [[ "$2" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "$1 must be a number"; exit 1; }; }
num seconds "$SECS"; num WORK "$WORK"; num COOL "$COOL"; num CYCLES "$CYCLES"; num SPEED "$SPEED"; num VEL "$VEL"
case "$MAX_TEMP" in 5[0-9]|6[0-9]|7[0-9]|80) ;; *) echo "MAX_TEMP must be 50..80"; exit 1 ;; esac
case "$TEMP_PAUSE" in [4-7][0-9]) ;; *) echo "TEMP_PAUSE must be 40..79"; exit 1 ;; esac
case "$TEMP_RESUME" in [3-7][0-9]) ;; *) echo "TEMP_RESUME must be 30..79"; exit 1 ;; esac
[ "$TEMP_PAUSE" -lt "$MAX_TEMP" ] || { echo "TEMP_PAUSE ($TEMP_PAUSE) must be below MAX_TEMP ($MAX_TEMP)"; exit 1; }
[ "$TEMP_RESUME" -lt "$TEMP_PAUSE" ] || { echo "TEMP_RESUME ($TEMP_RESUME) must be below TEMP_PAUSE ($TEMP_PAUSE)"; exit 1; }
[ "${SECS%.*}" -ge 30 ] || { echo "seconds must be 30 or more"; exit 1; }
case "$PICO" in 0) PICO_ARG="" ;; 1) PICO_ARG="--pico" ;; *) echo "PICO must be 0 or 1"; exit 1 ;; esac
cd ~/nextech/arm_ik || exit 1
[ -r "$MOTION" ] || { echo "no motion file $MOTION"; exit 1; }
PY=~/miniforge3/envs/gmr/bin/python   # has xrobotoolkit_sdk (PICO), numpy and redis, as for the IK
"$PY" play_motion.py --motion "$MOTION" --speed "$SPEED" --max-vel "$VEL" --check || exit 1
if ps -eo cmd | grep -E 'arm_sdk_bridge|arm_ik_dryrun|play_motion|arm_sim\.py|sim_run\.sh' | grep -v grep; then
  echo "ABORT: a bridge / IK / playback / simulator is already running (stop it first)"; exit 1
fi
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}
TS=$(date +%Y%m%d_%H%M%S)
L=~/nextech_setup/logs
echo "play $MODE, $SECS s, motion $MOTION, work $WORK s, cool >= $COOL s and <= $TEMP_RESUME C, early rest at $TEMP_PAUSE C," \
     "bridge stop at $MAX_TEMP C, cycles ${CYCLES/#0/unlimited}, speed x$SPEED, max-vel $VEL rad/s, pico $PICO"
nohup python3 -u arm_sdk_bridge.py $SEND --max-vel "$VEL" --duration "$SECS" --max-temp "$MAX_TEMP" \
  --waist-correct "" > "$L/play_bridge_${MODE}_$TS.txt" 2>&1 < /dev/null &
BR_PID=$!
echo "bridge pid $BR_PID"
sleep 4
nohup "$PY" -u play_motion.py --motion "$MOTION" --work "$WORK" --cool "$COOL" --temp-pause "$TEMP_PAUSE" \
  --temp-resume "$TEMP_RESUME" --cycles "$CYCLES" --duration $(( ${SECS%.*} - 2 )) --speed "$SPEED" --max-vel "$VEL" \
  $PICO_ARG > "$L/play_motion_${MODE}_$TS.txt" 2>&1 < /dev/null &
PLAY_PID=$!
echo "play pid $PLAY_PID"
# When the playback ends (done, stopped or failed), end the bridge too, so the next start is not refused. By then
# the weight is 0 (or the bridge has stopped on its own: the playback's commands go stale in 0.3 s).
nohup bash -c "while kill -0 $PLAY_PID 2>/dev/null; do sleep 1; done; sleep 2; kill -TERM $BR_PID 2>/dev/null" \
  > /dev/null 2>&1 < /dev/null &
sleep 4
echo "--- bridge ---"; grep -E 'lowstate|SEND|LOG ONLY|kp=|monitors|waist target|waist correction' "$L/play_bridge_${MODE}_$TS.txt"
echo "--- play ---"; grep -E 'play_motion:|work |PICO|REFUSED|ENGAGE|no bridge' "$L/play_motion_${MODE}_$TS.txt"
echo "logs: $L/play_bridge_${MODE}_$TS.txt $L/play_motion_${MODE}_$TS.txt"
echo "follow: tail -f $L/play_motion_${MODE}_$TS.txt    stop: bash ~/nextech/arm_ik/stop.sh"
