#!/bin/bash
# Simulator run on the G1: arm_sim.py (physics) + arm_sdk_bridge.py --sim + IK (scripted scenario or live PICO).
# No robot I/O at all (the bridge uses Redis sim_lowcmd / sim_lowstate instead of DDS).
#   sim_run.sh [sim_suite|live] [payload_kg] [video(0/1)] [bridge_max_vel] ["extra bridge args"] ["extra IK args"] [payload_offset_m]
SC=${1:-sim_suite}; KG=${2:-0}; VIDEO=${3:-0}; VEL=${4:-1.0}; BRARGS=${5:-}; IKARGS=${6:-}; OFF=${7:-0.05}
cd ~/nextech/arm_ik || exit 1
PY=~/miniforge3/envs/gmr/bin/python
L=~/nextech_setup/logs
TAG=${SC}_${KG}kg_off${OFF}_$(date +%H%M%S)
if [ -n "$SIM_DUR" ]; then DUR=$SIM_DUR; elif [ "$SC" = live ]; then DUR=120; else DUR=24; fi
VARG=""; [ "$VIDEO" = 1 ] && VARG="--video $L/arm_sim_$TAG.mp4"
MUJOCO_GL=egl $PY -u arm_sim.py --duration $((DUR + 9)) --payload-kg "$KG" --payload-offset "$OFF" --tag "$TAG" $VARG > "$L/sim_$TAG.txt" 2>&1 &
SIM=$!
sleep 4
python3 -u arm_sdk_bridge.py --sim --max-vel "$VEL" --duration $((DUR + 4)) $BRARGS > "$L/simbridge_$TAG.txt" 2>&1 &
BR=$!
sleep 2
if [ "$SC" = live ]; then
  $PY -u arm_ik_dryrun.py --arms both --duration $DUR $IKARGS > "$L/simik_$TAG.txt" 2>&1
else
  $PY -u arm_ik_dryrun.py --selftest --selftest-redis --scenario "$SC" --duration $DUR $IKARGS > "$L/simik_$TAG.txt" 2>&1
fi
wait $BR; wait $SIM
echo "=== $TAG ==="
grep -E 'SIM MODE|RAMP|TRACK|EMERGENCY|STOPPED|released' "$L/simbridge_$TAG.txt"
grep -E 'A ->|grip ->|B ->|start pose|EMERGENCY|timing' "$L/simik_$TAG.txt" | head -9
grep -E 'simulated|video:' "$L/sim_$TAG.txt"
[ "$SC" = sim_suite ] && $PY analyze_sim.py "$TAG"
