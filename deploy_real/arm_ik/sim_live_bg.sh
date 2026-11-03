#!/bin/bash
# Start the simulator with live PICO input in the background. No robot I/O (bridge --sim, no DDS).
#   sim_live_bg.sh [payload_kg] [seconds] ["extra IK args"]
# Bridge and IK joint speed 0.5 rad/s as planned for the first real run; bridge waist = its defaults.
# Also samples the CPU of the simulator / bridge / IK / stream / camera processes (cpu_sim_live_HHMMSS.txt).
cd ~/nextech/arm_ik || exit 1
L=~/nextech_setup/logs
SECS=${2:-120}
SIM_DUR=$SECS nohup bash sim_run.sh live "${1:-0}" 0 0.5 "" "--max-vel 0.5 ${3:-}" > $L/sim_live.txt 2>&1 < /dev/null &
echo "sim live started (pid $!, ${SECS} s)"
nohup bash cpu_sampler.sh $((SECS + 15)) > $L/cpu_sim_live_$(date +%H%M%S).txt 2>&1 < /dev/null &
