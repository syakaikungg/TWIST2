#!/bin/bash
# Start the simulator with live PICO input in the background. No robot I/O (bridge --sim, no DDS).
#   sim_live_bg.sh [payload_kg] [seconds] ["extra IK args"] [max_vel]
# Bridge and IK joint speed limit max_vel rad/s (default 0.5, as planned for the first real run);
# bridge waist = its defaults.
# Also samples the CPU of the simulator / bridge / IK / stream / camera processes (cpu_sim_live_HHMMSS.txt).
cd ~/nextech/arm_ik || exit 1
L=~/nextech_setup/logs
SECS=${2:-120}
VEL=${4:-0.5}
SIM_DUR=$SECS nohup bash sim_run.sh live "${1:-0}" 0 "$VEL" "" "--max-vel $VEL ${3:-}" > $L/sim_live.txt 2>&1 < /dev/null &
echo "sim live started (pid $!, ${SECS} s, max-vel ${VEL} rad/s)"
nohup bash cpu_sampler.sh $((SECS + 15)) > $L/cpu_sim_live_$(date +%H%M%S).txt 2>&1 < /dev/null &
