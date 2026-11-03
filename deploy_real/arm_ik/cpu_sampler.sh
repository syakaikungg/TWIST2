#!/bin/bash
# Read only: every ~5 s, overall CPU and the current CPU of the python processes (simulator, bridge, IK,
# stream, camera). cpu_sampler.sh <seconds>
END=$(( $(date +%s) + ${1:-120} ))
while [ "$(date +%s)" -lt "$END" ]; do
  echo "== $(date -u +%T) UTC"
  top -b -d 1 -n 2 -w 200 | awk '/^top -/{n++} n==2' | grep -E '^%Cpu|python' | head -12 | cut -c1-110
  ps -eo pid,args | grep -E 'arm_sim|arm_sdk_bridge|arm_ik_dryrun|stream_sim|simcam|cam_to_pico' | grep -v grep | cut -c1-110
  sleep 4
done
