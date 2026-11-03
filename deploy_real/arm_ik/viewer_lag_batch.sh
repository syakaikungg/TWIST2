#!/bin/bash
# Simulator only (no robot I/O). While a sim_suite runs, sample the CPU of stream_sim.py on the G1
# (the laptop viewer measures itself); then compare the hand lag at 0.5 and 1.0 rad/s. ~2 min.
cd ~/nextech/arm_ik || exit 1
PY=~/miniforge3/envs/gmr/bin/python
for vel in 0.5 1.0; do
  bash sim_run.sh sim_suite 0 0 $vel "--waist-kp 60,60,60 --waist-kd 1.5,1.5,1.5" "--max-vel $vel" > /tmp/vl_$vel.txt 2>&1 &
  R=$!
  if [ "$vel" = 0.5 ]; then
    sleep 15
    echo "--- G1 CPU while the simulator runs (%CPU) ---"
    ps -eo pcpu,etime,cmd | grep -E 'stream_sim|arm_sim.py|arm_sdk_bridge|arm_ik_dryrun' | grep -v grep | cut -c1-110
  fi
  wait $R
  echo "--- hand lag at $vel rad/s (sim_suite) ---"
  $PY diagnose_abs2.py sim_suite 2>&1 | grep -E 'right:|fwd|left :|up  ' | head -4
done
