#!/bin/bash
# Wait (up to 4 min) for the live simulator run to finish, then summarise it. Read only.
for i in $(seq 1 240); do
  pgrep -f 'arm_sim.py' > /dev/null || pgrep -f 'arm_ik_dryrun.py' > /dev/null || break
  sleep 1
done
~/miniforge3/envs/gmr/bin/python ~/nextech/arm_ik/analyze_live_sim.py
