#!/bin/bash
# Before/after comparison of IK collision avoidance in the simulator, then 2.0 kg at 5/10/15 cm. No robot I/O.
cd ~/nextech/arm_ik || exit 1
out() { grep -E '===|EMERGENCY STOP: track|timing|collision margin|simulated|video:|weight [0-9]|contact'; }
echo "##### collision avoidance OFF (default monitors)"
bash sim_run.sh sim_suite 0 0 1.0 "" "--collision-margin 0" 2>&1 | out
echo "##### collision avoidance ON 4 cm (default monitors)"
bash sim_run.sh sim_suite 0 1 1.0 "" "" 2>&1 | out
for off in 0.05 0.10 0.15; do
  echo "##### 2.0 kg at ${off} m, collision avoidance ON"
  bash sim_run.sh sim_suite 2.0 0 1.0 "" "" $off 2>&1 | out
done
