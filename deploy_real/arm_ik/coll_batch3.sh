#!/bin/bash
# Collision avoidance v3 (margin back to the start value on B). 0 kg with video, 2.0 kg at 0.15 m. No robot I/O.
cd ~/nextech/arm_ik || exit 1
out() { grep -E '===|EMERGENCY STOP|released|RAMP_DOWN|timing|collision margin|B ->|start pose|video:|weight [0-9]|contact'; }
echo "##### v3, 0 kg"
bash sim_run.sh sim_suite 0 1 1.0 "" "--max-vel 1.0" 2>&1 | out
grep -E 'B ->|start pose reached' ~/nextech_setup/logs/simik_sim_suite_0kg_*.txt | tail -2
echo "##### v3, 2.0 kg at 0.15 m"
bash sim_run.sh sim_suite 2.0 0 1.0 "" "--max-vel 1.0" 0.15 2>&1 | out
python3 - <<'EOF'
import csv, glob, os
p = max(glob.glob(os.path.expanduser("~/nextech_setup/logs/arm_ik_selftest_sim_suite_*.csv")), key=os.path.getmtime)
rows = list(csv.DictReader(open(p)))
lp = [float(r["loop_ms"]) for r in rows]
i = max(range(len(lp)), key=lambda k: lp[k])
print(f"slowest IK loop {lp[i]:.0f} ms at t={rows[i]['t']}s; loops over 20 ms: {sum(x > 20 for x in lp)} of {len(lp)}")
EOF
