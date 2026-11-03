#!/bin/bash
# Collision avoidance v2 (margin ratchets up to 4 cm; IK speed matched to the bridge's 1 rad/s). No robot I/O.
cd ~/nextech/arm_ik || exit 1
out() { grep -E '===|EMERGENCY STOP: track|timing|collision margin|video:|weight [0-9]|contact'; }
echo "##### v2, 0 kg"
bash sim_run.sh sim_suite 0 1 1.0 "" "--max-vel 1.0" 2>&1 | out
echo "##### v2, 2.0 kg at 0.10 m"
bash sim_run.sh sim_suite 2.0 0 1.0 "" "--max-vel 1.0" 0.10 2>&1 | out
python3 - <<'EOF'
import csv, glob, os
p = max(glob.glob(os.path.expanduser("~/nextech_setup/logs/arm_ik_selftest_sim_suite_*.csv")), key=os.path.getmtime)
rows = list(csv.DictReader(open(p)))
lp = [float(r["loop_ms"]) for r in rows]
i = max(range(len(lp)), key=lambda k: lp[k])
print(f"slowest IK loop {lp[i]:.0f} ms at t={rows[i]['t']}s (loop {i} of {len(lp)}); loops over 20 ms: {sum(x > 20 for x in lp)}")
EOF
