#!/bin/bash
# IK self-test for 25 s (no Redis, no robot I/O) and count slow loops.
cd ~/nextech/arm_ik || exit 1
~/miniforge3/envs/gmr/bin/python arm_ik_dryrun.py --selftest --duration 25 2>&1 | grep timing
python3 - <<'EOF'
import csv, glob, os
p = max(glob.glob(os.path.expanduser("~/nextech_setup/logs/arm_ik_selftest_none_*.csv")), key=os.path.getmtime)
rows = list(csv.DictReader(open(p)))
print("loops over 20 ms:", [(r["t"], r["loop_ms"]) for r in rows if float(r["loop_ms"]) > 20])
EOF
