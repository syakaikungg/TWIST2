#!/bin/bash
# Wait for the live simulator run to finish, then: events/summary, absolute-mode tracking, loop timings.
for i in $(seq 1 240); do
  pgrep -f 'arm_sim.py' > /dev/null || pgrep -f 'arm_ik_dryrun.py' > /dev/null || break
  sleep 1
done
PY=~/miniforge3/envs/gmr/bin/python
cd ~/nextech/arm_ik
$PY analyze_live_sim.py
echo "--- absolute-mode tracking ---"
$PY diagnose_abs.py "$(ls -t ~/nextech_setup/logs/arm_ik_live_*.csv | head -1)"
echo "--- loop timings ---"
python3 - <<'EOF'
import csv, glob, os
import numpy as np
L = os.path.expanduser("~/nextech_setup/logs")
newest = lambda p: max(glob.glob(f"{L}/{p}"), key=os.path.getmtime)
ik = list(csv.DictReader(open(newest("arm_ik_live_*.csv"))))
lp = np.array([float(r["loop_ms"]) for r in ik]); ikm = np.array([float(r["ik_ms"]) for r in ik])
print(f"IK loop: avg {lp.mean():.1f} / p95 {np.percentile(lp, 95):.1f} / max {lp.max():.1f} ms; IK solve max {ikm.max():.1f} ms")
br = list(csv.DictReader(open(newest("arm_sdk_bridge_sim_*.csv"))))
t = np.array([float(r["t"]) for r in br]); dt = np.diff(t) * 1000
print(f"bridge loop period: avg {dt.mean():.1f} / p95 {np.percentile(dt, 95):.1f} / max {dt.max():.1f} ms (target 20)")
sim = list(csv.DictReader(open(newest("arm_sim_live_*.csv"))))
w = np.array([float(r["wall"]) for r in sim]); st = np.array([float(r["t"]) for r in sim])
print(f"sim: {st[-1] / (w[-1] - w[0]):.2f}x real time, record period max {np.diff(w).max() * 1000:.0f} ms")
EOF
