#!/bin/bash
# v3 settings, 2.0 kg at 0.05 m and 0.10 m: right-wrist torque maxima and contact totals. No robot I/O.
cd ~/nextech/arm_ik || exit 1
for off in 0.05 0.10; do
  bash sim_run.sh sim_suite 2.0 0 1.0 "" "--max-vel 1.0" $off > /tmp/cb4.txt 2>&1
  tag=$(grep -o 'sim_suite_2.0kg_off[0-9.]*_[0-9]*' /tmp/cb4.txt | head -1)
  python3 - "$tag" <<'EOF'
import csv, os, sys
tag = sys.argv[1]
rows = list(csv.DictReader(open(os.path.expanduser(f"~/nextech_setup/logs/arm_sim_{tag}.csv"))))
mx = lambda k: max(abs(float(r[k])) for r in rows)
pairs = {}
for r in rows:
    for p in filter(None, r["contact_pairs"].split(";")):
        pairs[p] = pairs.get(p, 0) + 0.02
print(f"{tag}: right wrist pitch max {mx('tau_right_wrist_pitch_joint'):.2f} Nm, yaw max {mx('tau_right_wrist_yaw_joint'):.2f} Nm, "
      f"right shoulder roll max {mx('tau_right_shoulder_roll_joint'):.1f} Nm; contacts total {sum(pairs.values()):.2f}s {dict((k, round(v, 2)) for k, v in pairs.items())}")
EOF
  grep -E 'EMERGENCY STOP: track' /tmp/cb4.txt
done
