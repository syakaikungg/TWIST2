#!/usr/bin/env python3
"""One line per simulator run: waist pitch torque / tilt (command - simulated) and whether the bridge tripped."""
import csv
import os
import re
import sys

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
tag, label = sys.argv[1], sys.argv[2]
rows = [r for r in csv.DictReader(open(f"{L}/arm_sim_{tag}.csv")) if float(r["weight"]) > 0.9]
br = open(f"{L}/simbridge_{tag}.txt").read()
trip = re.search(r"EMERGENCY STOP: (tracking error[^(]*)", br)
if not rows:
    print(f"{label:34s} no samples at full weight; bridge: {trip.group(1) if trip else 'no trip'}")
    sys.exit()
tau = np.abs([float(r["tau_waist_pitch_joint"]) for r in rows])
tilt = np.abs([float(r["cmd_waist_pitch_joint"]) - float(r["q_waist_pitch_joint"]) for r in rows])
roll = np.abs([float(r["cmd_waist_roll_joint"]) - float(r["q_waist_roll_joint"]) for r in rows])
sh = max(abs(float(r[k])) for r in rows for k in ("tau_left_shoulder_pitch_joint", "tau_right_shoulder_pitch_joint"))
print(f"{label:34s} waist pitch torque max {tau.max():5.1f} / 95% {np.percentile(tau, 95):5.1f} Nm, "
      f"tilt max {tilt.max():.3f} / 95% {np.percentile(tilt, 95):.3f} rad, roll tilt max {roll.max():.3f}; "
      f"shoulder pitch max {sh:.1f} Nm; bridge: {trip.group(1).strip() if trip else 'no trip'}")
