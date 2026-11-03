#!/usr/bin/env python3
"""One line per simulator run, for the waist correction: tilt from the held waist pose (not from the corrected
command), size of the correction, waist pitch torque, wobble, bridge trip. Read only.
  waist_line2.py <sim_run tag> <label>"""
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
trip = trip.group(1).strip() if trip else "no trip"
if not rows:
    print(f"{label:34s} no samples at full weight; bridge: {trip}")
    sys.exit()
col = lambda k: np.array([float(r[k]) for r in rows])
parts = []
for j in ("pitch", "roll"):
    q, cmd = col(f"q_waist_{j}_joint"), col(f"cmd_waist_{j}_joint")
    hold = cmd[0]                       # the correction is 0 when the weight reaches 1
    tilt, corr = np.abs(hold - q), np.abs(cmd - hold)
    parts.append(f"{j} tilt max {tilt.max():.3f}/95% {np.percentile(tilt, 95):.3f} corr max {corr.max():.3f}")
q = col("q_waist_pitch_joint")
ma = np.convolve(q, np.ones(25) / 25, mode="same")
wob = np.std((q - ma)[25:-25]) * 1000 if len(q) > 60 else float("nan")   # 0.5 s high-pass, mrad RMS
dq = np.abs(np.diff(q)) / 0.02
tau = np.abs(col("tau_waist_pitch_joint"))
print(f"{label:34s} {parts[0]}; {parts[1]}; pitch torque max {tau.max():4.1f} Nm; wobble {wob:.1f} mrad, |dq| max {dq.max():.2f} rad/s; bridge: {trip}")
