#!/usr/bin/env python3
"""Summarise the newest live simulator run: IK events, bridge states, sim torques/contacts/tracking, timing."""
import csv
import glob
import os
import re

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
newest = lambda pat: max(glob.glob(f"{L}/{pat}"), key=os.path.getmtime)
ik_txt, br_txt, sim_txt = newest("simik_live_*.txt"), newest("simbridge_live_*.txt"), newest("sim_live_*.txt")
print("logs:", os.path.basename(ik_txt), os.path.basename(br_txt), os.path.basename(sim_txt))
print("--- IK events ---")
for line in open(ik_txt):
    if re.search(r"A ->|A ignored|A pressed|grip|B ->|start pose|EMERGENCY|timing", line):
        print("  " + line.strip())
print("--- bridge events ---")
for line in open(br_txt):
    if re.search(r"RAMP|TRACK|released|EMERGENCY|STOPPED", line):
        print("  " + line.strip())
print("--- simulator summary ---")
txt = open(sim_txt).read()
print(txt[txt.find("simulated"):].strip())

ik_csv = newest("arm_ik_live_*.csv")
rows = list(csv.DictReader(open(ik_csv)))
f = np.array([int(r["follow"]) for r in rows])
dr = np.array([int(r["drill"]) for r in rows])
t = np.array([float(r["t"]) for r in rows])
print(f"--- IK log ---\nfollow time {f.mean() * t[-1]:.1f}s, grip presses {int((np.diff(f) == 1).sum())}, "
      f"drill ON {int((np.diff(dr) == 1).sum())} times ({dr.mean() * t[-1]:.1f}s)")
lp = np.array([float(r["loop_ms"]) for r in rows])
print(f"IK loop avg {lp.mean():.1f} ms, p95 {np.percentile(lp, 95):.1f} ms, max {lp.max():.0f} ms, over 20 ms: {(lp > 20).sum()}")
