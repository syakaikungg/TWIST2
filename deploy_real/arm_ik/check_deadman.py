#!/usr/bin/env python3
"""Check the dead-man scenario log: drill only while grip+trigger, arms frozen while the grip is released."""
import csv
import glob
import os
import sys

import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.expanduser(
    "~/nextech_setup/logs/arm_ik_selftest_deadman_*.csv")), key=os.path.getmtime)
rows = list(csv.DictReader(open(path)))
t = np.array([float(r["t"]) for r in rows])
f = np.array([int(r["follow"]) for r in rows])
dr = np.array([int(r["drill"]) for r in rows])
tr = np.array([float(r["trigger"]) for r in rows])
q = np.array([[float(r[k]) for k in r if k.startswith("q_right")] for r in rows])
print("log:", path)
for name, t0, t1 in [("grip released, trigger pressed", 3.9, 4.5), ("grip held, trigger pressed", 5.6, 6.2),
                     ("grip released, no trigger", 3.6, 4.9)]:
    m = (t >= t0) & (t < t1)
    print(f"{name:32s} {t0}-{t1}s: follow={f[m].max()} drill max={dr[m].max()} trigger max={tr[m].max():.1f} "
          f"right-arm joint change {np.abs(np.diff(q[m], axis=0)).max():.4f} rad/step")
m = (t >= 1.6) & (t < 3.4)
print(f"{'grip held, hand drifting':32s} 1.6-3.4s: right-arm joint change max {np.abs(np.diff(q[m], axis=0)).max():.4f} rad/step")
