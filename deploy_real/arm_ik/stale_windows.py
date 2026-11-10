#!/usr/bin/env python3
"""Per-5s summary of stale data and trigger presses in an arm_ik_dryrun live CSV."""
import csv
import sys

import numpy as np

rows = list(csv.DictReader(open(sys.argv[1])))
t = np.array([float(r["t"]) for r in rows])
stale = np.array([int(r["stale"]) for r in rows])
trig = np.array([float(r["trigger"]) for r in rows])
for w in np.arange(0, t[-1], 5.0):
    m = (t >= w) & (t < w + 5)
    print(f"{w:4.0f}-{w + 5:3.0f}s stale {stale[m].mean():4.0%}  trigger>0.6 {int((trig[m] > 0.6).sum()):3d} loops")
