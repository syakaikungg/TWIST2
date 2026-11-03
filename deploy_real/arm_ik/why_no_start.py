#!/usr/bin/env python3
"""Newest live IK log: did any button / grip / trigger register, was the PICO data fresh, did poses change?"""
import csv
import glob
import os

import numpy as np

p = max(glob.glob(os.path.expanduser("~/nextech_setup/logs/arm_ik_live_*.csv")), key=os.path.getmtime)
rows = list(csv.DictReader(open(p)))
col = lambda k: np.array([float(r[k]) for r in rows])
print("log", os.path.basename(p), "loops", len(rows))
for k in ("A", "B", "X", "Y"):
    print(f"  {k}: pressed in {int(col(k).sum())} loops")
print(f"  grip>0.6 in {int((col('grip') > 0.6).sum())} loops, trigger>0.6 in {int((col('trigger') > 0.6).sum())} loops")
print(f"  stale in {int(col('stale').sum())} loops ({col('stale').mean() * 100:.0f}%), head_ok {int(col('head_ok').sum())}, "
      f"right_ok {int(col('right_ok').sum())}; stamp age max {col('stamp_age_ms').max():.0f} ms")
for k in ("head", "ctrl_right", "ctrl_left"):
    xyz = np.stack([col(f"{k}_{a}") for a in "xyz"], 1)
    print(f"  {k} moved (range) {np.ptp(xyz, 0).round(3).tolist()} m")
