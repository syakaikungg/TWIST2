#!/usr/bin/env python3
"""Newest live run: when and why the IK program stopped, the state just before, and the PICO receive intervals
around it. Read only."""
import csv
import glob
import os
import time

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
r = list(csv.DictReader(open(max(glob.glob(f"{L}/arm_ik_live_*.csv"), key=os.path.getmtime))))
f = lambda k: np.array([float(x[k]) for x in r])
wall, est, age, fol, act, grip = f("wall"), f("estop"), f("stamp_age_ms"), f("follow"), f("active"), f("grip")
X, Y = f("X"), f("Y")
utc = lambda w: time.strftime("%H:%M:%S", time.gmtime(w)) + f".{int(w % 1 * 1000):03d}"
k = int(np.argmax(est > 0))
if est[k] == 0:
    print("no emergency stop in this run")
    raise SystemExit
print(f"estop at {utc(wall[k])} UTC; PICO data age then {age[k]:.0f} ms -> last PICO data at ~{utc(wall[k] - age[k] / 1000)} UTC")
j = max(0, k - 1)
print(f"just before: active {int(act[j])}, follow {int(fol[j])}, grip {grip[j]:.2f}, X {int(X[j])}, Y {int(Y[j])}")
# PICO arrivals: the data age drops when a new sample comes in.
arr = wall[1:][np.diff(age) < -5]
pre = arr[(arr > wall[k] - 10) & (arr <= wall[k])]
iv = np.diff(pre) * 1000
if len(iv):
    print(f"PICO receive interval in the 10 s before: median {np.median(iv):.0f} ms, max {iv.max():.0f} ms, "
          f">100 ms: {(iv > 100).sum()} times")
post = arr[arr > wall[k]]
if len(post):
    print(f"PICO data came back at {utc(post[0])} UTC (gap {(post[0] - (wall[k] - age[k] / 1000)):.1f} s)")
big = np.where(np.diff(arr) > 0.3)[0]
print("all PICO gaps > 0.3 s while running:", ", ".join(f"{utc(arr[i])} +{arr[i + 1] - arr[i]:.1f}s" for i in big[:12]) or "none")
