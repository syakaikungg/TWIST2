#!/usr/bin/env python3
"""Largest per-step command change and start offset in the newest bridge log-only CSV."""
import csv
import glob
import os

import numpy as np

path = max(glob.glob(os.path.expanduser("~/nextech_setup/logs/arm_sdk_bridge_logonly_*.csv")), key=os.path.getmtime)
paths = sorted(glob.glob(os.path.expanduser("~/nextech_setup/logs/arm_sdk_bridge_logonly_*.csv")), key=os.path.getmtime)[-6:]
for p in paths:
    rows = [r for r in csv.DictReader(open(p)) if r["state"] in ("RAMP_UP", "TRACK", "RAMP_DOWN", "ESTOP_RAMP")]
    if not rows:
        continue
    cmd_keys = [k for k in rows[0] if k.startswith("cmd_")]
    meas_keys = [k for k in rows[0] if k.startswith("meas_")]
    c = np.array([[float(r[k]) for k in cmd_keys] for r in rows])
    m0 = np.array([float(rows[0][k]) for k in meas_keys])
    w = np.array([float(r["weight"]) for r in rows])
    print(f"{os.path.basename(p)}: max step {np.abs(np.diff(c, axis=0)).max():.4f} rad/loop, "
          f"first cmd vs measured {np.abs(c[0] - m0).max():.4f} rad, max weight step {np.abs(np.diff(w)).max():.4f}")
