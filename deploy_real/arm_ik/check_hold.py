#!/usr/bin/env python3
"""From the newest bridge log: target/command vs measured before the first grip, and after B."""
import csv
import glob
import os

import numpy as np

path = max(glob.glob(os.path.expanduser("~/nextech_setup/logs/arm_sdk_bridge_logonly_*.csv")), key=os.path.getmtime)
rows = list(csv.DictReader(open(path)))
names = [k[4:] for k in rows[0] if k.startswith("tgt_")]
arr = lambda r, p: np.array([float(r[p + n]) for n in names])
eng = [i for i, r in enumerate(rows) if r["state"] in ("RAMP_UP", "TRACK")]
first_follow = next((i for i in eng if rows[i]["follow"] == "1"), None)
pre = [i for i in eng if first_follow is None or i < first_follow]
print("log:", os.path.basename(path))
if pre:
    tg = np.array([arr(rows[i], "tgt_") for i in pre])
    cm = np.array([arr(rows[i], "cmd_") for i in pre])
    me = np.array([arr(rows[i], "meas_") for i in pre])
    w = [float(rows[i]["weight"]) for i in pre]
    print(f"enabled, before the grip: {len(pre)} loops ({float(rows[pre[0]]['t']):.1f}-{float(rows[pre[-1]]['t']):.1f}s), "
          f"weight {w[0]:.2f}->{w[-1]:.2f}")
    d_tm, d_cm = np.abs(tg - me).max(0), np.abs(cm - me).max(0)
    print(f"  max |target - measured| {d_tm.max():.4f} rad ({names[int(d_tm.argmax())]})")
    print(f"  max |command - measured| {d_cm.max():.4f} rad ({names[int(d_cm.argmax())]})")
    print(f"  command change over the window {np.abs(cm[-1] - cm[0]).max():.4f} rad")
if first_follow is not None:
    start = arr(rows[pre[0]], "meas_") if pre else None
    track = [i for i in eng if i >= first_follow]
    last = track[-1]
    print(f"after the grip: command moved up to {np.abs(arr(rows[last], 'cmd_') - start).max():.3f} rad from the start pose")
    rel = [i for i, r in enumerate(rows) if r["state"] == "RAMP_DOWN"]
    if rel:
        c0, c1 = arr(rows[rel[0]], "cmd_"), arr(rows[rel[-1]], "cmd_")
        w_back = next((float(rows[i]["weight"]) for i in rel if np.abs(arr(rows[i], "cmd_") - start).max() < 0.02), None)
        print(f"pause after B: |command - start pose| {np.abs(c0 - start).max():.4f} rad when RAMP_DOWN began, "
              f"{np.abs(c1 - start).max():.4f} rad at weight 0; back within 0.02 rad while weight was "
              f"{w_back if w_back is None else round(w_back, 2)} (reason '{rows[rel[0]]['reason']}')")
