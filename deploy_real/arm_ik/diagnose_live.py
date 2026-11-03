#!/usr/bin/env python3
"""Why did the live (simulator) teleop feel bad? Controller motion vs IK hand vs bridge command, per axis."""
import csv
import glob
import math
import os
import re

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
newest = lambda pat: max(glob.glob(f"{L}/{pat}"), key=os.path.getmtime)
ik_csv, ik_txt = newest("arm_ik_live_*.csv"), newest("simik_live_*.txt")
br_csv = newest("arm_sdk_bridge_sim_*.csv")
rows = list(csv.DictReader(open(ik_csv)))
col = lambda k: np.array([float(r[k]) for r in rows])
t, f, stale = col("t"), col("follow").astype(int), col("stale").astype(int)
yaw = [float(y) for y in re.findall(r"head yaw (-?\d+) deg", open(ik_txt).read())]
print("IK log", os.path.basename(ik_csv), "| head yaw at grip(s):", yaw)
segs = []
s = None
for i in range(len(f)):
    if f[i] and s is None:
        s = i
    if not f[i] and s is not None:
        segs.append((s, i)); s = None
if s is not None:
    segs.append((s, len(f)))
dt = np.median(np.diff(t))
for k, (a, b) in enumerate(segs):
    yw = math.radians(yaw[k] if k < len(yaw) else 0.0)
    Rz = np.array([[math.cos(-yw), -math.sin(-yw), 0], [math.sin(-yw), math.cos(-yw), 0], [0, 0, 1]])
    print(f"grip segment {k}: {t[a]:.1f}-{t[b - 1]:.1f}s ({t[b - 1] - t[a]:.1f}s)")
    for side in ("right", "left"):
        c = np.stack([col(f"ctrl_{side}_{x}") for x in "xyz"], 1)[a:b]
        h = np.stack([col(f"hand_{side}_{x}") for x in "xyz"], 1)[a:b]
        dc = (c - c[0]) @ Rz.T
        want = dc.copy()
        n = np.linalg.norm(want, axis=1)
        clamp = n > 0.4
        want[clamp] *= (0.4 / n[clamp])[:, None]
        dh = h - h[0]
        print(f"  {side} hand: controller moved {np.ptp(dc, 0).round(2) * 100} cm (x fwd, y left, z up); "
              f"IK hand moved {np.ptp(dh, 0).round(2) * 100} cm; reach clamp {clamp.mean() * 100:.0f}% of the time")
        for i, ax in enumerate("xyz"):
            if dc[:, i].std() < 0.01:
                continue
            corr = np.corrcoef(dc[:, i], dh[:, i])[0, 1]
            lags = range(0, int(1.5 / dt))
            cc = [np.corrcoef(dc[:len(dc) - L_, i], dh[L_:, i])[0, 1] for L_ in lags]
            best = int(np.nanargmax(cc))
            gap = np.abs(want[:, i] - dh[:, i])
            print(f"     {ax}: corr {corr:+.2f}, best when IK lags {best * dt:.2f}s (corr {cc[best]:+.2f}), "
                  f"IK hand short of the wanted offset by >5 cm {np.mean(gap > 0.05) * 100:.0f}% of the time "
                  f"(max {gap.max() * 100:.0f} cm)")
print(f"stale PICO data during grip: {int((stale & f).sum())} loops; IK loop max {col('loop_ms').max():.0f} ms")
b = list(csv.DictReader(open(br_csv)))
tg = np.array([[float(r[k]) for k in r if k.startswith("tgt_")] for r in b if r["state"] in ("TRACK",)])
cm = np.array([[float(r[k]) for k in r if k.startswith("cmd_")] for r in b if r["state"] in ("TRACK",)])
if len(tg):
    lag = np.abs(tg - cm).max(1)
    print(f"bridge (TRACK): command behind the IK angles by >0.05 rad {np.mean(lag > 0.05) * 100:.0f}% of the time, "
          f"max {lag.max():.2f} rad (rate limit 0.5 rad/s)")
