#!/usr/bin/env python3
"""Summarise an arm_ik_dryrun live CSV: clutch/drill events, timing, joint-limit hits, hand motion per second."""
import csv
import sys

import numpy as np

LIMITS = {  # from GMR g1_mocap_29dof.xml
    "left_shoulder_pitch_joint": (-3.089, 1.149), "left_shoulder_roll_joint": (-0.6, 2.252),
    "left_shoulder_yaw_joint": (-1.4, 2.0), "left_elbow_joint": (-1.047, 1.7),
    "left_wrist_roll_joint": (-1.972, 1.972), "left_wrist_pitch_joint": (-1.614, 1.614),
    "left_wrist_yaw_joint": (-1.614, 1.614),
    "right_shoulder_pitch_joint": (-3.089, 1.149), "right_shoulder_roll_joint": (-2.252, 0.6),
    "right_shoulder_yaw_joint": (-2.0, 1.4), "right_elbow_joint": (-1.047, 1.7),
    "right_wrist_roll_joint": (-1.972, 1.972), "right_wrist_pitch_joint": (-1.614, 1.614),
    "right_wrist_yaw_joint": (-1.614, 1.614),
}

rows = list(csv.DictReader(open(sys.argv[1])))
t = np.array([float(r["t"]) for r in rows])
act = np.array([int(r["active"]) for r in rows])
drill = np.array([int(r["drill"]) for r in rows])
trig = np.array([float(r["trigger"]) for r in rows])
stale = np.array([int(r["stale"]) for r in rows])
ik = np.array([float(r["ik_ms"]) for r in rows])
loop = np.array([float(r["loop_ms"]) for r in rows])
hand = {s: np.array([[float(r[f"hand_{s}_{a}"]) for a in "xyz"] for r in rows]) for s in ("left", "right")}
q = {n: np.array([float(r[f"q_{n}"]) for r in rows]) for n in LIMITS}

def edges(x):
    d = np.diff(x)
    return t[1:][d == 1], t[1:][d == -1]

on, off = edges(act)
print(f"duration {t[-1]:.1f}s, {len(t)} loops ({len(t) / t[-1]:.1f} Hz)")
print(f"clutch: active ON at {np.round(on, 1).tolist()}  OFF at {np.round(off, 1).tolist()}  "
      f"active total {act.sum() / len(t) * t[-1]:.1f}s")
d_on, d_off = edges(drill)
print(f"drill: ON {len(d_on)} times at {np.round(d_on, 1).tolist()}, total {drill.sum() / len(t) * t[-1]:.1f}s; "
      f"trigger max {trig.max():.2f}; trigger>0.6 while paused: {int(((trig > 0.6) & (act == 0)).sum())} loops")
print(f"stale loops: {int(stale.sum())} ({stale.mean() * 100:.1f}%), stale during active: {int((stale & act).sum())}")
print(f"timing: ik avg {ik.mean():.2f} / max {ik.max():.2f} ms; loop avg {loop.mean():.2f} / p95 "
      f"{np.percentile(loop, 95):.2f} / max {loop.max():.2f} ms (budget 20 ms)")

print("joint ranges during the run (min..max) and limit hits (within 0.03 rad):")
for n, (lo, hi) in LIMITS.items():
    v = q[n]
    hits = int(((v < lo + 0.03) | (v > hi - 0.03)).sum())
    if v.max() - v.min() > 0.05 or hits:
        print(f"  {n:28s} {v.min():+.2f}..{v.max():+.2f}  limit [{lo:+.2f},{hi:+.2f}]  hits {hits}")

print("right hand motion per 2 s window (robot frame: x fwd, y left, z up), range in cm and dominant axis:")
h = hand["right"]
for w0 in np.arange(0, t[-1], 2.0):
    m = (t >= w0) & (t < w0 + 2.0)
    if m.sum() < 5:
        continue
    rng = (h[m].max(0) - h[m].min(0)) * 100
    seg = act[m].mean()
    dom = "xyz"[int(np.argmax(rng))] if rng.max() > 3 else "-"
    wr = q["right_wrist_roll_joint"][m]; wy = q["right_wrist_yaw_joint"][m]; wp = q["right_wrist_pitch_joint"][m]
    wrist = max(wr.max() - wr.min(), wy.max() - wy.min(), wp.max() - wp.min())
    print(f"  {w0:5.1f}-{w0 + 2:5.1f}s active {seg:.0%}  dx {rng[0]:5.1f} dy {rng[1]:5.1f} dz {rng[2]:5.1f}  "
          f"dominant {dom}  wrist-joint span {np.degrees(wrist):5.1f} deg")
lh = hand["left"]
print(f"left hand total range (cm): {np.round((lh.max(0) - lh.min(0)) * 100, 1).tolist()}")
end = {n: round(float(q[n][-1]), 2) for n in LIMITS if n.startswith("right")}
print(f"right arm at the end: {end}")
