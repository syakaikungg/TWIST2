#!/usr/bin/env python3
"""Analyse an arm_ik_dryrun live CSV (with A/B and controller columns) plus its stdout.

Usage: analyze_live2.py <csv> <stdout.txt>
"""
import csv
import math
import re
import sys

import numpy as np

LIMITS = {
    "right_shoulder_pitch_joint": (-3.089, 1.149), "right_shoulder_roll_joint": (-2.252, 0.6),
    "right_shoulder_yaw_joint": (-2.0, 1.4), "right_elbow_joint": (-1.047, 1.7),
    "right_wrist_roll_joint": (-1.972, 1.972), "right_wrist_pitch_joint": (-1.614, 1.614),
    "right_wrist_yaw_joint": (-1.614, 1.614),
    "left_shoulder_pitch_joint": (-3.089, 1.149), "left_shoulder_roll_joint": (-0.6, 2.252),
    "left_shoulder_yaw_joint": (-1.4, 2.0), "left_elbow_joint": (-1.047, 1.7),
    "left_wrist_roll_joint": (-1.972, 1.972), "left_wrist_pitch_joint": (-1.614, 1.614),
    "left_wrist_yaw_joint": (-1.614, 1.614),
}

rows = list(csv.DictReader(open(sys.argv[1])))
out = open(sys.argv[2]).read() if len(sys.argv) > 2 else ""
col = lambda k, f=float: np.array([f(r[k]) for r in rows])
t, act, drill, trig, stale = col("t"), col("active", int), col("drill", int), col("trigger"), col("stale", int)
A, B, ik, loop = col("A", int), col("B", int), col("ik_ms"), col("loop_ms")
hand = np.stack([col(f"hand_right_{a}") for a in "xyz"], 1)
ctrl = np.stack([col(f"ctrl_right_{a}") for a in "xyz"], 1)
q = {n: col(f"q_{n}") for n in LIMITS}

def rising(x):
    return t[1:][np.diff(x) == 1]

print("events from stdout:")
for line in out.splitlines():
    if re.search(r"A ->|B ->|stale|ignored|returning", line):
        print("  " + line.strip())
print(f"A presses at {np.round(rising(A), 1).tolist()}; B presses at {np.round(rising(B), 1).tolist()}")
print(f"active from {np.round(rising(act), 1).tolist()} to {np.round(t[1:][np.diff(act) == -1], 1).tolist()}, "
      f"total {act.mean() * t[-1]:.1f}s")
print(f"drill ON {len(rising(drill))} times, total {drill.mean() * t[-1]:.1f}s; "
      f"trigger>0.6 while paused {int(((trig > 0.6) & (act == 0)).sum())} loops")
print(f"stale loops {int(stale.sum())} ({stale.mean() * 100:.1f}%), during active {int((stale & act).sum())}")
print(f"timing: ik avg(active) {ik[act == 1].mean() if act.any() else 0:.2f} / max {ik.max():.2f} ms; "
      f"loop avg {loop.mean():.2f} / p95 {np.percentile(loop, 95):.2f} / max {loop.max():.2f} ms")

print("joint ranges and limit hits (within 0.03 rad):")
for n, (lo, hi) in LIMITS.items():
    v = q[n]
    hits = int(((v < lo + 0.03) | (v > hi - 0.03)).sum())
    if v.max() - v.min() > 0.05 or hits:
        print(f"  {n:28s} {v.min():+.2f}..{v.max():+.2f}  limit [{lo:+.2f},{hi:+.2f}]  hits {hits}")

# Direction check per active segment: controller motion (rotated into the operator's facing direction
# at engage, from the stdout yaw) vs hand motion, both relative to the engage instant.
yaws = [math.radians(float(m)) for m in re.findall(r"ACTIVE \(head yaw (-?\d+) deg\)", out)]
starts = np.where(np.diff(act) == 1)[0] + 1
ends = np.where(np.diff(act) == -1)[0] + 1
for k, s in enumerate(starts):
    e = ends[ends > s][0] if (ends > s).any() else len(t)
    yaw = yaws[k] if k < len(yaws) else 0.0
    c, sn = math.cos(-yaw), math.sin(-yaw)
    Rz = np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]])
    dc = (ctrl[s:e] - ctrl[s]) @ Rz.T
    dh = hand[s:e] - hand[s]
    print(f"segment {k}: {t[s]:.1f}-{t[e - 1]:.1f}s, head yaw {math.degrees(yaw):.0f} deg")
    for i, a in enumerate("xyz"):
        if dc[:, i].std() < 0.01:
            print(f"  {a}: controller hardly moved (std {dc[:, i].std() * 100:.1f} cm)")
            continue
        corr = np.corrcoef(dc[:, i], dh[:, i])[0, 1]
        gain = np.polyfit(dc[:, i], dh[:, i], 1)[0]
        print(f"  {a}: controller range {np.ptp(dc[:, i]) * 100:5.1f} cm, hand range {np.ptp(dh[:, i]) * 100:5.1f} cm, "
              f"corr {corr:+.2f}, gain {gain:+.2f}")
    w = {n: np.ptp(q[n][s:e]) for n in ("right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint")}
    print("  wrist joint spans (deg): " + ", ".join(f"{n.split('_')[2]} {math.degrees(v):.0f}" for n, v in w.items()))

print("per 3 s window (robot axes, cm): controller range vs hand range")
for w0 in np.arange(0, t[-1], 3.0):
    m = (t >= w0) & (t < w0 + 3)
    if m.sum() < 5:
        continue
    cr, hr = np.ptp(ctrl[m], 0) * 100, np.ptp(hand[m], 0) * 100
    print(f"  {w0:4.0f}s act {act[m].mean():.0%} ctrl dx{cr[0]:5.1f} dy{cr[1]:5.1f} dz{cr[2]:5.1f} | "
          f"hand dx{hr[0]:5.1f} dy{hr[1]:5.1f} dz{hr[2]:5.1f}")
print("right arm at end:", {n.split('_', 1)[1]: round(float(q[n][-1]), 2) for n in LIMITS if n.startswith("right")})
