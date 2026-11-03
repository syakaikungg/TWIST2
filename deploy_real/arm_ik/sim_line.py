#!/usr/bin/env python3
"""One line per simulator run: joint torque peaks, contact time, bridge trip; for reach_fwd also how far the
simulated hands overshoot the held target and how late they get there. Read only.
  sim_line.py <sim_run tag> <label>"""
import csv
import glob
import os
import re
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.expanduser("~/nextech/arm_ik"))
import arm_sim  # noqa: E402  (same model build as the simulator)

L = os.path.expanduser("~/nextech_setup/logs")
tag, label = sys.argv[1], sys.argv[2]
sc = tag.split("_0kg")[0]
rows = list(csv.DictReader(open(f"{L}/arm_sim_{tag}.csv")))
full = [r for r in rows if float(r["weight"]) > 0.9]
br = open(f"{L}/simbridge_{tag}.txt").read()
trip = re.search(r"EMERGENCY STOP: (tracking error[^(]*)", br)
trip = trip.group(1).strip() if trip else "no trip"
if not full:
    print(f"{label:40s} no samples at full weight; bridge: {trip}")
    sys.exit()
peak = lambda keys: max(abs(float(r[f"tau_{k}"])) for r in full for k in keys)
side = ("left", "right")
sh = peak([f"{s}_shoulder_{a}_joint" for s in side for a in ("pitch", "roll", "yaw")])
el = peak([f"{s}_elbow_joint" for s in side])
wr = peak([f"{s}_wrist_{a}_joint" for s in side for a in ("roll", "pitch", "yaw")])
wp = peak(["waist_pitch_joint"])
wall = np.array([float(r["wall"]) for r in rows])
dt = float(np.median(np.diff(wall)))
contact = sum(1 for r in full if float(r["contacts"] or 0) > 0) * dt
line = (f"{label:40s} torque max shoulder {sh:4.1f}/25, elbow {el:4.1f}/25, wrist {wr:3.1f}/5, waist pitch {wp:4.1f} Nm; "
        f"contact {contact:.2f}s; bridge: {trip}")

if sc == "reach_fwd":
    # Scenario time -> wall clock from the IK log of this run (newest; runs are sequential).
    ik = list(csv.DictReader(open(max(glob.glob(f"{L}/arm_ik_selftest_reach_fwd_*.csv"), key=os.path.getmtime))))
    it, iw = np.array([float(r["t"]) for r in ik]), np.array([float(r["wall"]) for r in ik])
    tsc = np.interp(wall, iw, it)
    m = arm_sim.build_model(0.0, 0.05)
    d = mujoco.MjData(m)
    qadr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in arm_sim.JOINT_NAMES}
    hb = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_rubber_hand") for s in side}
    hand = {s: [] for s in side}
    for r in rows:
        for n in arm_sim.JOINT_NAMES:
            d.qpos[qadr[n]] = float(r[f"q_{n}"])
        mujoco.mj_kinematics(m, d)
        for s in side:
            hand[s].append(d.xpos[hb[s]].copy())
    win = lambda a, b: (tsc >= a) & (tsc < b)
    parts = []
    for s in side:
        h = np.array(hand[s])
        out, back = np.median(h[win(9.0, 10.0)], 0), np.median(h[win(13.0, 13.5)], 0)
        span = np.linalg.norm(out - back)
        u = (out - back) / span
        p = (h - back) @ u                       # 0 = back pose, span = held forward pose
        over_out = max(0.0, p[win(6.5, 10.0)].max() - span)
        over_back = max(0.0, -p[win(11.5, 13.5)].min())
        k = np.where(win(5.0, 10.0) & (p >= 0.9 * span))[0]
        late = tsc[k[0]] - 7.0 if len(k) else float("nan")   # the scripted hand target is there at 7.0 s
        parts.append(f"{s[0].upper()} move {span * 100:.0f}cm overshoot {over_out * 100:.1f}/{over_back * 100:.1f}cm "
                     f"90% at {late:+.2f}s")
    line += "; " + ", ".join(parts)
print(line)
