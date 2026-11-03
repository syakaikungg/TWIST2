#!/usr/bin/env python3
"""Newest live absolute-mode run: wanted hand (operator, scaled) vs IK hand vs simulated hand, relative to the
shoulders; median/95% errors, reach-clamp time, direction agreement, lags. Read only."""
import csv
import glob
import math
import os
import re
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.expanduser("~/nextech/arm_ik"))
import arm_sim  # noqa: E402  (same model build as the simulator)

L = os.path.expanduser("~/nextech_setup/logs")
newest = lambda p: max(glob.glob(f"{L}/{p}"), key=os.path.getmtime)
kind = sys.argv[1] if len(sys.argv) > 1 else "live"   # "live" or a scenario name, e.g. "sim_suite"
ik_pat = "arm_ik_live_*.csv" if kind == "live" else f"arm_ik_selftest_{kind}_*.csv"
ik_rows = list(csv.DictReader(open(newest(ik_pat))))
sim_rows = list(csv.DictReader(open(newest(f"arm_sim_{kind}_*.csv"))))
out = open(newest(f"simik_{kind}_*.txt")).read()
m = re.search(r"robot reach ([\d.]+) m, scale ([\d.]+).*?robot shoulders (.*)", out)
reach, scale = float(m.group(1)), float(m.group(2))
sh = {s: np.array(eval(v)) for s, v in re.findall(r"(left|right) (\[[^\]]+\])", m.group(3))}
yaw = math.radians(float(re.findall(r"head yaw (-?\d+) deg", out)[0]))
col = lambda k: np.array([float(r[k]) for r in ik_rows])
wall, f = col("wall"), col("follow").astype(int)
sel = f == 1
Ry = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
head = np.stack([col(f"head_{a}") for a in "xyz"], 1)

# Simulated hands from the logged simulator joint angles (same model as arm_sim.py).
sm = arm_sim.build_model(0.0, 0.05)
sd = mujoco.MjData(sm)
qadr = {n: sm.jnt_qposadr[mujoco.mj_name2id(sm, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in arm_sim.JOINT_NAMES}
hb = {s: mujoco.mj_name2id(sm, mujoco.mjtObj.mjOBJ_BODY, f"{s}_rubber_hand") for s in sh}
sb = {s: mujoco.mj_name2id(sm, mujoco.mjtObj.mjOBJ_BODY, f"{s}_shoulder_roll_link") for s in sh}
s_wall = np.array([float(r["wall"]) for r in sim_rows])
s_hand = {s: [] for s in sh}
for r in sim_rows:
    for n in arm_sim.JOINT_NAMES:
        sd.qpos[qadr[n]] = float(r[f"q_{n}"])
    mujoco.mj_kinematics(sm, sd)
    for s in sh:
        s_hand[s].append(sd.xpos[hb[s]] - sd.xpos[sb[s]])
idx = np.clip(np.searchsorted(s_wall, wall), 0, len(s_wall) - 1)


def lag(a, b, dt=0.02):
    best = max(range(0, 50), key=lambda L_: np.corrcoef(a[:len(a) - L_], b[L_:])[0, 1] if len(a) > L_ + 10 else -1)
    return best * dt


print(f"follow {sel.sum() * 0.02:.1f}s, head yaw {math.degrees(yaw):.0f} deg, reach {reach} m, scale {scale}")
for s in sh:
    c = np.stack([col(f"ctrl_{s}_{x}") for x in "xyz"], 1)
    op_sh = head + Ry @ np.array([-0.10, 0.0, -0.25]) + Ry @ np.array([0.0, 0.19 if s == "left" else -0.19, 0.0])
    want = scale * ((c - op_sh) @ Ry)
    n = np.linalg.norm(want, axis=1, keepdims=True)
    clamped = (n[:, 0] > 0.95 * reach)
    want = np.where(n > 0.95 * reach, want * 0.95 * reach / n, want)
    ik = np.stack([col(f"hand_{s}_{x}") for x in "xyz"], 1) - sh[s]
    sim = np.array(s_hand[s])[idx]
    w, i_, s_ = want[sel], ik[sel], sim[sel]
    e_ik, e_sim = np.linalg.norm(w - i_, axis=1), np.linalg.norm(w - s_, axis=1)
    print(f"{s}: wanted vs IK hand median {np.median(e_ik) * 100:.1f} / 95% {np.percentile(e_ik, 95) * 100:.1f} cm; "
          f"wanted vs simulated hand median {np.median(e_sim) * 100:.1f} / 95% {np.percentile(e_sim, 95) * 100:.1f} cm; "
          f"reach clamp {clamped[sel].mean() * 100:.0f}% of the follow time")
    for k, ax in enumerate(("fwd", "left", "up")):
        print(f"   {ax:4s}: corr wanted-IK {np.corrcoef(w[:, k], i_[:, k])[0, 1]:+.2f}, wanted-sim {np.corrcoef(w[:, k], s_[:, k])[0, 1]:+.2f}; "
              f"lag IK {lag(w[:, k], i_[:, k]):.2f}s, sim {lag(w[:, k], s_[:, k]):.2f}s")
