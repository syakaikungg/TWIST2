#!/usr/bin/env python3
"""Absolute-mode check on the newest IK log: wanted hand (from the operator, scaled) vs IK hand, both relative
to the shoulders; direction agreement per axis; approach time after the grip. Usage: diagnose_abs.py [csv]"""
import csv
import glob
import math
import os
import re
import sys

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
path = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(f"{L}/arm_ik_*.csv"), key=os.path.getmtime)
txt = max(glob.glob(f"{L}/simik_*.txt"), key=os.path.getmtime)
out = open(txt).read()
m = re.search(r"robot reach ([\d.]+) m, scale ([\d.]+).*?robot shoulders (.*)", out)
reach, scale = float(m.group(1)), float(m.group(2))
sh = {s: np.array(eval(v)) for s, v in re.findall(r"(left|right) (\[[^\]]+\])", m.group(3))}
yaws = [math.radians(float(y)) for y in re.findall(r"head yaw (-?\d+) deg", out)]
rows = list(csv.DictReader(open(path)))
col = lambda k: np.array([float(r[k]) for r in rows])
t, f = col("t"), col("follow").astype(int)
head = np.stack([col(f"head_{a}") for a in "xyz"], 1)
print(f"log {os.path.basename(path)}; reach {reach} m, scale {scale}, shoulders {sh}")
starts = list(np.where(np.diff(f) == 1)[0] + 1)
for k, a in enumerate(starts):
    b = next((i for i in range(a, len(f)) if not f[i]), len(f))
    yw = yaws[k] if k < len(yaws) else 0.0
    Ry = np.array([[math.cos(yw), -math.sin(yw), 0], [math.sin(yw), math.cos(yw), 0], [0, 0, 1]])
    print(f"grip {k}: {t[a]:.1f}-{t[b - 1]:.1f}s, head yaw {math.degrees(yw):.0f} deg")
    for s in sh:
        c = np.stack([col(f"ctrl_{s}_{x}") for x in "xyz"], 1)[a:b]
        h = np.stack([col(f"hand_{s}_{x}") for x in "xyz"], 1)[a:b]
        centre = head[a:b] + (Ry @ np.array([-0.10, 0.0, -0.25]))
        op_sh = centre + Ry @ np.array([0.0, 0.19 if s == "left" else -0.19, 0.0])
        want = scale * ((c - op_sh) @ Ry)        # Rz(-yaw) applied to row vectors
        n = np.linalg.norm(want, axis=1, keepdims=True)
        want = np.where(n > 0.95 * reach, want * 0.95 * reach / n, want)
        got = h - sh[s]
        err = np.linalg.norm(want - got, axis=1)
        conv = next((i for i in range(len(err)) if err[i] < 0.03), None)
        tail = err[conv:] if conv is not None else err
        print(f"  {s}: wanted vs IK hand error at grip {err[0] * 100:.0f} cm -> within 3 cm after "
              f"{'never' if conv is None else f'{t[a + conv] - t[a]:.1f}s'}; afterwards median {np.median(tail) * 100:.1f} cm, "
              f"max {tail.max() * 100:.1f} cm")
        for i, ax in enumerate(("fwd", "left", "up")):
            if want[:, i].std() > 0.01:
                print(f"     {ax}: corr(wanted, IK hand) {np.corrcoef(want[:, i], got[:, i])[0, 1]:+.2f}")
