#!/usr/bin/env python3
"""Map simulator contacts and torque peaks to the IK scenario phases (by wall clock). Usage: analyze_sim.py <tag>"""
import csv
import glob
import os
import sys

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
tag = sys.argv[1]
sim = list(csv.DictReader(open(f"{L}/arm_sim_{tag}.csv")))
ik_path = max(glob.glob(f"{L}/arm_ik_selftest_sim_suite_*.csv"), key=os.path.getmtime)
ik = list(csv.DictReader(open(ik_path)))
ik_t0 = float(ik[0]["wall"]) - float(ik[0]["t"])
PH = [("start", 0.0, 5.0), ("forward", 5.0, 7.0), ("up", 7.0, 9.0), ("left", 9.0, 11.0), ("twist", 11.0, 13.0),
      ("together", 13.0, 16.0), ("to_torso", 16.0, 19.0), ("hold/B", 19.0, 30.0)]


def phase(wall):
    t = wall - ik_t0
    return next((n for n, a, b in PH if a <= t < b), "after" if t >= 0 else "before IK")


names = [k[4:] for k in sim[0] if k.startswith("tau_")]
lim = {n: (5.0 if ("wrist_pitch" in n or "wrist_yaw" in n) else (25.0 if any(k in n for k in ("shoulder", "elbow", "wrist")) else 50.0)) for n in names}
print(f"ik log {os.path.basename(ik_path)}; sim log arm_sim_{tag}.csv")
by_phase = {}
for r in sim:
    p = phase(float(r["wall"]))
    e = by_phase.setdefault(p, {"rows": 0, "contacts": {}, "tau": {n: 0.0 for n in names}, "w": []})
    e["rows"] += 1
    e["w"].append(float(r["weight"]))
    for pair in filter(None, r["contact_pairs"].split(";")):
        e["contacts"][pair] = e["contacts"].get(pair, 0) + 1
    for n in names:
        e["tau"][n] = max(e["tau"][n], abs(float(r[f"tau_{n}"])))
for p, *_ in PH + [("after", 0, 0), ("before IK", 0, 0)]:
    if p not in by_phase:
        continue
    e = by_phase[p]
    top = sorted(e["tau"].items(), key=lambda kv: -kv[1] / lim[kv[0]])[:3]
    print(f"{p:10s} weight {min(e['w']):.2f}-{max(e['w']):.2f}  top torque " +
          ", ".join(f"{n.replace('_joint', '')} {v:.1f}/{lim[n]:.0f}" for n, v in top))
    for pair, k in sorted(e["contacts"].items(), key=lambda kv: -kv[1]):
        print(f"           contact {pair}: {k * 0.02:.2f}s")
