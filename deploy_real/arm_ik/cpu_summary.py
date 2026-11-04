#!/usr/bin/env python3
"""Summarise the newest cpu_sim_live_*.txt (cpu_sampler.sh): CPU % per process role (median / max), whole-machine
busy %, optionally before vs after a UTC time.  cpu_summary.py [HH:MM:SS split]. Read only."""
import glob
import os
import re
import sys

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
path = max(glob.glob(f"{L}/cpu_sim_live_*.txt"), key=os.path.getmtime)
split = sys.argv[1] if len(sys.argv) > 1 else None
ROLES = [("arm_sim.py", "simulator"), ("arm_sdk_bridge", "bridge"), ("arm_ik_dryrun", "IK"),
         ("stream_sim", "viewer stream"), ("simcam", "camera (simcam)"), ("cam_to_pico", "camera (cam_to_pico)")]
samples, args = [], {}
t = None
for line in open(path):
    m = re.match(r"== (\d\d:\d\d:\d\d) UTC", line)
    if m:
        t = m.group(1)
        samples.append({"t": t, "busy": None, "pid": {}})
        continue
    if not samples:
        continue
    if line.startswith("%Cpu"):
        idle = float(re.search(r"([\d.]+) id", line).group(1))
        samples[-1]["busy"] = 100 - idle
        continue
    f = line.split()
    if len(f) >= 12 and f[0].isdigit() and "python" in f[11]:
        samples[-1]["pid"][f[0]] = float(f[8])
    elif len(f) >= 2 and f[0].isdigit():
        args[f[0]] = " ".join(f[1:])
role_of = {p: next((r for k, r in ROLES if k in a), None) for p, a in args.items()}
print(f"{os.path.basename(path)}: {len(samples)} samples, {samples[0]['t']}-{samples[-1]['t']} UTC")
groups = [("all", samples)] if not split else [(f"before {split}", [s for s in samples if s["t"] < split]),
                                               (f"from {split}", [s for s in samples if s["t"] >= split])]
for name, ss in groups:
    if not ss:
        continue
    busy = [s["busy"] for s in ss if s["busy"] is not None]
    out = [f"machine busy {np.median(busy):.0f}% (max {max(busy):.0f}%) of 8 cores"]
    for _, r in ROLES:
        v = [c for s in ss for p, c in s["pid"].items() if role_of.get(p) == r]
        if v:
            out.append(f"{r} {np.median(v):.0f}% (max {max(v):.0f}%)")
    print(f"  {name} ({len(ss)} samples): " + ", ".join(out))
