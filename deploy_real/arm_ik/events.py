#!/usr/bin/env python3
"""Today's live simulator runs: IK / bridge events, PICO data gaps > 0.1 s, and for each stop / B / gap the PICO
inputs logged in the 2 s before (A, B, X, Y, right grip, right trigger). Read only.  events.py [run tags...]"""
import csv
import glob
import os
import re
import sys
import time

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
utc = lambda w: time.strftime("%H:%M:%S", time.gmtime(w)) + f".{int(w % 1 * 10)}"
tags = sys.argv[1:] or [os.path.basename(p)[6:-4] for p in sorted(glob.glob(f"{L}/simik_live_*.txt"))
                        if time.time() - os.path.getmtime(p) < 3600]
for tag in tags:
    ik_txt, br_txt = f"{L}/simik_{tag}.txt", f"{L}/simbridge_{tag}.txt"
    t_ik = os.path.getmtime(ik_txt)
    ik_csv = min(glob.glob(f"{L}/arm_ik_live_*.csv"), key=lambda p: abs(os.path.getmtime(p) - t_ik))
    br_csv = min(glob.glob(f"{L}/arm_sdk_bridge_sim_*.csv"), key=lambda p: abs(os.path.getmtime(p) - os.path.getmtime(br_txt)))
    r = list(csv.DictReader(open(ik_csv)))
    f = lambda k: np.array([float(x[k]) for x in r])
    t, wall, age = f("t"), f("wall"), f("stamp_age_ms")
    extra = [k for k in ("lmenu", "rmenu", "lclick", "rclick") if k in r[0]]   # logged from 13:07 UTC on
    btn = {k: f(k) for k in ("A", "B", "X", "Y", *extra)}
    grip, trig, act, fol, est = f("grip"), f("trigger"), f("active"), f("follow"), f("estop")
    print(f"########## {tag}  ({utc(wall[0])} - {utc(wall[-1])} UTC)")
    ev = []
    for line in open(ik_txt):
        m = re.match(r"\[\s*([\d.]+)s\] (.*)", line)
        if m and not m.group(2).startswith("active="):
            ev.append((float(np.interp(float(m.group(1)), t, wall)), "IK     " + m.group(2)[:90]))
    b0 = time.mktime(time.strptime(os.path.basename(br_csv)[-19:-4], "%Y%m%d_%H%M%S"))
    for line in open(br_txt):
        m = re.match(r"\[\s*([\d.]+)s\] (.*)", line)
        if m:
            ev.append((b0 + float(m.group(1)), "bridge " + m.group(2)[:90] + "  (time +-1 s)"))
    bad = age > 100
    gaps, i = [], 0
    while i < len(age):
        if bad[i]:
            j = i
            while j + 1 < len(age) and bad[j + 1]:
                j += 1
            k = i + int(np.argmax(age[i:j + 1]))
            gaps.append((wall[k] - age[k] / 1000, age[k] / 1000))
            i = j + 1
        else:
            i += 1
    for s, d in gaps:
        ev.append((s, f"PICO   data gap {d:.2f} s"))
    for k in btn:
        for i in np.where(np.diff(btn[k]) > 0)[0]:
            ev.append((wall[i + 1], f"press  {k}"))
    for i in np.where(np.diff((grip > 0.5).astype(int)) != 0)[0]:
        ev.append((wall[i + 1], "grip   " + ("held" if grip[i + 1] > 0.5 else "released")))
    for w, s in sorted(ev):
        print(f"  {utc(w)}  {s}")
    print("  --- 2 s before each stop / B / gap >= 0.1 s ---")
    marks = [(w, s) for w, s in ev if "EMERGENCY" in s or "B ->" in s or s.startswith("PICO")]
    for w, s in marks:
        k = (wall >= w - 2.0) & (wall < w)
        if k.sum() == 0:
            continue
        pressed = [n for n in btn if btn[n][k].max() > 0]
        left = (f", left grip max {f('lgrip')[k].max():.2f} trigger max {f('ltrig')[k].max():.2f}" if extra else "")
        print(f"  {utc(w)} {s.strip()[:60]}: buttons {pressed or 'none'}, grip {grip[k].min():.2f}-{grip[k].max():.2f}, "
              f"trigger max {trig[k].max():.2f}{left}, active {int(act[k][-1])} follow {int(fol[k][-1])} estop {int(est[k][-1])}")
