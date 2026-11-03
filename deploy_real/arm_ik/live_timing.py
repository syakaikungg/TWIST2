#!/usr/bin/env python3
"""Newest live simulator run, in 20 s bins (UTC): IK loop / IK solve time, bridge period, simulator real-time
ratio and how far physics fell behind, PICO data age / stale. Read only.  live_timing.py [bin_s]"""
import csv
import glob
import os
import time

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
B = float(os.sys.argv[1]) if len(os.sys.argv) > 1 else 20.0
newest = lambda p: max(glob.glob(f"{L}/{p}"), key=os.path.getmtime)
rd = lambda p: list(csv.DictReader(open(p)))
ik, sim = rd(newest("arm_ik_live_*.csv")), rd(newest("arm_sim_live_*.csv"))
bp = newest("arm_sdk_bridge_sim_*.csv")
br = rd(bp)
f = lambda rows, k: np.array([float(r[k]) for r in rows])
iw, loop, ikms, stale, sage = f(ik, "wall"), f(ik, "loop_ms"), f(ik, "ik_ms"), f(ik, "stale"), f(ik, "stamp_age_ms")
sw, st = f(sim, "wall"), f(sim, "t")
slag = (sw - sw[0]) - (st - st[0])
bt = f(br, "t") + time.mktime(time.strptime(os.path.basename(bp)[-19:-4], "%Y%m%d_%H%M%S"))
bper = np.r_[np.nan, np.diff(f(br, "t")) * 1000]
print(f"IK loops {len(ik)}, sim rows {len(sim)}, bridge rows {len(br)}; whole run: IK loop p95 {np.percentile(loop, 95):.1f} / max {loop.max():.1f} ms, "
      f"bridge period max {np.nanmax(bper):.0f} ms, physics behind max {slag.max() * 1000:.0f} ms, PICO stale {stale.mean() * 100:.1f}% of loops")
print("UTC bin   | IK loop avg/max ms | IK solve avg/max | bridge period max | sim RT ratio, behind max | PICO age med/max ms, stale %")
t0 = np.floor(iw[0] / B) * B
while t0 < iw[-1]:
    a, b = t0, t0 + B
    ki, ks, kb = (iw >= a) & (iw < b), (sw >= a) & (sw < b), (bt >= a) & (bt < b)
    if ki.sum() > 5 and ks.sum() > 5:
        i0, i1 = np.where(ks)[0][[0, -1]]
        ratio = (st[i1] - st[i0]) / (sw[i1] - sw[i0])
        bmax = np.nanmax(bper[kb]) if kb.sum() > 1 else float("nan")
        print(f"{time.strftime('%H:%M:%S', time.gmtime(a))}  | {loop[ki].mean():5.1f} / {loop[ki].max():5.1f}      | {ikms[ki].mean():4.1f} / {ikms[ki].max():4.1f}  | "
              f"{bmax:5.0f}             | {ratio:.3f}, {slag[ks].max() * 1000:4.0f} ms          | {np.median(sage[ki]):5.0f} / {sage[ki].max():6.0f}, {stale[ki].mean() * 100:4.1f}")
    t0 = b
