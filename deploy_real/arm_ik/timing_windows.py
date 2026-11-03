#!/usr/bin/env python3
"""IK loop time, bridge period and simulator real-time ratio inside given UTC windows vs the rest of the same
runs (first/last 2 s of each run left out). Read only.
  timing_windows.py "2026-11-03 12:24:56.088" "2026-11-03 12:24:58.173" [more start/end pairs]"""
import calendar
import csv
import glob
import os
import sys
import time
from datetime import datetime

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
utc = lambda s: calendar.timegm(datetime.strptime(s, "%Y-%m-%d %H:%M:%S.%f").timetuple()) + float("0." + s.split(".")[1])
wins = [(utc(a), utc(b)) for a, b in zip(sys.argv[1::2], sys.argv[2::2])]
lo, hi = min(w[0] for w in wins) - 60, max(w[1] for w in wins) + 60


def rows_of(path):
    return list(csv.DictReader(open(path)))


def split(wall, w0, w1):
    inside = (wall >= w0) & (wall <= w1)
    rest = ~inside & (wall > wall[0] + 2) & (wall < wall[-1] - 2)
    return inside, rest


for w0, w1 in wins:
    print(f"=== window {time.strftime('%H:%M:%S', time.gmtime(w0))}-{time.strftime('%H:%M:%S', time.gmtime(w1))} UTC ===")
    for p in sorted(glob.glob(f"{L}/arm_ik_selftest_*.csv")):
        if not lo < os.path.getmtime(p) < hi + 60:
            continue
        r = rows_of(p)
        wall = np.array([float(x["wall"]) for x in r])
        if not (wall[0] <= w1 and wall[-1] >= w0):
            continue
        ins, rest = split(wall, w0, w1)
        loop, ikm, per = (np.array([float(x["loop_ms"]) for x in r]), np.array([float(x["ik_ms"]) for x in r]),
                          np.r_[np.nan, np.diff(wall) * 1000])
        print(f"IK  {os.path.basename(p)}: loop {loop[ins].mean():.1f}/{loop[ins].max():.1f} ms (rest {loop[rest].mean():.1f}/{loop[rest].max():.1f}), "
              f"ik {ikm[ins].mean():.1f}/{ikm[ins].max():.1f} ms (rest {ikm[rest].mean():.1f}/{ikm[rest].max():.1f}), "
              f"period max {np.nanmax(per[ins]):.0f} ms (rest {np.nanmax(per[rest]):.0f})")
    for p in sorted(glob.glob(f"{L}/arm_sim_*.csv")):
        if not lo < os.path.getmtime(p) < hi + 60:
            continue
        r = rows_of(p)
        wall = np.array([float(x["wall"]) for x in r])
        if not (wall[0] <= w1 and wall[-1] >= w0):
            continue
        ts = np.array([float(x["t"]) for x in r])
        ins, rest = split(wall, w0, w1)
        lag = (wall - wall[0]) - (ts - ts[0])          # physics behind the wall clock [s]
        k = np.where(ins)[0]
        ratio = (ts[k[-1]] - ts[k[0]]) / (wall[k[-1]] - wall[k[0]]) if len(k) > 1 else float("nan")
        per = np.r_[np.nan, np.diff(wall) * 1000]
        print(f"SIM {os.path.basename(p)}: real-time ratio {ratio:.3f}, physics behind wall max {lag[ins].max() * 1000:.0f} ms "
              f"(rest {lag[rest].max() * 1000:.0f}), log period max {np.nanmax(per[ins]):.0f} ms (rest {np.nanmax(per[rest]):.0f})")
    for p in sorted(glob.glob(f"{L}/arm_sdk_bridge_sim_*.csv")):
        if not lo < os.path.getmtime(p) < hi + 60:
            continue
        stamp = os.path.basename(p)[-19:-4]      # local (G1) time the log was opened, ~ the bridge t = 0
        t0 = time.mktime(time.strptime(stamp, "%Y%m%d_%H%M%S"))
        r = rows_of(p)
        t = np.array([float(x["t"]) for x in r])
        wall = t0 + t
        if not (wall[0] <= w1 + 1 and wall[-1] >= w0 - 1):
            continue
        ins, rest = split(wall, w0 - 1, w1 + 1)      # +-1 s: the file name has 1 s resolution
        per = np.r_[np.nan, np.diff(t) * 1000]
        print(f"BR  {os.path.basename(p)}: period mean {np.nanmean(per[ins]):.1f} / max {np.nanmax(per[ins]):.1f} ms "
              f"(rest {np.nanmean(per[rest]):.1f} / {np.nanmax(per[rest]):.1f})")
