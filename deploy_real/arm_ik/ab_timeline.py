#!/usr/bin/env python3
"""One live run, one time window: A/B presses, grip, IK events and the bridge state / weight, on one clock.
The bridge log has no wall clock: it is aligned on the first rising edge of `active` it saw, which is the
IK's first A -> ENABLED (one 20 ms loop later). Read only.  ab_timeline.py <tag> <HH:MM:SS from> <HH:MM:SS to>"""
import calendar
import csv
import glob
import os
import re
import sys
import time

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
tag, a, b = sys.argv[1:4]
utc = lambda w: time.strftime("%H:%M:%S", time.gmtime(w)) + f".{int(w % 1 * 10)}"
ik_txt, br_txt = f"{L}/simik_{tag}.txt", f"{L}/simbridge_{tag}.txt"
ik_csv = min(glob.glob(f"{L}/arm_ik_live_*.csv"), key=lambda p: abs(os.path.getmtime(p) - os.path.getmtime(ik_txt)))
stamp = re.search(r"log=\S*arm_sdk_bridge_sim_(\d{8}_\d{6})\.csv", open(br_txt).read()).group(1)
br_csv = f"{L}/arm_sdk_bridge_sim_{stamp}.csv"
ik, br = list(csv.DictReader(open(ik_csv))), list(csv.DictReader(open(br_csv)))
fi = lambda k: np.array([float(x[k]) for x in ik])
wall, t_ik = fi("wall"), fi("t")
bt, bact = np.array([float(x["t"]) for x in br]), np.array([int(x["active"]) for x in br])
first_ik = wall[int(np.argmax(fi("active") > 0))]
off = first_ik + 0.02 - bt[int(np.argmax(bact > 0))]
bw = bt + off
day = time.strftime("%Y-%m-%d", time.gmtime(wall[0]))
w0, w1 = (calendar.timegm(time.strptime(f"{day} {s}", "%Y-%m-%d %H:%M:%S")) for s in (a, b))
ev = []
for line in open(ik_txt):
    m = re.match(r"\[\s*([\d.]+)s\] (.*)", line)
    if m and not m.group(2).startswith("active="):
        ev.append((float(np.interp(float(m.group(1)), t_ik, wall)), "IK      " + m.group(2)[:80]))
prev = None
for i, x in enumerate(br):
    st = x["state"]
    if st != prev:
        ev.append((bw[i], f"bridge  -> {st} (weight {float(x['weight']):.2f})"))
        prev = st
for k in ("A", "B"):
    v = fi(k)
    for i in np.where(np.diff(v) > 0)[0]:
        j = int(np.searchsorted(bw, wall[i + 1]))
        j = min(j, len(br) - 1)
        ev.append((wall[i + 1], f"press {k}   (bridge {br[j]['state']}, weight {float(br[j]['weight']):.2f})"))
g = fi("grip")
on = (g > 0.6).astype(int)
for i in np.where(np.diff(on) != 0)[0]:
    j = min(int(np.searchsorted(bw, wall[i + 1])), len(br) - 1)
    ev.append((wall[i + 1], f"grip {'held    ' if on[i + 1] else 'released'} (bridge {br[j]['state']}, weight {float(br[j]['weight']):.2f})"))
for w, s in sorted(ev):
    if w0 <= w <= w1:
        print(f"{utc(w)}  {s}")
