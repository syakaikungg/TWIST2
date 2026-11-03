#!/usr/bin/env python3
"""Newest live run: every stretch where the PICO data was older than 100 ms (start UTC, longest age), and the
age percentiles while data was flowing. Read only."""
import csv
import glob
import os
import time

import numpy as np

L = os.path.expanduser("~/nextech_setup/logs")
r = list(csv.DictReader(open(max(glob.glob(f"{L}/arm_ik_live_*.csv"), key=os.path.getmtime))))
wall, age = np.array([float(x["wall"]) for x in r]), np.array([float(x["stamp_age_ms"]) for x in r])
utc = lambda w: time.strftime("%H:%M:%S", time.gmtime(w)) + f".{int(w % 1 * 10)}"
bad = age > 100
eps, i = [], 0
while i < len(age):
    if bad[i]:
        j = i
        while j + 1 < len(age) and bad[j + 1]:
            j += 1
        k = i + int(np.argmax(age[i:j + 1]))
        eps.append((wall[k] - age[k] / 1000, age[k] / 1000))
        i = j + 1
    else:
        i += 1
flow = age[~bad]
print(f"PICO data age while flowing: median {np.median(flow):.0f} ms, 99% {np.percentile(flow, 99):.0f} ms")
print(f"stretches older than 100 ms: {len(eps)}")
for s, d in eps:
    print(f"  from {utc(s)} UTC for {d:.2f} s")
