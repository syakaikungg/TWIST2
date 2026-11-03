#!/usr/bin/env python3
"""10 s at 50 Hz: are head/controller poses arriving, and how regularly does the PICO timestamp advance? Read only."""
import time

import numpy as np
import xrobotoolkit_sdk as xr

xr.init()
time.sleep(0.5)
n = 0
nz = {"head": 0, "left": 0, "right": 0}
stamps, changes = [], []
last_stamp, last_change = None, time.time()
max_gap = 0.0
t0 = time.time()
while time.time() - t0 < 10.0:
    now = time.time()
    s = xr.get_time_stamp_ns()
    if s != last_stamp:
        if last_stamp is not None:
            max_gap = max(max_gap, now - last_change)
            changes.append(now)
        last_stamp, last_change = s, now
    stamps.append(s)
    for k, f in (("head", xr.get_headset_pose), ("left", xr.get_left_controller_pose), ("right", xr.get_right_controller_pose)):
        if np.linalg.norm(np.array(f())[3:7]) > 0.5:
            nz[k] += 1
    n += 1
    time.sleep(0.02)
max_gap = max(max_gap, time.time() - last_change)
xr.close()
rate = len(changes) / 10.0
print(f"samples {n}; poses present: head {nz['head']}, left {nz['left']}, right {nz['right']} of {n}")
print(f"timestamp advanced {len(changes)} times (~{rate:.0f}/s), longest pause {max_gap * 1000:.0f} ms, "
      f"timestamp span {(stamps[-1] - stamps[0]) / 1e9:.2f} s")
