#!/usr/bin/env python3
"""Record which PICO buttons/analog inputs change over a short window (read-only, no robot commands)."""
import sys
import time

import xrobotoolkit_sdk as xr

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
DIGITAL = ["get_A_button", "get_B_button", "get_X_button", "get_Y_button", "get_left_menu_button",
           "get_right_menu_button", "get_left_axis_click", "get_right_axis_click"]
ANALOG = ["get_left_trigger", "get_right_trigger", "get_left_grip", "get_right_grip"]

xr.init()
prev = {n: False for n in DIGITAL + ANALOG}
counts = {n: 0 for n in DIGITAL + ANALOG}
t0 = time.time()
print(f"recording {DURATION:.0f}s ...", flush=True)
while time.time() - t0 < DURATION:
    t = time.time() - t0
    for n in DIGITAL:
        v = bool(getattr(xr, n)())
        if v and not prev[n]:
            counts[n] += 1
            print(f"[{t:5.1f}s] {n[4:]} pressed", flush=True)
        prev[n] = v
    for n in ANALOG:
        v = float(getattr(xr, n)()) > 0.6
        if v and not prev[n]:
            counts[n] += 1
            print(f"[{t:5.1f}s] {n[4:]} > 0.6", flush=True)
        prev[n] = v
    time.sleep(0.01)
xr.close()
print("SUMMARY presses:", {k[4:]: v for k, v in counts.items()})
