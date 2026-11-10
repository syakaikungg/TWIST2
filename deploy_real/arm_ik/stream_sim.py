#!/usr/bin/env python3
"""Print the arm simulator state (Redis `sim_qpos`) as JSON lines on stdout. Read only.

Run over SSH by the laptop viewer (sim_viewer.py), so no network port has to be opened.
"""
import sys
import time

import redis

r = redis.Redis(host="127.0.0.1", port=6379)
last = None
try:
    while True:
        raw = r.get("sim_qpos")
        if raw and raw != last:
            last = raw
            sys.stdout.write(raw.decode() + "\n")
            sys.stdout.flush()
        time.sleep(0.02)
except (BrokenPipeError, KeyboardInterrupt):
    pass
