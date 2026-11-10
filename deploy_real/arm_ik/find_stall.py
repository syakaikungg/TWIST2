#!/usr/bin/env python3
"""Where are the slow IK loops in the newest live and sim_suite logs?"""
import csv
import glob
import os

L = os.path.expanduser("~/nextech_setup/logs")
for pat in ("arm_ik_live_*.csv", "arm_ik_selftest_sim_suite_*.csv"):
    p = max(glob.glob(f"{L}/{pat}"), key=os.path.getmtime)
    rows = list(csv.DictReader(open(p)))
    slow = [(float(r["loop_ms"]), r["t"], r["active"], r["follow"]) for r in rows if float(r["loop_ms"]) > 20]
    print(os.path.basename(p), "loops over 20 ms:", slow,
          "| last loop t =", rows[-1]["t"], "| ik_ms at the slow loop:",
          [r["ik_ms"] for r in rows if float(r["loop_ms"]) > 20])
