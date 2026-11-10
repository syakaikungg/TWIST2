#!/usr/bin/env python3
"""Rebuild an approximate input record (*_input.jsonl, for arm_ik_dryrun.py --replay) from the CSV and the text
log of an IK run made before the inputs were recorded (before 2026-11-05). Read only, writes one new file.

The CSV has the head and controller POSITIONS and the buttons, not the orientations. Rebuilt:
- head: logged position; orientation = the head yaw printed at the latest grip ("grip -> ... (head yaw N deg"),
  level (no pitch or roll). Good enough to compare the forward direction (--heading shared vs per_arm); the
  neck-pivot shoulder estimate cannot be judged from it (no head pitch).
- controllers: logged positions; orientation = identity (unknown), the same in both runs being compared.
- data timestamp: a new one whenever stamp_age_ms is small or starts again from a smaller value (new data
  in that loop), else the same (so data gaps replay).

  python csv_to_input.py arm_ik_live_X.csv live_ik_send_X.txt out_input.jsonl
"""
import csv
import json
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

M = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # PICO axes -> robot axes (as in the IK)
GRIP = re.compile(r"\[\s*([\d.]+)s\] grip -> (\w+) arm FOLLOW \(head yaw (-?\d+) deg")


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    csv_path, txt_path, out_path = sys.argv[1:]
    grips = []
    with open(txt_path, errors="replace") as f:
        for line in f:
            m = GRIP.search(line)
            if m:
                grips.append((float(m.group(1)), float(m.group(3))))
    if not grips:
        sys.exit(f"no 'grip -> ... (head yaw N deg)' lines in {txt_path}")
    grips.sort()
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    stamp, prev_age, n = 1, None, 0
    with open(out_path, "w") as out:
        for r in rows:
            t = float(r["t"])
            yaw = grips[0][1]
            for tg, yg in grips:
                if tg <= t + 1e-6:
                    yaw = yg
            age = float(r.get("stamp_age_ms") or 0.0)
            if prev_age is None or age < prev_age or age <= 15.0:   # new PICO data since the last loop
                stamp += 1
            prev_age = age
            q_head = list(R.from_rotvec([0.0, np.radians(yaw), 0.0]).as_quat())   # robot yaw = turn about PICO +y

            def pose(prefix, ok_key, q):
                if ok_key and int(r.get(ok_key, 1) or 0) == 0:
                    return None
                p = M.T @ np.array([float(r[f"{prefix}_{a}"]) for a in "xyz"])
                return [round(float(v), 5) for v in list(p) + q]

            d = {"head": pose("head", "head_ok", q_head),
                 "left": pose("ctrl_left", None, [0.0, 0.0, 0.0, 1.0]),
                 "right": pose("ctrl_right", "right_ok", [0.0, 0.0, 0.0, 1.0]),
                 "trigger": float(r["trigger"]), "grip": float(r["grip"]), "lgrip": float(r.get("lgrip") or 0.0),
                 "ltrig": float(r.get("ltrig") or 0.0),
                 "A": bool(int(r["A"])), "B": bool(int(r["B"])), "X": bool(int(r["X"])), "Y": bool(int(r["Y"])),
                 "rclick": bool(int(r.get("rclick") or 0)), "lclick": bool(int(r.get("lclick") or 0)),
                 "stamp": stamp * 20_000_000}
            out.write(json.dumps({"t": round(t, 4), "d": d}) + "\n")
            n += 1
    print(f"{out_path}: {n} records, {len(grips)} grips, {rows[-1]['t']} s")


if __name__ == "__main__":
    main()
