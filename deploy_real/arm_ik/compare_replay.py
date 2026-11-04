#!/usr/bin/env python3
"""Compare the hand positions of two IK runs on the same inputs (two --replay runs of one *_input.jsonl, e.g.
the current settings and --heading per_arm --neck-back 0 --neck-down 0), while each arm follows. Read only.

  python compare_replay.py A.csv B.csv
"""
import csv
import sys

import numpy as np


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    hand = {s: np.array([[float(r[f"hand_{s}_{a}"]) for a in "xyz"] for r in rows]) for s in ("left", "right")
            if f"hand_{s}_x" in rows[0]}
    fol = {s: np.array([int(r.get(f"follow_{s}", 0) or 0) for r in rows]) for s in hand}
    return t, hand, fol


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    ta, ha, fa = load(sys.argv[1])
    tb, hb, fb = load(sys.argv[2])
    idx = np.clip(np.searchsorted(tb, ta), 0, len(tb) - 1)   # B row at or after each A row
    for s in ha:
        if s not in hb:
            continue
        on = (fa[s] == 1) | (fb[s][idx] == 1)
        if not on.any():
            print(f"{s}: never follows")
            continue
        dist = np.linalg.norm(ha[s] - hb[s][idx], axis=1)
        dist_on = dist[on]
        print(f"{s}: while following ({on.sum() / 50:.0f} s): hand difference mean {dist_on.mean() * 100:.1f} cm, "
              f"p95 {np.percentile(dist_on, 95) * 100:.1f} cm, max {dist_on.max() * 100:.1f} cm")
        # the 5 seconds with the largest difference
        secs = {}
        for t, d, o in zip(ta, dist, on):
            if o:
                k = int(t)
                secs[k] = max(secs.get(k, 0.0), d)
        top = sorted(secs.items(), key=lambda kv: -kv[1])[:5]
        print("   largest at " + ", ".join(f"{k} s {v * 100:.1f} cm" for k, v in sorted(top)))


if __name__ == "__main__":
    main()
