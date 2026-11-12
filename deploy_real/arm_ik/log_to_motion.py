#!/usr/bin/env python3
"""Cut a stretch of a teleop run into a motion file for play_motion.py (the "teach by teleop" way).

Every teleop run already leaves its logs on the G1 (~/nextech_setup/logs): the bridge's CSV (the commanded
angles cmd_*, what the arms really got, rate-limited) and the IK's CSV (q_*). Pick the seconds you liked:

  python3 log_to_motion.py ~/nextech_setup/logs/arm_sdk_bridge_send_20261112_150000.csv --from 40 --to 70 \
      --out motions/teleop_1500.json
  python3 play_motion.py --motion motions/teleop_1500.json --check

--from / --to are seconds of the CSV's `t` column (the bridge's and IK's own clocks; the bridge's log prints the
same seconds, e.g. "[  35.2s] TRACK"). Only rows where the bridge tracked (state TRACK) are used for a bridge CSV.
The rest pose (where the arms cool down between work periods) comes from --rest-from (default: the designed
motion's low-load rest, motions/drill_work.json), or the clip's first frame with --rest-from first.
--pingpong plays the clip forward then backward, so the loop closes without a glide back to the start.
With mujoco and the TWIST2 model at hand it also reports the gravity torques of the right arm with the real
tool (tool_df033d.py), like design_motion.py.
"""
import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np

from play_motion import NAMES, LO, HI


def read_clip(path, t_from, t_to):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: empty")
    cols = rows[0].keys()
    if all(f"cmd_{n}" in cols for n in NAMES):
        pre, kind = "cmd_", "bridge"
    elif all(f"q_{n}" in cols for n in NAMES):
        pre, kind = "q_", "ik"
    else:
        raise SystemExit(f"{path}: no cmd_* (bridge) or q_* (IK) columns for the 14 arm joints")
    t, q = [], []
    for r in rows:
        try:
            tt = float(r["t"])
        except (KeyError, ValueError):
            continue
        if tt < t_from or (t_to > 0 and tt > t_to):
            continue
        if kind == "bridge" and r.get("state") != "TRACK":
            continue
        try:
            v = [float(r[pre + n]) for n in NAMES]
        except ValueError:
            continue
        if np.isfinite(v).all():
            t.append(tt)
            q.append(v)
    if len(t) < 10:
        raise SystemExit(f"{path}: fewer than 10 usable rows between {t_from} and {t_to} s"
                         + (" in TRACK" if kind == "bridge" else ""))
    return kind, np.array(t), np.array(q)


def resample(t, q, rate):
    tt = np.arange(t[0], t[-1], 1.0 / rate)
    return np.stack([np.interp(tt, t, q[:, k]) for k in range(q.shape[1])], axis=1)


def smooth(q, n):
    if n <= 1:
        return q
    k = np.ones(n) / n
    pad = np.pad(q, ((n // 2, n - 1 - n // 2), (0, 0)), mode="edge")
    return np.stack([np.convolve(pad[:, j], k, mode="valid") for j in range(q.shape[1])], axis=1)


def torque_check(loop, rest):
    """Gravity torques of the right arm with the real tool, if mujoco and the model are there; else None."""
    try:
        import design_motion as dm
        md = dm.Model(0.0, 0.0, "df033d")
    except Exception as e:
        print(f"(no torque check: {e})")
        return None
    taus, clear = [], []
    for f in loop[::5]:
        md.set(dict(zip(NAMES, f)))
        taus.append(md.gravity(dm.RIGHT))
        clear.append(md.clearance())
    taus, clear = np.array(taus), np.array(clear)
    md.set(dict(zip(NAMES, rest)))
    return {"right_tau_rms_Nm": dict(zip(dm.RIGHT, np.round(np.sqrt((taus ** 2).mean(axis=0)), 2).tolist())),
            "right_tau_max_Nm": dict(zip(dm.RIGHT, np.round(np.abs(taus).max(axis=0), 2).tolist())),
            "rest_tau_Nm": dict(zip(dm.RIGHT, np.round(md.gravity(dm.RIGHT), 2).tolist())),
            "min_clearance_m": {"right_vs_body": round(float(clear[:, 0].min()), 3),
                                "right_vs_left_arm": round(float(clear[:, 1].min()), 3),
                                "left_vs_body": round(float(clear[:, 2].min()), 3)}}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="the bridge's arm_sdk_bridge_*.csv or the IK's arm_ik_*.csv of a teleop run")
    ap.add_argument("--from", dest="t_from", type=float, required=True, help="start [s, the CSV's t]")
    ap.add_argument("--to", dest="t_to", type=float, required=True, help="end [s, the CSV's t]")
    ap.add_argument("--out", required=True)
    ap.add_argument("--rest-from", default=str(Path(__file__).with_name("motions") / "drill_work.json"),
                    help="a motion file whose rest pose to use, or 'first' (the clip's first frame)")
    ap.add_argument("--smooth", type=int, default=5, help="moving average over this many frames (50 Hz)")
    ap.add_argument("--pingpong", action="store_true", help="forward then backward (no glide at the seam)")
    ap.add_argument("--rate", type=float, default=50.0)
    args = ap.parse_args()

    kind, t, q = read_clip(args.csv, args.t_from, args.t_to)
    loop = smooth(resample(t, q, args.rate), args.smooth)
    if args.pingpong:
        loop = np.concatenate([loop, loop[-2::-1]])
    loop = np.clip(loop, LO, HI)
    if args.rest_from == "first":
        rest = loop[0].copy()
    else:
        rm = json.loads(Path(args.rest_from).read_text())
        rest = np.array([dict(zip(rm["names"], rm["rest"]))[n] for n in NAMES])
    vel = np.abs(np.diff(loop, axis=0)).max(axis=1) * args.rate
    check = {"from_s": args.t_from, "to_s": args.t_to, "rows": int(len(t)), "loop_s": round(len(loop) / args.rate, 2),
             "peak_joint_speed": round(float(vel.max()), 3),
             "seam_rad": round(float(np.abs(loop[-1] - loop[0]).max()), 3)}
    tc = torque_check(loop, rest)
    if tc:
        check.update(tc)
    out = {"format": "g1_arm_motion/1", "name": Path(args.out).stem,
           "description": f"teleop clip {args.t_from}-{args.t_to} s of {Path(args.csv).name} ({kind} log)",
           "created": time.strftime("%Y-%m-%d %H:%M:%S"), "source": "log_to_motion.py", "rate": args.rate,
           "names": NAMES, "rest": [round(float(v), 4) for v in rest],
           "loop": [[round(float(v), 4) for v in f] for f in loop], "check": check}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, separators=(",", ":")))
    print(json.dumps(check, indent=1, ensure_ascii=False))
    print(f"wrote {args.out}: {len(loop)} frames ({len(loop) / args.rate:.1f} s), rest from {args.rest_from}")


if __name__ == "__main__":
    main()
