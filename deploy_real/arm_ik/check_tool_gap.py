#!/usr/bin/env python3
"""How close the drill (and the hands) came to the body and the other arm, from a log with joint angles.

Reads an IK log (arm_ik_*.csv) or an arm_sim log (arm_sim_*.csv): both have q_<joint> columns. Builds the
G1 model with the drill's collision capsules on the right wrist (sim_drill.py) and a box on the left hand,
and prints, over the run, the smallest distance from the drill to the body (pelvis, torso, hips, knees) and
to the left arm (forearm links and hand), with the time and the closest body.

  python check_tool_gap.py arm_ik_selftest_drill_cross_....csv [--hand]   (--hand: right hand box, no drill)
"""
import argparse
import csv
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_drill  # noqa: E402

MODELS = [os.path.join(HERE, "..", "..", "assets", "g1", "g1_sim2sim_29dof.xml"),
          os.path.expanduser("~/nextech/TWIST2/assets/g1/g1_sim2sim_29dof.xml")]
BODY = ["pelvis", "torso_link"] + [f"{s}_{p}" for s in ("left", "right")
                                   for p in ("hip_pitch_link", "hip_roll_link", "hip_yaw_link", "knee_link")]
LEFT_ARM = ["left_elbow_link", "left_wrist_roll_link", "left_wrist_pitch_link", "left_wrist_yaw_link",
            "left_rubber_hand"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--hand", action="store_true", help="the right hand (box) instead of the drill")
    args = ap.parse_args()
    xml = next(p for p in MODELS if os.path.isfile(p))
    spec = mujoco.MjSpec.from_file(xml)
    if args.hand:
        sim_drill.add_hand_collision(spec, "right_rubber_hand")
    else:
        sim_drill.add_drill(spec, collision=True)
    sim_drill.add_hand_collision(spec, "left_rubber_hand")
    m = spec.compile()
    d = mujoco.MjData(m)

    def geoms(bodies):
        out = []
        for bn in bodies:
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
            out += [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and (m.geom_contype[g] or m.geom_conaffinity[g])]
        return out

    tool = geoms(["right_rubber_hand"] if args.hand else list(sim_drill.DRILL_BODIES))
    groups = {"body": geoms(BODY), "left arm": geoms(LEFT_ARM)}
    rows = list(csv.DictReader(open(args.log, newline="")))
    qcols = [k for k in rows[0] if k.startswith("q_")]
    qadr = {}
    for k in qcols:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, k[2:])
        if j >= 0:
            qadr[k] = m.jnt_qposadr[j]
    best = {g: (1.0, None, None) for g in groups}
    for r in rows:
        for k, a in qadr.items():
            d.qpos[a] = float(r[k])
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)   # not needed for distances, but keeps the geom poses current
        for name, gs in groups.items():
            for a in tool:
                for b in gs:
                    dist = mujoco.mj_geomDistance(m, d, a, b, 0.3, None)
                    if dist < best[name][0]:
                        best[name] = (dist, float(r["t"]), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[b]))
    what = "right hand" if args.hand else "drill"
    for name, (dist, t, bn) in best.items():
        if bn is None:
            print(f"{what} - {name}: farther than 30 cm all the time")
        else:
            print(f"{what} - {name}: closest {dist * 100:.1f} cm at t={t:.2f} s ({bn})"
                  + ("  << TOUCHING" if dist <= 0 else ""))


if __name__ == "__main__":
    main()
