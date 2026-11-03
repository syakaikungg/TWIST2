#!/usr/bin/env python3
"""Per-geom collision settings of upper-body bodies, joint torque ranges, and actuator gains of a G1 model."""
import sys

import mujoco
import numpy as np

path = sys.argv[1]
m = mujoco.MjModel.from_xml_path(path)
bn = lambda b: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
TYPES = {2: "sphere", 3: "capsule", 5: "cylinder", 6: "box", 7: "mesh"}
for g in range(m.ngeom):
    b = bn(m.geom_bodyid[g])
    if any(k in b for k in ("torso", "shoulder", "elbow", "wrist", "hand", "pelvis", "waist")) and (m.geom_contype[g] or m.geom_conaffinity[g]):
        print(f"geom {g:2d} body {b:28s} {TYPES.get(int(m.geom_type[g]), m.geom_type[g]):8s} contype {m.geom_contype[g]} conaff {m.geom_conaffinity[g]} "
              f"size {np.round(m.geom_size[g], 3).tolist()}")
print("joint actuatorfrcrange (limited?):")
for j in range(m.njnt):
    n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    if n and any(k in n for k in ("shoulder", "elbow", "wrist", "waist")):
        print(f"  {n:28s} limited={bool(m.jnt_actfrclimited[j])} range={np.round(m.jnt_actfrcrange[j], 1).tolist()} "
              f"jrange={np.round(m.jnt_range[j], 3).tolist()}")
print("actuator gain/bias (first arm actuator):", m.actuator_gainprm[3][:3], m.actuator_biasprm[3][:3])
print("total mass", round(float(m.body_mass.sum()), 2), "kg; right hand chain masses:",
      {bn(b): round(float(m.body_mass[b]), 3) for b in range(m.nbody) if "right" in bn(b) and any(k in bn(b) for k in ("elbow", "wrist", "hand"))})
