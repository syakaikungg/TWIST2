#!/usr/bin/env python3
"""Inspect candidate G1 29DOF physics models: actuators, torque limits, collision geoms and self-collision filters."""
import glob
import os
import sys

import mujoco
import numpy as np

cands = sys.argv[1:] or sorted(set(glob.glob(os.path.expanduser("~/nextech/TWIST2/assets/**/g1*29*.xml"), recursive=True)))
for path in cands:
    print("=" * 80)
    print(path)
    try:
        m = mujoco.MjModel.from_xml_path(path)
    except Exception as e:
        print("  load failed:", e)
        continue
    print(f"  nq={m.nq} nv={m.nv} nu={m.nu} ngeom={m.ngeom} nbody={m.nbody} timestep={m.opt.timestep}")
    jn = lambda j: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    acts = []
    for a in range(m.nu):
        j = m.actuator_trnid[a, 0]
        acts.append((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a), jn(j),
                     bool(m.actuator_ctrllimited[a]), np.round(m.actuator_ctrlrange[a], 1).tolist(),
                     bool(m.actuator_forcelimited[a]), np.round(m.actuator_forcerange[a], 1).tolist(),
                     int(m.actuator_gaintype[a]), int(m.actuator_biastype[a])))
    for a in acts:
        if any(k in a[1] for k in ("shoulder", "elbow", "wrist", "waist")):
            print("  act", a)
    # collision geoms per body
    coll = {}
    for g in range(m.ngeom):
        if m.geom_contype[g] or m.geom_conaffinity[g]:
            b = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
            coll.setdefault(b, []).append((int(m.geom_type[g]), int(m.geom_contype[g]), int(m.geom_conaffinity[g])))
    print("  bodies with collision geoms:", {b: len(v) for b, v in coll.items()})
    print("  contype/conaffinity sets:", sorted({(c, a) for v in coll.values() for _, c, a in v}))
    print(f"  excludes: {m.nexclude}, disableflags={m.opt.disableflags}, filterparent={'on' if not (m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT) else 'off'}")
    print("  joint damping/armature (arms):",
          {jn(j): (round(float(m.dof_damping[m.jnt_dofadr[j]]), 3), round(float(m.dof_armature[m.jnt_dofadr[j]]), 4))
           for j in range(m.njnt) if jn(j) and ("wrist" in jn(j) or "elbow" in jn(j)) and "right" in jn(j)})
    hand = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)
            if "hand" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "") or "rubber" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "")]
    print("  hand bodies:", hand)
