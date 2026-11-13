"""The real tool on the right wrist (2026-11-12 photos): a Makita DF033D driver drill clamped by its grip to a
stainless plate on a quick coupler at the wrist flange, where the hand was.

Makita's specs: length 149 mm, width 66 mm, height 229 mm (top of the head to the bottom of the battery), net
weight 0.72 kg without battery, 0.9-1.1 kg with a battery (EPTA). Placement from the photos, scaled by those
sizes (estimates, not measured): the chuck points along the forearm (the wrist yaw link's x), its tip about 20 cm
past the flange; the head's axis about 10 cm above the forearm axis; the grip and battery below it (with the wrist
at 0 and the forearm level: head up, battery down, as a person holds a drill); the coupler about 4 cm long. A
trigger servo sits beside the grip.
Masses: drill 0.95 kg with its battery (within Makita's 0.9-1.1 kg); the coupler, plate and servo are guesses
(0.20 + 0.15 + 0.08 kg). About 1.38 kg in all, centre of mass about 9 cm past the flange and 2 cm above the forearm
axis. Change PARTS when the real values are known.

  import tool_df033d
  tool_df033d.add_tool(spec)          # MjSpec of the G1 model, before compile(); takes the right hand's mesh off
  tool_df033d.chuck(m, d)             # (tip position, chuck direction) in the world
"""
import mujoco
import numpy as np

import sim_drill

PARENT = "right_wrist_yaw_link"
FLANGE = list(sim_drill.FLANGE)          # where the hand was bolted, in the wrist yaw frame
BODY = "tool_df033d"
CHUCK_TIP = np.array([0.200, 0.0, 0.10])     # from the flange, in the wrist yaw frame
CHUCK_DIR = np.array([1.0, 0.0, 0.0])
TEAL, BLACK, STEEL, ALU = [0.0, 0.55, 0.55, 1.0], [0.08, 0.08, 0.08, 1.0], [0.72, 0.72, 0.75, 1.0], [0.8, 0.8, 0.82, 1.0]
# name, type, size (MuJoCo half sizes / radius+half length), pos from the flange, mass [kg], rgba, along x
PARTS = (
    ("coupler", "cyl", [0.034, 0.019], [0.019, 0.0, 0.0], 0.20, ALU, True),
    ("plate", "box", [0.065, 0.002, 0.070], [0.105, -0.030, -0.020], 0.15, STEEL, False),   # on the outer side
    ("head", "cyl", [0.033, 0.057], [0.108, 0.0, 0.10], 0.45, TEAL, True),      # 149 mm with the chuck
    ("chuck", "cyl", [0.013, 0.0175], [0.1825, 0.0, 0.10], 0.05, BLACK, True),
    ("grip", "box", [0.025, 0.022, 0.065], [0.087, 0.0, 0.005], 0.22, BLACK, False),
    ("battery", "box", [0.040, 0.033, 0.024], [0.087, 0.0, -0.074], 0.23, TEAL, False),   # bottom 229 mm below the top
    ("servo", "box", [0.020, 0.012, 0.025], [0.130, 0.0, 0.030], 0.08, BLACK, False),
)
MASS = sum(p[4] for p in PARTS)
_Z_TO_X = [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0]


def add_tool(spec, collision=True, mass_scale=1.0):
    """Add the drill with its masses (the right hand's mesh is taken off, its body stays as the IK frame).
    collision=True: the parts also collide (group 3 copies), for distance checks and the simulator's contacts."""
    wrist = spec.body(PARENT)
    sim_drill._remove_hand(spec, sim_drill.HAND)
    body = wrist.add_body(name=BODY, pos=FLANGE)
    types = {"cyl": mujoco.mjtGeom.mjGEOM_CYLINDER, "box": mujoco.mjtGeom.mjGEOM_BOX}
    for name, t, size, pos, mass, rgba, along_x in PARTS:
        size3 = list(size) + [0.0] * (3 - len(size))
        g = body.add_geom(name=f"df033d_{name}", type=types[t], size=size3, pos=pos, rgba=rgba,
                          contype=0, conaffinity=0, group=1, mass=mass * mass_scale)
        if along_x:
            g.quat = _Z_TO_X
        if collision:
            c = body.add_geom(name=f"df033d_coll_{name}", type=types[t], size=size3, pos=pos,
                              rgba=[1.0, 0.3, 0.3, 0.3], group=3, density=0.0)
            if along_x:
                c.quat = _Z_TO_X
    for link in ("wrist_pitch_link", "wrist_roll_link", "wrist_yaw_link"):
        sim_drill._exclude(spec, BODY, f"right_{link}")
    return spec


def chuck(m, d):
    """(tip position, chuck direction) in the world frame."""
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, BODY)
    R = d.xmat[b].reshape(3, 3)
    return d.xpos[b] + R @ CHUCK_TIP, R @ CHUCK_DIR
