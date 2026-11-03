#!/usr/bin/env python3
"""A drill you can see spinning, for the simulator and its viewers (looks only: no mass, no contacts).

arm_sim.py adds it to the right hand, spins it while IK's Redis `drill_cmd` says ON, and puts the state in
`sim_qpos` as {"drill": {"on": 0/1, "angle": rad, "rps": rev/s}}. A viewer that draws the robot from
`sim_qpos` shows it with three lines:

    import sim_drill
    sim_drill.add_drill(spec)            # spec = mujoco.MjSpec.from_file(...), before spec.compile()
    view = sim_drill.DrillView(m)        # m = the compiled model
    view.apply(m, msg.get("drill"))      # every frame, before mj_kinematics / mj_forward

The shape is made up (the real attachment is not finished): a clamp over the fingers, a motor body, and in
front of it the chuck and bit, along the hand's forward axis (x of right_wrist_yaw_link). The chuck has a
yellow stripe and the bit a flat blade, so the turning is easy to see. The ring on the motor and the lamp
on the clamp's wrist side (what the head camera sees) turn green while ON. Shown at 3 rev/s (not the real
speed) so that 30-50 fps video does not alias it.
"""
import math

import mujoco
import numpy as np

PARENT = "right_wrist_yaw_link"
BIT_BODY = "drill_bit"
LAMP_GEOMS = ("drill_ring", "drill_lamp")   # green while ON
RING_OFF = [0.25, 0.25, 0.25, 1.0]
RING_ON = [0.1, 0.95, 0.2, 1.0]
_Z_TO_X = [math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0]   # cylinders run along local z


def _geom(body, name, gtype, size, pos, rgba, along_x=False):
    g = body.add_geom(name=name, type=gtype, size=size, pos=pos, rgba=rgba,
                      contype=0, conaffinity=0, group=1, density=0.0)
    if along_x:
        g.quat = _Z_TO_X
    return g


def add_drill(spec, parent=PARENT, prefix="drill"):
    """Add the drill bodies under `parent` (the right wrist yaw link). Returns the spec."""
    wrist = spec.body(parent)
    if wrist is None:
        raise ValueError(f"no body {parent!r} in the model")
    cyl, box, sph = mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_SPHERE
    # Positions in the wrist yaw frame: the hand runs x 0.04-0.17 m, centred near y 0.01, z 0.01.
    body = wrist.add_body(name=f"{prefix}_body", pos=[0.0, 0.012, 0.010])
    _geom(body, f"{prefix}_clamp", box, [0.040, 0.036, 0.056], [0.140, 0.0, 0.0], [0.22, 0.22, 0.25, 1.0])
    _geom(body, f"{prefix}_motor", cyl, [0.030, 0.040, 0], [0.220, 0.0, 0.0], [0.95, 0.55, 0.05, 1.0], True)
    _geom(body, "drill_lamp", box, [0.0015, 0.034, 0.054], [0.0985, 0.0, 0.0], RING_OFF)   # wrist-side face
    _geom(body, "drill_ring", cyl, [0.0315, 0.007, 0], [0.205, 0.0, 0.0], RING_OFF, True)
    bit = body.add_body(name=BIT_BODY, pos=[0.260, 0.0, 0.0])        # turns about its x axis
    _geom(bit, f"{prefix}_chuck", cyl, [0.017, 0.018, 0], [0.018, 0.0, 0.0], [0.08, 0.08, 0.08, 1.0], True)
    _geom(bit, f"{prefix}_stripe", box, [0.018, 0.005, 0.004], [0.018, 0.0, 0.016], [1.0, 0.85, 0.0, 1.0])
    _geom(bit, f"{prefix}_shaft", cyl, [0.0035, 0.035, 0], [0.071, 0.0, 0.0], [0.75, 0.75, 0.78, 1.0], True)
    _geom(bit, f"{prefix}_blade", box, [0.028, 0.009, 0.0012], [0.078, 0.0, 0.0], [0.35, 0.4, 0.5, 1.0])
    _geom(bit, f"{prefix}_tip", sph, [0.005, 0, 0], [0.106, 0.0, 0.0], [0.35, 0.4, 0.5, 1.0])
    return spec


class DrillView:
    """Turns the bit and colours the ring and lamp of a model built with add_drill()."""

    def __init__(self, m):
        self.bit = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, BIT_BODY)
        self.lamps = [g for g in (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in LAMP_GEOMS) if g >= 0]
        self.ok = self.bit >= 0
        self.q0 = m.body_quat[self.bit].copy() if self.ok else None

    def apply(self, m, state):
        """state: the "drill" entry of sim_qpos (None or missing -> stopped). Changes the model only."""
        if not self.ok:
            return
        state = state or {}
        a = float(state.get("angle", 0.0))
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, self.q0, np.array([math.cos(a / 2), math.sin(a / 2), 0.0, 0.0]))
        m.body_quat[self.bit] = q
        for g in self.lamps:
            m.geom_rgba[g] = RING_ON if state.get("on") else RING_OFF


class DrillSpin:
    """The bit's angle from the ON/OFF command, with a short spin-up and a longer coast down."""

    def __init__(self, rps=3.0, spin_up=0.25, spin_down=0.6):
        self.rps, self.up, self.down = rps, spin_up, spin_down
        self.speed = self.angle = 0.0
        self.on = False

    def update(self, on, dt):
        self.on = bool(on)
        target = self.rps if self.on else 0.0
        step = (self.rps / self.up if target > self.speed else self.rps / self.down) * dt
        self.speed = min(target, self.speed + step) if target > self.speed else max(target, self.speed - step)
        self.angle = (self.angle + 2 * math.pi * self.speed * dt) % (2 * math.pi)
        return self.state()

    def state(self):
        return {"on": int(self.on), "angle": round(self.angle, 4), "rps": round(self.speed, 3)}
