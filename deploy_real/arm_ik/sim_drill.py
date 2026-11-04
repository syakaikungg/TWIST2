#!/usr/bin/env python3
"""A drill you can see spinning, for the simulator and its viewers (looks only: no mass, no contacts).

arm_sim.py adds it to the right wrist, spins it while IK's Redis `drill_cmd` says ON, and puts the state in
`sim_qpos` as {"drill": {"on": 0/1, "angle": rad, "rps": rev/s}}. A viewer that draws the robot from
`sim_qpos` shows it with three lines:

    import sim_drill
    sim_drill.add_drill(spec)            # spec = mujoco.MjSpec.from_file(...), before spec.compile()
    view = sim_drill.DrillView(m)        # m = the compiled model
    view.apply(m, msg.get("drill"))      # every frame, before mj_kinematics / mj_forward

The shape is made up (the real adapter is not finished). As on the real robot, the right hand is taken off
and the drill sits straight on the wrist: an adapter plate on the wrist flange, a motor body, then the chuck
and bit, along the forearm's axis (x of right_wrist_yaw_link). The chuck has a yellow stripe and the bit a
flat blade, so the turning is easy to see. The two rings on the motor (what the head camera sees) turn green
while ON. Shown at 3 rev/s (not the real speed) so that 30-50 fps video does not alias it. The physics is
unchanged: the hand's mesh had no mass of its own, and the right_rubber_hand body stays as the IK's frame.
"""
import math

import mujoco
import numpy as np

PARENT = "right_wrist_yaw_link"
HAND = "right_rubber_hand"     # its mesh is removed; the body (the IK frame, at the flange) stays
FLANGE = [0.0415, -0.003, 0.0]  # where the hand was bolted, in the wrist yaw frame (= HAND body pos)
BIT_BODY = "drill_bit"
LAMP_GEOMS = ("drill_ring", "drill_lamp")   # green while ON (front and rear ring on the motor)
RING_OFF = [0.25, 0.25, 0.25, 1.0]
RING_ON = [0.1, 0.95, 0.2, 1.0]
_Z_TO_X = [math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0]   # cylinders run along local z


def _geom(body, name, gtype, size, pos, rgba, along_x=False):
    g = body.add_geom(name=name, type=gtype, size=size, pos=pos, rgba=rgba,
                      contype=0, conaffinity=0, group=1, density=0.0)
    if along_x:
        g.quat = _Z_TO_X
    return g


def _remove_hand(spec, hand):
    """Take the hand's mesh off (the body stays: the IK and the payload use its frame)."""
    body = spec.body(hand)
    if body is None:
        return 0
    try:
        geoms = list(body.geoms)
    except AttributeError:     # older MjSpec without .geoms
        geoms = list(body.find_all(mujoco.mjtObj.mjOBJ_GEOM))
    for g in geoms:
        try:
            spec.delete(g)
        except Exception:      # cannot delete: hide it instead
            g.rgba = [0.0, 0.0, 0.0, 0.0]
    return len(geoms)


def add_drill(spec, parent=PARENT, prefix="drill", remove_hand=HAND, flange=FLANGE):
    """Add the drill bodies on the wrist flange of `parent` and take the hand off. Returns the spec.

    remove_hand=None keeps the hand's mesh (the drill then goes through it)."""
    wrist = spec.body(parent)
    if wrist is None:
        raise ValueError(f"no body {parent!r} in the model")
    if remove_hand:
        _remove_hand(spec, remove_hand)
    cyl, box, sph = mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_SPHERE
    # Positions from the flange face, along x. The wrist link ends there with a ~3 cm radius.
    body = wrist.add_body(name=f"{prefix}_body", pos=list(flange))
    _geom(body, f"{prefix}_adapter", cyl, [0.033, 0.007, 0], [0.007, 0.0, 0.0], [0.22, 0.22, 0.25, 1.0], True)
    _geom(body, f"{prefix}_motor", cyl, [0.030, 0.050, 0], [0.064, 0.0, 0.0], [0.95, 0.55, 0.05, 1.0], True)
    _geom(body, "drill_lamp", cyl, [0.0315, 0.008, 0], [0.026, 0.0, 0.0], RING_OFF, True)    # rear ring
    _geom(body, "drill_ring", cyl, [0.0315, 0.006, 0], [0.100, 0.0, 0.0], RING_OFF, True)    # front ring
    _geom(body, f"{prefix}_nose", cyl, [0.022, 0.010, 0], [0.124, 0.0, 0.0], [0.22, 0.22, 0.25, 1.0], True)
    bit = body.add_body(name=BIT_BODY, pos=[0.134, 0.0, 0.0])        # turns about its x axis
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
