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
# Collision shapes (add_drill(collision=True)): capsules along x from the flange, covering the motor, the chuck
# and the bit. Hidden (group 3), no mass. name, radius, x from, x to.
COLL_CAPSULES = (("drill_coll_motor", 0.034, 0.0, 0.134), ("drill_coll_chuck", 0.018, 0.134, 0.188),
                 ("drill_coll_bit", 0.006, 0.188, 0.245))
DRILL_BODIES = ("drill_body", "drill_bit")
# A box around a hand (the hands have no collision shape in the G1 models), in the hand body's frame.
# The right hand's mesh spans x 0..0.132, y -0.019..0.048, z -0.043..0.063; the left one is its mirror in y.
HAND_BOX_HALF = [0.066, 0.034, 0.053]
HAND_BOX_POS = [0.066, 0.015, 0.010]
COLL_RGBA = [1.0, 0.3, 0.3, 0.35]


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


def _exclude(spec, b1, b2):
    """No contacts between two bodies (e.g. a tool and the wrist links it is bolted next to)."""
    if spec.body(b1) is not None and spec.body(b2) is not None:
        e = spec.add_exclude()
        e.bodyname1, e.bodyname2 = b1, b2


def add_hand_collision(spec, hand):
    """Give a hand body (left_rubber_hand / right_rubber_hand) a box collision shape. Returns the geom or None."""
    body = spec.body(hand)
    if body is None:
        return None
    sign = 1.0 if hand.startswith("right") else -1.0
    g = body.add_geom(name=f"{hand}_coll", type=mujoco.mjtGeom.mjGEOM_BOX, size=list(HAND_BOX_HALF),
                      pos=[HAND_BOX_POS[0], sign * HAND_BOX_POS[1], HAND_BOX_POS[2]], rgba=COLL_RGBA,
                      group=3, density=0.0)
    side = hand.split("_")[0]
    for link in ("wrist_pitch_link", "wrist_roll_link"):
        _exclude(spec, hand, f"{side}_{link}")
    return g


def add_drill(spec, parent=PARENT, prefix="drill", remove_hand=HAND, flange=None, collision=False):
    """Add the drill bodies on the wrist flange of `parent` and take the hand off. Returns the spec.

    flange: where the drill starts, in the wrist frame (default: the hand body's position, else FLANGE).
    remove_hand=None keeps the hand's mesh (the drill then goes through it).
    collision=True also adds the collision capsules (COLL_CAPSULES, hidden, no mass), for the IK's
    collision avoidance and the simulator's contacts. Without it the drill is looks only."""
    wrist = spec.body(parent)
    if wrist is None:
        raise ValueError(f"no body {parent!r} in the model")
    if flange is None:
        hb = spec.body(HAND)
        flange = list(hb.pos) if hb is not None else FLANGE
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
    if collision:
        cap = mujoco.mjtGeom.mjGEOM_CAPSULE
        for name, r, x0, x1 in COLL_CAPSULES:
            body.add_geom(name=name, type=cap, size=[r, (x1 - x0) / 2 - r, 0], pos=[(x0 + x1) / 2, 0.0, 0.0],
                          quat=_Z_TO_X, rgba=COLL_RGBA, group=3, density=0.0)
        for link in ("wrist_pitch_link", "wrist_roll_link"):
            _exclude(spec, f"{prefix}_body", parent.replace("wrist_yaw_link", link))
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
