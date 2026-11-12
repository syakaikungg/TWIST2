#!/usr/bin/env python3
"""Design the exhibition's "working with the driver" arm motion for play_motion.py, with low motor load.

The real G1 heats the right shoulder pitch at about 1 C/s while the arm with the tool is held forward (6-8 Nm,
2026-11-10). Heat goes with torque squared, so the motion keeps the upper arm hanging and moves mostly the
forearm and the wrist: the driver points forward and down, does a small push in and back (driving a screw), a
wrist roll twist, and moves sideways to the next screw by turning the upper arm about its own axis (shoulder
yaw, which gravity does not load while the upper arm hangs).

On the simulator's model (arm_sim.build_model: pelvis fixed, the drill's collision capsules, a payload for the
tool) it searches
  rest pose: right arm hanging, driver pointing down (lowest torque, kept away from the thigh), used to cool down
  work pose: driver tip in front of the body, pointing forward and down, lowest gravity torque
then solves the key poses around the work pose (tip position and driver direction, damped least squares),
joins them with minimum-jerk segments under a joint speed limit, and checks every frame: joint speed, gravity
torque, distance from the body and the left arm, joint limits. Writes a motion file (JSON) for play_motion.py.

  python3 design_motion.py --out motions/drill_work.json [--payload-kg 1.0] [--max-vel 0.4]
Needs mujoco and the TWIST2 model (~/nextech/TWIST2/assets/g1/g1_sim2sim_29dof.xml), as arm_sim.py.
"""
import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np

import arm_sim
import tool_df033d

LEFT = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
        "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"]
RIGHT = [n.replace("left_", "right_") for n in LEFT]
ARM = LEFT + RIGHT
TIP_X = 0.245                        # driver tip from the flange (sim_drill.COLL_CAPSULES), along the forearm x
# Body parts the arms must stay away from (collision geoms of these bodies).
BODY = ["pelvis", "torso_link", "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link",
        "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link"]
LEFT_ARM_BODIES = ["left_elbow_link", "left_wrist_roll_link", "left_wrist_pitch_link", "left_wrist_yaw_link",
                   "left_rubber_hand"]
RIGHT_TOOL_BODIES = ["right_elbow_link", "right_wrist_roll_link", "right_wrist_pitch_link", "right_wrist_yaw_link",
                     "drill_body", tool_df033d.BODY]
# Torque weights for the load: the wrist pitch / yaw motors are the small 5 Nm ones.
TAU_W = {"right_shoulder_pitch_joint": 1.0, "right_shoulder_roll_joint": 1.0, "right_shoulder_yaw_joint": 1.0,
         "right_elbow_joint": 1.0, "right_wrist_roll_joint": 1.0, "right_wrist_pitch_joint": 6.0,
         "right_wrist_yaw_joint": 6.0}
TAU_W.update({n.replace("right_", "left_"): w for n, w in TAU_W.items()})


class Model:
    def __init__(self, payload_kg, payload_offset, tool="df033d"):
        self.tool = tool
        self.m = arm_sim.build_model(payload_kg, payload_offset, drill=True, tool=tool)
        self.d = mujoco.MjData(self.m)
        m = self.m
        jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in arm_sim.JOINT_NAMES}
        self.qadr = {n: int(m.jnt_qposadr[j]) for n, j in jid.items()}
        self.vadr = {n: int(m.jnt_dofadr[j]) for n, j in jid.items()}
        self.range = {n: tuple(m.jnt_range[j]) for n, j in jid.items()}
        bid = lambda b: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
        self.drill = bid("drill_body" if tool == "drill" else tool_df033d.BODY)
        self.hand_l = bid("left_rubber_hand")

        def coll(bodies):
            ids = {bid(b) for b in bodies}
            return [g for g in range(m.ngeom) if m.geom_bodyid[g] in ids and m.geom_contype[g]]
        self.g_body = coll(BODY)
        self.g_left = coll(LEFT_ARM_BODIES)
        self.g_right = coll(RIGHT_TOOL_BODIES)
        self.fromto = np.zeros(6)

    def set(self, q):
        """q: dict joint -> angle (others 0: waist upright, as the bridge's --waist-level)."""
        self.d.qpos[:] = 0.0
        self.d.qvel[:] = 0.0
        for n, v in q.items():
            self.d.qpos[self.qadr[n]] = v
        mujoco.mj_forward(self.m, self.d)

    def tip(self):
        """The driver's tip (the provisional drill's bit, or the DF033D's chuck)."""
        if self.tool == "df033d":
            return tool_df033d.chuck(self.m, self.d)[0]
        return self.d.xpos[self.drill] + self.d.xmat[self.drill].reshape(3, 3)[:, 0] * TIP_X

    def axis(self):
        return self.d.xmat[self.drill].reshape(3, 3)[:, 0].copy()   # both point along the forearm

    def gravity(self, names):
        return np.array([self.d.qfrc_bias[self.vadr[n]] for n in names])

    def dist(self, ga, gb, cap=0.15):
        best = cap
        for a in ga:
            for b in gb:
                best = min(best, mujoco.mj_geomDistance(self.m, self.d, a, b, cap, self.fromto))
        return best

    def clearance(self):
        """(right arm + driver vs body, right vs left arm, left arm vs body) [m], capped at 0.15."""
        return (self.dist(self.g_right, self.g_body), self.dist(self.g_right, self.g_left),
                self.dist(self.g_left, self.g_body))


def axis_angles(a):
    """Driver direction: degrees below horizontal, heading in degrees (0 = forward, + = to the robot's left)."""
    return math.degrees(math.asin(-a[2])), math.degrees(math.atan2(a[1], a[0]))


def load(md, names=RIGHT):
    tau = md.gravity(names)
    return float(sum(TAU_W[n] * t * t for n, t in zip(names, tau))), tau


def search_left(md):
    """Left arm (no tool): lowest load with 5 cm from the body."""
    best = None
    for sp in np.arange(-0.10, 0.41, 0.05):
        for sr in np.arange(0.05, 0.41, 0.05):
            for el in np.arange(0.80, 1.56, 0.05):
                q = {n: 0.0 for n in ARM}
                q.update({LEFT[0]: sp, LEFT[1]: sr, LEFT[3]: el})
                md.set(q)
                c, _ = load(md, LEFT)
                if best is not None and c >= best[0]:
                    continue
                if md.dist(md.g_left, md.g_body) < 0.05:
                    continue
                best = (c, q)
    return {n: best[1][n] for n in LEFT}


def search_rest(md, left, clear=0.04):
    """Right arm hanging with the driver pointing down: lowest load with `clear` m from the thigh."""
    cands = []
    for sp in np.arange(-0.20, 0.51, 0.05):
        for sr in np.arange(-0.40, -0.04, 0.05):
            for el in np.arange(0.60, 1.56, 0.05):
                for wp in np.arange(-0.60, 0.61, 0.20):
                    q = dict(left, **{n: 0.0 for n in RIGHT})
                    q.update({RIGHT[0]: sp, RIGHT[1]: sr, RIGHT[3]: el, RIGHT[5]: wp})
                    md.set(q)
                    cands.append((load(md)[0], q))
    cands.sort(key=lambda c: c[0])
    for c, q in cands:
        md.set(q)
        if md.dist(md.g_right, md.g_body) >= clear:
            return q
    raise SystemExit("no rest pose found")


def search_work(md, left, down=(30.0, 50.0), tip_x=0.22, tip_z=(0.50, 0.85)):
    """Candidate work poses, lowest load first: driver tip in front of the body, pointing forward and down `down`
    degrees, 6 cm clear of the body. A generator: the caller takes the first whose whole loop stays clear."""
    cands = []
    for sp in np.arange(-0.40, 0.41, 0.05):
        for sr in np.arange(-0.35, -0.04, 0.05):
            for el in np.arange(0.20, 1.31, 0.05):
                for wp in np.arange(-0.80, 0.81, 0.10):
                    q = dict(left, **{n: 0.0 for n in RIGHT})
                    q.update({RIGHT[0]: sp, RIGHT[1]: sr, RIGHT[3]: el, RIGHT[5]: wp})
                    md.set(q)
                    dn, hd = axis_angles(md.axis())
                    tp = md.tip()
                    if not (down[0] <= dn <= down[1] and abs(hd) <= 12.0 and tp[0] >= tip_x
                            and tip_z[0] <= tp[2] <= tip_z[1]):
                        continue
                    cands.append((load(md)[0], q))
    cands.sort(key=lambda c: c[0])
    for c, q in cands[:3000]:
        md.set(q)
        if md.dist(md.g_right, md.g_body) >= 0.06:
            yield q


def solve(md, q0, tip_goal, axis_goal, roll, iters=200):
    """Damped least squares on the right shoulder (3), elbow and wrist pitch / yaw for the driver tip position
    and direction; wrist roll set to `roll`; pulled lightly toward q0 (the work pose)."""
    J = [RIGHT[0], RIGHT[1], RIGHT[2], RIGHT[3], RIGHT[5], RIGHT[6]]
    q = dict(q0)
    q[RIGHT[4]] = roll
    m, d = md.m, md.d
    jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    cols = [md.vadr[n] for n in J]
    for _ in range(iters):
        md.set(q)
        tp, ax = md.tip(), md.axis()
        e = np.concatenate([tip_goal - tp, np.cross(ax, axis_goal) * 0.3])
        if np.linalg.norm(e[:3]) < 2e-4 and np.linalg.norm(e[3:]) < 2e-4:
            break
        mujoco.mj_jac(m, d, jacp, jacr, tp, md.drill)
        A = np.vstack([jacp[:, cols], jacr[:, cols] * 0.3])
        lam = 1e-3
        dq = A.T @ np.linalg.solve(A @ A.T + lam * np.eye(6), e)
        # null space: toward q0 (keeps the upper arm hanging)
        Nproj = np.eye(len(J)) - np.linalg.pinv(A) @ A
        dq += Nproj @ (0.05 * np.array([q0[n] - q[n] for n in J]))
        for k, n in enumerate(J):
            lo, hi = md.range[n]
            q[n] = float(np.clip(q[n] + dq[k], lo + 0.05, hi - 0.05))
    return q


def rot_about_z(v, ang):
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


def keyframes(md, work, spots=(0.0, -0.07, -0.14), push=0.04, twist=0.4, turn_per_m=150.0):
    """Key poses of one loop and the time to hold each (seconds). The loop starts and ends at the first screw's
    approach pose. Screws at sideways offsets `spots` from the work pose [m] (- = outward, to the robot's right:
    inward the battery under the wrist comes to the hip); the driver turns turn_per_m deg per m with them. Each:
    approach (driver `push` m back), push in while twisting the wrist roll (driving the screw), hold, back out
    while untwisting."""
    md.set(work)
    tip0, ax0 = md.tip(), md.axis()
    keys = []

    def screw(offset_y, turn_deg, label):
        ax = rot_about_z(ax0, math.radians(turn_deg))
        spot = tip0 + np.array([0.0, offset_y, 0.0])
        qa = solve(md, work, spot - ax * push, ax, 0.0)
        qp = solve(md, qa, spot, ax, twist)
        keys.append((qa, 0.3, label + " approach"))
        keys.append((qp, 0.6, label + " push + twist"))
        keys.append((qa, 0.2, label + " back"))

    for k, y in enumerate(spots):
        screw(y, y * turn_per_m, f"screw {k + 1} ({y * 100:+.0f} cm)")
    keys.append((keys[0][0], 0.0, "screw 1 approach (loop)"))
    return keys


def min_jerk(a, b, n):
    s = np.linspace(0.0, 1.0, n)
    s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return a[None, :] + (b - a)[None, :] * s[:, None]


def segment(qa, qb, rate, vmax, tmin):
    """Minimum-jerk from qa to qb (arrays), the time chosen so the peak speed is at most vmax."""
    dq = float(np.max(np.abs(qb - qa)))
    T = max(tmin, 1.875 * dq / vmax)
    n = max(2, int(round(T * rate)) + 1)
    return min_jerk(qa, qb, n)[1:]


def vec(q):
    return np.array([q[n] for n in ARM])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(Path(__file__).with_name("motions") / "drill_work.json"))
    ap.add_argument("--tool", choices=["df033d", "drill"], default="df033d",
                    help="df033d = the real tool with estimated masses (tool_df033d.py); drill = provisional shape "
                         "with --payload-kg")
    ap.add_argument("--payload-kg", type=float, default=1.0, help="--tool drill only: tool mass")
    ap.add_argument("--payload-offset", type=float, default=0.08, help="--tool drill only: centre of mass past the "
                    "flange [m]")
    ap.add_argument("--max-vel", type=float, default=0.4, help="joint speed limit of the motion [rad/s]")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--spots", default="0,-0.07,-0.14",
                    help="sideways offsets of the screws from the work pose [m] (- = outward)")
    ap.add_argument("--push", type=float, default=0.04, help="push in along the driver [m]")
    ap.add_argument("--twist", type=float, default=0.4, help="wrist roll twist while pushing in [rad]")
    ap.add_argument("--down", default="30,50", help="driver angle below horizontal at the work pose [deg]")
    ap.add_argument("--clear", type=float, default=0.04, help="distance kept from the body and the left arm [m]")
    args = ap.parse_args()

    t0 = time.time()
    md = Model(args.payload_kg, args.payload_offset, args.tool)
    left = search_left(md)          # the left arm stays here all the time
    rest = search_rest(md, left)
    dn = [float(v) for v in args.down.split(",")]
    spots = [float(v) for v in args.spots.split(",")]
    tried = 0
    for work in search_work(md, left, down=(dn[0], dn[1])):
        tried += 1
        keys = keyframes(md, work, spots=spots, push=args.push, twist=args.twist)
        frames = [vec(keys[0][0])]
        labels = []
        for (qa, hold, la), (qb, _, lb) in zip(keys[:-1], keys[1:]):
            if hold > 0:
                frames += [frames[-1]] * int(round(hold * args.rate))
            frames += list(segment(vec(qa), vec(qb), args.rate, args.max_vel, 0.6))
            labels.append(la)
        loop = np.array(frames)
        ok = True
        for f in loop[::5]:             # the whole loop (every 0.1 s) stays clear of the body by args.clear
            md.set(dict(zip(ARM, f)))
            if md.dist(md.g_right, md.g_body) < args.clear or md.dist(md.g_right, md.g_left) < args.clear:
                ok = False
                break
        if ok:
            break
    else:
        raise SystemExit("no work pose whose loop stays clear")
    print(f"search {time.time() - t0:.1f} s ({tried} work poses tried)")

    # Checks on every frame.
    dt = 1.0 / args.rate
    vel = np.abs(np.diff(loop, axis=0)) / dt
    taus, clear, tips = [], [], []
    for f in loop:
        md.set(dict(zip(ARM, f)))
        taus.append(md.gravity(RIGHT))
        clear.append(md.clearance())
        tips.append(md.tip())
    taus, clear, tips = np.array(taus), np.array(clear), np.array(tips)
    lim_margin = min(min(f[k] - md.range[n][0], md.range[n][1] - f[k]) for f in loop for k, n in enumerate(ARM))

    def pose_report(name, q):
        md.set(q)
        c, tau = load(md)
        dn_, hd = axis_angles(md.axis())
        tl = md.gravity(LEFT)
        return {"name": name, "right_tau_Nm": dict(zip(RIGHT, np.round(tau, 2).tolist())),
                "left_tau_max_Nm": round(float(np.abs(tl).max()), 2),
                "tip_m": np.round(md.tip(), 3).tolist(), "driver_down_deg": round(dn_, 1),
                "driver_heading_deg": round(hd, 1), "clearance_m": np.round(md.clearance(), 3).tolist()}

    rms = np.sqrt(np.mean(taus ** 2, axis=0))
    report = {
        "tool": args.tool,
        "payload_kg": round(tool_df033d.MASS, 2) if args.tool == "df033d" else args.payload_kg,
        "max_vel": args.max_vel,
        "loop_s": round(len(loop) / args.rate, 2),
        "peak_joint_speed": round(float(vel.max()), 3),
        "right_tau_rms_Nm": dict(zip(RIGHT, np.round(rms, 2).tolist())),
        "right_tau_max_Nm": dict(zip(RIGHT, np.round(np.abs(taus).max(axis=0), 2).tolist())),
        "min_clearance_m": {"right_vs_body": round(float(clear[:, 0].min()), 3),
                            "right_vs_left_arm": round(float(clear[:, 1].min()), 3),
                            "left_vs_body": round(float(clear[:, 2].min()), 3)},
        "joint_limit_margin_rad": round(float(lim_margin), 3),
        "tip_range_m": np.round(tips.max(axis=0) - tips.min(axis=0), 3).tolist(),
        "poses": [pose_report("rest", rest), pose_report("work", work)],
        "keys": labels,
    }
    out = {
        "format": "g1_arm_motion/1",
        "name": "drill_work",
        "description": "driver work: upper arm hanging, driver forward and down; three screws (push, wrist twist, "
                       "back) with sideways steps by the shoulder yaw. Rest: arms hanging, driver down.",
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": "design_motion.py",
        "rate": args.rate,
        "names": ARM,
        "rest": [round(float(v), 4) for v in vec(rest)],
        "loop": [[round(float(v), 4) for v in f] for f in loop],
        "check": report,
    }
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=None, separators=(",", ":")))
    print(json.dumps(report, indent=1, ensure_ascii=False))
    print(f"wrote {path} ({len(loop)} frames, {len(loop) / args.rate:.1f} s per loop) in {time.time() - t0:.1f} s")


if __name__ == "__main__":
    main()
