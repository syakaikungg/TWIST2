#!/usr/bin/env python3
"""Arm simulator for the G1 29DOF: pelvis and legs welded, arms + waist driven like the real arm_sdk.

No robot I/O. Reads the bridge's command from Redis `sim_lowcmd` (weight, q, kp, kd for arms + waist, as
arm_sdk_bridge.py --sim writes it) and applies, per joint,
    tau = w * (kp (q_cmd - q) - kd dq) + (1 - w) * (kp_b (q_builtin - q) - kd_b dq)
clipped to the motor limits (wrist pitch/yaw 5 Nm, as on the real G1), where "builtin" stands in for the
robot's own controller holding its standing arm posture. Steps MuJoCo in real time and writes
`sim_lowstate` (q, dq, tau) for the bridge and `sim_qpos` (joint angles by name) for viewers.
Records self-contacts (arm vs torso/head/pelvis/other arm), torques, tracking error and joint-limit proximity.
The right wrist carries a drill in place of the hand (sim_drill.py, no mass): it spins while IK's Redis
`drill_cmd` says ON, and its state goes into `sim_qpos` as "drill" for the viewers. Its collision capsules and
a box on each remaining hand count as arm bodies in the contact check. --no-drill leaves it out.

  python arm_sim.py --duration 40 [--payload-kg 1.5 --payload-offset 0.05] [--video out.mp4]
"""
import argparse
import csv
import json
import os
import time
from pathlib import Path

import mujoco
import numpy as np
import redis

import sim_drill

MODEL = str(Path.home() / "nextech" / "TWIST2" / "assets" / "g1" / "g1_sim2sim_29dof.xml")
ARM_NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
JOINT_NAMES = ARM_NAMES + ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
# Standing posture measured on the real G1 under the built-in controller (rt/lowstate, 2026-11-03).
BUILTIN_POSE = [0.310, 0.408, -0.194, 0.903, 0.040, -0.094, 0.042,
                0.316, -0.219, 0.120, 0.895, 0.066, 0.016, -0.088,
                -0.054, -0.037, 0.057]
WRIST_5NM = ["left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"]
# Motor rotor inertia (armature) by motor type, as in MuJoCo Menagerie's unitree_g1. Without it the light
# wrist links make the 1 kHz PD unstable (the TWIST2 sim2sim model has none on the arms).
ARMATURE = {"N5020": 0.003609725, "W4010": 0.00425, "N7520-14": 0.01017752}
JOINT_MOTOR = {n: "N5020" for n in ARM_NAMES}
JOINT_MOTOR.update({n: "W4010" for n in WRIST_5NM})
JOINT_MOTOR.update({"waist_yaw_joint": "N7520-14", "waist_roll_joint": "N5020", "waist_pitch_joint": "N5020"})
ARM_BODY_KEYS = ("shoulder", "elbow", "wrist", "rubber_hand", "drill")   # drill_body, drill_bit, drill_payload


def build_model(payload_kg, payload_offset, drill=True):
    spec = mujoco.MjSpec.from_file(MODEL)
    keep = set(JOINT_NAMES)
    for a in list(spec.actuators):
        if a.target not in keep:
            spec.delete(a)
    for s in list(spec.sensors):
        if s.objtype == mujoco.mjtObj.mjOBJ_JOINT and s.objname not in keep:
            spec.delete(s)
    for j in list(spec.joints):          # free joint and legs removed -> welded to the world
        if j.name not in keep:
            spec.delete(j)
    for k in list(spec.keys):            # keyframes refer to the full qpos
        spec.delete(k)
    for j in spec.joints:                # the model gives the right wrist 25 Nm; the real motors are 5 Nm
        if j.name in WRIST_5NM:
            j.actfrclimited = mujoco.mjtLimited.mjLIMITED_TRUE
            j.actfrcrange = [-5.0, 5.0]
        if j.name in JOINT_MOTOR:
            j.armature = ARMATURE[JOINT_MOTOR[j.name]]
    if payload_kg > 0:
        wrist = spec.body("right_wrist_yaw_link")
        hand = spec.body("right_rubber_hand")
        d = np.array(hand.pos, dtype=float)
        d = d / (np.linalg.norm(d) or 1.0)
        b = wrist.add_body(name="drill_payload", pos=(np.array(hand.pos) + payload_offset * d).tolist())
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], mass=payload_kg,
                   contype=0, conaffinity=0, rgba=[1.0, 0.5, 0.0, 0.0 if drill else 1.0])   # hidden by the drill
    if drill:   # no mass; its collision capsules make contacts with the body / other arm (none at rest)
        sim_drill.add_drill(spec, collision=True)
    for hand in (["left_rubber_hand"] if drill else ["left_rubber_hand", "right_rubber_hand"]):
        sim_drill.add_hand_collision(spec, hand)   # the hands have no collision shape in the model
    return spec.compile()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--builtin-kp", type=float, default=60.0, help="assumed gains of the robot's own arm control")
    ap.add_argument("--builtin-kd", type=float, default=1.5)
    ap.add_argument("--payload-kg", type=float, default=0.0, help="dummy drill mass at the right hand")
    ap.add_argument("--payload-offset", type=float, default=0.05, help="beyond the hand along the forearm [m]")
    ap.add_argument("--no-drill", action="store_true", help="no drill (the hand stays)")
    ap.add_argument("--drill-rps", type=float, default=3.0, help="drawn spin speed while ON [rev/s] (not the real one)")
    ap.add_argument("--video", default="", help="mp4 path (offscreen render, needs EGL)")
    ap.add_argument("--tag", default="")
    ap.add_argument("--log-dir", default=str(Path.home() / "nextech_setup" / "logs"))
    args = ap.parse_args()

    m = build_model(args.payload_kg, args.payload_offset, drill=not args.no_drill)
    d = mujoco.MjData(m)
    drill_view = sim_drill.DrillView(m)
    spin = sim_drill.DrillSpin(rps=args.drill_rps)
    drill_on, drill_wall = False, 0.0
    jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
    qadr = np.array([m.jnt_qposadr[j] for j in jid])
    vadr = np.array([m.jnt_dofadr[j] for j in jid])
    act = []
    for n in JOINT_NAMES:
        a = next(i for i in range(m.nu) if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[i, 0]) == n)
        act.append(a)
    act = np.array(act)
    tau_lim = np.array([m.jnt_actfrcrange[j][1] if m.jnt_actfrclimited[j] else 1e9 for j in jid])
    jrange = np.array([m.jnt_range[j] for j in jid])
    builtin = np.array(BUILTIN_POSE)
    d.qpos[qadr] = builtin
    mujoco.mj_forward(m, d)

    names_b = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "world" for b in range(m.nbody)]
    arm_b = [any(k in n for k in ARM_BODY_KEYS) for n in names_b]
    side_b = ["left" if n.startswith("left") else ("right" if n.startswith(("right", "drill")) else "")
              for n in names_b]
    bname = lambda b: names_b[b]
    is_arm = lambda b: arm_b[b]
    side = lambda b: side_b[b]

    def self_contacts():
        out = {}
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if not (is_arm(b1) or is_arm(b2)):
                continue
            if is_arm(b1) and is_arm(b2) and side(b1) == side(b2):
                kind = "same-arm"
            elif is_arm(b1) and is_arm(b2):
                kind = "arm-arm"
            else:
                kind = "arm-body"
            key = tuple(sorted((bname(b1), bname(b2))))
            out[key] = (kind, min(out.get(key, ("", 1.0))[1], float(c.dist)))
        return out

    baseline = self_contacts()
    rds = redis.Redis(host="127.0.0.1", port=6379)
    rds.delete("sim_lowcmd")
    renderer = frames = None
    if args.video:
        os.environ.setdefault("MUJOCO_GL", "egl")
        try:
            renderer = mujoco.Renderer(m, 480, 640)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [0.25, 0.0, 1.0]   # behind the robot, slightly above (operator's view)
            cam.distance, cam.azimuth, cam.elevation = 1.8, 0.0, -25.0
            frames = []
        except Exception as e:
            print("offscreen rendering unavailable:", e)
            renderer = None

    tag = args.tag or time.strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"arm_sim_{tag}.csv"
    logf = open(log_path, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "wall", "weight", "cmd_age_ms", "contacts", "contact_pairs"] + [f"q_{n}" for n in JOINT_NAMES]
                 + [f"cmd_{n}" for n in JOINT_NAMES] + [f"tau_{n}" for n in JOINT_NAMES])
    print(f"arm_sim: model {os.path.basename(MODEL)} (pelvis+legs welded), timestep {m.opt.timestep}s, "
          f"payload {args.payload_kg} kg, wrist pitch/yaw limit 5 Nm, motor armature added, "
          f"drill {'off' if args.no_drill else 'drawn'}, log {log_path}")
    if baseline:
        print("contacts already present in the standing pose (ignored as baseline):", list(baseline))

    stats = {"tau_max": np.zeros(len(JOINT_NAMES)), "sat_steps": np.zeros(len(JOINT_NAMES)),
             "err_max": np.zeros(len(JOINT_NAMES)), "near_limit": np.zeros(len(JOINT_NAMES)), "steps": 0}
    contacts = {}
    cmd = {"weight": 0.0, "q": builtin.copy(), "kp": np.array(60.0), "kd": np.array(1.5), "t": 0.0}
    t0 = time.time()
    next_io = next_log = next_frame = 0.0
    try:
        while True:
            wall = time.time() - t0
            if wall > args.duration:
                break
            if wall >= next_io:          # 200 Hz I/O with the bridge
                next_io += 0.005
                raw = rds.get("sim_lowcmd")
                if raw:
                    c = json.loads(raw)
                    cmd = {"weight": float(c["weight"]), "q": np.array(c["q"]), "kp": np.array(c["kp"], dtype=float),
                           "kd": np.array(c["kd"], dtype=float), "t": float(c["t"])}   # scalar or per joint
                if time.time() - cmd["t"] > 0.3:     # no fresh command: the robot's own control only
                    cmd["weight"] = 0.0
                raw = rds.get("drill_cmd")            # IK's drill ON/OFF (stale -> OFF)
                try:
                    c = json.loads(raw) if raw else {}
                    drill_on = bool(c.get("on")) and time.time() - float(c.get("t", 0.0)) < 0.3
                except (ValueError, TypeError):
                    drill_on = False
                rds.set("sim_lowstate", json.dumps({"t": time.time(), "names": JOINT_NAMES,
                                                    "q": d.qpos[qadr].tolist(), "dq": d.qvel[vadr].tolist(),
                                                    "tau": d.actuator_force[act].tolist()}))
            # Advance physics to the wall clock (at most 20 ms of physics per pass, so the I/O keeps running).
            n_steps = 0
            while d.time < wall and n_steps < int(0.02 / m.opt.timestep):
                n_steps += 1
                q, dq = d.qpos[qadr], d.qvel[vadr]
                w = np.clip(cmd["weight"], 0.0, 1.0)
                tau = (w * (cmd["kp"] * (cmd["q"] - q) - cmd["kd"] * dq)
                       + (1 - w) * (args.builtin_kp * (builtin - q) - args.builtin_kd * dq))
                stats["sat_steps"] += np.abs(tau) > tau_lim
                d.ctrl[act] = np.clip(tau, -tau_lim, tau_lim)
                mujoco.mj_step(m, d)
                stats["steps"] += 1
                stats["tau_max"] = np.maximum(stats["tau_max"], np.abs(d.actuator_force[act]))
                if w > 0.9:
                    stats["err_max"] = np.maximum(stats["err_max"], np.abs(cmd["q"] - d.qpos[qadr]))
                stats["near_limit"] += (d.qpos[qadr] < jrange[:, 0] + 0.05) | (d.qpos[qadr] > jrange[:, 1] - 0.05)
                sc = self_contacts()
                for k, (kind, dist) in sc.items():
                    if k in baseline:
                        continue
                    e = contacts.setdefault(k, {"kind": kind, "first_t": d.time, "steps": 0, "min_dist": 1.0})
                    e["steps"] += 1
                    e["min_dist"] = min(e["min_dist"], dist)
            if wall >= next_log:         # 50 Hz record + viewer stream
                next_log += 0.02
                q = d.qpos[qadr]
                drill_state = spin.update(drill_on, wall - drill_wall)
                drill_wall = wall
                rds.set("sim_qpos", json.dumps({"t": time.time(), "joints": dict(zip(JOINT_NAMES, q.tolist())),
                                                "weight": cmd["weight"], "payload_kg": args.payload_kg,
                                                "drill": drill_state,
                                                "contacts": [list(k) for k in self_contacts() if k not in baseline]}))
                pairs = [f"{a}|{b}" for (a, b) in self_contacts() if (a, b) not in baseline]
                log.writerow([f"{d.time:.3f}", f"{time.time():.3f}", f"{cmd['weight']:.3f}",
                              f"{(time.time() - cmd['t']) * 1000:.0f}", len(pairs), ";".join(pairs)]
                             + [f"{v:.4f}" for v in q] + [f"{v:.4f}" for v in cmd["q"]]
                             + [f"{v:.2f}" for v in d.actuator_force[act]])
            if renderer is not None and wall >= next_frame:
                next_frame += 1.0 / 30
                drill_view.apply(m, spin.state())
                mujoco.mj_kinematics(m, d)
                renderer.update_scene(d, camera=cam)
                frames.append(renderer.render().copy())
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass
    finally:
        logf.close()

    rt = d.time / max(1e-6, time.time() - t0)
    print(f"simulated {d.time:.1f}s ({rt:.2f}x real time), {stats['steps']} steps")
    print("max |torque| per joint (Nm) / limit, and steps above the limit (clipped):")
    for i, n in enumerate(JOINT_NAMES):
        print(f"  {n:28s} {stats['tau_max'][i]:6.2f} / {tau_lim[i]:4.0f}  "
              f"({stats['tau_max'][i] / tau_lim[i] * 100:5.1f}%)  clipped {int(stats['sat_steps'][i])}  "
              f"max err(w>0.9) {stats['err_max'][i]:.3f} rad  near-limit steps {int(stats['near_limit'][i])}")
    if contacts:
        print("self-contacts (not present at rest):")
        for k, e in sorted(contacts.items(), key=lambda kv: kv[1]["first_t"]):
            print(f"  {e['kind']:8s} {k[0]} <-> {k[1]}: first at {e['first_t']:.2f}s, "
                  f"{e['steps'] * m.opt.timestep:.2f}s in contact, deepest {e['min_dist'] * 1000:.1f} mm")
    else:
        print("self-contacts (not present at rest): none")
    summary = {"payload_kg": args.payload_kg, "tau_max": dict(zip(JOINT_NAMES, stats["tau_max"].round(2).tolist())),
               "tau_limit": dict(zip(JOINT_NAMES, tau_lim.tolist())),
               "err_max": dict(zip(JOINT_NAMES, stats["err_max"].round(3).tolist())),
               "contacts": {f"{k[0]}|{k[1]}": e for k, e in contacts.items()}, "log": str(log_path)}
    (log_dir / f"arm_sim_{tag}_summary.json").write_text(json.dumps(summary, indent=1))
    if frames:
        import imageio
        imageio.mimsave(args.video, frames, fps=30)
        print(f"video: {args.video} ({len(frames)} frames)")
    print(f"log: {log_path}")


if __name__ == "__main__":
    main()
