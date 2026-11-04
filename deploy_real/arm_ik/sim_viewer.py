#!/usr/bin/env python3
"""Show the G1 arm simulator (running on the robot's onboard PC) in a MuJoCo window on this laptop.

Joint angles arrive over SSH: `stream_sim.py` on the G1 prints Redis `sim_qpos` as JSON lines, so no
network port is opened on either side. The model is the same TWIST2 g1_sim2sim_29dof.xml (this PC's copy
of the repo, same commit) with the pelvis and legs welded, as in arm_sim.py. Bodies in contact turn red.
The right hand carries the drill drawn by sim_drill.py (put it next to this file): it spins and its lamps
turn green while the simulator says the drill is ON.

  ~/nextech_viewer/venv/bin/python sim_viewer.py [--payload-kg 1.5] [--no-drill]
"""
import argparse
import json
import os
import subprocess
import threading
import time

# WSLg falls back to software OpenGL (llvmpipe: ~6 CPU cores busy) unless the D3D12 driver is asked for,
# which renders on the laptop's GPU. Must be set before the GL context is created.
os.environ.setdefault("GALLIUM_DRIVER", "d3d12")

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
import numpy as np  # noqa: E402

try:
    import sim_drill  # noqa: E402
except ImportError:      # no sim_drill.py next to this file: draw without the drill
    sim_drill = None

MODEL = "/mnt/c/Users/masak/sakai/job/Xnova/TWIST2/assets/g1/g1_sim2sim_29dof.xml"
JOINT_NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
]


def build_model(payload_kg, payload_offset, drill=True):
    spec = mujoco.MjSpec.from_file(MODEL)
    keep = set(JOINT_NAMES)
    for a in list(spec.actuators):
        if a.target not in keep:
            spec.delete(a)
    for s in list(spec.sensors):
        if s.objtype == mujoco.mjtObj.mjOBJ_JOINT and s.objname not in keep:
            spec.delete(s)
    for j in list(spec.joints):
        if j.name not in keep:
            spec.delete(j)
    for k in list(spec.keys):
        spec.delete(k)
    if payload_kg > 0:
        wrist, hand = spec.body("right_wrist_yaw_link"), spec.body("right_rubber_hand")
        d = np.array(hand.pos, dtype=float)
        d = d / (np.linalg.norm(d) or 1.0)
        b = wrist.add_body(name="drill_payload", pos=(np.array(hand.pos) + payload_offset * d).tolist())
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0], mass=payload_kg,
                   contype=0, conaffinity=0, rgba=[1.0, 0.5, 0.0, 0.0 if drill else 1.0])   # hidden by the drill
    if drill:
        sim_drill.add_drill(spec)
    return spec.compile()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="unitree@192.168.123.164")
    ap.add_argument("--key", default="/home/masak/.ssh/id_ed25519_g1")
    ap.add_argument("--payload-kg", type=float, default=0.0, help="draw the dummy drill mass (display only)")
    ap.add_argument("--payload-offset", type=float, default=0.05)
    ap.add_argument("--no-drill", action="store_true", help="leave out the drill drawn at the right hand")
    ap.add_argument("--pretty", action="store_true", help="keep shadows/reflections (slower on a laptop GPU)")
    args = ap.parse_args()

    drill = sim_drill is not None and not args.no_drill
    if sim_drill is None and not args.no_drill:
        print("viewer: sim_drill.py not found next to sim_viewer.py, drawing without the drill", flush=True)
    m = build_model(args.payload_kg, args.payload_offset, drill=drill)
    if not args.pretty:
        m.mat_reflectance[:] = 0.0
        m.vis.quality.shadowsize = 1024
    d = mujoco.MjData(m)
    qadr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES}
    body_id = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b): b for b in range(m.nbody)}
    rgba0 = m.geom_rgba.copy()
    drill_view = sim_drill.DrillView(m) if drill else None
    mujoco.mj_forward(m, d)

    latest = {"msg": None, "n": 0, "arr": []}   # arr: arrival times since the last log line
    proc = subprocess.Popen(
        ["ssh", "-i", args.key, "-o", "BatchMode=yes", "-o", "ServerAliveInterval=10", args.host,
         "python3 -u ~/nextech/arm_ik/stream_sim.py"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    def reader():
        while True:
            line = proc.stdout.readline()
            if not line:
                latest["eof"] = True
                return
            try:
                latest["msg"] = json.loads(line)
                latest["n"] += 1
                latest["arr"].append(time.monotonic())
            except Exception as e:   # keep reading whatever happens to one line
                latest["err"] = repr(e)

    threading.Thread(target=reader, daemon=True).start()
    print("viewer: waiting for the simulator on the G1 (start ~/nextech/arm_ik/sim_run.sh there) ...", flush=True)

    last_print = 0.0
    last_t = None
    frames, sync_ms = 0, []
    # Side panels off: less to draw every frame (Tab / Shift+Tab bring them back).
    with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as v:
        # Behind the robot, slightly above: the robot's left is on the screen's left (same view as the operator).
        v.cam.lookat[:] = [0.25, 0.0, 1.0]
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.8, 0.0, -25.0
        if not args.pretty:   # shadows, reflections and the skybox cost the most on an integrated GPU
            with v.lock():    # the passive viewer syncs user_scn's render flags into its own scene
                v.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                v.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                v.user_scn.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0
        while v.is_running():
            msg = latest["msg"]
            if msg and msg["t"] != last_t:   # only recompute when a new state arrived
                last_t = msg["t"]
                for n, q in msg["joints"].items():
                    if n in qadr:
                        d.qpos[qadr[n]] = q
                m.geom_rgba[:] = rgba0
                for pair in msg.get("contacts", []):
                    for bn in pair:
                        b = body_id.get(bn)
                        if b is not None:
                            m.geom_rgba[m.geom_bodyid == b] = [1.0, 0.1, 0.1, 1.0]
                if drill_view is not None:   # after the colour reset: the lamps are coloured here
                    drill_view.apply(m, msg.get("drill"))
                mujoco.mj_kinematics(m, d)
            t_sync = time.perf_counter()
            v.sync()
            sync_ms.append((time.perf_counter() - t_sync) * 1000)
            frames += 1
            now = time.time()
            if now - last_print > 2.0:
                fps = frames / (now - last_print) if last_print else 0.0
                last_print, frames = now, 0
                sm = np.array(sync_ms) if sync_ms else np.zeros(1)
                sync_ms = []
                arr, latest["arr"] = latest["arr"], []
                gap = np.diff(arr).max() * 1000 if len(arr) > 1 else float("nan")
                if msg:
                    age = now - float(msg["t"])
                    dr = msg.get("drill") or {}
                    print(f"states {latest['n']} (+{len(arr)}, max gap {gap:.0f} ms), "
                          f"view {fps:.0f} fps (sync avg {sm.mean():.1f} / max {sm.max():.1f} ms), "
                          f"weight {msg.get('weight', 0):.2f}, drill {'ON' if dr.get('on') else 'off'}, "
                          f"data age {age:.2f}s, contacts {msg.get('contacts', [])}"
                          f"{', stream ended' if latest.get('eof') else ''}{', last error ' + latest['err'] if latest.get('err') else ''}",
                          flush=True)
                else:
                    print(f"no data yet (ssh {'running' if proc.poll() is None else 'exited'})", flush=True)
            time.sleep(1 / 60)
    proc.terminate()


if __name__ == "__main__":
    main()
