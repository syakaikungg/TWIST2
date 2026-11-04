#!/usr/bin/env python3
"""Turn a recorded simulator run into an MP4 seen from the front of the robot (or another view).

The recording is what stream_sim.py prints (Redis `sim_qpos` as JSON lines), saved to a file during a live
session, e.g. on the G1:

    timeout 660 python3 -u stream_sim.py > ~/nextech_setup/logs/rec_sim_qpos_HHMMSS.jsonl

Afterwards (no Redis, no robot I/O; only reads the file):

    python render_front.py rec.jsonl out.mp4 [--view front|back|left|right|drill] [--fps 30]
                           [--width 1280 --height 720] [--start S --end S] [--max-gap 1.0]

Without a recording, arm_sim.py's own log of the run works too (joint angles and contacts at 50 Hz). It has
no drill state, so give the IK's log of the same run for that (its `drill` column, replayed with the same
spin-up and coast as the simulator):

    python render_front.py arm_sim_live_..._HHMMSS.csv out.mp4 --drill-csv arm_ik_..._HHMMSS.csv

The model is the viewers' one (pelvis and legs welded, the drill on the right wrist from sim_drill.py when it
is next to this file). Bodies in contact turn red, the drill spins and its rings turn green while ON, as in
sim_viewer.py. Joint angles are held between samples (sim_qpos comes at 50 Hz). Pauses longer than --max-gap
(e.g. while the simulator was restarted) are shortened to that length.

Offscreen rendering needs an OpenGL backend: MUJOCO_GL=egl (default here, works on the G1), glfw or osmesa.
Encoding: the `ffmpeg` on PATH or imageio-ffmpeg's copy (libx264), else GStreamer's nvv4l2h264enc (Jetson).
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import sim_drill  # noqa: E402
except ImportError:
    sim_drill = None

MODEL_CANDIDATES = [
    os.path.join(HERE, "..", "..", "assets", "g1", "g1_sim2sim_29dof.xml"),        # in the TWIST2 repo
    os.path.expanduser("~/nextech/TWIST2/assets/g1/g1_sim2sim_29dof.xml"),        # G1
    "/mnt/c/Users/masak/sakai/job/Xnova/TWIST2/assets/g1/g1_sim2sim_29dof.xml",   # laptop (WSL)
]
JOINT_NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
]
# lookat, distance, azimuth, elevation. The robot faces +x: azimuth 180 looks at its front.
VIEWS = {
    "front": ([0.15, 0.0, 0.92], 1.5, 180.0, -8.0),
    "back": ([0.25, 0.0, 1.0], 1.8, 0.0, -25.0),        # same as sim_viewer.py (the operator's view)
    "left": ([0.15, 0.0, 0.95], 2.0, 270.0, -10.0),     # from the robot's left side
    "right": ([0.15, 0.0, 0.95], 2.0, 90.0, -10.0),     # from the robot's right side (drill side)
    "drill": ([0.25, -0.15, 0.95], 1.3, 150.0, -15.0),  # front right, closer
}


def find_model(path):
    for p in ([path] if path else MODEL_CANDIDATES):
        if p and os.path.isfile(os.path.expanduser(p)):
            return os.path.expanduser(p)
    raise SystemExit("model not found; pass --model (tried: %s)" % ", ".join(MODEL_CANDIDATES))


def build_model(xml, drill, width, height):
    spec = mujoco.MjSpec.from_file(xml)
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
    if drill:
        sim_drill.add_drill(spec)
    m = spec.compile()
    m.vis.global_.offwidth = max(width, m.vis.global_.offwidth)
    m.vis.global_.offheight = max(height, m.vis.global_.offheight)
    m.mat_reflectance[:] = 0.0
    return m


def load_sim_csv(path):
    """arm_sim.py's log -> the same (t, msg) samples as a recording (wall time, joints, contacts)."""
    msgs = []
    bad = 0
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                joints = {k[2:]: float(v) for k, v in r.items() if k.startswith("q_")}
                pairs = [p.split("|") for p in (r.get("contact_pairs") or "").split(";") if "|" in p]
                msgs.append((float(r["wall"]), {"t": float(r["wall"]), "joints": joints, "contacts": pairs}))
            except Exception:
                bad += 1
    return msgs, bad


def add_drill_from_ik_csv(msgs, path, rps):
    """Fill each sample's "drill" from the IK log's drill column (wall time), replaying the spin."""
    on_t, on_v = [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                on_t.append(float(r["wall"]))
                on_v.append(int(float(r["drill"])))
            except Exception:
                pass
    on_t, on_v = np.array(on_t), np.array(on_v)
    spin = sim_drill.DrillSpin(rps=rps)
    prev = None
    n_on = 0
    for t, msg in msgs:
        k = np.searchsorted(on_t, t, side="right") - 1
        on = bool(on_v[k]) if k >= 0 and t - on_t[k] < 0.3 else False
        msg["drill"] = spin.update(on, 0.0 if prev is None else min(t - prev, 0.1))
        prev = t
        n_on += on
    return n_on


def load_recording(path):
    if path.endswith(".csv"):
        return load_sim_csv(path)
    msgs = []
    bad = 0
    with open(path) as f:
        for line in f:
            try:
                msg = json.loads(line)
                msgs.append((float(msg["t"]), msg))
            except Exception:
                bad += 1
    msgs.sort(key=lambda x: x[0])
    # stream_sim prints whatever is in Redis first, which can be minutes old: drop a stale head
    while len(msgs) > 1 and msgs[1][0] - msgs[0][0] > 5.0:
        msgs.pop(0)
    return msgs, bad


def open_encoder(out, w, h, fps):
    exe = shutil.which("ffmpeg")
    if exe is None:
        try:
            import imageio_ffmpeg
            exe = imageio_ffmpeg.get_ffmpeg_exe()
        except Exception:
            exe = None
    if exe:
        cmd = [exe, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
               "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", out]
        return subprocess.Popen(cmd, stdin=subprocess.PIPE), "ffmpeg (%s)" % exe
    gst = shutil.which("gst-launch-1.0")
    if gst:
        pipe = (f"fdsrc fd=0 ! rawvideoparse width={w} height={h} format=rgb framerate={fps}/1 ! videoconvert ! "
                f"video/x-raw,format=I420 ! nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! "
                f"nvv4l2h264enc bitrate=8000000 ! h264parse ! qtmux ! filesink location={out}")
        return subprocess.Popen([gst, "-q", "-e"] + pipe.split(), stdin=subprocess.PIPE), "gstreamer nvv4l2h264enc"
    raise SystemExit("no encoder: install ffmpeg or imageio-ffmpeg, or run on the Jetson with GStreamer")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("out")
    ap.add_argument("--view", choices=sorted(VIEWS), default="front")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--start", type=float, default=0.0, help="seconds from the first sample")
    ap.add_argument("--end", type=float, default=0.0, help="seconds from the first sample (0 = to the end)")
    ap.add_argument("--max-gap", type=float, default=1.0, help="shorten pauses longer than this [s]")
    ap.add_argument("--drill-csv", default="", help="the IK's log of the run (for an arm_sim CSV input)")
    ap.add_argument("--drill-rps", type=float, default=3.0)
    ap.add_argument("--model", default="")
    ap.add_argument("--no-drill", action="store_true")
    ap.add_argument("--shadows", action="store_true")
    args = ap.parse_args()

    msgs, bad = load_recording(args.recording)
    if not msgs:
        raise SystemExit("no samples in %s" % args.recording)
    if args.drill_csv and sim_drill is not None:
        n_on = add_drill_from_ik_csv(msgs, args.drill_csv, args.drill_rps)
        print(f"drill: ON in {n_on} of {len(msgs)} samples (from {os.path.basename(args.drill_csv)})", flush=True)
    t0 = msgs[0][0]
    span = msgs[-1][0] - t0
    print(f"recording: {len(msgs)} samples over {span:.1f} s ({bad} unreadable lines)", flush=True)

    # Timeline: sample times with long pauses squeezed to --max-gap
    ts = np.array([t for t, _ in msgs]) - t0
    keep = (ts >= args.start) & ((ts <= args.end) if args.end > 0 else True)
    idx = np.nonzero(keep)[0]
    if len(idx) == 0:
        raise SystemExit("nothing between --start and --end")
    vt = np.zeros(len(idx))   # video time of each kept sample
    for k in range(1, len(idx)):
        vt[k] = vt[k - 1] + min(ts[idx[k]] - ts[idx[k - 1]], args.max_gap)
    n_frames = int(vt[-1] * args.fps) + 1

    drill = sim_drill is not None and not args.no_drill
    m = build_model(find_model(args.model), drill, args.width, args.height)
    d = mujoco.MjData(m)
    qadr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES}
    body_id = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b): b for b in range(m.nbody)}
    rgba0 = m.geom_rgba.copy()
    drill_view = sim_drill.DrillView(m) if drill else None
    renderer = mujoco.Renderer(m, args.height, args.width)
    cam = mujoco.MjvCamera()
    look, dist, az, el = VIEWS[args.view]
    cam.lookat[:] = look
    cam.distance, cam.azimuth, cam.elevation = dist, az, el

    enc, how = open_encoder(args.out, args.width, args.height, args.fps)
    print(f"rendering {n_frames} frames ({n_frames / args.fps:.1f} s of video, view {args.view}, "
          f"{args.width}x{args.height} @ {args.fps} fps), encoder {how}", flush=True)
    t_start = time.time()
    last_report = t_start
    j = 0
    shown = -1
    for f in range(n_frames):
        ft = f / args.fps
        while j + 1 < len(idx) and vt[j + 1] <= ft:
            j += 1
        if j != shown:                      # a new sample: pose, contact colours, drill
            shown = j
            msg = msgs[idx[j]][1]
            for n, q in msg.get("joints", {}).items():
                if n in qadr:
                    d.qpos[qadr[n]] = q
            m.geom_rgba[:] = rgba0
            for pair in msg.get("contacts", []):
                for bn in pair:
                    b = body_id.get(bn)
                    if b is not None:
                        m.geom_rgba[m.geom_bodyid == b] = [1.0, 0.1, 0.1, 1.0]
            if drill_view is not None:
                drill_view.apply(m, msg.get("drill"))
            mujoco.mj_kinematics(m, d)
        renderer.update_scene(d, camera=cam)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = bool(args.shadows)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        enc.stdin.write(renderer.render().tobytes())
        now = time.time()
        if now - last_report > 10.0 or f == n_frames - 1:
            last_report = now
            done = (f + 1) / n_frames
            eta = (now - t_start) / done * (1 - done)
            print(f"  {f + 1}/{n_frames} frames ({done * 100:.0f}%), {now - t_start:.0f} s so far, "
                  f"about {eta:.0f} s left", flush=True)
    enc.stdin.close()
    rc = enc.wait()
    size = os.path.getsize(args.out) if os.path.exists(args.out) else 0
    print(f"video: {args.out} ({size / 1e6:.1f} MB, encoder exit {rc}, {time.time() - t_start:.0f} s)", flush=True)
    sys.exit(0 if rc == 0 and size > 0 else 1)


if __name__ == "__main__":
    main()
