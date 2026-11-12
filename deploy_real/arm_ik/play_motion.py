#!/usr/bin/env python3
"""Play a fixed arm motion again and again with cool-down breaks, through arm_sdk_bridge.py (instead of the IK).

For the exhibition: no operator in the headset. A motion file (design_motion.py, or a teleop log cut with
log_to_motion.py) holds a rest pose and a loop of 14 arm joint angles. The cycle is
  glide to rest -> WORK: glide to the loop's start, play the loop over and over for --work s -> glide to rest ->
  COOL: hold the rest pose for at least --cool s and until the hottest arm motor is at or below
  --temp-resume C -> WORK again ...
WORK ends early when that motor reaches --temp-pause C (--temp-scope all: arms and waist, as before; the waist
does not move in the playback, its heat comes from holding the off-centre arm and the legs, so by default it is
shown but only the bridge's --max-temp stop watches it). The bridge's own monitors stay (its --max-temp stop,
tracking error, lowstate, stop.sh). At the end (--duration, --cycles, SIGTERM / Ctrl+C) the arms glide back to the
pose they had at the start, then the bridge hands them back to the robot's controller (weight -> 0 in 2 s).

Like the IK it writes Redis `arm_ik_q` (50 Hz, the bridge needs it fresh), `drill_cmd` (always OFF),
`teleop_state/ik` and `teleop_display` (PICO video overlay), and it reads `arm_state` (measured angles) and
`teleop_state/bridge` (state, hottest motor). It sends nothing to the robot itself.
Stops: `stop.sh` (Redis teleop_stop) and everything the bridge watches; with --pico also the PICO's left X / Y
(EMERGENCY STOP, latched) and right A (pause at rest / resume). Without PICO data the playback carries on.

  python3 play_motion.py --motion motions/drill_work.json --work 30 --cool 60 --duration 600
  python3 play_motion.py --motion ... --check        # only check the file (speeds, limits) and print it
"""
import argparse
import csv
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np
import redis

NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
# Same joint limits and margin as arm_sdk_bridge.py (it clamps the command to them anyway).
LIMITS = {
    "left_shoulder_pitch_joint": (-3.089, 1.149), "left_shoulder_roll_joint": (-0.6, 2.252),
    "left_shoulder_yaw_joint": (-1.4, 2.0), "left_elbow_joint": (-1.047, 1.7),
    "left_wrist_roll_joint": (-1.972, 1.972), "left_wrist_pitch_joint": (-1.614, 1.614),
    "left_wrist_yaw_joint": (-1.614, 1.614),
    "right_shoulder_pitch_joint": (-3.089, 1.149), "right_shoulder_roll_joint": (-2.252, 0.6),
    "right_shoulder_yaw_joint": (-2.0, 1.4), "right_elbow_joint": (-1.047, 1.7),
    "right_wrist_roll_joint": (-1.972, 1.972), "right_wrist_pitch_joint": (-1.614, 1.614),
    "right_wrist_yaw_joint": (-1.614, 1.614),
}
LO = np.array([LIMITS[n][0] + 0.05 for n in NAMES])
HI = np.array([LIMITS[n][1] - 0.05 for n in NAMES])
STALE_S = 0.5


class StopRequested(Exception):
    pass


def joint_ja(name):
    side = "左" if name.startswith("left") else ("右" if name.startswith("right") else "")
    for key, ja in (("shoulder", "肩"), ("elbow", "肘"), ("wrist", "手首"), ("waist", "腰"),
                    ("hip", "股関節"), ("knee", "膝"), ("ankle", "足首")):
        if key in name:
            return side + ja
    return name or "モーター"


def load_motion(path):
    """The motion file -> (dict, rest array, loop array [frames x 14]). Joints in NAMES order."""
    m = json.loads(Path(path).read_text())
    names = m["names"]
    if sorted(names) != sorted(NAMES):
        raise SystemExit(f"{path}: names must be the 14 arm joints")
    idx = [names.index(n) for n in NAMES]
    rest = np.array(m["rest"], dtype=float)[idx]
    loop = np.array(m["loop"], dtype=float)[:, idx]
    if loop.ndim != 2 or len(loop) < 2:
        raise SystemExit(f"{path}: loop needs at least 2 frames")
    return m, rest, loop


def check_motion(m, rest, loop, rate, speed, max_vel):
    """Problems that stop the playback (list of str) and the peak joint speed while playing [rad/s]."""
    bad = []
    for label, arr in (("rest", rest[None, :]), ("loop", loop)):
        lo = arr < LO - 1e-6
        hi = arr > HI + 1e-6
        if lo.any() or hi.any():
            k = int(np.argmax((lo | hi).any(axis=0)))
            bad.append(f"{label}: {NAMES[k]} outside the joint limits (margin 0.05 rad)")
        if not np.isfinite(arr).all():
            bad.append(f"{label}: not a number")
    src_rate = float(m.get("rate", 50.0))
    vel = np.abs(np.diff(loop, axis=0)).max(axis=1) * src_rate * speed if len(loop) > 1 else np.zeros(1)
    peak = float(vel.max())
    if peak > max_vel + 1e-6:
        k = int(np.argmax(vel))
        bad.append(f"loop frame {k}: {peak:.2f} rad/s at speed x{speed} (limit --max-vel {max_vel})")
    return bad, peak


def min_jerk(a, b, T, t):
    s = min(1.0, max(0.0, t / T)) if T > 0 else 1.0
    s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return a + (b - a) * s


def glide_time(a, b, vel, tmin=1.0):
    """Minimum-jerk time from a to b with the peak joint speed at most vel (peak = 1.875 x mean)."""
    return max(tmin, 1.875 * float(np.max(np.abs(b - a))) / vel)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motion", default=str(Path(__file__).with_name("motions") / "drill_work.json"))
    ap.add_argument("--work", type=float, default=30.0, help="seconds of the loop per cycle (0 = STAND-BY: only "
                    "hold the rest pose under the bridge, soft waist, until the end or a stop)")
    ap.add_argument("--cool", type=float, default=60.0, help="minimum rest between cycles [s]")
    ap.add_argument("--temp-pause", type=float, default=65.0,
                    help="end the work early when the hottest arm motor reaches this [C] (0 = off)")
    ap.add_argument("--temp-resume", type=float, default=58.0,
                    help="the rest lasts until the hottest arm motor is at or below this [C]")
    ap.add_argument("--temp-scope", choices=["arm", "all"], default="arm",
                    help="motors for --temp-pause/--temp-resume: arm = arms only (default), all = arms + waist")
    ap.add_argument("--cycles", type=int, default=0, help="number of work periods (0 = until --duration)")
    ap.add_argument("--duration", type=float, default=600.0, help="total seconds, end included (0 = no limit)")
    ap.add_argument("--speed", type=float, default=1.0, help="play the loop faster (>1) or slower (<1)")
    ap.add_argument("--glide-vel", type=float, default=0.3, help="peak joint speed of the glides [rad/s]")
    ap.add_argument("--max-vel", type=float, default=0.5, help="refuse a loop faster than this [rad/s] "
                    "(the bridge rate-limits to its own --max-vel anyway)")
    ap.add_argument("--pico", action="store_true", help="read the PICO (xrobotoolkit_sdk): left X/Y = stop, "
                    "right A = pause at rest / resume")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--check", action="store_true", help="check the motion file and exit")
    ap.add_argument("--redis-host", default="127.0.0.1")
    ap.add_argument("--log-dir", default=str(Path.home() / "nextech_setup" / "logs"))
    args = ap.parse_args()

    m, rest, loop = load_motion(args.motion)
    bad, peak = check_motion(m, rest, loop, args.rate, args.speed, args.max_vel)
    src_rate = float(m.get("rate", 50.0))
    loop_s = len(loop) / src_rate / args.speed
    seam = float(np.max(np.abs(loop[-1] - loop[0])))
    print(f"play_motion: {args.motion} '{m.get('name', '')}' ({m.get('source', '?')}), loop {loop_s:.1f} s "
          f"({len(loop)} frames at {src_rate:g} Hz, speed x{args.speed}), peak {peak:.2f} rad/s, "
          f"seam {seam:.3f} rad{' (glides back to the start between loops)' if seam > 0.02 else ''}")
    for k, v in (m.get("check") or {}).items():
        if k in ("right_tau_rms_Nm", "min_clearance_m", "payload_kg"):
            print(f"  design check {k}: {v}")
    if bad:
        print("REFUSED:\n  " + "\n  ".join(bad))
        sys.exit(1)
    if args.check:
        return
    if args.temp_pause and args.temp_resume >= args.temp_pause:
        sys.exit("--temp-resume must be below --temp-pause")

    rds = redis.Redis(host=args.redis_host, port=6379)

    def on_sigterm(signum, frame):
        raise StopRequested("SIGTERM")
    signal.signal(signal.SIGTERM, on_sigterm)

    xr = None
    if args.pico:
        try:
            import xrobotoolkit_sdk as xr
            xr.init()
        except Exception as e:   # the playback does not need the PICO
            print(f"PICO not available ({e}): stop with stop.sh / g1.sh stop")
            xr = None

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"play_motion_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    logf = open(log_path, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "phase", "cycle", "bridge", "weight", "temp", "temp_joint", "leg_temp", "pico", "reason"]
                 + [f"q_{n}" for n in NAMES] + [f"meas_{n}" for n in NAMES])
    print(f"work {args.work:g} s, cool >= {args.cool:g} s and until <= {args.temp_resume:g} C, "
          f"early rest at {args.temp_pause:g} C, cycles {args.cycles or 'unlimited'}, "
          f"duration {args.duration:g} s, glide {args.glide_vel} rad/s, pico {'on' if xr else 'off'}, log {log_path}")

    def bridge_status():
        raw = rds.hget("teleop_state", "bridge")
        if not raw:
            return None
        b = json.loads(raw)
        return b if time.time() - float(b["t"]) < STALE_S else None

    def measured():
        raw = rds.get("arm_state")
        if not raw:
            return None
        s = json.loads(raw)
        if time.time() - float(s["t"]) > STALE_S:
            return None
        by = dict(zip(s["names"], s["q"]))
        return np.array([by[n] for n in NAMES])

    pico = {"stamp": None, "change": 0.0, "A": False}

    def read_pico(now):
        """(fresh, X or Y, A pressed now). Never raises."""
        if xr is None:
            return False, False, False
        try:
            stamp = int(xr.get_time_stamp_ns())
            if stamp != pico["stamp"]:
                pico["stamp"], pico["change"] = stamp, now
            fresh = now - pico["change"] < 0.3
            xy = fresh and (bool(xr.get_X_button()) or bool(xr.get_Y_button()))
            a = bool(xr.get_A_button()) and fresh
            press = a and not pico["A"]
            pico["A"] = a
            return fresh, xy, press
        except Exception:
            return False, False, False

    # --- state ---
    phase, reason = "WAIT_BRIDGE", ""
    cycle = 0
    q = None               # what is published
    q_start = None         # measured at engage: where the arms go back at the end
    glide = None           # (from, to, T, t0, next phase)
    t_phase = 0.0
    loop_pos = 0.0         # frame index into the loop (float)
    work_t = 0.0           # seconds of WORK in this cycle (loop playing)
    paused = False         # right A: stay at rest
    estop, active = False, False
    finishing = False
    stats = []             # per cycle: start temp, end temp, work s, why it ended, cool s
    t0 = time.time()
    dt = 1.0 / args.rate
    next_t = t0
    last_print = -10.0

    def set_phase(p, now, why=""):
        nonlocal phase, t_phase
        if p != phase:
            b = bridge_status() or {}
            print(f"[{now - t0:6.1f}s] {phase} -> {p}{' (' + why + ')' if why else ''}  "
                  f"hottest {b.get('temp_joint', '?')} {b.get('temp', 0):.0f} C, arm {b.get('arm_joint', '?')} "
                  f"{b.get('arm_temp', 0):.0f} C, waist {b.get('waist_temp', 0):.0f} C, leg {b.get('leg_temp', 0):.0f} C")
        phase, t_phase = p, now

    def start_glide(target, then, now, vel=None):
        nonlocal glide
        T = glide_time(q, target, vel or args.glide_vel)
        glide = (q.copy(), target.copy(), T, now, then)
        set_phase("GLIDE_" + then, now)

    def trip(now, why):
        nonlocal estop, reason
        if not estop:
            estop, reason = True, why
            set_phase("ESTOP", now, why)

    def display(now, b, temp, tj, leg, lj, wt=0.0, wj=""):
        tl = f"{joint_ja(tj)} {temp:.0f}℃" if temp > 0 else ""
        if phase == "ESTOP":
            lv, ja, en = "stop", f"非常停止（PCから再起動が必要）{reason}", "E-STOP - restart from PC"
        elif phase == "WORK":
            left = max(0.0, args.work - work_t)
            lv, ja, en = "ok", f"自動再生 {cycle}回目 作業中 あと{left:.0f}秒 ／ {tl}", f"PLAY #{cycle} {left:.0f}s left"
        elif phase == "COOL":
            if args.work <= 0:
                lv, ja, en = "idle", f"待機中（腕は休みの姿勢）／ {tl}", "STAND-BY"
            elif paused:
                lv, ja, en = "idle", f"一時停止中（右Aで再開）／ {tl}", "PAUSED - A to resume"
            else:
                left = max(0.0, args.cool - (now - t_phase))
                more = f"あと{left:.0f}秒" if left > 0 else f"{args.temp_resume:.0f}℃まで冷ます"
                lv, ja, en = "idle", f"休憩中 {more} ／ {tl}", f"COOL DOWN {left:.0f}s"
        elif phase.startswith("GLIDE"):
            lv, ja, en = "wait", "姿勢を移動中" + (" ／ 終了します" if finishing else ""), "MOVING"
        elif phase in ("ENGAGE", "WAIT_BRIDGE"):
            lv, ja, en = "wait", "自動再生の準備中", "PREPARING"
        else:
            lv, ja, en = "wait", "終了", "DONE"
        if xr is not None and not pico_fresh and phase != "ESTOP":
            ja += " ／ PICO未接続（停止はPCから）"
        return {"t": now, "level": lv, "ja": ja, "en": en, "ik": "PLAY_" + phase,
                "bridge": b.get("state") if b else None, "weight": b.get("weight") if b else None,
                "reason": reason or (b.get("reason", "") if b else ""), "drill": 0,
                "temp": round(temp, 1), "temp_joint": tj, "leg_temp": round(leg, 1), "leg_joint": lj,
                "waist_temp": round(wt, 1), "waist_joint": wj}

    pico_fresh = False
    try:
        while True:
            now = time.time()
            t = now - t0
            b = bridge_status()
            meas = measured()
            temp = float(b.get("temp", 0.0) or 0.0) if b else 0.0
            tj = str(b.get("temp_joint", "") or "") if b else ""
            wt = float(b.get("waist_temp", 0.0) or 0.0) if b else 0.0
            wj = str(b.get("waist_joint", "") or "") if b else ""
            if args.temp_scope == "arm" and b and "arm_temp" in b:
                temp, tj = float(b.get("arm_temp") or 0.0), str(b.get("arm_joint") or "")
            leg = float(b.get("leg_temp", 0.0) or 0.0) if b else 0.0
            lj = str(b.get("leg_joint", "") or "") if b else ""
            bstate = b.get("state") if b else None
            pico_fresh, xy, a_press = read_pico(now)

            # --- stops ---
            if xy:
                trip(now, "PICO left X/Y")
            if rds.exists("teleop_stop"):
                trip(now, "stop command (teleop_stop)")
            if active and bstate in ("ESTOP_RAMP", "STOPPED"):
                trip(now, "bridge stopped: " + str(b.get("reason", "")))
            if active and phase not in ("ENGAGE",) and b is None:
                trip(now, "no bridge state")
            if active and phase not in ("ENGAGE", "RELEASE") and bstate in ("WAIT", "RAMP_DOWN"):
                trip(now, "bridge released the arms")
            if args.duration and t > args.duration - 2.0 and not estop and phase not in ("RELEASE", "DONE"):
                trip(now, "duration ended before the release")   # safety net; the end normally comes earlier

            # --- the end: glide back to the start pose, then release ---
            def finish_now(why):
                nonlocal finishing
                if not finishing and not estop and q is not None and active:
                    finishing = True
                    print(f"[{t:6.1f}s] finishing: {why}")
                    start_glide(q_start, "RELEASE", now)
            if args.duration and q is not None and active and not finishing:
                left = args.duration - t
                if left <= glide_time(q, q_start, args.glide_vel) + 6.0:
                    finish_now("duration")

            # --- phases ---
            if estop:
                active = False
                if q is None and meas is not None:
                    q = meas
            elif phase == "WAIT_BRIDGE":
                if b is not None and meas is not None and bstate == "WAIT":
                    q = meas.copy()
                    q_start = q.copy()
                    active = True
                    set_phase("ENGAGE", now, "arms measured, weight up")
                elif t > 10.0:
                    print("no bridge in WAIT within 10 s (is arm_sdk_bridge.py running, and not stopped?)")
                    break
            elif phase == "ENGAGE":
                if bstate == "TRACK":
                    start_glide(rest, "COOL", now)
                elif now - t_phase > 8.0:
                    trip(now, f"bridge not in TRACK after 8 s ({bstate})")
            elif phase.startswith("GLIDE_"):
                a, bq, T, tg, then = glide
                q = min_jerk(a, bq, T, now - tg)
                if now - tg >= T:
                    q = bq.copy()
                    glide = None
                    if then == "WORK":
                        loop_pos, work_t = 0.0, 0.0
                        cycle += 1
                        stats.append({"cycle": cycle, "t": round(t, 1), "temp_start": temp, "joint_start": tj})
                    elif then == "COOL" and stats and "work_s" in stats[-1]:
                        pass
                    elif then == "LOOP":   # seam of a recorded clip: back at the start, play on
                        loop_pos = 0.0
                        set_phase("WORK", now)
                        then = None
                    if then == "RELEASE":
                        active = False
                        set_phase("RELEASE", now, "weight -> 0, the robot's controller takes the arms")
                    elif then:
                        set_phase(then, now)
            elif phase == "WORK":
                work_t += dt
                end_why = ""
                if args.temp_pause and temp >= args.temp_pause:
                    end_why = f"{tj} {temp:.0f} C >= {args.temp_pause:g}"
                elif work_t >= args.work:
                    end_why = "work time"
                elif a_press:
                    end_why, paused = "right A: pause", True
                if end_why:
                    stats[-1].update({"work_s": round(work_t, 1), "end": end_why, "temp_end": temp, "joint_end": tj})
                    print(f"[{t:6.1f}s] work #{cycle} ends after {work_t:.1f} s: {end_why}")
                    start_glide(rest, "COOL", now)
                else:
                    loop_pos += args.speed * src_rate / args.rate
                    if loop_pos >= len(loop) - 1:
                        if seam > 0.02:
                            q = loop[-1].copy()
                            start_glide(loop[0], "LOOP", now)
                        else:
                            loop_pos -= len(loop) - 1
                    if phase == "WORK":
                        i = int(loop_pos)
                        f = loop_pos - i
                        q = loop[i] * (1 - f) + loop[min(i + 1, len(loop) - 1)] * f
            elif phase == "COOL":
                q = rest.copy()
                if a_press:
                    paused = not paused
                    print(f"[{t:6.1f}s] right A: {'pause at rest' if paused else 'resume'}")
                cooled = (now - t_phase >= args.cool) and temp <= args.temp_resume
                if stats and "cool_s" not in stats[-1] and "work_s" in stats[-1] and cooled:
                    stats[-1]["cool_s"] = round(now - t_phase, 1)
                # time for one more work period (at least 10 s of it) and the way back?
                need = (glide_time(q, loop[0], args.glide_vel) + min(args.work, 10.0)
                        + glide_time(loop[0], q_start, args.glide_vel) + 8.0) if args.work > 0 \
                    else glide_time(q, q_start, args.glide_vel) + 8.0   # stand-by: only the way back
                if args.cycles and cycle >= args.cycles:
                    finish_now(f"{cycle} cycles done")
                elif args.duration and args.duration - t < need:
                    finish_now("not enough time left for another work period")
                elif cooled and not paused and args.work > 0:
                    start_glide(loop[0], "WORK", now)
            elif phase == "RELEASE":
                if bstate in ("WAIT", None) or now - t_phase > 5.0:
                    set_phase("DONE", now)
                    break

            # --- outputs ---
            if q is not None:
                qp = np.clip(q, LO, HI)
                rds.set("arm_ik_q", json.dumps({"t": now, "active": bool(active and not estop), "follow": active,
                                                "follow_arms": ["left", "right"] if active else [],
                                                "estop": estop, "estop_reason": reason, "names": NAMES,
                                                "q": [float(v) for v in qp], "source": "play_motion"}))
            rds.set("drill_cmd", json.dumps({"t": now, "on": 0, "trigger": 0.0}))
            rds.hset("teleop_state", "ik", json.dumps({"t": now, "state": "PLAY_" + phase, "reason": reason}))
            rds.set("teleop_display", json.dumps(display(now, b, temp, tj, leg, lj, wt, wj), ensure_ascii=False))
            nan = [np.nan] * 14
            log.writerow([f"{t:.3f}", phase, cycle, bstate, b.get("weight") if b else "", f"{temp:.1f}", tj,
                          f"{leg:.1f}", int(pico_fresh), reason]
                         + [f"{v:.4f}" for v in (q if q is not None else nan)]
                         + [f"{v:.4f}" for v in (meas if meas is not None else nan)])
            if t - last_print >= 10.0:
                last_print = t
                print(f"[{t:6.1f}s] {phase} cycle {cycle} bridge {bstate} hottest {tj} {temp:.0f} C "
                      f"waist {wj} {wt:.0f} C leg {leg:.0f} C")
            if estop and (bstate in ("STOPPED", None) or now - t_phase > 6.0):
                break
            next_t += dt
            time.sleep(max(0.0, next_t - time.time()))
    except (KeyboardInterrupt, StopRequested) as e:
        # Ctrl+C / SIGTERM: an emergency stop (the bridge releases in 3 s); no glide back.
        estop, reason = True, "Ctrl+C" if isinstance(e, KeyboardInterrupt) else "SIGTERM"
        rds.set("arm_ik_q", json.dumps({"t": time.time(), "active": False, "follow": False, "follow_arms": [],
                                        "estop": True, "estop_reason": reason, "names": NAMES,
                                        "q": [float(v) for v in np.clip(q if q is not None else rest, LO, HI)],
                                        "source": "play_motion"}))
        print(f"[{time.time() - t0:6.1f}s] EMERGENCY STOP: {reason}")
    logf.close()
    if xr is not None:
        try:
            xr.close()
        except Exception:
            pass
    print("cycles:")
    for s in stats:
        print("  #{cycle} at {t}s: {js} {ts:.0f} C -> work {w} s ({e}) -> {je} {te:.0f} C, cool {c} s".format(
            cycle=s["cycle"], t=s["t"], js=s.get("joint_start", ""), ts=s.get("temp_start", 0),
            w=s.get("work_s", "?"), e=s.get("end", "-"), je=s.get("joint_end", ""), te=s.get("temp_end", 0),
            c=s.get("cool_s", "-")))
    print(f"end: {phase} {reason}; log {log_path}")


if __name__ == "__main__":
    main()
