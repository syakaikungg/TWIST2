#!/usr/bin/env python3
"""Bridge Redis `arm_ik_q` -> G1 29DOF `rt/arm_sdk` LowCmd (layout of Unitree's g1_arm7_sdk_dds_example).

Default is LOG ONLY: it builds, at 50 Hz, the LowCmd it would send and writes it to CSV. Nothing is
published unless --send is given. It subscribes to rt/lowstate (read only) for the measured joint angles.

  python3 arm_sdk_bridge.py --duration 30            # log only
  python3 arm_sdk_bridge.py --send ...               # publishes to rt/arm_sdk (only after an explicit go)

States: WAIT -> RAMP_UP (weight 0->1 over --ramp-up s, command starts at the measured angles) -> TRACK
-> RAMP_DOWN (`active` false: weight ->0 over --ramp-down s, command held) -> WAIT.
EMERGENCY STOP (weight ->0 over --estop-ramp s, drill forced OFF, then STOPPED until restarted):
  arm_ik_q says estop (left X/Y, PICO lost, stop.sh, IK program exit), arm_ik_q older than --stale s while
  engaged (IK program died), `teleop_stop` set in Redis (stop.sh), Ctrl+C / SIGTERM, and while engaged:
  |command - measured| > --max-track-err rad for --track-err-time s, any motor hotter than --max-temp C,
  rt/lowstate silent for 0.2 s.
Commands are clamped to the joint limits and rate-limited to --max-vel rad/s.
The measured arm angles are written to Redis `arm_state` every loop; the IK program takes them as its
start pose when A is pressed, so the arms do not move until the grip is held. The waist is held at the
angles measured when RAMP_UP starts (the official example drives it to 0 instead), then --waist-level brings
roll and pitch slowly to 0 while in TRACK (the robot's own standing pose leans ~6 deg forward, which costs
waist torque and heat; the IK model also assumes waist 0). Waist gains default to
TWIST2's 150/4 (chosen 2026-11-03), and --waist-correct (roll,pitch by default) slowly adds the measured waist
sag to the waist target while in TRACK (integral correction, limited in size and rate); it starts from 0 and
eases back to 0 on pause, return-to-start and emergency stop. --waist-level-rate sets the leveling speed.
The state report (Redis teleop_state/bridge) includes ramp_left, the seconds left in RAMP_UP, and the hottest
commanded motor (temp, temp_joint) for the operator display and the IK's temperature pause.
The emergency-stop ramp is 3 s (was 1 s until 2026-11-04): the command is frozen and the robot's own controller
takes the arms back over that time; with 1 s the arms swung back at up to 1.5 rad/s on the real G1.
The CSV also logs the pelvis IMU (rt/lowstate imu_state rpy), to tell a pelvis tilt from a waist tilt.
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

# Motor indices (Unitree G1JointIndex). Same joint set as the official arm7 example: both arms + waist.
ARM_NAMES = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
ARM_IDX = list(range(15, 29))
WAIST_NAMES = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
WAIST_IDX = [12, 13, 14]
WEIGHT_IDX = 29  # kNotUsedJoint: its q is the arm_sdk weight (1 = arm_sdk, 0 = built-in controller)
JOINT_IDX = ARM_IDX + WAIST_IDX
JOINT_NAMES = ARM_NAMES + WAIST_NAMES
# Limits from GMR's g1_mocap_29dof.xml, shrunk by a margin.
LIMITS = {
    "left_shoulder_pitch_joint": (-3.089, 1.149), "left_shoulder_roll_joint": (-0.6, 2.252),
    "left_shoulder_yaw_joint": (-1.4, 2.0), "left_elbow_joint": (-1.047, 1.7),
    "left_wrist_roll_joint": (-1.972, 1.972), "left_wrist_pitch_joint": (-1.614, 1.614),
    "left_wrist_yaw_joint": (-1.614, 1.614),
    "right_shoulder_pitch_joint": (-3.089, 1.149), "right_shoulder_roll_joint": (-2.252, 0.6),
    "right_shoulder_yaw_joint": (-2.0, 1.4), "right_elbow_joint": (-1.047, 1.7),
    "right_wrist_roll_joint": (-1.972, 1.972), "right_wrist_pitch_joint": (-1.614, 1.614),
    "right_wrist_yaw_joint": (-1.614, 1.614),
    "waist_yaw_joint": (-1.57, 1.57), "waist_roll_joint": (-0.52, 0.52), "waist_pitch_joint": (-0.52, 0.52),
}
MARGIN = 0.05
LO = np.array([LIMITS[n][0] + MARGIN for n in JOINT_NAMES])
HI = np.array([LIMITS[n][1] - MARGIN for n in JOINT_NAMES])
FAKE_STATE = [0.0, 0.4, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Waist kp, kd (yaw, roll, pitch). stock: what the robot's own controller sends on rt/lowcmd while standing
# (read 2026-11-03 11:53 UTC). twist2: TWIST2 configs/g1.yaml. Waist kp is capped at 60, at 150 with twist2.
WAIST_GAINS = {
    "stock": ([40.0, 29.0, 29.0], [2.56, 1.81, 1.81]),
    "twist2": ([150.0, 150.0, 150.0], [4.0, 4.0, 4.0]),
}


class StopRequested(Exception):
    pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--send", action="store_true", help="actually publish rt/arm_sdk (default: log only)")
    ap.add_argument("--net", default="eth0", help="network interface to the robot")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl+C)")
    ap.add_argument("--kp", type=float, default=60.0, help="arms, <= 60 (official example)")
    ap.add_argument("--kd", type=float, default=1.5, help="arms, <= 1.5 (official example)")
    ap.add_argument("--waist-gains", choices=sorted(WAIST_GAINS), default="twist2",
                    help="waist gain set: stock = kp 40/29/29 kd 2.56/1.81/1.81, twist2 = kp 150 kd 4")
    ap.add_argument("--waist-kp", default="", help="override: waist yaw,roll,pitch kp (<= 60, <= 150 with twist2)")
    ap.add_argument("--waist-kd", default="", help="override: waist yaw,roll,pitch kd")
    ap.add_argument("--waist-correct", default="roll,pitch",
                    help="integral waist correction on these joints (\"\" = off)")
    ap.add_argument("--waist-ki", type=float, default=1.0, help="waist correction gain [1/s]")
    ap.add_argument("--waist-corr-max", type=float, default=0.15, help="waist correction limit [rad]")
    ap.add_argument("--waist-corr-rate", type=float, default=0.1, help="waist correction change limit [rad/s]")
    ap.add_argument("--waist-level", default="roll,pitch",
                    help="in TRACK, bring these waist joints slowly to 0 (upright) instead of holding the start "
                         "angle (\"\" = hold the start angle, as before 2026-11-04)")
    ap.add_argument("--waist-level-rate", type=float, default=0.03, help="--waist-level speed [rad/s]")
    ap.add_argument("--max-vel", type=float, default=1.0, help="command rate limit [rad/s]")
    ap.add_argument("--ramp-up", type=float, default=3.0, help="weight 0->1 time [s]")
    ap.add_argument("--ramp-down", type=float, default=2.0, help="weight 1->0 time on pause [s]")
    ap.add_argument("--estop-ramp", type=float, default=3.0, help="weight ->0 time on emergency stop [s]")
    ap.add_argument("--stale", type=float, default=0.3, help="arm_ik_q older than this while engaged -> stop [s]")
    ap.add_argument("--max-track-err", type=float, default=0.3, help="|command - measured| limit [rad]")
    ap.add_argument("--track-err-time", type=float, default=0.5, help="how long the limit may be exceeded [s]")
    ap.add_argument("--max-temp", type=float, default=60.0, help="motor temperature limit [C]")
    ap.add_argument("--fake-state", action="store_true", help="no rt/lowstate: use the default arm pose")
    ap.add_argument("--sim", action="store_true",
                    help="drive arm_sim.py through Redis (sim_lowcmd / sim_lowstate) instead of the robot; no DDS")
    ap.add_argument("--redis-host", default="127.0.0.1")
    ap.add_argument("--log-dir", default=str(Path.home() / "nextech_setup" / "logs"))
    args = ap.parse_args()
    if args.sim and args.send:
        ap.error("--sim and --send cannot be combined")
    kp, kd = min(args.kp, 60.0), min(args.kd, 1.5)
    wkp, wkd = WAIST_GAINS[args.waist_gains]
    if args.waist_kp:
        wkp = [float(v) for v in args.waist_kp.split(",")]
    if args.waist_kd:
        wkd = [float(v) for v in args.waist_kd.split(",")]
    wcap = 150.0 if args.waist_gains == "twist2" else 60.0
    wkp = [min(v, wcap) for v in wkp]
    corr_joints = [j for j in args.waist_correct.split(",") if j]
    if set(corr_joints) - {"yaw", "roll", "pitch"}:
        ap.error("--waist-correct takes yaw, roll, pitch")
    corr_mask = np.array([n.split("_")[1] in corr_joints for n in WAIST_NAMES])
    level_joints = [j for j in args.waist_level.split(",") if j]
    if set(level_joints) - {"yaw", "roll", "pitch"}:
        ap.error("--waist-level takes yaw, roll, pitch")
    level_mask = np.array([n.split("_")[1] in level_joints for n in WAIST_NAMES])
    KP = np.array([kp] * len(ARM_IDX) + wkp)     # per joint, JOINT_IDX order
    KD = np.array([kd] * len(ARM_IDX) + wkd)
    rds = redis.Redis(host=args.redis_host, port=6379)

    def on_sigterm(signum, frame):
        raise StopRequested("SIGTERM")
    signal.signal(signal.SIGTERM, on_sigterm)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    # rt/lowstate arrives at ~1 kHz: the callback only keeps the latest message; the 50 Hz loop decodes it.
    state = {"msg": None, "q": None, "tau": None, "temp": None, "temp_cmd_max": 0.0, "temp_cmd_id": -1,
             "temp_all_max": 0.0, "temp_all_id": -1, "t": 0.0, "n": 0, "rpy": None}
    publisher = None

    def decode_state():
        if args.sim:   # simulator joint state from Redis (no temperatures in the sim)
            raw = rds.get("sim_lowstate")
            if raw:
                s = json.loads(raw)
                by = dict(zip(s["names"], s["q"]))
                bt = dict(zip(s["names"], s["tau"]))
                state["q"] = np.array([by[n] for n in JOINT_NAMES])
                state["tau"] = np.array([bt[n] for n in JOINT_NAMES])
                state["temp"] = np.full(len(JOINT_NAMES), 30.0)
                raw_t = rds.get("sim_temp")   # test hook: {"joint": name, "temp": C} sets one motor's temperature
                if raw_t:
                    ft = json.loads(raw_t)
                    if ft.get("joint") in JOINT_NAMES:
                        state["temp"][JOINT_NAMES.index(ft["joint"])] = float(ft["temp"])
                k = int(np.argmax(state["temp"]))
                state["temp_cmd_max"], state["temp_cmd_id"] = float(state["temp"][k]), JOINT_IDX[k]
                state["t"] = float(s["t"])
                state["n"] += 1
            return
        msg = state["msg"]
        if msg is None:
            return
        ms = msg.motor_state
        state["q"] = np.array([ms[i].q for i in JOINT_IDX])
        state["tau"] = np.array([ms[i].tau_est for i in JOINT_IDX])
        state["temp"] = np.array([max(ms[i].temperature) for i in JOINT_IDX])
        k = int(np.argmax(state["temp"]))
        state["temp_cmd_max"], state["temp_cmd_id"] = float(state["temp"][k]), JOINT_IDX[k]
        temps = [max(ms[i].temperature) for i in range(29)]
        k = int(np.argmax(temps))
        state["temp_all_max"], state["temp_all_id"] = float(temps[k]), k
        try:
            state["rpy"] = np.array([float(v) for v in msg.imu_state.rpy])   # pelvis IMU, log only
        except (AttributeError, TypeError):
            state["rpy"] = None

    if args.sim:
        t0 = time.time()
        while state["q"] is None and time.time() - t0 < 5.0:
            decode_state()
            time.sleep(0.05)
        if state["q"] is None:
            print("no sim_lowstate within 5 s (is arm_sim.py running?)")
            sys.exit(1)
        print("SIM MODE: arm_sim.py via Redis, no DDS. sim arm+waist", np.round(state["q"], 3).tolist())
    elif not args.fake_state or args.send:
        ChannelFactoryInitialize(0, args.net)

        def on_state(msg):
            state["msg"] = msg
            state["t"] = time.time()
            state["n"] += 1

        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(on_state, 10)
        t0 = time.time()
        while state["msg"] is None and time.time() - t0 < 5.0:
            time.sleep(0.05)
        decode_state()
        if state["q"] is None:
            print("no rt/lowstate within 5 s (is the robot on, and is --net right?)")
            if not args.fake_state:
                sys.exit(1)
        else:
            print("rt/lowstate ok: arm+waist measured", np.round(state["q"], 3).tolist())
    if args.send:
        from unitree_sdk2py.core.channel import ChannelPublisher
        publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        publisher.Init()
        print("SEND MODE: publishing rt/arm_sdk")
    elif not args.sim:
        print("LOG ONLY: rt/arm_sdk is not published")

    def measured():
        return state["q"].copy() if state["q"] is not None else np.array(FAKE_STATE)

    if rds.delete("teleop_stop"):
        print("cleared an old teleop_stop flag (restart from the PC = reset)")
    cmd = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()
    dt = 1.0 / args.rate
    max_step = args.max_vel * dt

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    run_kind = "send" if args.send else ("sim" if args.sim else "logonly")
    log_path = log_dir / "arm_sdk_bridge_{}_{}.csv".format(run_kind, time.strftime("%Y%m%d_%H%M%S"))
    logf = open(str(log_path), "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "state", "weight", "age_ms", "active", "follow", "ik_estop", "published", "reason",
                  "track_err", "temp_max", "temp_max_id", "lowstate_age_ms"]
                 + ["tgt_" + n for n in JOINT_NAMES] + ["cmd_" + n for n in JOINT_NAMES]
                 + ["meas_" + n for n in JOINT_NAMES] + ["tau_" + n for n in JOINT_NAMES]
                 + ["temp_" + n for n in JOINT_NAMES] + ["wcorr_" + n for n in WAIST_NAMES]
                 + ["imu_roll", "imu_pitch", "imu_yaw"] + ["whold_" + n for n in WAIST_NAMES])
    print("monitors: track_err>{} rad for {}s, motor temp>{} C, lowstate silent>0.2 s".format(
        args.max_track_err, args.track_err_time, args.max_temp))
    print("waist correction: {}".format("off" if not corr_mask.any() else "{} ki={} max={} rad rate={} rad/s".format(
        ",".join(corr_joints), args.waist_ki, args.waist_corr_max, args.waist_corr_rate)))
    print("waist target: {}".format("hold the start angles" if not level_mask.any() else
                                    "{} to 0 at {} rad/s in TRACK".format(",".join(level_joints), args.waist_level_rate)))
    print("arms kp={} kd={}, waist kp={} kd={}, max_vel={} rad/s ramp_up={}s ramp_down={}s estop_ramp={}s stale={}s log={}".format(
        kp, kd, wkp, wkd, args.max_vel, args.ramp_up, args.ramp_down, args.estop_ramp, args.stale, log_path))

    mode, weight, q_cmd, reason = "WAIT", 0.0, None, ""
    waist_hold = None
    wcorr = np.zeros(len(WAIST_NAMES))
    err_since = None
    t_start = time.time()
    next_t = t_start
    leaving = False

    def report(now):
        ramp_left = (1.0 - weight) * args.ramp_up if mode == "RAMP_UP" else 0.0   # for the operator display
        k = state["temp_cmd_id"]
        rds.hset("teleop_state", "bridge", json.dumps({"t": now, "state": mode, "weight": round(weight, 3),
                                                        "ramp_left": round(ramp_left, 2), "reason": reason,
                                                        "send": bool(args.send), "sim": bool(args.sim),
                                                        "temp": round(state["temp_cmd_max"], 1),
                                                        "temp_joint": JOINT_NAMES[JOINT_IDX.index(k)]
                                                        if k in JOINT_IDX else "",
                                                        "max_temp": args.max_temp}))

    def estop(t, why):
        nonlocal mode, reason
        if mode not in ("ESTOP_RAMP", "STOPPED"):
            reason = why
            print("[{:6.1f}s] EMERGENCY STOP: {} (weight {:.2f} -> 0 in {:.1f}s)".format(t, why, weight, args.estop_ramp))
            mode = "ESTOP_RAMP" if weight > 0.0 else "STOPPED"
            rds.set("drill_cmd", json.dumps({"t": time.time(), "on": 0, "trigger": 0.0, "forced_by": "bridge: " + why}))

    def write_cmd():
        cmd.motor_cmd[WEIGHT_IDX].q = float(weight)
        for k, i in enumerate(JOINT_IDX):
            cmd.motor_cmd[i].q = float(q_cmd[k])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = float(KP[k])
            cmd.motor_cmd[i].kd = float(KD[k])
        cmd.crc = crc.Crc(cmd)
        if args.sim:   # same fields as the LowCmd, to the simulator
            rds.set("sim_lowcmd", json.dumps({"t": time.time(), "weight": float(weight), "names": JOINT_NAMES,
                                              "q": [float(v) for v in q_cmd], "kp": KP.tolist(), "kd": KD.tolist()}))
        if publisher is not None:
            publisher.Write(cmd)
            return 1
        return 0

    while True:
        try:
            now = time.time()
            t = now - t_start
            if args.duration and t > args.duration and not leaving:
                leaving = True
                estop(t, "bridge duration ended")

            # Measured angles for the IK program's start pose.
            decode_state()
            meas_now = measured()
            rds.set("arm_state", json.dumps({"t": now, "names": ARM_NAMES, "q": [float(v) for v in meas_now[:14]],
                                             "waist": [float(v) for v in meas_now[14:]],
                                             "fake": state["q"] is None}))

            # Latest IK command from Redis.
            target, age, active, follow, ik_estop, ik_reason = None, float("inf"), False, False, False, ""
            raw = rds.get("arm_ik_q")
            if raw:
                msg = json.loads(raw)
                age = now - float(msg["t"])
                active, follow = bool(msg.get("active")), bool(msg.get("follow"))
                ik_estop, ik_reason = bool(msg.get("estop")), msg.get("estop_reason", "")
                by_name = dict(zip(msg["names"], msg["q"]))
                waist = waist_hold if waist_hold is not None else meas_now[14:]
                target = np.array([by_name[n] for n in ARM_NAMES] + [float(v) for v in waist])
            fresh = target is not None and age < args.stale
            engaged = mode in ("RAMP_UP", "TRACK", "RAMP_DOWN")

            # Emergency stops.
            if rds.exists("teleop_stop"):
                estop(t, "stop command (teleop_stop)")
            if fresh and ik_estop:
                estop(t, "IK: " + ik_reason)
            if engaged and not fresh:
                estop(t, "arm_ik_q stale {:.0f} ms (IK program stopped?)".format(min(age, 99.0) * 1000))

            # Normal state machine.
            if mode == "WAIT" and fresh and active and not ik_estop:
                mode, q_cmd, reason = "RAMP_UP", meas_now.copy(), ""
                waist_hold = meas_now[14:].copy()
                target[14:] = waist_hold
                print("[{:6.1f}s] RAMP_UP from measured angles (waist held at {})".format(
                    t, np.round(waist_hold, 3).tolist()))
            elif mode in ("RAMP_UP", "TRACK") and not active:
                mode, reason = "RAMP_DOWN", "paused (active=false)"
                print("[{:6.1f}s] RAMP_DOWN (paused)".format(t))
            elif mode == "RAMP_UP" and weight >= 1.0:
                mode = "TRACK"
                print("[{:6.1f}s] TRACK".format(t))

            if mode == "RAMP_UP":
                weight = min(1.0, weight + dt / args.ramp_up)
            elif mode == "RAMP_DOWN":
                weight = max(0.0, weight - dt / args.ramp_down)
                if weight <= 0.0:
                    mode, waist_hold = "WAIT", None
                    print("[{:6.1f}s] released (weight 0) -> WAIT".format(t))
            elif mode == "ESTOP_RAMP":
                weight = max(0.0, weight - dt / args.estop_ramp)
                if weight <= 0.0:
                    mode = "STOPPED"
                    print("[{:6.1f}s] STOPPED (weight 0, built-in controller). Restart from the PC to resume.".format(t))

            # --waist-level: in TRACK the held waist angle walks to 0 (upright on the pelvis) on the chosen joints.
            if level_mask.any() and mode == "TRACK" and waist_hold is not None:
                lstep = args.waist_level_rate * dt
                waist_hold = np.where(level_mask, waist_hold + np.clip(-waist_hold, -lstep, lstep), waist_hold)
                if target is not None:
                    target[14:] = waist_hold

            # Integral waist correction: in TRACK the shortfall (held - measured) is slowly added to the waist
            # target, within --waist-corr-max and --waist-corr-rate; in every other state it eases back to 0.
            if corr_mask.any():
                if mode == "TRACK" and waist_hold is not None:
                    want = np.clip(wcorr + args.waist_ki * (waist_hold - meas_now[14:]) * dt,
                                   -args.waist_corr_max, args.waist_corr_max)
                else:
                    want = np.zeros(len(WAIST_NAMES))
                step = args.waist_corr_rate * dt
                wcorr = np.where(corr_mask, wcorr + np.clip(want - wcorr, -step, step), 0.0)
                if target is not None and waist_hold is not None:
                    target[14:] = waist_hold + wcorr
                if mode == "ESTOP_RAMP" and waist_hold is not None and q_cmd is not None:
                    q_cmd[14:] = q_cmd[14:] + np.clip(waist_hold + wcorr - q_cmd[14:], -max_step, max_step)

            # Command: toward the target (rate-limited, clamped). On a pause (RAMP_DOWN) it keeps easing toward
            # the IK's last pose (the start pose after B) while the weight drops; held on an emergency stop.
            if mode in ("RAMP_UP", "TRACK", "RAMP_DOWN") and target is not None and fresh:
                # The goal is clamped; the command only ever moves toward it at max_step, so a start
                # outside the margin (measured angle near a hard limit) eases in instead of jumping.
                goal = np.clip(target, LO, HI)
                q_cmd = q_cmd + np.clip(goal - q_cmd, -max_step, max_step)

            # Safety monitors on the measured state, while the bridge owns the arms. The tracking check is
            # skipped on a pause release (RAMP_DOWN), where the built-in controller is taking the arms back.
            track_err = float(np.abs(q_cmd - meas_now).max()) if q_cmd is not None else 0.0
            if mode in ("RAMP_UP", "TRACK") and track_err > args.max_track_err:
                err_since = now if err_since is None else err_since
                if now - err_since >= args.track_err_time:
                    estop(t, "tracking error {:.2f} rad for {:.1f} s".format(track_err, now - err_since))
            else:
                err_since = None
            if mode in ("RAMP_UP", "TRACK", "RAMP_DOWN") and state["q"] is not None:
                if state["temp_cmd_max"] > args.max_temp:   # a motor this bridge commands (arms, waist)
                    estop(t, "motor {} at {:.0f} C".format(state["temp_cmd_id"], state["temp_cmd_max"]))
                if now - state["t"] > 0.2:
                    estop(t, "rt/lowstate silent {:.0f} ms".format((now - state["t"]) * 1000))
            if state["temp_all_max"] > args.max_temp and int(t) % 10 == 0 and abs(t - round(t)) < dt:
                print("[{:6.1f}s] WARNING: motor {} (not commanded here) at {:.0f} C".format(
                    t, state["temp_all_id"], state["temp_all_max"]))

            published = 0
            if q_cmd is not None and mode in ("RAMP_UP", "TRACK", "RAMP_DOWN", "ESTOP_RAMP"):
                published = write_cmd()
            elif q_cmd is not None and mode in ("WAIT", "STOPPED") and weight <= 0.0 and published == 0:
                pass  # nothing to send: built-in controller owns the arms
            report(now)

            meas = measured()
            tg = target if target is not None else np.full(len(JOINT_NAMES), np.nan)
            qc = q_cmd if q_cmd is not None else np.full(len(JOINT_NAMES), np.nan)
            nan17 = np.full(len(JOINT_NAMES), np.nan)
            tau = state["tau"] if state["tau"] is not None else nan17
            temp = state["temp"] if state["temp"] is not None else nan17
            ls_age = (now - state["t"]) * 1000 if state["q"] is not None else -1
            log.writerow(["{:.3f}".format(t), mode, "{:.3f}".format(weight), "{:.0f}".format(min(age, 99.0) * 1000),
                          int(active), int(follow), int(ik_estop), published, reason,
                          "{:.4f}".format(track_err), "{:.0f}".format(state["temp_all_max"]), state["temp_all_id"],
                          "{:.0f}".format(ls_age)]
                         + ["{:.4f}".format(v) for v in tg] + ["{:.4f}".format(v) for v in qc]
                         + ["{:.4f}".format(v) for v in meas] + ["{:.2f}".format(v) for v in tau]
                         + ["{:.0f}".format(v) for v in temp] + ["{:.4f}".format(v) for v in wcorr]
                         + (["{:.4f}".format(v) for v in state["rpy"]] if state["rpy"] is not None else ["nan"] * 3)
                         + (["{:.4f}".format(v) for v in waist_hold] if waist_hold is not None else ["nan"] * 3))

            if leaving and mode == "STOPPED":
                break
            next_t += dt
            time.sleep(max(0.0, next_t - time.time()))
        except (KeyboardInterrupt, StopRequested) as e:
            leaving = True
            estop(time.time() - t_start, "Ctrl+C" if isinstance(e, KeyboardInterrupt) else "SIGTERM")
    logf.close()
    print("lowstate messages received: {}; log: {}".format(state["n"], log_path))


if __name__ == "__main__":
    main()
