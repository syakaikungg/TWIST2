#!/usr/bin/env python3
"""Arm IK for the G1 from PICO controllers. Sends NOTHING to the robot (no DDS).

Reads headset/controller poses from xrobotoolkit_sdk, solves arm-only IK with mink on GMR's G1 29DOF
model, and writes the 14 arm joint angles, the drill on/off value and the teleop state to the local Redis.

Operator controls (one person, headset on):
  right A        enable / pause teleop. Enabling takes the arms' measured angles (Redis `arm_state`, written
                 by the bridge from rt/lowstate) as the start pose, so nothing moves until a grip is held.
                 Pause: the bridge hands the arms back to the built-in controller.
  right / left grip
                 dead-man per arm: the right arm follows the right hand only while the right grip is held, the
                 left arm the left hand while the left grip is held; release = that arm stops where it is
                 (--grip-mode right: the right grip moves both arms, as before 2026-11-04)
  right trigger  drill ON while held (only while enabled and the right arm follows)
  right stick press (--return-button, default rclick)
                 glide back to the start pose measured at enable, then pause. Not B: in the PICO app's
                 Remote Vision B switches the camera view between one eye and both eyes.
  left X or Y    EMERGENCY STOP (latched until this program is restarted from the PC)
Also an emergency stop: PICO data missing for 0.3 s while following (a grip held), or for more than
--stale-hold s (default 2) while enabled but not following, `teleop_stop` set in Redis (stop.sh),
Ctrl+C / SIGTERM. A shorter gap while not following (Wi-Fi hiccup) only holds: the arms stay where they
are, and teleop carries on by itself when the data is back (hold the grip again to move).
The state for the operator's display (PICO video overlay) is written to Redis `teleop_display`.
Collision avoidance keeps each arm (and what it holds) away from the body and the two forearms apart. With
--tool drill (default) the right arm carries the drill of sim_drill.py (capsules from the wrist flange, about
24 cm long); --tool hand checks both hands instead (a box around each, as the real hands are still on).

  python arm_ik_dryrun.py --arms both
  python arm_ik_dryrun.py --selftest [--scenario deadman|estop_x|pico_stale|per_arm|drill_cross|none] [--selftest-redis]
"""
import argparse
import csv
import gc
import json
import math
import signal
import time
from pathlib import Path

import mink
import mujoco
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation as R

import sim_drill   # next to this file: the drill's shape (also drawn by the simulator and its viewers)

from general_motion_retargeting import ROBOT_XML_DICT

# Unitree G1 motor order: left arm 15-21, right arm 22-28.
ARM_JOINTS = {
    "left": [
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    ],
    "right": [
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    ],
}
ALL_NAMES = ARM_JOINTS["left"] + ARM_JOINTS["right"]
# Default arm pose used by TWIST2 (deploy_real/data_utils/params.py): arms out, elbows bent forward.
DEFAULT_Q = {
    "left": [0.0, 0.4, 0.0, 1.2, 0.0, 0.0, 0.0],
    "right": [0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0],
}
HAND_BODY = {"left": "left_rubber_hand", "right": "right_rubber_hand"}
STALE_S = 0.3
# Collision avoidance in the IK: arm links vs the body, and forearm vs forearm.
ARM_LINKS = ["shoulder_yaw_link", "elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link"]
FOREARM_LINKS = ["elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link"]
BODY_LINKS = ["pelvis", "torso_link"] + [f"{s}_{p}" for s in ("left", "right")
                                         for p in ("hip_pitch_link", "hip_roll_link", "hip_yaw_link", "knee_link")]

# PICO / OpenXR axes (x right, y up, z back) -> robot axes (x forward, y left, z up).
M = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pose7(p):
    """[x, y, z, qx, qy, qz, qw] -> (pos, rotmat), or None if the quaternion is empty."""
    p = np.asarray(p, dtype=float)
    if p.shape[0] < 7 or np.linalg.norm(p[3:7]) < 0.5:
        return None
    return p[:3].copy(), R.from_quat(p[3:7]).as_matrix()


def head_yaw(R_head):
    """Yaw of the headset's forward direction, in robot axes."""
    fwd = (M @ R_head @ M.T) @ np.array([1.0, 0.0, 0.0])
    return math.atan2(fwd[1], fwd[0])


def tool_bodies(sides, tool):
    """Bodies at the end of each arm that take part in the collision avoidance."""
    return {s: (list(sim_drill.DRILL_BODIES) if (s == "right" and tool == "drill") else [HAND_BODY[s]])
            for s in sides}


def build_arm_model(sides, tool="drill"):
    """GMR's G1 29DOF model with only the chosen arms' joints left free (base, legs, waist welded), plus the
    collision shapes of what each arm holds: the drill on the right wrist (tool=drill), a box on each hand."""
    keep = {j for s in sides for j in ARM_JOINTS[s]}
    spec = mujoco.MjSpec.from_file(str(ROBOT_XML_DICT["unitree_g1"]))
    for s in sides:
        if s == "right" and tool == "drill":
            sim_drill.add_drill(spec, collision=True)
        else:
            sim_drill.add_hand_collision(spec, HAND_BODY[s])
    for a in list(spec.actuators):
        if a.target not in keep:
            spec.delete(a)
    for s in list(spec.sensors):
        if s.objtype == mujoco.mjtObj.mjOBJ_JOINT and s.objname not in keep:
            spec.delete(s)
    for j in list(spec.joints):
        if j.name not in keep:
            spec.delete(j)
    return spec.compile()


class SdkSource:
    """Live PICO data through xrobotoolkit_sdk."""

    def __init__(self):
        import xrobotoolkit_sdk as xr
        self.xr = xr
        xr.init()

    def read(self, t):
        xr = self.xr
        return {
            "head": pose7(xr.get_headset_pose()),
            "left": pose7(xr.get_left_controller_pose()),
            "right": pose7(xr.get_right_controller_pose()),
            "trigger": float(xr.get_right_trigger()),
            "grip": float(xr.get_right_grip()),
            "A": bool(xr.get_A_button()), "B": bool(xr.get_B_button()),
            "X": bool(xr.get_X_button()), "Y": bool(xr.get_Y_button()),
            "stamp": int(xr.get_time_stamp_ns()),
            # Right stick press: return-to-start button (--return-button rclick).
            "rclick": bool(xr.get_right_axis_click()),
            # Logged only (to find which input goes with a stop or a data gap).
            "lgrip": self._opt("get_left_grip", float, 0.0), "ltrig": self._opt("get_left_trigger", float, 0.0),
            "lmenu": self._opt("get_left_menu_button", bool, False),
            "rmenu": self._opt("get_right_menu_button", bool, False),
            "lclick": self._opt("get_left_axis_click", bool, False),
            "laxis": self._opt("get_left_axis", lambda v: [float(x) for x in v], [0.0, 0.0]),
            "raxis": self._opt("get_right_axis", lambda v: [float(x) for x in v], [0.0, 0.0]),
        }

    def _opt(self, name, conv, default):
        """Logged-only inputs: tolerate an SDK build without that getter."""
        fn = getattr(self.xr, name, None)
        try:
            return conv(fn()) if fn is not None else default
        except Exception:
            return default

    def close(self):
        self.xr.close()


class SyntheticSource:
    """Scripted inputs for checking the IK and the stops without a PICO.

    scenario "none": A at 1 s, grip held 1-10 s; 2 s bumps on the right controller (out and back):
      forward 0.15 m, up 0.15 m, left 0.15 m, twist +45 deg; trigger 9-10 s; B at 12 s; A at 14 s.
    scenario "deadman": A at 1 s; forward ramp 1.5-6 s; grip held 1.5-3.5 s and 5-7 s; trigger 3.8-4.6 s
      (grip released -> drill must stay OFF) and 5.5-6.3 s (grip held -> drill ON).
    scenario "estop_x": A at 1 s, grip from 1.5 s, trigger 3-6 s, left X at 4 s.
    scenario "pico_stale": A at 1 s, grip from 1.5 s, PICO stream freezes at 4 s.
    scenario "hold": A at 1 s, no grip until 4 s (arms must not move), grip 4-6 s with a forward drift,
      B at 7 s (glide back to the start pose, then pause).
    scenario "sim_suite": A at 1 s, grip 4.5-19.5 s (after the bridge's 3 s weight ramp); right-hand bumps
      forward/up/left 0.15 m and twist 45 deg (2 s each, from 5 s), both hands toward each other 0.2 m
      (13-16 s), right hand back+left toward the torso (16-19 s); B at 20 s.
    scenario "reach_fwd": A at 1 s, grip 4.5-13.5 s; both hands from the chest to fully forward at shoulder
      height (5-7 s), held, back (10-12 s); trigger 7.5-9.5 s (drill ON while reaching); return button at 14 s.
    scenario "per_arm": A at 1 s; both controllers sway forward/back from 4 s; right grip only 4.5-7 s, left grip
      only 8-10.5 s, both 11.5-13 s (each arm must move only while its own grip is held); trigger 5-6 s (right
      arm following -> drill ON) and 9-9.8 s (only the left arm following -> drill stays OFF); return at 14 s.
    scenario "drill_cross": A at 1 s; right grip only 4.5-12 s: the right hand goes across in front of the
      body toward the left forearm and turns 100 deg so that the drill points at it (5-8 s), stays (8-10 s),
      comes back (10-12 s); the left arm stays. With --tool drill the drill must keep clear of the body and the left arm
      (check with check_tool_gap.py).
    Other scenarios hold the left grip together with the right one (both arms follow, as before).
    scenario "buttons": A at 1 s, grip 1.5-6 s with a forward drift, B at 3 s and 5 s (Remote Vision's
      eye switch: must change nothing), return button at 7 s (glide back, pause), A at 10 s (enabled again).
    scenario "stale_hold": A at 1 s, no grip; PICO frozen 3.0-3.5 s and 4.0-4.45 s (holds, no stop); grip
      5-7 s with a forward drift (follows); frozen again from 8 s for good (stop after --stale-hold).
    "B" in a scenario means the return button (--return-button): the synthetic press goes to that input.
    """

    SUITE = [("forward", 5.0, 7.0), ("up", 7.0, 9.0), ("left", 9.0, 11.0), ("twist", 11.0, 13.0),
             ("together", 13.0, 16.0), ("to_torso", 16.0, 19.0)]

    PHASES = [("forward", 1.0, 3.0), ("up", 3.0, 5.0), ("left", 5.0, 7.0), ("twist", 7.0, 9.0)]

    def __init__(self, scenario, ret_key="rclick"):
        self.scenario = scenario
        self.ret_key = ret_key

    def read(self, t):
        sc = self.scenario
        head = (np.array([0.0, 1.6, 0.0]), np.eye(3))
        base = np.array([0.25, 1.1, -0.35])            # right hand, in front of the chest (PICO axes)
        off, rot = np.zeros(3), np.eye(3)
        d = {"trigger": 0.0, "grip": 0.0, "A": 1.0 <= t < 1.1, "B": False, "X": False, "Y": False, "rclick": False}
        ret = False   # the return-button press of the scenario
        lgrip = None  # left grip of the scenario; None = same as the right grip
        stamp = int(t * 1e9)
        loff = np.zeros(3)
        if sc == "sim_suite":
            for name, t0, t1 in self.SUITE:
                if t0 <= t < t1:
                    a = math.sin(math.pi * (t - t0) / (t1 - t0))
                    if name == "forward":
                        off = np.array([0.0, 0.0, -0.15 * a])
                    elif name == "up":
                        off = np.array([0.0, 0.15 * a, 0.0])
                    elif name == "left":
                        off = np.array([-0.15 * a, 0.0, 0.0])
                    elif name == "twist":
                        rot = R.from_rotvec([0.0, 0.0, -math.radians(45) * a]).as_matrix()
                    elif name == "together":            # hands toward the midline, in front of the chest
                        off = np.array([-0.20 * a, 0.0, 0.0])
                        loff = np.array([0.20 * a, 0.0, 0.0])
                    else:                                # right hand back toward the body and across
                        off = np.array([-0.15 * a, 0.0, 0.25 * a])
            d["grip"] = 1.0 if 4.5 <= t < 19.5 else 0.0
            ret = 20.0 <= t < 20.1
        elif sc == "reach_fwd":   # both hands from the chest to fully forward at shoulder height, hold, back
            a = float(np.clip((t - 5.0) / 2.0, 0, 1)) - float(np.clip((t - 10.0) / 2.0, 0, 1))
            chest_r, far_r = np.array([0.25, 1.1, -0.35]), np.array([0.19, 1.35, -0.55])
            off = a * (far_r - chest_r)
            loff = a * (np.array([-0.19, 1.35, -0.55]) - np.array([-0.25, 1.1, -0.35]))
            d["grip"] = 1.0 if 4.5 <= t < 13.5 else 0.0
            d["trigger"] = 1.0 if 7.5 <= t < 9.5 else 0.0
            ret = 14.0 <= t < 14.1
        elif sc == "temp_pause":   # with a temperature fed in by a test script: warn, pause, A refused, A again
            sway = 0.08 * math.sin(2 * math.pi * max(0.0, t - 4.0) / 3.0)
            off, loff = np.array([0.0, 0.0, -sway]), np.array([0.0, 0.0, sway])
            d["grip"] = 1.0 if 4.5 <= t < 27.0 else 0.0
            d["A"] = (1.0 <= t < 1.1) or (18.0 <= t < 18.1) or (22.0 <= t < 22.1)
        elif sc in ("estop_reach", "pause_reach"):   # both hands fully forward, then X (or A) while extended
            a = float(np.clip((t - 5.0) / 2.0, 0, 1))
            off = a * (np.array([0.19, 1.35, -0.55]) - np.array([0.25, 1.1, -0.35]))
            loff = a * (np.array([-0.19, 1.35, -0.55]) - np.array([-0.25, 1.1, -0.35]))
            d["grip"] = 1.0 if 4.5 <= t < 9.0 else 0.0
            if sc == "estop_reach":
                d["X"] = 9.0 <= t < 9.1
            else:
                d["A"] = (1.0 <= t < 1.1) or (9.0 <= t < 9.1)
        elif sc == "per_arm":
            sway = 0.08 * math.sin(2 * math.pi * max(0.0, t - 4.0) / 3.0)
            off, loff = np.array([0.0, 0.0, -sway]), np.array([0.0, 0.0, sway])
            d["grip"] = 1.0 if (4.5 <= t < 7.0 or 11.5 <= t < 13.0) else 0.0
            lgrip = 1.0 if (8.0 <= t < 10.5 or 11.5 <= t < 13.0) else 0.0
            d["trigger"] = 1.0 if (5.0 <= t < 6.0 or 9.0 <= t < 9.8) else 0.0
            ret = 14.0 <= t < 14.1
        elif sc == "drill_cross":
            a = float(np.clip((t - 5.0) / 3.0, 0, 1)) - float(np.clip((t - 10.0) / 2.0, 0, 1))
            off = a * (np.array([-0.25, 0.80, -0.15]) - base)   # across, toward the left forearm, nearer
            rot = R.from_rotvec([0.0, math.radians(100) * a, 0.0]).as_matrix()   # drill turned to point left
            d["grip"] = 1.0 if 4.5 <= t < 12.0 else 0.0
            lgrip = 0.0
            ret = 13.0 <= t < 13.1
        elif sc == "none":
            for name, t0, t1 in self.PHASES:
                if t0 <= t < t1:
                    a = math.sin(math.pi * (t - t0) / (t1 - t0))
                    if name == "forward":
                        off = np.array([0.0, 0.0, -0.15 * a])   # PICO -z = forward
                    elif name == "up":
                        off = np.array([0.0, 0.15 * a, 0.0])    # PICO +y = up
                    elif name == "left":
                        off = np.array([-0.15 * a, 0.0, 0.0])   # PICO -x = left
                    else:
                        rot = R.from_rotvec([0.0, 0.0, -math.radians(45) * a]).as_matrix()
            d["grip"] = 1.0 if 1.0 <= t < 10.0 else 0.0
            d["trigger"] = 1.0 if 9.0 <= t < 10.0 else 0.0
            d["A"] = (1.0 <= t < 1.1) or (14.0 <= t < 14.1)
            ret = 12.0 <= t < 12.1
        else:
            off = np.array([0.0, 0.0, -0.05 * max(0.0, min(t, 6.0) - 1.5)])   # slow forward drift
            if sc == "deadman":
                d["grip"] = 1.0 if (1.5 <= t < 3.5 or 5.0 <= t < 7.0) else 0.0
                d["trigger"] = 1.0 if (3.8 <= t < 4.6 or 5.5 <= t < 6.3) else 0.0
            elif sc == "estop_x":
                d["grip"] = 1.0 if t >= 1.5 else 0.0
                d["trigger"] = 1.0 if 3.0 <= t < 6.0 else 0.0
                d["X"] = 4.0 <= t < 4.1
            elif sc == "pico_stale":
                d["grip"] = 1.0 if t >= 1.5 else 0.0
                if t >= 4.0:
                    stamp = int(4.0 * 1e9)
            elif sc == "hold":
                off = np.array([0.0, 0.0, -0.05 * max(0.0, min(t, 6.0) - 4.0)])
                d["grip"] = 1.0 if 4.0 <= t < 6.0 else 0.0
                ret = 7.0 <= t < 7.1
            elif sc == "buttons":
                d["grip"] = 1.0 if 1.5 <= t < 6.0 else 0.0
                d["B"] = (3.0 <= t < 3.1) or (5.0 <= t < 5.1)
                ret = 7.0 <= t < 7.1
                d["A"] = (1.0 <= t < 1.1) or (10.0 <= t < 10.1)
            elif sc == "stale_hold":
                off = np.array([0.0, 0.0, -0.05 * max(0.0, min(t, 7.0) - 5.0)])
                d["grip"] = 1.0 if 5.0 <= t < 7.0 else 0.0
                for f0, f1 in ((3.0, 3.5), (4.0, 4.45), (8.0, 1e9)):
                    if f0 <= t < f1:
                        stamp = int(f0 * 1e9)
        if ret and self.ret_key in d:
            d[self.ret_key] = True
        d["lgrip"] = d["grip"] if lgrip is None else lgrip
        d.update({"head": head, "left": (np.array([-0.25, 1.1, -0.35]) + loff, np.eye(3)),
                  "right": (base + off, rot), "stamp": stamp})
        return d

    def close(self):
        pass


class StopRequested(Exception):
    pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arms", choices=["right", "both"], default="both")
    ap.add_argument("--rate", type=float, default=50.0, help="loop rate [Hz]")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to run (0 = until Ctrl+C)")
    ap.add_argument("--mode", choices=["absolute", "relative"], default="absolute",
                    help="absolute: operator hand relative to the operator's (estimated) shoulder -> robot hand "
                         "relative to the robot's shoulder; relative: clutch, hand moves by the controller delta")
    ap.add_argument("--human-arm", type=float, default=0.60, help="operator shoulder-to-controller reach [m] (absolute)")
    ap.add_argument("--abs-scale", type=float, default=0.0, help="absolute position scale (0 = robot reach / --human-arm)")
    ap.add_argument("--human-shoulder-width", type=float, default=0.38, help="operator shoulder width [m] (absolute)")
    ap.add_argument("--head-down", type=float, default=0.25, help="headset to shoulder line, down [m] (absolute)")
    ap.add_argument("--head-back", type=float, default=0.10, help="headset to shoulder line, back [m] (absolute)")
    ap.add_argument("--orient-cost", type=float, default=0.3, help="weight of the hand orientation in the IK")
    ap.add_argument("--orient-pitch", type=float, default=0.0, help="extra pitch of the hand vs the controller [deg]")
    ap.add_argument("--lm-damping", type=float, default=1.0,
                    help="IK hand-task damping (larger = smoother but slower to catch up)")
    ap.add_argument("--reach-frac", type=float, default=0.95, help="absolute mode: hand target within this x arm reach")
    ap.add_argument("--max-forward", type=float, default=0.0,
                    help="absolute mode: hand target at most this far in front of the shoulder [m] (0 = no limit)")
    ap.add_argument("--hand-speed", type=float, default=0.3, help="absolute mode: max hand target speed [m/s]")
    ap.add_argument("--hand-turn", type=float, default=1.5, help="absolute mode: max hand target turn rate [rad/s]")
    ap.add_argument("--scale", type=float, default=1.0, help="relative mode: controller -> hand position scale")
    ap.add_argument("--rot-scale", type=float, default=0.7, help="relative mode: controller -> hand rotation scale")
    ap.add_argument("--max-vel", type=float, default=3.0, help="joint velocity limit [rad/s]")
    ap.add_argument("--max-reach", type=float, default=0.4, help="max hand offset from the grip point [m]")
    ap.add_argument("--posture-cost", type=float, default=0.05)
    ap.add_argument("--collision-margin", type=float, default=0.04,
                    help="keep arm links this far from the body / other forearm [m] (0 = off)")
    ap.add_argument("--tool", choices=["drill", "hand"], default="drill",
                    help="what the right arm holds, for the collision avoidance: the drill on the wrist "
                         "(default) or the hand (both hands checked)")
    ap.add_argument("--return-button", choices=["rclick", "B", "none"], default="rclick",
                    help="glide back to the start pose, then pause: right stick press (default), B, or none. "
                         "B is also Remote Vision's one-eye/both-eyes switch.")
    ap.add_argument("--grip-mode", choices=["per_arm", "right"], default="per_arm",
                    help="per_arm: each grip moves its own arm (default); right: the right grip moves both arms")
    ap.add_argument("--temp-warn", type=float, default=0.0,
                    help="hottest motor the bridge commands (arms, waist) at or above this: warning on the display "
                         "[C]; 0 = 5 C below the bridge's --max-temp (55 C with the default 60 C)")
    ap.add_argument("--temp-pause", type=float, default=0.0,
                    help="at or above this: glide back to the start pose and pause to cool down, before the "
                         "bridge's emergency stop (--max-temp, 60 C) [C]; 0 = off (default: warning only, "
                         "chosen 2026-11-04)")
    ap.add_argument("--temp-resume", type=float, default=55.0,
                    help="after a temperature pause, A is refused until the hottest motor is at or below this [C]")
    ap.add_argument("--stale-hold", type=float, default=2.0,
                    help="PICO data gap tolerated while enabled but not following: the arms hold and teleop "
                         "resumes when the data is back; longer -> emergency stop [s]. While following: 0.3 s.")
    ap.add_argument("--redis-host", default="127.0.0.1")
    ap.add_argument("--no-redis", action="store_true")
    ap.add_argument("--selftest", action="store_true", help="synthetic inputs, no PICO, no Redis")
    ap.add_argument("--scenario", default="none",
                    choices=["none", "deadman", "estop_x", "pico_stale", "hold", "sim_suite", "reach_fwd",
                             "buttons", "stale_hold", "per_arm", "drill_cross", "estop_reach", "pause_reach",
                             "temp_pause"])
    ap.add_argument("--selftest-redis", action="store_true", help="with --selftest: still write to Redis")
    ap.add_argument("--log-dir", default=str(Path.home() / "nextech_setup" / "logs"))
    args = ap.parse_args()
    if args.selftest:
        args.no_redis = args.no_redis or not args.selftest_redis
        args.duration = args.duration or {"none": 16.0, "hold": 12.0, "sim_suite": 24.0, "reach_fwd": 17.0,
                                          "buttons": 12.0, "stale_hold": 12.0, "per_arm": 16.0,
                                          "drill_cross": 14.0, "estop_reach": 15.0,
                                          "pause_reach": 15.0, "temp_pause": 28.0}.get(args.scenario, 9.0)

    def on_sigterm(signum, frame):
        raise StopRequested("SIGTERM")
    signal.signal(signal.SIGTERM, on_sigterm)

    sides = ["right"] if args.arms == "right" else ["left", "right"]
    model = build_arm_model(sides, args.tool)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    side_idx = {s: np.array([i for i, n in enumerate(names) if n in ARM_JOINTS[s]]) for s in sides}   # in cfg.q
    q0 = np.array([dict(zip(ARM_JOINTS[s], DEFAULT_Q[s]))[n] for n in names for s in sides if n in ARM_JOINTS[s]])
    cfg = mink.Configuration(model)
    cfg.update(q0)

    frame_tasks = {s: mink.FrameTask(HAND_BODY[s], "body", position_cost=1.0, orientation_cost=args.orient_cost,
                                     lm_damping=args.lm_damping) for s in sides}

    # Absolute mode geometry: robot shoulder points (shoulder_roll_link, pelvis welded) and straight-arm reach.
    geo = mujoco.MjData(model)
    mujoco.mj_kinematics(model, geo)
    rob_sh = {s: geo.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_shoulder_roll_link")].copy()
              for s in sides}
    reach = 0.0
    for s in sides:
        ej = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{s}_elbow_joint")
        hb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, HAND_BODY[s])
        for e in np.linspace(model.jnt_range[ej][0], model.jnt_range[ej][1], 40):
            geo.qpos[:] = 0.0
            geo.qpos[model.jnt_qposadr[ej]] = e
            mujoco.mj_kinematics(model, geo)
            reach = max(reach, float(np.linalg.norm(geo.xpos[hb] - rob_sh[s])))
    abs_scale = args.abs_scale or reach / args.human_arm
    R_fix = R.from_euler("y", args.orient_pitch, degrees=True).as_matrix()
    posture = mink.PostureTask(model, cost=args.posture_cost)
    posture.set_target(q0)
    base_limits = [mink.ConfigurationLimit(model), mink.VelocityLimit(model, {n: args.max_vel for n in names})]
    # An arm whose grip is not held is frozen inside the IK too, so that keeping clear of the other arm is
    # done by the arm that moves (overriding the held arm after the solve would undo that).
    hold_limits = {s: mink.VelocityLimit(model, {n: 1e-6 for n in ARM_JOINTS[s]}) for s in sides}
    limits = list(base_limits)

    def coll_geoms(bodies):
        ids = []
        for bn in bodies:
            b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bn)
            if b >= 0:
                ids += [g for g in mink.get_body_geom_ids(model, b) if model.geom_contype[g] or model.geom_conaffinity[g]]
        return ids

    tools = tool_bodies(sides, args.tool)
    coll_pairs = [(coll_geoms([f"{s}_{l}" for l in ARM_LINKS] + tools[s]), coll_geoms(BODY_LINKS)) for s in sides]
    if len(sides) == 2:
        coll_pairs.append((coll_geoms([f"left_{l}" for l in FOREARM_LINKS] + tools["left"]),
                           coll_geoms([f"right_{l}" for l in FOREARM_LINKS] + tools["right"])))

    def min_pair_distance():
        best = 1.0
        for ga, gb in coll_pairs:
            for a in ga:
                for b in gb:
                    best = min(best, mujoco.mj_geomDistance(model, cfg.data, a, b, 1.0, None))
        return best

    coll = {"limit": None, "margin": 0.0}

    def set_collision_limit():
        """Collision avoidance with the requested margin, never tighter than the current pose allows
        (otherwise the arms would be pushed away the moment the grip is pressed)."""
        nonlocal limits
        if args.collision_margin <= 0:
            limits = list(base_limits)
            return None
        dmin = min_pair_distance()
        margin = max(0.0, min(args.collision_margin, dmin - 0.005))
        coll["limit"] = mink.CollisionAvoidanceLimit(
            model, geom_pairs=coll_pairs, minimum_distance_from_collisions=margin,
            collision_detection_distance=margin + 0.08)
        coll["margin"] = coll["margin0"] = margin
        limits = list(base_limits) + [coll["limit"]]
        return dmin, margin

    def reset_collision_margin():
        """Back to the enable-time margin, so B can glide all the way back to the start pose."""
        lim = coll["limit"]
        if lim is not None:
            coll["margin"] = coll["margin0"]
            lim.minimum_distance_from_collisions = coll["margin0"]
            lim.collision_detection_distance = coll["margin0"] + 0.08

    def widen_collision_margin():
        """Ratchet the margin up toward the requested value as the arms move away from the body."""
        lim = coll["limit"]
        if lim is None or coll["margin"] >= args.collision_margin or returning:
            return
        new = min(args.collision_margin, min_pair_distance() - 0.005)
        if new > coll["margin"] + 0.002:
            coll["margin"] = new
            lim.minimum_distance_from_collisions = new
            lim.collision_detection_distance = new + 0.08

    dt = 1.0 / args.rate
    max_step = args.max_vel * dt

    def hand_pose(side):
        T = cfg.get_transform_frame_to_world(HAND_BODY[side], "body")
        return T.translation().copy(), T.rotation().as_matrix().copy()

    def set_target(side, p, Rm):
        frame_tasks[side].set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(Rm), p))

    default_pose = {s: hand_pose(s) for s in sides}   # replaced by the measured start pose at enable
    for s in sides:
        set_target(s, *default_pose[s])
    home_q = q0.copy()
    fixed_out = {}   # measured angles of arms outside the IK model (--arms right), captured at enable
    jlo, jhi = model.jnt_range[:, 0] + 1e-3, model.jnt_range[:, 1] - 1e-3

    rds = None if args.no_redis else __import__("redis").Redis(host=args.redis_host, port=6379)
    if rds is not None and rds.delete("teleop_stop"):
        print("cleared an old teleop_stop flag (restart from the PC = reset)")
    source = SyntheticSource(args.scenario, args.return_button) if args.selftest else SdkSource()

    def bridge_status():
        """The bridge's latest state report (Redis teleop_state/bridge) if fresh, else None."""
        raw = rds.hget("teleop_state", "bridge") if rds is not None else None
        if not raw:
            return None
        b = json.loads(raw)
        return b if time.time() - float(b["t"]) < 0.5 else None

    def bridge_tracking():
        """True when the bridge owns the arms at full weight (TRACK). Without Redis (offline self-test): True."""
        if rds is None:
            return True
        b = bridge_status()
        return b is not None and b.get("state") == "TRACK"

    def read_measured():
        """Arm angles measured by the bridge from rt/lowstate (Redis `arm_state`), if fresh."""
        raw = rds.get("arm_state") if rds is not None else None
        if not raw:
            return None
        msg = json.loads(raw)
        if time.time() - float(msg["t"]) > STALE_S:
            return None
        return dict(zip(msg["names"], msg["q"]))
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    tag = "selftest_" + args.scenario if args.selftest else "live"
    log_path = log_dir / f"arm_ik_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    logf = open(log_path, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "wall", "active", "follow", "estop", "drill", "trigger", "grip", "stale", "ik_ms", "loop_ms",
                  "A", "B", "X", "Y", "stamp_age_ms", "head_ok", "right_ok"]
                 + [f"q_{n}" for n in ALL_NAMES] + [f"hand_{s}_{a}" for s in sides for a in "xyz"]
                 + [f"ctrl_{s}_{a}" for s in ("left", "right") for a in "xyz"] + [f"head_{a}" for a in "xyz"]
                 + ["lgrip", "ltrig", "lmenu", "rmenu", "lclick", "rclick", "laxis_x", "laxis_y", "raxis_x", "raxis_y"]
                 + ["state", "stale_ms", "follow_left", "follow_right"])

    active = follow = drill = returning = False
    stale_holding = False          # data gap while enabled but not following: hold, resume when it is back
    cooling = False                # paused by --temp-pause: A waits for --temp-resume
    missing_since = None           # when the PICO data went bad (frozen stamp or missing pose)
    estop, estop_reason = False, ""
    prev = {"A": False, "B": False, "ret": False, "grip": False}
    grip_on = {"left": False, "right": False}
    follow_s = {s: False for s in sides}   # per arm: following its controller (follow = any of them)
    grip_wait_noted = False
    clutch = {}
    last_stamp, last_stamp_change = None, time.time()
    ik_times, loop_times = [], []
    checks = {}
    rate = RateLimiter(frequency=args.rate, warn=False)
    # Everything loaded so far (torch, mujoco, mink) is long-lived: keep it out of the collector, which
    # otherwise does a ~150 ms full pass about 17 s into the run.
    gc.collect()
    gc.freeze()
    t_start = time.time()
    last_print = -1.0
    q14 = None
    if args.mode == "absolute":
        print(f"arm_ik_dryrun: ABSOLUTE mode, robot reach {reach:.3f} m, scale {abs_scale:.2f} "
              f"(human arm {args.human_arm} m), robot shoulders " +
              ", ".join(f"{s} {np.round(rob_sh[s], 3).tolist()}" for s in sides))
    else:
        print(f"arm_ik_dryrun: RELATIVE mode, scale={args.scale} rot_scale={args.rot_scale} max_reach={args.max_reach}m")
    print(f"arm_ik_dryrun: arms={args.arms} rate={args.rate}Hz max_vel={args.max_vel} selftest={args.selftest}/"
          f"{args.scenario} log={log_path}")

    def ik_state():
        if estop:
            return "ESTOP"
        if not active:
            return "PAUSED"
        if stale_holding:
            return "STALE_HOLD"
        if returning:
            return "RETURNING"
        return "FOLLOW" if follow else "ENABLED"

    def hot_motor():
        """(temperature, joint name) of the hottest motor the bridge commands, from its state report; (0, "")."""
        b = bridge_status()
        if not b:
            return 0.0, ""
        return float(b.get("temp", 0.0) or 0.0), str(b.get("temp_joint", "") or "")

    def temp_warn():
        if args.temp_warn > 0:
            return args.temp_warn
        b = bridge_status()
        return float(b.get("max_temp", 60.0) if b else 60.0) - 5.0

    def joint_ja(name):
        side = "左" if name.startswith("left") else ("右" if name.startswith("right") else "")
        for key, ja in (("shoulder", "肩"), ("elbow", "肘"), ("wrist", "手首"), ("waist", "腰")):
            if key in name:
                return side + ja
        return name or "モーター"

    def display(state, drill_out, now):
        """One line for the operator (PICO video overlay): what the arms do now and what to press.
        level: stop / warn / wait / idle / ready / ok."""
        b = bridge_status()
        temp_now, temp_joint = hot_motor()
        if state == "ESTOP":
            lv, ja, en = "stop", "非常停止（PCから再起動が必要）", "E-STOP - restart from PC"
        elif b is not None and b.get("state") in ("ESTOP_RAMP", "STOPPED"):
            # the bridge stopped on its own monitors (tracking error, IK command gap, temperature, lowstate,
            # stop.sh): the arms are back with the robot's controller even if the IK still follows the grips
            lv, ja, en = "stop", "非常停止（ブリッジ・PCから再起動が必要）", "E-STOP (bridge) - restart from PC"
        elif state == "STALE_HOLD":
            lv, ja, en = "warn", "通信待ち（データが戻れば再開）", "WAITING FOR PICO DATA"
        elif state == "PAUSED" and cooling and round(temp_now) > args.temp_resume:
            lv, ja, en = ("warn", f"冷却中 {joint_ja(temp_joint)} {temp_now:.0f}℃（{args.temp_resume:.0f}℃以下で右A）",
                          f"COOLING {temp_now:.0f} C - A at {args.temp_resume:.0f} C")
        elif state == "PAUSED":
            lv, ja, en = "idle", "一時停止中（右Aで開始）", "PAUSED - press A"
        elif state == "RETURNING" and cooling:
            lv, ja, en = ("warn", f"熱いので最初の姿勢に戻り中（{joint_ja(temp_joint)} {temp_now:.0f}℃）",
                          f"HOT {temp_now:.0f} C - RETURNING TO START")
        elif state == "RETURNING":
            lv, ja, en = "wait", "最初の姿勢に戻り中", "RETURNING TO START"
        elif state == "FOLLOW":
            on = [s for s in ("right", "left") if follow_s.get(s)]
            ja_s = "両手" if len(on) == 2 else ("右" if on == ["right"] else "左")
            en_s = "BOTH" if len(on) == 2 else ("R" if on == ["right"] else "L")
            lv, ja, en = "ok", f"追従中（{ja_s}）", f"FOLLOWING {en_s}"
        elif b is None:
            lv, ja, en = "warn", "ブリッジ待ち", "WAITING FOR BRIDGE"
        elif b.get("state") == "RAMP_UP":
            left = float(b.get("ramp_left", 0.0))
            lv, ja, en = "wait", f"準備中 あと{left:.1f}秒（そのあと握れば動く）", f"GET READY {left:.1f} s"
        elif b.get("state") == "TRACK":
            lv, ja, en = "ready", "握れば動きます", "READY - hold grip"
        else:
            lv, ja, en = "wait", "ブリッジ準備中", "BRIDGE " + str(b.get("state"))
        if state in ("FOLLOW", "ENABLED") and lv in ("ok", "ready") and temp_now >= temp_warn():
            lv = "warn"
            ja = ja + f" ／ {joint_ja(temp_joint)} {temp_now:.0f}℃ 腕を下ろして休ませて"
            en = en + f" / HOT {temp_now:.0f} C"
        if drill_out:
            ja, en = ja + " ／ ドリルON", en + " / DRILL ON"
        return {"t": now, "level": lv, "ja": ja, "en": en, "ik": state,
                "bridge": b.get("state") if b else None, "weight": b.get("weight") if b else None,
                "reason": estop_reason or (b.get("reason", "") if b else ""), "drill": int(bool(drill_out)),
                "temp": round(temp_now, 1), "temp_joint": temp_joint}

    def publish(now, drill_out, trig):
        if rds is None:
            return
        rds.set("arm_ik_q", json.dumps({"t": now, "active": active, "follow": follow,
                                        "follow_arms": [s for s in sides if follow_s[s]], "estop": estop,
                                        "estop_reason": estop_reason, "names": ALL_NAMES, "q": q14}))
        rds.set("drill_cmd", json.dumps({"t": now, "on": int(drill_out), "trigger": trig}))
        state = ik_state()
        rds.hset("teleop_state", "ik", json.dumps({"t": now, "state": state, "reason": estop_reason}))
        rds.set("teleop_display", json.dumps(display(state, drill_out, now), ensure_ascii=False))

    def trip(t, reason):
        nonlocal estop, estop_reason, active, follow, returning, stale_holding
        if not estop:
            estop, estop_reason = True, reason
            active = follow = returning = stale_holding = False
            follow_s.update({s: False for s in follow_s})
            print(f"[{t:6.1f}s] EMERGENCY STOP: {reason}")

    try:
        while True:
            t_loop = time.perf_counter()
            t = time.time() - t_start
            if args.duration and t > args.duration:
                break
            d = source.read(t)

            # Data freshness.
            now_w = time.time()
            if d["stamp"] != last_stamp:
                last_stamp, last_stamp_change = d["stamp"], now_w
            pose_missing = d["head"] is None or d["right"] is None
            stale = (now_w - last_stamp_change) > STALE_S or pose_missing
            if stale and missing_since is None:
                missing_since = now_w if pose_missing else last_stamp_change
            gap_s = (now_w - missing_since) if missing_since is not None else 0.0   # length of the current gap
            if not stale:
                missing_since = None
            stale_s = gap_s if stale else 0.0

            # Emergency stops (latched).
            if d["X"] or d["Y"]:
                trip(t, "left X/Y pressed")
            if stale and active:
                if follow:   # the arms were following the hands: stop at once
                    trip(t, f"PICO data missing > {STALE_S} s while following")
                elif stale_s > args.stale_hold:
                    trip(t, f"PICO data missing > {args.stale_hold} s")
                elif not stale_holding:
                    stale_holding = True
                    print(f"[{t:6.1f}s] PICO data gap while not following -> arms hold, resumes when data is back")
            if stale_holding and (not stale or not active):
                stale_holding = False
                if active and not estop:
                    print(f"[{t:6.1f}s] PICO data back after {gap_s:.2f} s -> ENABLED (hold the grip to move)")
            if rds is not None and rds.exists("teleop_stop"):
                trip(t, "stop command (teleop_stop)")

            # Temperature: the hottest motor the bridge commands at --temp-pause -> glide back, then pause to
            # cool down (the bridge's own emergency stop at --max-temp would release the arms abruptly).
            temp_now, temp_joint = hot_motor() if rds is not None else (0.0, "")
            if (args.temp_pause > 0 and active and not estop and not returning
                    and temp_now >= args.temp_pause):
                follow, returning, cooling = False, True, True
                follow_s.update({s: False for s in follow_s})
                reset_collision_margin()
                for s in sides:
                    set_target(s, *default_pose[s])
                print(f"[{t:6.1f}s] {temp_joint} at {temp_now:.0f} C (>= {args.temp_pause:.0f}) -> gliding back to "
                      f"the start pose, then pause to cool down (A again at <= {args.temp_resume:.0f} C)")

            # A: enable (start pose = measured arm angles) / pause. Return button (right stick press by default):
            # glide back to the start pose, then pause. B is left to Remote Vision. Ignored after an emergency stop.
            ret_now = bool(d.get(args.return_button, False)) if args.return_button != "none" else False
            if not estop:
                if d["A"] and not prev["A"]:
                    if stale:
                        print(f"[{t:6.1f}s] A pressed but PICO data is stale -> ignored")
                    elif not active and cooling and round(temp_now) > args.temp_resume:
                        print(f"[{t:6.1f}s] A ignored: {temp_joint} still at {temp_now:.0f} C "
                              f"(cooling down to {args.temp_resume:.0f} C)")
                    elif active:
                        active, returning, follow = False, False, False
                        follow_s.update({s: False for s in follow_s})
                        print(f"[{t:6.1f}s] A -> paused")
                    else:
                        meas = read_measured()
                        if meas is None and rds is not None:
                            print(f"[{t:6.1f}s] A ignored: no fresh arm_state (is arm_sdk_bridge.py running?)")
                        else:
                            if cooling:
                                print(f"[{t:6.1f}s] cooled down ({temp_joint} {temp_now:.0f} C)")
                            cooling = False
                            if meas is not None:
                                home_q = np.clip(np.array([meas[n] for n in names]), jlo, jhi)
                                fixed_out = {n: meas[n] for n in ALL_NAMES if n not in names}
                            else:
                                home_q = cfg.q.copy()   # offline self-test without the bridge
                            cfg.update(home_q)
                            posture.set_target(home_q)
                            cl = set_collision_limit()
                            for s in sides:
                                default_pose[s] = hand_pose(s)
                                set_target(s, *default_pose[s])
                            active, returning, follow = True, False, False
                            src = "measured" if meas is not None else "IK default (no bridge)"
                            col = (f"collision margin {cl[1] * 100:.1f} cm (closest arm-body gap now {cl[0] * 100:.1f} cm)"
                                   if cl else "collision avoidance off")
                            how = ("hold a grip to move that arm (right grip = right arm, left grip = left arm)"
                                   if args.grip_mode == "per_arm" else "hold the right grip to move")
                            print(f"[{t:6.1f}s] A -> ENABLED, start pose = {src} angles, {col}; {how}")
                if ret_now and not prev["ret"] and active:
                    follow, returning = False, True
                    follow_s.update({s: False for s in follow_s})
                    reset_collision_margin()
                    for s in sides:
                        set_target(s, *default_pose[s])
                    print(f"[{t:6.1f}s] {args.return_button} -> gliding back to the start pose, then pause")
                if d["B"] and not prev["B"] and args.return_button != "B":
                    print(f"[{t:6.1f}s] B pressed (Remote Vision eye switch) -> no teleop action")
            prev["A"], prev["B"], prev["ret"] = d["A"], d["B"], ret_now

            # Dead-man per arm (hysteresis): the right grip moves the right arm, the left grip the left arm
            # (--grip-mode right: the right grip moves both). Each new grip re-anchors that arm's clutch: no jump.
            for g, key in (("right", "grip"), ("left", "lgrip")):
                v = float(d.get(key, 0.0))
                if grip_on[g] and v < 0.4:
                    grip_on[g] = False
                elif not grip_on[g] and v > 0.6:
                    grip_on[g] = True
            can_follow = active and not stale and not estop and not returning
            want = {s: can_follow and d[s] is not None
                    and (grip_on["right"] if args.grip_mode == "right" else grip_on[s]) for s in sides}
            if any(want[s] and not follow_s[s] for s in sides) and not bridge_tracking():
                # The bridge is still ramping its weight: nothing follows until it fully owns the arms.
                if not grip_wait_noted:
                    print(f"[{t:6.1f}s] grip held, waiting for the bridge to reach TRACK (weight ramp)")
                    grip_wait_noted = True
                want = {s: want[s] and follow_s[s] for s in sides}
            if not any(grip_on.values()):
                grip_wait_noted = False
            for s in sides:
                if want[s] and not follow_s[s]:
                    yaw0 = head_yaw(d["head"][1])
                    clutch[s] = {"Rz": rot_z(-yaw0), "yaw0": yaw0, "c": d[s], "h": hand_pose(s),
                                 "tgt": hand_pose(s)}   # rate-limited target (absolute mode) starts at the hand
                    print(f"[{t:6.1f}s] grip -> {s} arm FOLLOW (head yaw {math.degrees(yaw0):.0f} deg)")
                elif follow_s[s] and not want[s]:
                    if active:
                        print(f"[{t:6.1f}s] grip released -> {s} arm holds")
                    set_target(s, *hand_pose(s))   # freeze the target where the arm is now
                follow_s[s] = want[s]
            follow = any(follow_s.values())

            # Hand targets, in the operator's facing direction at the grip (head yaw).
            if follow:
                for s in sides:
                    if not follow_s[s] or d[s] is None:
                        continue
                    (c0p, c0R), (h0p, h0R) = clutch[s]["c"], clutch[s]["h"]
                    cp, cR = d[s]
                    if args.mode == "absolute":
                        # Operator hand relative to the operator's shoulder (estimated from the headset),
                        # scaled, placed at the robot's shoulder. Orientation: controller -> hand, fixed map.
                        Ryaw = rot_z(clutch[s]["yaw0"])
                        centre = M @ d["head"][0] + Ryaw @ np.array([-args.head_back, 0.0, -args.head_down])
                        side_y = (0.5 if s == "left" else -0.5) * args.human_shoulder_width
                        op_sh = centre + Ryaw @ np.array([0.0, side_y, 0.0])
                        v = abs_scale * (clutch[s]["Rz"] @ (M @ cp - op_sh))
                        if args.max_forward > 0:      # keep the arms from reaching far out (waist load)
                            v[0] = min(v[0], args.max_forward)
                        n = np.linalg.norm(v)
                        if n > args.reach_frac * reach:
                            v *= args.reach_frac * reach / n
                        want_p, want_R = rob_sh[s] + v, clutch[s]["Rz"] @ (M @ cR @ M.T) @ R_fix
                        # Move the target toward the wanted pose at a limited speed: a slow approach right
                        # after the grip, and no jumps when the operator moves fast.
                        p0, R0 = clutch[s]["tgt"]
                        step = want_p - p0
                        n = np.linalg.norm(step)
                        if n > args.hand_speed * dt:
                            step *= args.hand_speed * dt / n
                        rv = R.from_matrix(want_R @ R0.T).as_rotvec()
                        a = np.linalg.norm(rv)
                        if a > args.hand_turn * dt:
                            rv *= args.hand_turn * dt / a
                        clutch[s]["tgt"] = (p0 + step, R.from_rotvec(rv).as_matrix() @ R0)
                        set_target(s, *clutch[s]["tgt"])
                        continue
                    dp = args.scale * (clutch[s]["Rz"] @ M @ (cp - c0p))
                    n = np.linalg.norm(dp)
                    if n > args.max_reach:
                        dp *= args.max_reach / n
                    dR = clutch[s]["Rz"] @ M @ (cR @ c0R.T) @ M.T @ clutch[s]["Rz"].T
                    dR = R.from_rotvec(args.rot_scale * R.from_matrix(dR).as_rotvec()).as_matrix()
                    set_target(s, h0p + dp, dR @ h0R)

            # IK step only while following or gliding back; otherwise the arms hold.
            t_ik = time.perf_counter()
            if follow or returning:
                widen_collision_margin()
                q_prev = cfg.q.copy()
                held = [] if returning else [hold_limits[s] for s in sides if not follow_s[s]]
                vel = mink.solve_ik(cfg, list(frame_tasks.values()) + [posture], dt, "daqp",
                                    damping=1e-3, limits=limits + held)
                cfg.integrate_inplace(vel, dt)
                q_new = q_prev + np.clip(cfg.q - q_prev, -max_step, max_step)
                if not returning:   # an arm whose grip is not held stays exactly where it is
                    for s in sides:
                        if not follow_s[s]:
                            q_new[side_idx[s]] = q_prev[side_idx[s]]
                cfg.update(q_new)
                if returning and np.max(np.abs(cfg.q - home_q)) < 0.02:
                    returning, active = False, False
                    print(f"[{t:6.1f}s] start pose reached -> paused")
            ik_ms = (time.perf_counter() - t_ik) * 1000.0

            # Drill: right trigger (hysteresis), only while the right arm follows and nothing tripped.
            trig = d["trigger"]
            if drill and trig < 0.4:
                drill = False
            elif not drill and trig > 0.6:
                drill = True
            drill_out = drill and follow_s.get("right", False) and not estop

            q_by_name = dict(zip(names, cfg.q))
            for s in ("left", "right"):
                if s not in sides:   # arm outside the IK model: hold what was measured at enable
                    q_by_name.update({n: fixed_out.get(n, v) for n, v in zip(ARM_JOINTS[s], DEFAULT_Q[s])})
            q14 = [float(q_by_name[n]) for n in ALL_NAMES]
            publish(time.time(), drill_out, trig)

            hands = [v for s in sides for v in hand_pose(s)[0]]
            ctrl = np.concatenate([(M @ d[s][0]) if d[s] is not None else np.zeros(3) for s in ("left", "right")]
                                  + [(M @ d["head"][0]) if d["head"] is not None else np.zeros(3)])
            loop_ms = (time.perf_counter() - t_loop) * 1000.0
            ik_times.append(ik_ms)
            loop_times.append(loop_ms)
            log.writerow([f"{t:.3f}", f"{time.time():.3f}", int(active), int(follow), int(estop), int(drill_out), f"{trig:.2f}",
                          f"{d['grip']:.2f}", int(stale), f"{ik_ms:.2f}", f"{loop_ms:.2f}",
                          int(d["A"]), int(d["B"]), int(d["X"]), int(d["Y"]),
                          f"{(time.time() - last_stamp_change) * 1000:.0f}",
                          int(d["head"] is not None), int(d["right"] is not None)]
                         + [f"{v:.4f}" for v in q14] + [f"{v:.4f}" for v in hands] + [f"{v:.4f}" for v in ctrl]
                         + [f"{d.get('lgrip', 0.0):.2f}", f"{d.get('ltrig', 0.0):.2f}"]
                         + [int(d.get(k, False)) for k in ("lmenu", "rmenu", "lclick", "rclick")]
                         + [f"{v:.2f}" for v in (list(d.get("laxis", [0.0, 0.0]))[:2] + list(d.get("raxis", [0.0, 0.0]))[:2])]
                         + [ik_state(), f"{stale_s * 1000:.0f}",
                            int(follow_s.get("left", False)), int(follow_s.get("right", False))])

            if args.selftest and args.scenario == "per_arm":
                for name, tm in (("start", 4.4), ("right_only", 7.4), ("left_only", 10.9), ("both", 13.4)):
                    if name not in checks and t >= tm:
                        checks[name] = {s: hand_pose(s)[0] for s in sides}
            if args.selftest and args.scenario == "none":
                for name, t0, t1 in SyntheticSource.PHASES:
                    if name not in checks and t >= (t0 + t1) / 2 and follow:
                        hp, hR = hand_pose("right")
                        h0p, h0R = clutch["right"]["h"]
                        rel = R.from_matrix(hR @ h0R.T).as_rotvec()
                        checks[name] = (hp - h0p, np.degrees(rel))

            if t - last_print >= 1.0:
                last_print = t
                r = [round(float(q_by_name[n]), 2) for n in ARM_JOINTS["right"]]
                fl = "".join(k for s, k in (("left", "L"), ("right", "R")) if follow_s.get(s)) or "-"
                print(f"[{t:6.1f}s] active={int(active)} follow={fl} estop={int(estop)} drill={int(drill_out)} "
                      f"ik={ik_ms:.1f}ms right_q={r}")
            rate.sleep()
    except (KeyboardInterrupt, StopRequested) as e:
        trip(time.time() - t_start, "Ctrl+C" if isinstance(e, KeyboardInterrupt) else "SIGTERM")
    finally:
        if not estop:
            trip(time.time() - t_start, "IK program exited")
        if rds is not None and q14 is not None:
            publish(time.time(), False, 0.0)
        source.close()
        logf.close()

    ik, lp = np.array(ik_times), np.array(loop_times)
    print(f"timing over {len(lp)} loops: ik avg {ik.mean():.2f} ms / max {ik.max():.2f} ms, "
          f"loop avg {lp.mean():.2f} ms / p95 {np.percentile(lp, 95):.2f} ms / max {lp.max():.2f} ms "
          f"(budget {dt * 1000:.1f} ms)")
    if args.selftest and args.scenario == "per_arm":
        order = [k for k in ("start", "right_only", "left_only", "both") if k in checks]
        for a, b in zip(order, order[1:]):
            moved = ", ".join(f"{s} {np.linalg.norm(checks[b][s] - checks[a][s]) * 100:.1f} cm" for s in sides)
            print(f"check per_arm {b:10s}: hands moved {moved}")
    else:
        for name, (dp, drot) in checks.items():
            print(f"check {name:7s}: hand moved x{dp[0]:+.3f} y{dp[1]:+.3f} z{dp[2]:+.3f} m, rot {np.round(drot, 1)} deg")
    print(f"log: {log_path}")


if __name__ == "__main__":
    main()
