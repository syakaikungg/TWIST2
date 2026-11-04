#!/usr/bin/env python3
"""IK model geometry for the absolute mode: shoulder positions, arm length, hand-frame axes. Read only."""
import mujoco
import numpy as np

from general_motion_retargeting import ROBOT_XML_DICT

m = mujoco.MjModel.from_xml_path(str(ROBOT_XML_DICT["unitree_g1"]))
d = mujoco.MjData(m)
bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
jq = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
d.qpos[:] = 0
d.qpos[2] = 0.793
d.qpos[3] = 1.0
pose = {"left": [0.0, 0.4, 0.0, 1.2, 0.0, 0.0, 0.0], "right": [0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0]}
names = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
for config in ("zero", "default", "forward"):
    for s in ("left", "right"):
        for i, n in enumerate(names):
            v = 0.0 if config == "zero" else pose[s][i]
            if config == "forward":   # arm straight forward: shoulder pitch -pi/2, elbow 0
                v = {"shoulder_pitch": -1.5708}.get(n, 0.0)
            d.qpos[jq(f"{s}_{n}_joint")] = v
    mujoco.mj_forward(m, d)
    print(f"== {config} arms ==")
    for s in ("left", "right"):
        sh, hand, wy = bid(f"{s}_shoulder_pitch_link"), bid(f"{s}_rubber_hand"), bid(f"{s}_wrist_yaw_link")
        R = d.xmat[hand].reshape(3, 3)
        print(f"  {s}: shoulder {d.xpos[sh].round(3)}, hand {d.xpos[hand].round(3)}, "
              f"|hand-shoulder| {np.linalg.norm(d.xpos[hand] - d.xpos[sh]):.3f} m; hand local x->world {R[:, 0].round(2)}, "
              f"y->{R[:, 1].round(2)}, z->{R[:, 2].round(2)}")
    if config == "zero":
        for s in ("left", "right"):
            print(f"  {s}_rubber_hand pos in parent {m.body_pos[bid(f'{s}_rubber_hand')].round(3)}, "
                  f"quat {m.body_quat[bid(f'{s}_rubber_hand')].round(3)}, parent {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[bid(f'{s}_rubber_hand')])}")
print("head-ish: torso_link pos", d.xpos[bid("torso_link")].round(3))
