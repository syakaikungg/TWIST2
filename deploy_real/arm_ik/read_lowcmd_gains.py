#!/usr/bin/env python3
"""Read the built-in controller's gains from rt/lowcmd for a few seconds (subscribe only, nothing published).
Also print the robot's lowstate mode and waist state for context."""
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

ChannelFactoryInitialize(0, "eth0")
cmds, states = [], []
a = ChannelSubscriber("rt/lowcmd", LowCmd_)
a.Init(lambda m: cmds.append(m), 10)
b = ChannelSubscriber("rt/lowstate", LowState_)
b.Init(lambda m: states.append(m), 10)
time.sleep(6.0)
print(f"rt/lowcmd: {len(cmds)} messages in 6 s; rt/lowstate: {len(states)}")
if states:
    s = states[-1]
    print(f"lowstate mode_machine={s.mode_machine} mode_pr={s.mode_pr}; waist q/tau: " +
          ", ".join(f"{i}:{s.motor_state[i].q:+.3f}/{s.motor_state[i].tau_est:+.2f}" for i in (12, 13, 14)))
if cmds:
    kp = np.array([[c.motor_cmd[i].kp for i in range(29)] for c in cmds[-300:]])
    kd = np.array([[c.motor_cmd[i].kd for i in range(29)] for c in cmds[-300:]])
    q = np.array([[c.motor_cmd[i].q for i in range(29)] for c in cmds[-300:]])
    print(f"lowcmd mode_pr={cmds[-1].mode_pr} mode_machine={cmds[-1].mode_machine}")
    groups = {"legs 0-11": range(0, 12), "waist 12-14": range(12, 15), "arms 15-28": range(15, 29)}
    for g, idx in groups.items():
        print(f"  {g}: kp " + " ".join(f"{kp[:, i].mean():.0f}" for i in idx))
        print(f"  {g}: kd " + " ".join(f"{kd[:, i].mean():.2f}" for i in idx))
    print("  waist q (cmd): " + " ".join(f"{q[:, i].mean():+.3f}" for i in (12, 13, 14)))
    print("  kp spread over the window (max-min) for waist:", [round(float(np.ptp(kp[:, i])), 2) for i in (12, 13, 14)])
