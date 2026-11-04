#!/usr/bin/env python3
"""Read rt/lowstate (and try the BMS topic) for a few seconds and summarise. Subscribes only; publishes nothing."""
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg import dds_ as hg

NAMES = {12: "waist_yaw", 13: "waist_roll", 14: "waist_pitch",
         15: "L_sh_pitch", 16: "L_sh_roll", 17: "L_sh_yaw", 18: "L_elbow", 19: "L_wr_roll", 20: "L_wr_pitch", 21: "L_wr_yaw",
         22: "R_sh_pitch", 23: "R_sh_roll", 24: "R_sh_yaw", 25: "R_elbow", 26: "R_wr_roll", 27: "R_wr_pitch", 28: "R_wr_yaw"}

ChannelFactoryInitialize(0, "eth0")
msgs, bms = [], []
s = ChannelSubscriber("rt/lowstate", hg.LowState_)
s.Init(lambda m: msgs.append(m), 10)
if hasattr(hg, "BmsState_"):
    for topic in ("rt/lf/bmsstate", "rt/bmsstate"):
        b = ChannelSubscriber(topic, hg.BmsState_)
        b.Init(lambda m, tp=topic: bms.append((tp, m)), 10)
t0 = time.time()
time.sleep(3.0)
n = len(msgs)
print(f"rt/lowstate: {n} messages in 3 s ({n / 3:.0f} Hz)")
if n:
    m = msgs[-1]
    print(f"mode_machine={m.mode_machine} mode_pr={m.mode_pr} tick={m.tick}")
    print("imu rpy (deg):", np.round(np.degrees(np.array(m.imu_state.rpy)), 1).tolist())
    for i, name in NAMES.items():
        ms = m.motor_state[i]
        print(f"  {i:2d} {name:10s} q={ms.q:+.3f} rad  dq={ms.dq:+.2f}  tau={ms.tau_est:+.2f}  temp={list(ms.temperature)}")
    temps = [max(m.motor_state[i].temperature) for i in range(29)]
    vols = [m.motor_state[i].vol for i in range(29)]
    print(f"all 29 motors: max temp {max(temps)} C (motor {int(np.argmax(temps))}), voltage {min(vols):.1f}-{max(vols):.1f} V")
if bms:
    tp, b = bms[-1]
    fields = {k: getattr(b, k) for k in ("soc", "current", "cycle", "bmsvoltage", "temperature") if hasattr(b, k)}
    print(f"BMS ({tp}):", fields)
else:
    print("BMS: no message on rt/lf/bmsstate or rt/bmsstate")
