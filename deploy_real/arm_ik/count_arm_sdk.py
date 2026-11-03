#!/usr/bin/env python3
"""Count messages on rt/arm_sdk (and rt/lowcmd) for 3 s. Subscribes only; publishes nothing."""
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

ChannelFactoryInitialize(0, "eth0")
seen = {"rt/arm_sdk": [], "rt/lowcmd": []}
subs = []
for topic in seen:
    s = ChannelSubscriber(topic, LowCmd_)
    s.Init(lambda m, tp=topic: seen[tp].append(m), 10)
    subs.append(s)
time.sleep(3.0)
for topic, msgs in seen.items():
    w = msgs[-1].motor_cmd[29].q if msgs else None
    print(f"{topic}: {len(msgs)} messages in 3 s" + (f", last weight(q of motor 29)={w:.3f}" if msgs else ""))
