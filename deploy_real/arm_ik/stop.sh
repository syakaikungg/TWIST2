#!/bin/bash
# Emergency stop for the arm teleop, from any terminal on the G1 (or: ssh unitree@192.168.123.164 ~/nextech/arm_ik/stop.sh).
# Sets `teleop_stop` in Redis: the IK program and the arm_sdk bridge both trip (drill OFF, arms handed back).
python3 -c "import json, time, redis; redis.Redis(host='127.0.0.1', port=6379).set('teleop_stop', json.dumps({'t': time.time(), 'by': 'stop.sh'})); print('teleop_stop set')"
