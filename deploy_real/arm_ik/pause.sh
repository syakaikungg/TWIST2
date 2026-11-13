#!/bin/bash
# Pause / resume the automatic playback (play_motion.py), on the G1: pause.sh = the work stops, the arms stay at the
# rest pose and the bridge keeps holding arms and waist softly (no hand-back to the robot's lock stand);
# pause.sh off = work again (after the usual cool-down). From the PC: g1.sh pause / g1.sh resume.
case "${1:-on}" in
  on)  python3 -c "import json, time, redis; redis.Redis(host='127.0.0.1', port=6379).set('play_hold', json.dumps({'t': time.time()})); print('play_hold set: pause')" ;;
  off) python3 -c "import redis; redis.Redis(host='127.0.0.1', port=6379).delete('play_hold'); print('play_hold deleted: resume')" ;;
  *) echo "usage: pause.sh [on|off]"; exit 1 ;;
esac
