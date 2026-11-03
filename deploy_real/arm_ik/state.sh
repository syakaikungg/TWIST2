#!/bin/bash
# Show the live teleop state (Redis teleop_state) and whether the programs are running. Read only.
python3 -c "
import json, time, redis
r = redis.Redis()
for k, v in r.hgetall('teleop_state').items():
    d = json.loads(v); print(k.decode(), d.get('state'), 'weight', d.get('weight', '-'), 'age %.1fs' % (time.time() - d['t']), d.get('reason', ''))
"
pgrep -af 'arm_sdk_bridge|arm_ik_dryrun' | grep -v pgrep || echo "no teleop programs running"
