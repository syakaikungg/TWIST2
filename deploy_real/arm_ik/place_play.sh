#!/bin/bash
# Run on the operator PC from this clone (e.g. ~/g1_teleop): copies the automatic playback files to the G1's
# ~/nextech/arm_ik/ over SSH. Files that are replaced are kept as *.bak_YYYYMMDD[_HHMMSS] on the G1. Moves nothing.
#   bash deploy_real/arm_ik/place_play.sh            (G1_HOST / G1_KEY as for g1.sh)
set -eu
HOST=${G1_HOST:-unitree@192.168.123.164}
KEY=${G1_KEY:-$HOME/.ssh/id_ed25519_g1}
cd "$(dirname "$0")"
FILES="play_motion.py design_motion.py tool_df033d.py log_to_motion.py run_play.sh run_live.sh arm_sim.py render_front.py"
for f in $FILES motions/drill_work.json; do [ -r "$f" ] || { echo "missing $f (wrong branch? git checkout claude/project-thread-a5zqv6)"; exit 1; }; done
B=bak_$(date +%Y%m%d_%H%M%S)
ssh -i "$KEY" -o ConnectTimeout=8 "$HOST" "cd ~/nextech/arm_ik && mkdir -p motions && for f in $FILES motions/drill_work.json; do [ -e \$f ] && cp -p \$f \$f.$B; done; ps -eo cmd | grep -E 'arm_sdk_bridge|arm_ik_dryrun|play_motion' | grep -v grep && echo 'NOTE: a bridge / IK / playback is running; the new files take effect at its next start' || true"
scp -i "$KEY" -o ConnectTimeout=8 $FILES "$HOST:nextech/arm_ik/"
scp -i "$KEY" -o ConnectTimeout=8 motions/drill_work.json "$HOST:nextech/arm_ik/motions/"
echo "--- placed (old ones kept as *.$B) ---"
md5sum $FILES motions/drill_work.json
ssh -i "$KEY" -o ConnectTimeout=8 "$HOST" "cd ~/nextech/arm_ik && md5sum $FILES motions/drill_work.json && ~/miniforge3/envs/gmr/bin/python play_motion.py --check"
