#!/bin/bash

source ~/miniconda3/bin/activate twist2

SCRIPT_DIR=$(dirname $(realpath $0))

# Find the latest recorded motion
latest_dir=$(ls -td ${SCRIPT_DIR}/recorded_motions/*/ 2>/dev/null | head -1)

if [ -z "$latest_dir" ]; then
    echo "❌ No recorded motions found in recorded_motions/"
    echo ""
    echo "Please record a motion first using:"
    echo "  1. bash teleop.sh"
    echo "  2. bash data_record.sh"
    echo "  3. Press Y on left controller to start/stop recording"
    exit 1
fi

# Find the latest episode in that directory
latest_episode=$(ls -td ${latest_dir}episode_*/ 2>/dev/null | head -1)

if [ -z "$latest_episode" ]; then
    echo "❌ No episodes found in ${latest_dir}"
    exit 1
fi

data_file="${latest_episode}data.json"

if [ ! -f "$data_file" ]; then
    echo "❌ data.json not found: ${data_file}"
    exit 1
fi

echo "=================================================================="
echo "Playing Recorded Motion"
echo "=================================================================="
echo "Data file: ${data_file}"
echo "Redis IP: localhost"
echo "Playback frequency: 30 Hz"
echo ""
echo "This will play back your recorded motion in a loop."
echo "Use with sim2sim.sh or sim2real_both_arms.sh"
echo "=================================================================="
echo ""

cd deploy_real

python server_motion_recorded.py \
    --data_file "${data_file}" \
    --redis_ip localhost \
    --playback_frequency 30
