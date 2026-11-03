#!/bin/bash
# Start one scripted simulator run in the background (no robot I/O); output in ~/nextech_setup/logs/sim_demo.txt
cd ~/nextech/arm_ik || exit 1
nohup bash sim_run.sh sim_suite "${1:-0}" 0 1.0 "" "--max-vel 1.0" > ~/nextech_setup/logs/sim_demo.txt 2>&1 < /dev/null &
echo "sim demo started (pid $!)"
