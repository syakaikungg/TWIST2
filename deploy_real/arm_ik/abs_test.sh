#!/bin/bash
# Absolute mode in the simulator (scripted inputs, video from behind), then the absolute-mode check. No robot I/O.
cd ~/nextech/arm_ik || exit 1
bash sim_run.sh sim_suite 0 1 "${1:-1.0}" "" "--max-vel ${1:-1.0}" 2>&1 | grep -E '===|EMERGENCY STOP|ABSOLUTE|A ->|grip|B ->|start pose|timing|video:|weight [0-9]|contact'
~/miniforge3/envs/gmr/bin/python diagnose_abs.py "$(ls -t ~/nextech_setup/logs/arm_ik_selftest_sim_suite_*.csv | head -1)"
