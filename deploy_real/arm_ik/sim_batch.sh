#!/bin/bash
# Batch of simulator runs (no robot I/O): default monitors, then tracking monitor relaxed to see every phase,
# then dummy drill masses at the right hand.
cd ~/nextech/arm_ik || exit 1
case "${1:-all}" in
  base)  bash sim_run.sh sim_suite 0 1 1.0
         bash sim_run.sh sim_suite 0 1 1.0 "--max-track-err 3.0" ;;
  mass)  for kg in 1.0 1.5 2.0; do bash sim_run.sh sim_suite $kg 0 1.0 "--max-track-err 3.0"; done ;;
  all)   bash "$0" base; bash "$0" mass ;;
esac
