#!/bin/bash
# Simulator only (no robot I/O). Waist gains stock 40/29, 60/1.5, TWIST2 150/4, each without / with the
# integral waist correction (roll,pitch; ki 1.0, max 0.15 rad, 0.1 rad/s), on sim_suite and reach_fwd. ~8 min.
cd ~/nextech/arm_ik || exit 1
PY=~/miniforge3/envs/gmr/bin/python
CORR="--waist-correct roll,pitch"
for sc in sim_suite reach_fwd; do
  for g in "stock 40/29|" "60/1.5|--waist-kp 60,60,60 --waist-kd 1.5,1.5,1.5" "150/4|--waist-gains twist2"; do
    label=${g%%|*}; ga=${g#*|}
    for c in off on; do
      ba="$ga"; [ "$c" = on ] && ba="$ga $CORR"
      tag=$(bash sim_run.sh "$sc" 0 0 0.5 "$ba" "--max-vel 0.5" 2>&1 | grep -o "${sc}_0kg_off0.05_[0-9]*" | head -1)
      $PY waist_line2.py "$tag" "$label corr $c [$sc]"
    done
  done
done
echo BATCH_DONE
