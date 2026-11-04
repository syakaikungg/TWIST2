#!/bin/bash
# Waist gains x reach limits, scenarios sim_suite and reach_fwd, simulator only (no robot I/O). ~7 min.
cd ~/nextech/arm_ik || exit 1
PY=~/miniforge3/envs/gmr/bin/python
run() {  # label scenario "bridge args" "ik args"
  local out tag
  out=$(bash sim_run.sh "$2" 0 0 0.5 "$3" "--max-vel 0.5 $4" 2>&1)
  tag=$(echo "$out" | grep -o "${2}_0kg_off0.05_[0-9]*" | head -1)
  $PY waist_line.py "$tag" "$1 [$2]"
}
for sc in sim_suite reach_fwd; do
  run "A waist 60/1.5, reach 95%"            $sc "--waist-kp 60,60,60 --waist-kd 1.5,1.5,1.5" ""
  run "B waist built-in, reach 95%"          $sc "" ""
  run "C built-in, reach 80%, fwd<=0.25m"    $sc "" "--reach-frac 0.80 --max-forward 0.25"
  run "D built-in, reach 75%, fwd<=0.20m"    $sc "" "--reach-frac 0.75 --max-forward 0.20"
  run "E ref TWIST2 150/4 (sim only)"        $sc "--waist-kp 150,150,150 --waist-kd 4,4,4" ""
done
