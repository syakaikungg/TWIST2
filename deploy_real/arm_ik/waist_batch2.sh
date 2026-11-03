#!/bin/bash
# Simulator only (no robot I/O). Reach narrowing with waist 60/1.5 and TWIST2 150/4, then the IK damping
# (smoothing) and hand speed limit vs lag / overshoot / contact / torque. ~10 min.
cd ~/nextech/arm_ik || exit 1
PY=~/miniforge3/envs/gmr/bin/python
W60="--waist-kp 60,60,60 --waist-kd 1.5,1.5,1.5"; W150="--waist-gains twist2"
sim() {  # scenario "bridge args" "ik args" -> prints the run tag
  bash sim_run.sh "$1" 0 0 0.5 "$2" "--max-vel 0.5 $3" 2>&1 | grep -o "${1}_0kg_off0.05_[0-9]*" | head -1
}
run() {  # label scenario "bridge args" "ik args"
  local tag; tag=$(sim "$2" "$3" "$4")
  $PY waist_line.py "$tag" "$1 [$2]"
}
echo "### reach narrowing (the 95% rows are in waist_batch.txt)"
for sc in sim_suite reach_fwd; do
  run "60/1.5, reach 80%, fwd<=0.25m" $sc "$W60" "--reach-frac 0.80 --max-forward 0.25"
  run "60/1.5, reach 75%, fwd<=0.20m" $sc "$W60" "--reach-frac 0.75 --max-forward 0.20"
  run "150/4, reach 80%, fwd<=0.25m"  $sc "$W150" "--reach-frac 0.80 --max-forward 0.25"
  run "150/4, reach 75%, fwd<=0.20m"  $sc "$W150" "--reach-frac 0.75 --max-forward 0.20"
done
echo "### IK damping / hand speed (waist 60/1.5 for sim_suite, 150/4 for reach_fwd so it does not trip)"
for set in "damping 1.0 (default)|" "damping 0.3|--lm-damping 0.3" "damping 0.1|--lm-damping 0.1" \
           "damping 0.1 + hand 0.6 m/s|--lm-damping 0.1 --hand-speed 0.6"; do
  label=${set%%|*}; ik=${set#*|}
  tag=$(sim sim_suite "$W60" "$ik")
  echo "$label [sim_suite]:"; $PY diagnose_abs2.py sim_suite 2>&1 | grep -E 'wanted vs|fwd :|up  :'
  $PY sim_line.py "$tag" "  $label [sim_suite]"
  tag=$(sim reach_fwd "$W150" "$ik")
  $PY sim_line.py "$tag" "  $label [reach_fwd]"
done
echo BATCH_DONE
