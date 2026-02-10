#!/bin/bash

source ~/miniconda3/bin/activate twist2

SCRIPT_DIR=$(dirname $(realpath $0))
config_path=${SCRIPT_DIR}/deploy_real/robot_control/configs/g1.yaml

# Network interface for robot communication
net=enp5s0

cd deploy_real

echo "=================================================================="
echo "Both Arms Control (10 Motors) - Real Robot"
echo "=================================================================="
echo "Left arm:  motors 15-19 (shoulder_pitch/roll/yaw, elbow, wrist_roll)"
echo "Right arm: motors 22-26 (shoulder_pitch/roll/yaw, elbow, wrist_roll)"
echo "Other 19 motors: HOLD CURRENT POSITION"
echo "Config: ${config_path}"
echo "Network: ${net}"
echo ""
echo "⚠️  CRITICAL WARNING ⚠️"
echo "  - Robot MUST be suspended!"
echo "  - 10 arm motors will follow recorded motion"
echo "  - Other 19 motors will maintain their position"
echo ""
echo "Instructions:"
echo "  1. Put robot in SDK mode (L2 + A on remote)"
echo "  2. Ensure robot is suspended"
echo "  3. Run run_motion_server.sh in another terminal"
echo "  4. Run this script"
echo "  5. Press START on remote controller"
echo "  6. Both arms will follow recorded motion (Swing or Walk)"
echo "  7. Press SELECT on remote to stop safely"
echo "=================================================================="
echo ""

python server_low_level_g1_real_both_arms.py \
    --config ${config_path} \
    --net ${net} \
    --device cuda
