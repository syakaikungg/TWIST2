#!/bin/bash

source ~/miniconda3/bin/activate twist2

SCRIPT_DIR=$(dirname $(realpath $0))
config_path=${SCRIPT_DIR}/deploy_real/robot_control/configs/g1.yaml

# ネットワークインターフェース
net=enp5s0

cd deploy_real

echo "=================================================================="
echo "G1 23-DOF Sim2Real リターゲティング"
echo "=================================================================="
echo "Redis (action_body_unitree_g1_with_hands) → 23モーター直接マッピング"
echo ""
echo "制御対象 (23モーター):"
echo "  左脚:  motors 0-5   (hip_pitch/roll/yaw, knee, ankle_pitch/roll)"
echo "  右脚:  motors 6-11  (hip_pitch/roll/yaw, knee, ankle_pitch/roll)"
echo "  腰:    motor 12     (yaw のみ)"
echo "  左腕:  motors 15-19 (shoulder_p/r/y, elbow, wrist_roll)"
echo "  右腕:  motors 22-26 (shoulder_p/r/y, elbow, wrist_roll)"
echo ""
echo "スキップ (23-DOF非存在): motors 13,14,20,21,27,28"
echo ""
echo "Config: ${config_path}"
echo "Network: ${net}"
echo ""
echo "⚠️  安全確認:"
echo "  - ロボットが吊るされていること"
echo "  - 全23関節がテレオペに追従します"
echo "  - 脚も動きます！周囲に注意"
echo "  - Selectボタンで安全に終了"
echo ""
echo "前提:"
echo "  1. Redis起動済み (sudo systemctl start redis-server)"
echo "  2. テレオペ起動済み (teleop.sh in gmr env)"
echo "  3. SDK mode (L2+A on remote)"
echo "  4. ロボット吊り下げ済み"
echo "=================================================================="
echo ""

python server_low_level_g1_real_fullbody_23dof.py \
    --config ${config_path} \
    --net ${net} \
    --device cuda
