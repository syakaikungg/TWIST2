#!/bin/bash
# TWIST2 23DOF G1 Sim2Real with RL Policy
#
# Usage:
#   bash sim2real_23dof.sh [policy_path]

source ~/miniconda3/bin/activate twist2
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

SCRIPT_DIR=$(dirname $(realpath $0))
POLICY_PATH=${1:-"${SCRIPT_DIR}/assets/ckpts/g1_23dof_trained.onnx"}
CONFIG_PATH="${SCRIPT_DIR}/deploy_real/robot_control/configs/g1.yaml"

# ネットワークインターフェース (ロボット接続用)
net=enp5s0

if [ ! -f "$POLICY_PATH" ]; then
    echo "Error: Policy file not found: $POLICY_PATH"
    echo ""
    echo "Available ONNX files:"
    find ${SCRIPT_DIR}/assets/ckpts -name "*.onnx" 2>/dev/null
    exit 1
fi

echo "=================================================================="
echo "TWIST2 23DOF Sim2Real (RL Policy)"
echo "=================================================================="
echo ""
echo "Policy:  ${POLICY_PATH}"
echo "Config:  ${CONFIG_PATH}"
echo "Network: ${net}"
echo ""
echo "制御方式: 学習済み23DOFポリシーによるPD制御"
echo "  - RL policy が 1162次元観測 → 23アクション を推論"
echo "  - 23関節をPD制御, 残り6モーターはデフォルト保持"
echo ""
echo "制御対象 (23モーター):"
echo "  左脚:  [0-5]   hip_pitch/roll/yaw, knee, ankle_pitch/roll"
echo "  右脚:  [6-11]  hip_pitch/roll/yaw, knee, ankle_pitch/roll"
echo "  腰:    [12]    waist_yaw"
echo "  左腕:  [15-19] shoulder_p/r/y, elbow, wrist_roll"
echo "  右腕:  [22-26] shoulder_p/r/y, elbow, wrist_roll"
echo ""
echo "⚠️  安全確認:"
echo "  - ロボットが吊り下げられていること"
echo "  - 全23関節がポリシーで制御されます"
echo "  - 脚も動きます！周囲に注意"
echo "  - [Select]ボタンで安全に終了"
echo ""
echo "操作手順:"
echo "  1. Redis起動済み確認"
echo "  2. テレオペ起動 (teleop.sh in gmr env)"
echo "  3. リモコンで SDK mode (L2+A)"
echo "  4. [START] → デフォルト姿勢へ移動"
echo "  5. [A] → ポリシー制御開始"
echo "=================================================================="
echo ""

cd deploy_real

python server_low_level_g1_real_23dof.py \
    --policy "${POLICY_PATH}" \
    --config "${CONFIG_PATH}" \
    --net ${net} \
    --device cuda
