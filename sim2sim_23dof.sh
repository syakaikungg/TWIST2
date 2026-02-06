#!/bin/bash
# TWIST2 23DOF G1 Sim2Sim Script
#
# 使用方法:
#   ./sim2sim_23dof.sh [policy_path]
#
# 例:
#   ./sim2sim_23dof.sh assets/ckpts/g1_23dof_policy.onnx

set -e

cd "$(dirname "$0")"

POLICY_PATH=${1:-"assets/ckpts/g1_23dof_policy.onnx"}
XML_PATH="assets/g1/g1_sim2sim_23dof.xml"

if [ ! -f "$POLICY_PATH" ]; then
    echo "Error: Policy file not found: $POLICY_PATH"
    echo ""
    echo "Please train a 23DOF policy first using:"
    echo "  ./train_23dof.sh"
    echo ""
    echo "Then export to ONNX using:"
    echo "  cd legged_gym/legged_gym/scripts"
    echo "  python save_onnx.py --task g1_23dof_future --run_name <your_run_name>"
    exit 1
fi

if [ ! -f "$XML_PATH" ]; then
    echo "Error: MuJoCo XML file not found: $XML_PATH"
    exit 1
fi

echo "=========================================="
echo "TWIST2 23DOF G1 Sim2Sim"
echo "Policy: $POLICY_PATH"
echo "XML: $XML_PATH"
echo "=========================================="

cd deploy_real
python server_low_level_g1_23dof_sim.py \
    --xml "../$XML_PATH" \
    --policy "../$POLICY_PATH" \
    --device cuda \
    --measure_fps

