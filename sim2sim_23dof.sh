#!/bin/bash
# TWIST2 23DOF G1 Sim2Sim Script
#
# 使用方法:
#   ./sim2sim_23dof.sh [policy_path]
#
# 例:
#   ./sim2sim_23dof.sh assets/ckpts/g1_23dof_policy.onnx

set -e

# Activate conda environment
source ~/miniconda3/bin/activate twist2
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

cd "$(dirname "$0")"

POLICY_PATH=${1:-"assets/ckpts/g1_23dof_trained.onnx"}
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

# Initialize Redis with default standing pose (23DOF)
redis-server --daemonize yes 2>/dev/null || true
python -c "
import redis, json, numpy as np
r = redis.Redis()
obs = np.array([0,0,0.793,0,0,0, -0.2,0,0,0.4,-0.2,0, -0.2,0,0,0.4,-0.2,0, 0, 0,0.4,0,1.2,0, 0,-0.4,0,1.2,0])
r.set('action_body_unitree_g1_23dof', json.dumps(obs.tolist()))
print('Default standing pose sent to Redis (29 dims)')
"

# カメラストリーミングオプション
# --stream_camera を追加すると ZMQ でカメラ映像を配信
# 受信: python deploy_real/data_utils/vision_client.py --server 127.0.0.1 --port 5555
STREAM_OPTS=""
if [ "${STREAM_CAMERA:-0}" = "1" ]; then
    STREAM_OPTS="--stream_camera --stream_port ${STREAM_PORT:-5555} --stream_fps ${STREAM_FPS:-30}"
    echo "Camera streaming enabled on port ${STREAM_PORT:-5555}"
fi

cd deploy_real
python server_low_level_g1_23dof_sim.py \
    --xml "../$XML_PATH" \
    --policy "../$POLICY_PATH" \
    --device cpu \
    --policy_frequency 50 \
    $STREAM_OPTS


