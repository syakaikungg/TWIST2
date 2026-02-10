#!/bin/bash
# RunPod データ転送スクリプト
# 使用方法: ./scripts/runpod_transfer.sh <runpod-host> <runpod-port>
# 例: ./scripts/runpod_transfer.sh 205.196.xx.xx 12345

set -e

HOST=$1
PORT=$2

if [ -z "$HOST" ] || [ -z "$PORT" ]; then
    echo "Usage: $0 <runpod-host> <runpod-port>"
    echo "Example: $0 205.196.xx.xx 12345"
    exit 1
fi

echo "=== RunPod データ転送 ==="
echo "Host: $HOST"
echo "Port: $PORT"
echo ""

# 転送
echo "[1/3] TWIST2コード転送 (37MB)..."
scp -P $PORT -i ~/.ssh/id_ed25519 /tmp/twist2_code.tar.gz root@$HOST:/workspace/

echo "[2/3] 23DOFモーションデータ転送 (2.4GB)..."
scp -P $PORT -i ~/.ssh/id_ed25519 /tmp/twist2_motion_23dof.tar.gz root@$HOST:/workspace/

echo "[3/3] IsaacGym転送 (193MB)..."
scp -P $PORT -i ~/.ssh/id_ed25519 /tmp/isaacgym.tar.gz root@$HOST:/workspace/

echo ""
echo "=== 転送完了 ==="
echo ""
echo "次のステップ: RunPodにSSH接続してセットアップスクリプトを実行"
echo "  ssh root@$HOST -p $PORT -i ~/.ssh/id_ed25519"
echo "  bash /workspace/TWIST2/scripts/runpod_setup.sh"


