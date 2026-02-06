#!/bin/bash
# RunPodから学習結果をダウンロードするスクリプト
# 使用方法: ./scripts/runpod_download_results.sh <runpod-host> <runpod-port>

set -e

HOST=$1
PORT=$2

if [ -z "$HOST" ] || [ -z "$PORT" ]; then
    echo "Usage: $0 <runpod-host> <runpod-port>"
    exit 1
fi

DEST="/home/xnova-1/workspace/sakai/TWIST2/TWIST2/logs"
mkdir -p "$DEST"

echo "=== RunPodから学習結果をダウンロード ==="
echo "Host: $HOST:$PORT"
echo "Destination: $DEST"
echo ""

scp -P $PORT -r -i ~/.ssh/id_ed25519 root@$HOST:/workspace/TWIST2/logs/ "$DEST/"

echo ""
echo "=== ダウンロード完了 ==="
echo "チェックポイント: $DEST"
ls -la "$DEST/"

