#!/bin/bash
# =============================================================================
# G1 カメラ → PICO VR ストリーミング起動スクリプト
#
# 使い方:
#   bash scripts/start_vr_camera.sh
#
# 前提:
#   - G1 ロボットが 192.168.123.164 で起動済み
#   - PICO VR が同じネットワークに接続済み
#   - SSH 鍵認証設定済み (unitree@192.168.123.164)
#
# PICO 側の操作:
#   1. XRobot APP を開く
#   2. PICO4U を選択 → Listen → 192.168.123.164 → Confirm
#   3. 何度 Listen し直しても自動再接続される
#
# 停止:
#   Ctrl+C または別ターミナルで:
#   ssh unitree@192.168.123.164 "pkill -f persistent_vr_server"
# =============================================================================

ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"
SSH="ssh ${ROBOT_USER}@${ROBOT_IP}"

echo "=========================================="
echo "G1 Camera → PICO VR Streaming"
echo "=========================================="
echo "Robot: ${ROBOT_IP}"
echo ""

# ロボット接続確認
echo "[1/4] ロボット接続確認..."
if ! ping -c 1 -W 2 ${ROBOT_IP} > /dev/null 2>&1; then
    echo "Error: ロボット (${ROBOT_IP}) に接続できません"
    exit 1
fi
echo "  OK"

# videohub_pc4 を停止 (RGB カメラを解放)
echo "[2/4] videohub_pc4 を停止..."
${SSH} "/unitree/sbin/mscli stopservice video_hub_pc4 2>/dev/null || true" || true
${SSH} "kill \$(pgrep -f videohub_pc4) 2>/dev/null || true" || true
sleep 3
# 確認
if ${SSH} "fuser /dev/video4 2>/dev/null" 2>/dev/null | grep -q .; then
    echo "  WARNING: /dev/video4 はまだ占有されています"
else
    echo "  OK (RGB camera free)"
fi

# 既存の VR サーバーを停止
echo "[3/4] 既存サーバーを停止..."
${SSH} "pkill -f persistent_vr_server 2>/dev/null || true" || true
${SSH} "fuser -k 13579/tcp 2>/dev/null; sleep 1" || true
echo "  OK"

# サーバー起動
echo "[4/4] VR カメラサーバー起動..."
${SSH} "nohup python3 -u ~/persistent_vr_server.py > /tmp/persistent.log 2>&1 &" || true

sleep 8

# 起動確認
echo ""
echo "--- サーバーログ ---"
${SSH} "head -15 /tmp/persistent.log 2>/dev/null" || true

echo ""
echo "=========================================="
echo "サーバー起動完了!"
echo ""
echo "PICO VR で以下を操作:"
echo "  1. XRobot APP → Remote Vision"
echo "  2. PICO4U を選択"
echo "  3. Listen → IP: ${ROBOT_IP} → Confirm"
echo ""
echo "ログ監視: ssh ${ROBOT_USER}@${ROBOT_IP} 'tail -f /tmp/persistent.log'"
echo "停止:     ssh ${ROBOT_USER}@${ROBOT_IP} 'pkill -f persistent_vr_server'"
echo "=========================================="
