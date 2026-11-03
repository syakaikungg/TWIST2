#!/bin/bash
# =============================================================================
# G1 カメラ → PICO 映像送信の起動・停止 (G1 内蔵PC 上で実行する)
#
#   bash cam_to_pico.sh start [multicast|v4l2|test] [g1_cam_to_pico.py の追加引数...]
#   bash cam_to_pico.sh stop
#   bash cam_to_pico.sh status
#   bash cam_to_pico.sh log
#   bash cam_to_pico.sh selftest [multicast|v4l2|test] [秒数] [追加引数...]
#
# 映像の元:
#   multicast (既定) : Unitree の videohub_pc4 が出している H.264 をそのまま使う。サービスは止めない。
#   v4l2             : /dev/video4 を直接開く。先に videohub_pc4 を止める必要がある (README 参照)。
#   test             : テスト模様。カメラなしで PICO までの経路を確かめる。
#
# PICO 側: XRobot アプリ → Remote Vision → カメラ (ZEDMINI か PICO4U) → Listen → 192.168.123.164 → Confirm
# =============================================================================
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
PY="${PY:-/usr/bin/python3}"   # ホストの python3 (gi + NVIDIA の GStreamer)。conda の gmr 環境は使わない
APP="$DIR/g1_cam_to_pico.py"
LOGDIR="$DIR/logs"
mkdir -p "$LOGDIR"
# 行頭が python の本体プロセスだけに当てる (ssh や bash -c の文字列に名前が入っていても誤検出しない)
PAT='^[^ ]*python[0-9.]* (-u )?[^ ]*g1_cam_to_pico\.py'

cmd="${1:-status}"
[ $# -gt 0 ] && shift

case "$cmd" in
  start)
    src="${1:-multicast}"
    [ $# -gt 0 ] && shift
    if pgrep -f "$PAT" > /dev/null; then
      echo "すでに動いています:"
      pgrep -af "$PAT"
      exit 0
    fi
    log="$LOGDIR/cam_$(date +%Y%m%d_%H%M%S).log"
    nohup "$PY" -u "$APP" --source "$src" "$@" > "$log" 2>&1 &
    ln -sf "$log" "$LOGDIR/latest.log"
    sleep 2
    tail -n 5 "$log"
    echo "ログ: $log"
    ;;
  stop)
    if pkill -INT -f "$PAT"; then
      sleep 2
      pgrep -f "$PAT" > /dev/null && pkill -KILL -f "$PAT"
      echo "止めました"
    else
      echo "動いていません"
    fi
    ;;
  status)
    pgrep -af "$PAT" || echo "動いていません"
    ss -ltn 2>/dev/null | grep ":13579" || true
    [ -e "$LOGDIR/latest.log" ] && tail -n 5 "$LOGDIR/latest.log"
    ;;
  log)
    tail -f "$LOGDIR/latest.log"
    ;;
  selftest)
    src="${1:-test}"
    secs="${2:-5}"
    [ $# -gt 0 ] && shift
    [ $# -gt 0 ] && shift
    "$PY" -u "$APP" --source "$src" --selftest "$secs" "$@"
    ;;
  *)
    sed -n 2,18p "$0"
    exit 1
    ;;
esac
