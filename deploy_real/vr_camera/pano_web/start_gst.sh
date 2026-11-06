#!/usr/bin/env bash
# Ubuntu: pano_server.py（ページ配信）と gst_sender.py（GStreamer でハード圧縮して送る）をまとめて起動する。
# 止めるときは Ctrl+C。引数はそのまま gst_sender.py に渡す。
#   ./start_gst.sh                  # X5 を探して 2880x1440 30fps
#   ./start_gst.sh --test           # カメラなしの試験映像
#   ./start_gst.sh --scale 1.5      # 1920x960 に縮めて送る
#   ./start_gst.sh --encoder x264enc  # ソフト圧縮と比べる
set -u
cd "$(dirname "$0")"
PY=/usr/bin/python3            # GStreamer の Python 部品は OS の python3 に入る（conda では動かない）
[ -x "$PY" ] || PY=python3
mkdir -p logs
"$PY" pano_server.py &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT
sleep 1.5
echo "遅延を測る時計: http://localhost:8080/clock.html"
"$PY" gst_sender.py "$@"
