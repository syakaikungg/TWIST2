#!/bin/bash
# =============================================================================
# G1 カメラ → PICO 映像送信の起動・停止 (G1 内蔵PC 上で実行する)
#
#   bash cam_to_pico.sh start [v4l2|test|sim|multicast] [g1_cam_to_pico.py の追加引数...]
#   bash cam_to_pico.sh stop
#   bash cam_to_pico.sh status
#   bash cam_to_pico.sh log
#   bash cam_to_pico.sh selftest [multicast|v4l2|test] [秒数] [追加引数...]
#   bash cam_to_pico.sh boot [追加引数...]   # G1 を起動し直したあとの準備 (videohub_pc4 を止めて実機カメラで待ち受け)
#
# シミュレーターの頭カメラ目線を送るとき (腕シムが Redis sim_qpos を書いている間):
#   bash cam_to_pico.sh simcam start      # 描画 (gmr 環境, GPU/EGL)。127.0.0.1:5601 で待つ
#   bash cam_to_pico.sh start sim --listen 192.168.123.164:13579
#   bash cam_to_pico.sh simcam bench 60   # 描画の重さだけ測る
#   bash cam_to_pico.sh simcam png        # 1 枚描いて /tmp/sim_head.png に保存
#
# 映像の元:
#   v4l2 (既定)      : /dev/video4 を直接開く。先に videohub_pc4 を止める必要がある (README 参照)。
#   multicast        : videohub_pc4 の H.264 マルチキャストを受ける (2026-11 の G1 では流れておらず使えない)。
#   test             : テスト模様。カメラなしで PICO までの経路を確かめる。
#
# 腕テレオペの状態 (Redis teleop_display) は映像の上に文字で重ねる (読むだけ)。重ねないときは --no-overlay。
#
# PICO 側: XRobot アプリ → Remote Vision → カメラ (ZEDMINI か PICO4U) → Listen → 192.168.123.164 → Confirm
# =============================================================================
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
PY="${PY:-/usr/bin/python3}"   # ホストの python3 (gi + NVIDIA の GStreamer)。conda の gmr 環境は使わない
APP="$DIR/g1_cam_to_pico.py"
LOGDIR="$DIR/logs"
mkdir -p "$LOGDIR"
# シムの頭カメラ描画 (sim_head_cam.py) は gmr 環境の python (mujoco, redis) で動かす
GMR_PY="${GMR_PY:-$HOME/miniforge3/envs/gmr/bin/python}"
SIM_MODEL="${SIM_MODEL:-$HOME/nextech/TWIST2/assets/g1/g1_sim2sim_29dof.xml}"
SIMPAT='^[^ ]*python[0-9.]* (-u )?[^ ]*sim_head_cam\.py'
# 行頭が python の本体プロセスだけに当てる (ssh や bash -c の文字列に名前が入っていても誤検出しない)
PAT='^[^ ]*python[0-9.]* (-u )?[^ ]*g1_cam_to_pico\.py'

cmd="${1:-status}"
[ $# -gt 0 ] && shift

case "$cmd" in
  start)
    src="${1:-v4l2}"
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
  boot)
    # G1 を起動し直したあとの準備 (sudo 不要)。G1 を再起動すると Unitree の映像サービス video_hub_pc4 が
    # 戻ってカメラを使ってしまうので止め、実機カメラ (960x540) で PICO を待つ。
    # USB の省電力を切る設定は sudo が要るので、起動コマンドの側で先に行う (README 参照)。
    MSCLI=/unitree/sbin/mscli
    if [ -x "$MSCLI" ]; then
      echo "video_hub_pc4 を止めます"
      "$MSCLI" stopservice video_hub_pc4 || true
      sleep 2
      "$MSCLI" listservice 2>/dev/null | grep -E "video_hub_pc4[^_]|video_hub_pc4$" || true
    else
      echo "注意: $MSCLI がありません (video_hub_pc4 は止めていません)"
    fi
    for d in /sys/bus/usb/devices/2-2.3 /sys/bus/usb/devices/2-2; do
      [ -e "$d/power/control" ] && echo "USB $(basename $d) $(cat $d/product 2>/dev/null): power/control=$(cat $d/power/control)"
    done
    if pgrep -f "$PAT" > /dev/null; then
      "$0" stop
    fi
    "$0" start v4l2 --cam-size 960x540 --listen 192.168.123.164:13579 "$@"
    ;;
  simcam)
    # シミュレーターの頭カメラ目線の描画 (start sim と組み合わせる)
    sub="${1:-status}"
    [ $# -gt 0 ] && shift
    case "$sub" in
      start)
        if pgrep -f "$SIMPAT" > /dev/null; then
          echo "すでに動いています:"; pgrep -af "$SIMPAT"; exit 0
        fi
        log="$LOGDIR/simcam_$(date +%Y%m%d_%H%M%S).log"
        MUJOCO_GL=egl nohup "$GMR_PY" -u "$DIR/sim_head_cam.py" --model "$SIM_MODEL" "$@" > "$log" 2>&1 &
        ln -sf "$log" "$LOGDIR/latest_simcam.log"
        sleep 3
        tail -n 5 "$log"
        echo "ログ: $log"
        ;;
      stop)
        if pkill -INT -f "$SIMPAT"; then
          sleep 2
          pgrep -f "$SIMPAT" > /dev/null && pkill -KILL -f "$SIMPAT"
          echo "止めました"
        else
          echo "動いていません"
        fi
        ;;
      status)
        pgrep -af "$SIMPAT" || echo "動いていません"
        [ -e "$LOGDIR/latest_simcam.log" ] && tail -n 5 "$LOGDIR/latest_simcam.log"
        ;;
      bench)
        MUJOCO_GL=egl "$GMR_PY" -u "$DIR/sim_head_cam.py" --model "$SIM_MODEL" --bench "${1:-60}" "${@:2}"
        ;;
      png)
        MUJOCO_GL=egl "$GMR_PY" -u "$DIR/sim_head_cam.py" --model "$SIM_MODEL" --png "${1:-/tmp/sim_head.png}" "${@:2}"
        ;;
      *)
        echo "simcam start|stop|status|bench [枚数]|png [保存先]"; exit 1
        ;;
    esac
    ;;
  *)
    sed -n 2,26p "$0"
    exit 1
    ;;
esac
