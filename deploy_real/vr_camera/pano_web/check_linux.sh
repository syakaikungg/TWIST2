#!/usr/bin/env bash
# Ubuntu で X5（ウェブカメラモード）が見えるか、ハード圧縮が使えるかを調べる（数秒で終わる）。
# 結果は画面と logs/check_linux_*.txt に出る。
cd "$(dirname "$0")"
mkdir -p logs
OUT="logs/check_linux_$(date +%Y%m%d_%H%M%S).txt"
{
  echo "== OS"
  (lsb_release -ds 2>/dev/null || grep PRETTY /etc/os-release); uname -r
  echo "== この PC の IP（PICO で https://<IP>:8443/ を開く）"
  hostname -I 2>/dev/null
  echo "== GPU"
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null || echo "nvidia-smi なし（NVIDIA のドライバーが無いと NVENC は使えない）"
  lspci 2>/dev/null | grep -iE "vga|3d controller"
  echo "== USB（Insta360 の機器）"
  lsusb 2>/dev/null | grep -iE "2e1a|insta|arashi" || echo "Insta360 らしい USB 機器が見えません（X5 の USB モードを「ウェブカメラ」に、ケーブルを挿し直す）"
  echo "== カメラ（/dev/video*）"
  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --list-devices 2>&1
    for d in /dev/video*; do
      [ -e "$d" ] || continue
      echo "--- $d"
      v4l2-ctl -d "$d" --list-formats-ext 2>/dev/null | grep -E "\[[0-9]+\]|Size|Interval" | head -60
    done
  else
    ls -l /dev/video* 2>/dev/null || echo "/dev/video* が一つもありません"
    echo "（詳しく見るには: sudo apt install v4l-utils）"
  fi
  echo "== GStreamer"
  gst-launch-1.0 --version 2>/dev/null | head -1 || echo "GStreamer が入っていません"
  echo "== ファイアウォール (ufw)"
  systemctl is-active ufw 2>/dev/null || true
  echo "== gst_sender.py --list（GStreamer から見たカメラと圧縮）"
  /usr/bin/python3 gst_sender.py --list 2>&1
} 2>&1 | tee "$OUT"
echo
echo "結果を保存しました: $OUT"
echo "足りないものがあれば:"
echo "  sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \\"
echo "       gstreamer1.0-plugins-ugly gstreamer1.0-nice gstreamer1.0-libav gir1.2-gst-plugins-bad-1.0 python3-gi python3-gst-1.0 v4l-utils"
