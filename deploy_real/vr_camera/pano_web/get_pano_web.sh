#!/usr/bin/env bash
# Ubuntu: GitHub から pano_web の最新版を取ってくる（既定は ~/pano_web）。何度実行しても上書きするだけ。
#   bash <(curl -fsSL https://raw.githubusercontent.com/syakaikungg/TWIST2/claude/project-thread-vu08p3/deploy_real/vr_camera/pano_web/get_pano_web.sh)
set -eu
BRANCH="${PANO_BRANCH:-claude/project-thread-vu08p3}"
BASE="https://raw.githubusercontent.com/syakaikungg/TWIST2/$BRANCH/deploy_real/vr_camera/pano_web"
DIR="${1:-$HOME/pano_web}"
mkdir -p "$DIR"
cd "$DIR"
for f in pano_server.py gst_sender.py start_gst.sh check_linux.sh get_pano_web.sh viewer.html viewer.js common.js \
         sender.html sender.js clock.html three.module.min.js THREE_LICENSE.txt README.md; do
  curl -fsSL -o "$f.new" "$BASE/$f?nocache=$RANDOM$RANDOM" && mv -f "$f.new" "$f"
  echo "取得: $f"
done
chmod +x start_gst.sh check_linux.sh get_pano_web.sh
echo "置き場所: $DIR"
echo "次: cd $DIR && ./check_linux.sh   （X5 と圧縮の確認）"
echo "    ./start_gst.sh                 （送信開始）"
