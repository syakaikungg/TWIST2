#!/usr/bin/env bash
# Ubuntu: GitHub から pano_web の最新版を取ってくる（既定は ~/pano_web）。何度実行しても上書きするだけ。
#   bash <(curl -fsSL https://raw.githubusercontent.com/syakaikungg/TWIST2/claude/project-thread-vu08p3/deploy_real/vr_camera/pano_web/get_pano_web.sh)
set -eu
BRANCH="${PANO_BRANCH:-claude/project-thread-vu08p3}"
# ブランチ名のままだと GitHub 側の古い控え（数分）が返ることがあるので、最新のコミットに直して取る
SHA="$(curl -fsSL -H 'Accept: application/vnd.github.sha' "https://api.github.com/repos/syakaikungg/TWIST2/commits/$BRANCH" 2>/dev/null || true)"
case "$SHA" in ([0-9a-f]*) REF="$SHA" ;; (*) REF="$BRANCH" ;; esac
echo "取得元: $BRANCH (${REF:0:7})"
BASE="https://raw.githubusercontent.com/syakaikungg/TWIST2/$REF/deploy_real/vr_camera/pano_web"
DIR="${1:-$HOME/pano_web}"
mkdir -p "$DIR"
cd "$DIR"
for f in pano_server.py gst_sender.py start_gst.sh check_linux.sh get_pano_web.sh viewer.html viewer.js common.js \
         sender.html sender.js clock.html three.module.min.js THREE_LICENSE.txt README.md; do
  curl -fsSL --retry 6 --retry-delay 3 --retry-all-errors -o "$f.new" "$BASE/$f" && mv -f "$f.new" "$f"
  echo "取得: $f"
done
chmod +x start_gst.sh check_linux.sh get_pano_web.sh
echo "置き場所: $DIR"
echo "次: cd $DIR && ./check_linux.sh   （X5 と圧縮の確認）"
echo "    ./start_gst.sh                 （送信開始）"
