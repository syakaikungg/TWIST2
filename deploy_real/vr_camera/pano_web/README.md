# pano_web: 360度カメラ → PICO のブラウザで見回す

Insta360 X5 などをウェブカメラとして PC につなぎ、PC の Chrome から WebRTC で PICO のブラウザへ流し、
WebXR で球の内側に貼って見回す。XRoboToolkit アプリは使わない（腕テレオペとは別の試験用）。

```
Insta360 X5 --USB--> PC の Chrome (sender.html) ==WebRTC 映像==> PICO のブラウザ (viewer.html, WebXR)
                          \__ pano_server.py: ページ配信とつなぐための情報の受け渡しだけ（映像は通らない）
```

## 使い方（Windows）

1. X5 を USB で PC につなぎ、X5 の画面で USB モードを「ウェブカメラ（USBカメラ）」にする。
2. `start_pano_web.bat` をダブルクリック。黒い画面に PICO で開く URL（`https://<PCのIP>:8443/`）が出て、
   ブラウザで送信ページ（`http://localhost:8080/sender.html`）が開く。
   - 初回に Windows のファイアウォールの確認が出たら「許可」（プライベートネットワーク）。
3. 送信ページで カメラ＝Insta360、解像度＝2880x1440 を選んで「カメラを開く」。「2:1 OK」と出れば正しい。
4. PICO のブラウザで黒い画面に出た `https://<PCのIP>:8443/` を開く。
   「この接続ではプライバシーが保護されません」と出たら「詳細設定」→「アクセスする」（自己署名の証明書のため）。
5. 「受信中」になったら「VRで見る」。
   - スティック左右: 正面を回す（PICO が覚える） / B・Y: 正面を戻す / トリガー: 情報表示の切替

PC と PICO は同じ Wi-Fi（同じ LAN）につなぐ。止めるときは黒い画面で Ctrl+C。

## 使い方（Ubuntu・ハード圧縮）

Chrome の代わりに GStreamer（`gst_sender.py`）が送る。NVIDIA の NVENC など、ハードの圧縮を自動で選ぶ
（無ければソフトの x264）。Windows の Chrome（ソフト圧縮）より、2880x1440・30fps を出しやすく遅れも減る見込み。

```
Insta360 X5 --USB--> gst_sender.py (GStreamer, NVENC) ==WebRTC==> PICO のブラウザ (viewer.html, 今までと同じ)
```

1. 取ってくる（`~/pano_web` に置かれる。更新も同じコマンド）:
   ```
   bash <(curl -fsSL https://raw.githubusercontent.com/syakaikungg/TWIST2/claude/project-thread-vu08p3/deploy_real/vr_camera/pano_web/get_pano_web.sh)
   ```
2. 足りない部品を入れる（初回だけ。数分）:
   ```
   sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly gstreamer1.0-nice gstreamer1.0-libav gir1.2-gst-plugins-bad-1.0 python3-gi python3-gst-1.0 v4l-utils
   ```
3. X5 を USB でつなぎ、USB モードを「ウェブカメラ」にして `cd ~/pano_web && ./check_linux.sh`。
   - 「カメラ」に Insta360 が出て、2880x1440（2:1）があるか
   - 「H.264 の圧縮」の自動で選ぶものが `nvh264enc`（NVIDIA）などハードのものか
4. `./start_gst.sh` で送信開始。PICO のブラウザで `https://<この PC の IP>:8443/` を開く（Windows のときと同じ）。
   - カメラなしで試す: `./start_gst.sh --test`
   - 縮めて送る: `./start_gst.sh --scale 1.5`（1920x960）／ ビットレート: `--mbps 20`
   - ソフト圧縮と比べる: `./start_gst.sh --encoder x264enc`
   - X5 が H.264 で出せるなら、PC で圧縮し直さずそのまま送る: `./start_gst.sh --passthrough`（試験的。
     PICO 側からの「キーフレームをください」に応えられないので、乱れたら次の区切りまで戻らない）
5. 遅延を測るときは、送信ページの代わりに `http://localhost:8080/clock.html` の時計をカメラに映す。
   **sender.html（Chrome の送信ページ）は同時に開かない**（どちらが答えるか分からなくなる）。

ログは画面と `logs/gst_sender_*.log`（5 秒ごとに 解像度・fps・Mbps・圧縮の名前・RTT）。
ハードの圧縮でエラーが出たら、次の候補（最後は x264）に自動で切り替え、PICO は数秒でつなぎ直す。
ufw を有効にしている場合は、PICO から届くように許可が要る（例: `sudo ufw allow from 192.168.123.0/24`）。

## ログ

- 黒い画面と `logs/pano_web_*.log` に、接続の経過と 5 秒ごとの送受信の様子
  （解像度・fps・Mbps・コーデック・エンコーダー名・受信待ち時間）が出る。
- 送信側の「OpenH264」はソフトでのエンコード（CPU を使う）。ハードなら別の名前（例: MediaFoundation）。

## 遅延を測る

送信ページの「遅延を測る時計を出す」にチェックし、時計を X5 に映す。表示ページ（PICO、または同じ PC の
別ウィンドウで `https://localhost:8443/`）に映った時刻と画面の時刻を一緒に写真に撮ると、差が遅延。

## うまくいかないとき

| 症状 | 見るところ |
| --- | --- |
| 送信ページに Insta360 が出ない | X5 の USB モードがウェブカメラか。ケーブルを挿し直す |
| 2:1 にならない（例 1920x1080） | 解像度で 2880x1440 を選び直す |
| PICO が「応答を待っています」のまま | 送信ページで「カメラを開く」を押したか |
| PICO でページが開かない | 同じ Wi-Fi か、ファイアウォールの許可、会社の Wi-Fi が端末同士の通信を止めていないか |
| 「VRで見る」が押せない | https で開いているか（http では VR に入れない） |

## ファイル

- `pano_server.py` サーバー（Python 3.8 以上、標準ライブラリのみ。証明書は初回に自動作成。
  `cryptography` か `openssl` が要る。conda / miniforge の Python なら openssl が同梱）
- `sender.html` / `sender.js` PC 側（カメラ → WebRTC）
- `viewer.html` / `viewer.js` PICO 側（WebRTC → 球に貼って WebXR）
- `common.js` 共通の小物
- `three.module.min.js` three.js r160（MIT、`THREE_LICENSE.txt`）
- `start_pano_web.bat` Windows 用の起動
- `gst_sender.py` Ubuntu 用の送信（GStreamer、ハード圧縮。`--list` でカメラと圧縮の一覧）
- `start_gst.sh` / `check_linux.sh` / `get_pano_web.sh` Ubuntu 用の起動・確認・取得
- `clock.html` 遅延を測る時計
