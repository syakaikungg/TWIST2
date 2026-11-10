# G1 頭カメラ → PICO 映像送信

G1 の頭の RealSense D435i (RGB) の映像を、PICO の XRobot アプリ (XRoboToolkit) の「Remote Vision」に送り、
映像を見ながらテレオペできるようにするためのもの。G1 内蔵PC (Orin NX, 192.168.123.164) 上で動かす。

2026-02 に作った `persistent_vr_server.py` (`scripts/start_vr_camera.sh` から起動していたもの) は
リポジトリに入っておらず G1 からも消えていたため、2026-11 に作り直した。

## ファイル

| ファイル | 内容 |
|---|---|
| `g1_cam_to_pico.py` | 本体。TCP 13579 で PICO を待ち、要求に合わせて H.264 を送る |
| `cam_to_pico.sh` | G1 上での起動・停止・状態・自己テスト |
| `fake_pico_client.py` | PICO の代わりに要求を送って受信を数えるテスト用クライアント |
| `sim_head_cam.py` | シミュレーターの G1 の頭カメラ目線を GPU で描き、`--source sim` に渡す |
| `status_overlay.py` | 腕テレオペの状態 (Redis `teleop_display`) を映像に文字で重ねる。`g1_cam_to_pico.py` が読み込む |
| `battery_dds.py` | G1 のバッテリー残量 (DDS `rt/lf/bmsstate` の soc) を読むだけ。右上に小さく「電池 51%」と出すのに使う |

## 仕組み

1. `g1_cam_to_pico.py` が TCP 13579 で待つ。
2. PICO の XRobot アプリで Remote Vision → カメラ (ZEDMINI か PICO4U) → Listen → `192.168.123.164` → Confirm。
   アプリが `OPEN_CAMERA` (幅・高さ・fps・ビットレート・送り先 IP/ポート) を送ってくる。
3. G1 から PICO の指定ポートに TCP 接続し、H.264 (Annex-B) を 1 フレームずつ `[4byte 長さ][データ]` で送る。
   幅 2000 以上 (ZEDMINI=2560x720 など) の要求は左右並びのステレオなので、同じ映像を左右に並べる。
4. エンコードは Jetson のハードウェアエンコーダー (`nvv4l2h264enc`)、縮小・切り取りは `nvvidconv`。

プロトコルは `deploy_real/data_utils/webcam_to_vr.py` と同じ。

## G1 を起動し直したあと

G1 を再起動すると Unitree の映像サービス `video_hub_pc4` が戻ってカメラを使い、USB の省電力の設定も元に戻る。
`bash cam_to_pico.sh boot` で、`video_hub_pc4` を止めて実機カメラ (960x540) で PICO を待つ (sudo 不要)。
腕テレオペの `~/nextech/arm_ik/after_boot.sh` (docker start などとまとめて本人が 1 回実行) から呼ぶ。

カメラの USB の省電力を切る設定 (sudo が要る、再起動で戻る)。2026-11-04 にカメラの USB リセットが 1 日 5 回あり、
毎回直前に `Disable of device-initiated U1/U2 failed` が出ていたため (効果はまだ確認中):

```bash
echo on | sudo tee /sys/bus/usb/devices/2-2.3/power/control /sys/bus/usb/devices/2-2/power/control
```

## カメラの解像度

D435i の RGB (`YUYV`) は 320x180〜960x540 (60fps まで)、1280x720・1920x1080 (30fps まで) を出せる。

- 既定は **960x540** (16:9 なので切らずにカメラの全体 約69°×42° が入る、片目 1280x720 へは 1.33 倍)。
  2026-11-04 の G1 では 1280x720 で 1 枚も来ないことがあり (USB が不安定)、960x540 は安定して出た。
- 640x480 などの 4:3 は、カメラ自体が左右を切った絵を出したうえに、片目の 16:9 に合わせて上下も切るので、
  視野が約54°×32° まで狭くなり、2 倍に拡大するので粗くなる。使わない。
- 切り取りが要るときは、一度 GPU 側 (NVMM) に移してから切る (普通のメモリのまま nvvidconv で切ると CPU が 1 コアの 55〜68% になった)。
- 拡大の補間は bilinear (`--interp`、nvvidconv の既定の Nearest はブロック状に粗い)。
- デバイスは `--device auto` (既定) で、PICO がつながるたびに RealSense で YUYV と指定の大きさを出すノードを探す。
  USB がリセットされると `/dev/video*` の番号が変わることがあるため。

## 使い方 (G1 上)

ホストの `/usr/bin/python3` を使う (gi と NVIDIA の GStreamer プラグインがあるため)。conda の gmr 環境では動かない。

```bash
cd ~/nextech/cam_to_pico
bash cam_to_pico.sh selftest test 5          # ネットワークなしでエンコードだけ試す
bash cam_to_pico.sh selftest v4l2 5          # 頭のカメラで 5 秒エンコードだけ試す (videohub_pc4 を止めてから)
bash cam_to_pico.sh start v4l2 --listen 192.168.123.164:13579   # PICO を待つ (960x540、--cam-size で変更)
bash cam_to_pico.sh status
bash cam_to_pico.sh stop
```

PICO なしで G1 上だけで試す:

```bash
bash cam_to_pico.sh start test
/usr/bin/python3 fake_pico_client.py --server 127.0.0.1:13579 --size 2560x720 --seconds 5
bash cam_to_pico.sh stop
```

## 映像の元

| `--source` | 内容 | 注意 |
|---|---|---|
| `multicast` | Unitree の `videohub_pc4` が出している H.264 RTP (230.1.1.1:1720) を受けてデコードし直す | 2026-11-03 の G1 では 230.1.1.1:1720 に何も流れておらず使えなかった |
| `v4l2` (既定) | D435i の RGB (YUYV、既定 960x540 30fps) を直接開く。デバイスは自動で探す | `videohub_pc4` が `/dev/video4` を使っているので、先に止める必要がある |
| `test` | テスト模様 | カメラなしで PICO までの経路確認用 |

`v4l2` を使うときの `videohub_pc4` の停止 (2026-02 の `scripts/start_vr_camera.sh` と同じ方法。システムのサービスなので本人の確認後に):

```bash
/unitree/sbin/mscli stopservice video_hub_pc4
```

止めると `mscli listservice` で `status:1 enable:0` になり、G1 を再起動しても自動では戻らない可能性がある
(2026-11-03 確認)。元に戻すとき:

```bash
/unitree/sbin/mscli startservice video_hub_pc4
/unitree/sbin/mscli listservice   # video_hub_pc4 が status:0 enable:1 に戻ったか確認
```

sudo は不要 (unitree ユーザーで実行できた)。`/dev/video4` も unitree ユーザーで開ける。

## シミュレーターの頭カメラ目線を送る (sim2sim)

腕のシミュレーター (`~/nextech/arm_ik/arm_sim.py`) が Redis の `sim_qpos` (JSON、`joints` に関節名→角度) を
50Hz で書いている間、その関節角を読んで (書き込みはしない) G1 の頭カメラ目線を描き、PICO に送る。

- カメラは Unitree の URDF の `d435_joint` と同じ位置・向き (torso_link から (0.0576, 0.0175, 0.4299)、下向き 0.831 rad)、縦の画角 42°。
- 頭の殻のメッシュ (`head_link` / `logo_link`) は視界をふさぐので描かない。影と反射も切って軽くしている。
- 右手のドリル (腕シムと同じ `deploy_real/arm_ik/sim_drill.py`、見た目だけ) も描き、`sim_qpos` の `drill` (`on` / `angle`) で回す。
  `sim_drill.py` はこのフォルダ → `../arm_ik` → `~/nextech/arm_ik` の順に探し (`--drill-dir` で指定も可)、見つからなければドリルなしで描く。`--no-drill` で消せる。
- 描画は gmr 環境の python + MuJoCo の EGL (GPU)。エンコードは実機カメラと同じハードウェア H.264。

```bash
bash cam_to_pico.sh simcam bench 60     # 描画の重さを測る (1 枚あたり ms)
bash cam_to_pico.sh simcam png          # 1 枚描いて /tmp/sim_head.png に保存して見え方を確認
bash cam_to_pico.sh simcam start        # 描画を 127.0.0.1:5601 で待たせる
bash cam_to_pico.sh start sim --listen 192.168.123.164:13579
# PICO: Remote Vision → ZEDMINI → Listen → 192.168.123.164 → Confirm (実機のときと同じ)
bash cam_to_pico.sh stop; bash cam_to_pico.sh simcam stop
```

実機カメラ (`start v4l2`) とシム (`start sim`) は、送信サーバーを起動するときに選ぶ (同時には動かせない)。
描画は送信サーバーがつないでいる間 (PICO が見ている間) だけ行う。

2026-11-03 の G1 での実測: 描画は 1 枚 平均 12.4 ms (1280x720、影・反射なし)、GPU (GR3D) 最大 22%。
PICO に 30fps で約 9 分配信して捨て 0、関節角の遅れ 0.01 秒。配信中も腕の IK ループ (平均 2.2 ms / 最大 9.7 ms)・
ブリッジ周期 (最大 23 ms) に影響は見られなかった。

## 腕テレオペの状態を映像に重ねる

腕の IK (`deploy_real/arm_ik/arm_ik_dryrun.py`) が Redis の `teleop_display` に 50 Hz で書く状態
(`{"t", "level", "ja", "en", ...}`、形式は `deploy_real/arm_ik/README.md`) を 1 秒に 10 回読んで (GET のみ)、
映像の上の中央に文字で出す。実機カメラでもシムでも同じ。左右並びのときは両目の同じ位置に出る。

| level | 文字の色 | 例 |
|---|---|---|
| `stop` | 赤 | 非常停止（PCから再起動が必要） |
| `warn` | 橙 | 通信待ち（データが戻れば再開） |
| `wait` | 黄 | 準備中 あと2.1秒（そのあと握れば動く） |
| `idle` | 灰 | 一時停止中（右Aで開始） |
| `ready` | 水色 | 握れば動きます |
| `ok` | 緑 | 追従中 |

- `t` が 0.5 秒より古いと「IK停止（状態が届いていません） N秒」(赤)、キーが無いと「IK未起動」(灰)、
  Redis につながらないと「状態を読めません（Redis）」(橙) と出す。
- 日本語フォントが G1 に無ければ (`fc-list :lang=ja` が空)、自動で英語 (`en`) を出す。`--overlay-lang ja|en` で固定できる。
- GStreamer の `textoverlay` (pango) が無い、または入れるとパイプラインが作れないときは、文字なしで映像だけ送る (ログに「注意」)。
- 右上に小さく G1 のバッテリー残量 (「電池 51%」、11/10〜)。送信プログラムが DDS の `rt/lf/bmsstate` を eth0 で読むだけで、テレオペが動いていなくても出る。20% 以下で橙、10% 以下で赤、15秒届かなければ「電池 ?」。unitree_sdk2py が無い・DDS を始められないときは出さない (映像は続く)。`--no-battery`、`--battery-xpad 0.10` (右端からの距離、片目の幅に対する割合)、`--battery-font-size`、テスト用 `--battery-fake 51`。
- 配信中のログの「送信 …」の行の最後に、いま出している表示 (`表示 [ready] 握れば動きます` など) が付く。
- 主なオプション: `--no-overlay` (重ねない)、`--overlay-pos top|bottom`、`--overlay-font-size 36` (既定は片目の高さ 720 で 30)、
  `--overlay-key`、`--redis-host` / `--redis-port` (既定 127.0.0.1:6379)。

## 腕テレオペと同時に使うとき

- 腕のブリッジは `rt/lowstate` が 0.2 秒、Redis の `arm_ik_q` が 0.3 秒古くなると非常停止する。
  映像を起動する前後でブリッジのループ時間を確認する。
- 実機テスト (`--send`) 中に PICO アプリの画面切替はしない。切替でコントローラーの送信が途切れないかは要確認。
- ポートは 13579 だけ使う (PC Service の 63901/34074/60061、Redis の 6379 とは重ならない)。

## よく使うオプション

- `--bitrate 4000000` : PICO の要求値を無視してビットレートを固定 (bps)
- `--aspect stretch` : 片目の縦横比に合わせて切り取らず引き伸ばす (既定は中央を切り取り)
- `--mono` : 幅が大きくても左右に並べない
- `--no-hevc` : PICO が H.265 を要求しても H.264 で送る
- `--encoder x264` : Jetson 以外でのテスト用 (ソフトエンコード)
