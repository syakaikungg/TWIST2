# arm_ik: G1 の腕テレオペ（PICO → IK → rt/arm_sdk）

NexTech Week 2026 秋の展示用。G1 の内蔵PC（`~/nextech/arm_ik/`）で動かす。脚は純正の制御のまま、両腕と腰を `rt/arm_sdk` で上書きする。

| ファイル | 役割 |
|---|---|
| `arm_ik_dryrun.py` | PICO（xrobotoolkit_sdk）から手の位置を読み、mink で腕14関節の IK を解いて Redis `arm_ik_q` / `drill_cmd` / `teleop_display` に書く。ロボットには何も送らない |
| `arm_sdk_bridge.py` | Redis `arm_ik_q` → `rt/arm_sdk` の LowCmd。既定はログのみで、`--send` のときだけ送信する。`--sim` は `arm_sim.py` と Redis でつなぐ |
| `arm_sim.py` | 骨盤と脚を固定した MuJoCo のシミュレーター（腕と腰を arm_sdk と同じ形で動かす）。右手を外した手首に見た目だけのドリルが付く |
| `sim_drill.py` | シミュレーターのドリルの形と回し方（ビューアーからも使う） |
| `sim_viewer.py` | ノートPC（WSL）のビューアー。G1の `stream_sim.py` を SSH で動かして関節角を受け取り、MuJoCo の窓に描く。`sim_drill.py` と同じフォルダ（`~/nextech_viewer/`）に置く |
| `sim_run.sh` / `sim_live_bg.sh` | シミュレーターでの自動テスト / PICO で操作する live |
| `run_live.sh` | 実機。`run_live.sh dry` はロボットに送らずに動かす（既定60秒）、`run_live.sh send` は `--send` で送る（既定180秒、本人の確認後にだけ使う）。上限は既定0.5 rad/s、手先0.6 m/s |
| `stop.sh` | 非常停止（Redis `teleop_stop`） |
| `render_front.py` | 記録（`stream_sim.py` の出力）か arm_sim の CSV から、前などから見た動画を作る（G1 では EGL＋imageio-ffmpeg） |
| `check_tool_gap.py` | IK か arm_sim のログから、ドリル（か手）が体と反対の腕にどこまで近づいたかを出す |
| そのほか | 解析やテスト用の小さなスクリプト |

## 操作（PICO）

- 右A：開始／一時停止。開始すると準備（重みを3秒で上げる）が始まる。準備が終わってからグリップを握ると、手についてくる
- 右グリップ：握っている間だけ追従する（デッドマン）
- 右トリガー：ドリル ON（追従中だけ）
- 右スティックの押し込み：最初の姿勢に戻って一時停止（`--return-button`。B は Remote Vision の片目/両目の切り替えなので、何もしない）
- 左X/Y：非常停止（PC から起動し直すまで止まったまま）

## 自動で止まる条件

- 追従中に PICO のデータが0.3秒途切れたら、非常停止
- 追従していないとき（グリップを離している）の途切れは、`--stale-hold`（2秒）までは腕をその場で保って待つだけ。データが戻れば自動で元に戻る。2秒を超えたら非常停止
- ブリッジ側：IK の指令が0.3秒途切れた、指令と実測のずれが0.3 rad を0.5秒、腕か腰のモーターが60℃超、rt/lowstate が0.2秒途切れた、`stop.sh`

## 表示用の状態（Redis `teleop_display`）

IK が毎ループ（50 Hz）書く JSON。PICO の映像に重ねる表示に使う。`t` が0.5秒より古ければ、IK が止まっている。

```json
{"t": 1762176000.0, "level": "ready", "ja": "握れば動きます", "en": "READY - hold grip",
 "ik": "ENABLED", "bridge": "TRACK", "weight": 1.0, "reason": "", "drill": 0}
```

`level` の値は次のとおり。

| level | 意味 | ja の例 |
|---|---|---|
| `stop` | 非常停止（IK か、ブリッジの見張り） | 非常停止（PCから再起動が必要）、非常停止（ブリッジ・PCから再起動が必要） |
| `warn` | PICO のデータ待ち、またはブリッジが応答していない | 通信待ち（データが戻れば再開） |
| `wait` | 準備中、または最初の姿勢に戻っている途中 | 準備中 あと2.1秒（そのあと握れば動く） |
| `idle` | 一時停止中 | 一時停止中（右Aで開始） |
| `ready` | 握れば動く | 握れば動きます |
| `ok` | 追従中 | 追従中 |

ドリルが ON のときは、文の後ろに「／ ドリルON」が付く。

## 腰

arm_sdk を有効にすると、腰（12〜14）もこちらの担当になる（公式の arm7 の例と同じ組み合わせ）。既定は TWIST2 の腰ゲイン（kp 150 / kd 4）と、roll・pitch の積分補正（`--waist-correct`）。シミュレーターでの腰の傾きは、最大0.03〜0.04 rad。

## シミュレーターのドリル

`arm_sim.py` は右手を外して、手首にドリルを付ける（重さはないので、重さは今までどおり `--payload-kg`）。IK の Redis `drill_cmd` が ON の間（追従中に右トリガー）、先が毎秒3回転で回り、モーターの2本の輪が緑になる。実際の回転数ではなく、映像で回っているのが分かる速さにしている（`--drill-rps`）。`--no-drill` で外せる。

形は仮のもの（アダプターが未完成のため）。実機と同じく右手は外し、手首の取り付け面（手が付いていた所）にアダプター、モーター、チャック（黄色の線）、ビット（平たい刃）が、前腕の向き（`right_wrist_yaw_link` の x）に並ぶ。取り付け面から先端まで約24cm。手のメッシュには重さがないので、外しても物理は変わらない。IK が動かす点は今までどおり `right_rubber_hand`（取り付け面の位置）。

状態は `sim_qpos` の `"drill": {"on": 0/1, "angle": rad, "rps": 回転/秒}` に入る。`sim_qpos` から絵を描くビューアーは、次の3行で同じドリルを出せる。

```python
import sim_drill
sim_drill.add_drill(spec)            # MjSpec.from_file(...) のあと、compile() の前
view = sim_drill.DrillView(m)        # compile したモデル
view.apply(m, msg.get("drill"))      # 毎フレーム、mj_kinematics / mj_forward の前
```

## ドリルと手の当たり判定

ドリルの当たり判定は仮の形（`sim_drill.COLL_CAPSULES`：取り付け面から、モーター・チャック・ビットを覆うカプセル3つ、約24.5cm）。G1 のモデルの手には当たり判定がないので、手には箱を足す（`add_hand_collision`）。

- IK（`--tool drill`、既定）：右腕＋ドリル、左腕＋左手の箱を、体（骨盤・胴・股関節・膝）から離し、左右の前腕どうしも離す（`--collision-margin`、既定4cm）。`--tool hand` はドリルの代わりに両手の箱で判定する（ドリルを付けていない実機用）
- 握っていない側の腕は、IK の計算の中でも止めておく。止めた腕を計算のあとで戻すと、反対の腕から離れる動きが消えてしまうため
- `arm_sim.py`：同じカプセルと左手の箱で、体や反対の腕との接触を数える（立ち姿勢では接触なし）
- 確認：自動テスト `--scenario drill_cross` で、右手を左の前腕の方へ持っていき、ドリルを向ける。`check_tool_gap.py` で、ドリルと左腕の距離が3.7cm、体とが4.0cm で止まることを確認した（クラウド）
