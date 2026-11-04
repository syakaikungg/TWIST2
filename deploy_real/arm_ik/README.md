# arm_ik: G1 の腕テレオペ（PICO → IK → rt/arm_sdk）

NexTech Week 2026 秋の展示用。G1 の内蔵PC（`~/nextech/arm_ik/`）で動かす。脚は純正の制御のまま、両腕と腰を `rt/arm_sdk` で上書きする。

| ファイル | 役割 |
|---|---|
| `arm_ik_dryrun.py` | PICO（xrobotoolkit_sdk）から手の位置を読み、mink で腕14関節の IK を解いて Redis `arm_ik_q` / `drill_cmd` / `teleop_display` に書く。ロボットには何も送らない |
| `arm_sdk_bridge.py` | Redis `arm_ik_q` → `rt/arm_sdk` の LowCmd。既定はログのみで、`--send` のときだけ送信する。`--sim` は `arm_sim.py` と Redis でつなぐ |
| `arm_sim.py` | 骨盤と脚を固定した MuJoCo のシミュレーター（腕と腰を arm_sdk と同じ形で動かす）。右手に見た目だけのドリルが付く |
| `sim_drill.py` | シミュレーターのドリルの形と回し方（ビューアーからも使う） |
| `sim_viewer.py` | ノートPC（WSL）のビューアー。G1の `stream_sim.py` を SSH で動かして関節角を受け取り、MuJoCo の窓に描く。`sim_drill.py` と同じフォルダ（`~/nextech_viewer/`）に置く |
| `sim_run.sh` / `sim_live_bg.sh` | シミュレーターでの自動テスト / PICO で操作する live |
| `run_live.sh` | 実機（`--send`）。本人の確認後にだけ使う |
| `stop.sh` | 非常停止（Redis `teleop_stop`） |
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
| `stop` | 非常停止 | 非常停止（PCから再起動が必要） |
| `warn` | PICO のデータ待ち、またはブリッジが応答していない | 通信待ち（データが戻れば再開） |
| `wait` | 準備中、または最初の姿勢に戻っている途中 | 準備中 あと2.1秒（そのあと握れば動く） |
| `idle` | 一時停止中 | 一時停止中（右Aで開始） |
| `ready` | 握れば動く | 握れば動きます |
| `ok` | 追従中 | 追従中 |

ドリルが ON のときは、文の後ろに「／ ドリルON」が付く。

## 腰

arm_sdk を有効にすると、腰（12〜14）もこちらの担当になる（公式の arm7 の例と同じ組み合わせ）。既定は TWIST2 の腰ゲイン（kp 150 / kd 4）と、roll・pitch の積分補正（`--waist-correct`）。シミュレーターでの腰の傾きは、最大0.03〜0.04 rad。

## シミュレーターのドリル

`arm_sim.py` は右手に見た目だけのドリルを付ける（重さも当たり判定もないので、物理の結果は変わらない。重さは今までどおり `--payload-kg`）。IK の Redis `drill_cmd` が ON の間（追従中に右トリガー）、先が毎秒3回転で回り、モーターの輪が緑になる。実際の回転数ではなく、映像で回っているのが分かる速さにしている（`--drill-rps`）。`--no-drill` で外せる。

形は仮のもの（アタッチメントが未完成のため）。指の上にクランプ、その先にモーター、チャック（黄色の線）、ビット（平たい刃）が、手の前方向（`right_wrist_yaw_link` の x）に並ぶ。手首から先端まで約37cm。

状態は `sim_qpos` の `"drill": {"on": 0/1, "angle": rad, "rps": 回転/秒}` に入る。`sim_qpos` から絵を描くビューアーは、次の3行で同じドリルを出せる。

```python
import sim_drill
sim_drill.add_drill(spec)            # MjSpec.from_file(...) のあと、compile() の前
view = sim_drill.DrillView(m)        # compile したモデル
view.apply(m, msg.get("drill"))      # 毎フレーム、mj_kinematics / mj_forward の前
```
