# TWIST2 23DOF クイックスタートガイド

## 環境一覧

| 環境名 | Python | 用途 |
|--------|--------|------|
| `twist2` | 3.8 | sim2sim / sim2real / モーション再生 |
| `gmr` | 3.10 | テレオペ（PICO VR → モーションリターゲティング） |

---

## 1. テレオペ（PICO VR → ロボット制御）

### 前提
- PICO VR が PC と同じネットワークに接続
- RoboticsServiceProcess が起動済み: `cd /opt/apps/roboticsservice && bash runService.sh`
- Redis 起動済み: `sudo systemctl start redis-server`

### 起動

```bash
# ターミナル1: テレオペ（gmr 環境）
conda activate gmr
bash teleop.sh
```

テレオペ単体ではロボットは動きません。以下の sim2sim または sim2real と組み合わせて使います。

---

## 2. Sim2Sim（MuJoCo シミュレーション検証）

### 2-A. テレオペ操作で sim2sim

```bash
# ターミナル1: テレオペ（gmr 環境）
conda activate gmr
bash teleop.sh

# ターミナル2: sim2sim（twist2 環境）
conda activate twist2
bash sim2sim_23dof.sh
```

### 2-B. オフラインモーション再生で sim2sim

```bash
# ターミナル1: モーション再生（twist2 環境）
conda activate twist2
bash run_motion_server_23dof.sh

# ターミナル2: sim2sim（twist2 環境）
conda activate twist2
bash sim2sim_23dof.sh
```

### 2-C. 仮想カメラ付き sim2sim

MuJoCo のレンダリング映像を ZMQ で配信:

```bash
# ターミナル1: テレオペまたはモーション再生（上記参照）

# ターミナル2: カメラ配信付き sim2sim
conda activate twist2
STREAM_CAMERA=1 bash sim2sim_23dof.sh

# ターミナル3: 映像受信（別ターミナル）
conda activate twist2
cd deploy_real
python data_utils/vision_client.py --server 127.0.0.1 --port 5555
```

### カスタムポリシーを使う場合

```bash
bash sim2sim_23dof.sh assets/ckpts/my_custom_policy.onnx
```

---

## 3. Sim2Real（実機制御）

### 前提
- G1 ロボットが起動済み、イーサネット接続済み
- `ping 192.168.123.164` が通ること
- リモコンで `L2+R2` → SDK（開発者）モード

### 起動

```bash
# ターミナル1: テレオペ（gmr 環境）
conda activate gmr
bash teleop.sh

# ターミナル2: sim2real（twist2 環境）
conda activate twist2
bash sim2real_23dof.sh
```

### リモコン操作手順

| 順番 | ボタン | 動作 |
|:----:|--------|------|
| 1 | `[START]` | デフォルト姿勢へ2秒かけて移動 |
| 2 | `[A]` | RL ポリシー制御開始（テレオペ追従） |
| 3 | `[Select]` | 安全に終了（ダンピングモードへ） |

### カスタムポリシーを使う場合

```bash
bash sim2real_23dof.sh assets/ckpts/my_custom_policy.onnx
```

---

## 4. G1 カメラ → PICO VR ストリーミング

G1 ロボットの RealSense RGB カメラ映像を PICO VR にリアルタイム配信。

### 起動（PC から実行）

```bash
bash scripts/start_vr_camera.sh
```

### PICO VR 側の操作

1. XRobot APP を開く
2. Remote Vision → **PICO4U** を選択
3. **Listen** → IP に `192.168.123.164` を入力 → **Confirm**

### 停止

```bash
ssh unitree@192.168.123.164 "pkill -f persistent_vr_server"
```

### ログ監視

```bash
ssh unitree@192.168.123.164 "tail -f /tmp/persistent.log"
```

---

## 全体構成図

```
┌─────────────────────────────────────────────────────────────┐
│  PICO VR (gmr 環境)                                         │
│  teleop.sh → xrobot_teleop_to_robot_w_hand.py               │
│                                                             │
│  VR ボディトラッキング → モーションリターゲティング (GMR)        │
│      ↓                                                      │
│  Redis: action_body_unitree_g1_23dof (29次元)                │
│  Redis: action_body_unitree_g1_with_hands (35次元)           │
└────────────────────────┬────────────────────────────────────┘
                         ↓ Redis
┌────────────────────────┴────────────────────────────────────┐
│  低レベルコントローラー (twist2 環境)                          │
│                                                             │
│  [sim2sim] sim2sim_23dof.sh                                 │
│    → server_low_level_g1_23dof_sim.py (MuJoCo)              │
│                                                             │
│  [sim2real] sim2real_23dof.sh                                │
│    → server_low_level_g1_real_23dof.py (実機)                │
│                                                             │
│  処理: Redis読取 → 観測構築(1162次元) → ONNX推論(23出力)      │
│       → PD制御 → MuJoCo / 実機モーター                       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  カメラストリーミング (ロボット上)                              │
│                                                             │
│  scripts/start_vr_camera.sh (PC から起動)                    │
│    → persistent_vr_server.py (ロボット上で実行)               │
│    → RealSense RGB → GStreamer NVENC H.264 → TCP → PICO VR  │
└─────────────────────────────────────────────────────────────┘
```

---

## トラブルシューティング

| 問題 | 解決方法 |
|------|----------|
| Redis 接続エラー | `sudo systemctl start redis-server` |
| ONNX ポリシーが見つからない | `ls assets/ckpts/*.onnx` で確認 |
| ロボットに ping が通らない | IP: `192.168.123.222`, サブネット: `255.255.255.0` を設定 |
| カメラが占有されている | `scripts/start_vr_camera.sh` が自動で `videohub_pc4` を停止 |
| テレオペの FPS が低い | GPU が弱い場合は `--target_fps 30` に下げる |
| VR 映像が静止する | PICO で Listen し直す（自動再接続される） |
