# PICO VR 接続ガイド

## ネットワーク構成

| デバイス | IP アドレス | 接続方式 |
|---|---|---|
| PC (ubuntu) | `192.168.123.175` (DHCP、変動の可能性あり) | 有線LAN (`enp5s0`) |
| PICO VR | `192.168.123.75` | WiFi (同一サブネット) |
| ゲートウェイ | `192.168.123.1` | - |

> ⚠️ PCのIPはDHCPで割り当てられているため変わる可能性がある。`ip -4 addr show enp5s0` で確認すること。

## XRoboToolkit アーキテクチャ

```
PICO VR (XRobot APP)
    │
    │  TCP port 63901
    ▼
RoboticsServiceProcess (/opt/apps/roboticsservice/)
    │
    │  gRPC port 60061 (127.0.0.1)
    ▼
Python SDK (xrobotoolkit_sdk / xrt.init())
    │
    ▼
teleop.sh → xrobot_teleop_to_robot_w_hand.py
```

### ポート一覧

| ポート | プロトコル | バインド | 用途 |
|---|---|---|---|
| **63901** | TCP | `0.0.0.0` (全IF) | PICO VR の XRobot APP が接続するポート |
| **60061** | TCP | `127.0.0.1` | Python SDK (gRPC) 内部通信 |
| **59313** | UDP | `0.0.0.0` (全IF) | デバイス検出/ブロードキャスト |

## 起動手順

### 1. RoboticsServiceProcess を起動（必須・最初に）

```bash
cd /opt/apps/roboticsservice && bash runService.sh
```

確認:
```bash
# プロセスが動いているか
pgrep -a RoboticsServiceProcess

# ポートが開いているか
ss -tlnp 'sport = :63901'
ss -tlnp 'sport = :60061'
```

### 2. teleop.sh を起動

```bash
cd ~/workspace/sakai/TWIST2
bash teleop.sh
```

### 3. PICO VR の XRobot APP で接続

- IPアドレス: PCの現在のIP（例: `192.168.123.175`）
- ポート: デフォルト（63901）

## トラブルシューティング

### PICO VR から接続できない

1. **RoboticsServiceProcess が起動しているか確認**
   ```bash
   pgrep -a RoboticsServiceProcess
   ```
   起動していなければ `cd /opt/apps/roboticsservice && bash runService.sh`

2. **PCのIPアドレスを確認（DHCP で変わることがある）**
   ```bash
   ip -4 addr show enp5s0 | grep inet
   ```

3. **PICO VR と同一ネットワークか確認**
   ```bash
   ping 192.168.123.75
   ```

4. **ポートが開いているか確認**
   ```bash
   ss -tlnp 'sport = :63901'
   ```

5. **ファイアウォール確認**
   ```bash
   sudo ufw status
   # 有効なら無効化
   sudo ufw disable
   ```

### XRobot SDK が body data を取得できない

- `RoboticsServiceProcess` が起動していないと、`xrt.init()` は成功するが gRPC 接続（60061）がないためデータが来ない
- teleop のループは回るが body tracking データが None になる

### IPアドレスが変わった場合

- PCのIPがDHCPで変わった場合、PICO VR の XRobot APP で新しいIPに更新する必要がある
- `teleop.sh` 内の `redis_ip` は `localhost` なので影響なし

## 設定ファイル

### PC側

- サービス設定: `/opt/apps/roboticsservice/setting.ini`
- SDK設定: `PXREASetting.ini`（カレントディレクトリに配置、なければデフォルト `127.0.0.1:60061`）

### PICO VR 側

XRobot APP の設定ファイル（adb 経由で取得/設定可能）:
```bash
# Pull from PICO
adb pull /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml video_source.yml

# Push to PICO
adb push video_source.yml /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml
```

## インストール情報

- パッケージ名: `roboticsservice`
- インストールパス: `/opt/apps/roboticsservice/`
- インストール方法:
  ```bash
  sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
  ```
- .deb ファイル: `/home/ubuntu/Downloads/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb`
