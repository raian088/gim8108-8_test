# GIM8108-8 Motor Driver & GUI — ROS2

GIM8108-8 ブラシレスジンバルモーター（ODrive / CyberBeast）向けの ROS2 ドライバー＋マルチモーター制御 GUI です。  
USB Type-C 直結（ODrive USB SDK）と USB-CAN アダプター（SLCAN / SocketCAN）の両方に対応しています。

---

## 機能

- **マルチモーター管理** — モーターを動的に追加・削除、設定は自動保存
- **モーター制御** — 位置 / 速度 / トルクの各制御モード、スライダー＋絶対値入力＋相対移動ボタン
- **ソフトリミット** — ソフトウェアによる角度上下限
- **グローバル E-STOP** — 全モーター同時緊急停止
- **モーションレコーダー** — 手動でモーターを動かして軌跡を記録・再生
- **シーケンサー** — モーションのプレイリスト（順番・繰り返し・ウェイト付き）を保存/読み込み
- **リアルタイムグラフ** — 角度・速度・電流・温度・電圧を時系列グラフで表示
- **設定の永続化** — PID ゲイン・速度制限・電流制限・ソフトリミットを次回起動時に自動復元

---

## 依存関係

| カテゴリ | パッケージ |
|---|---|
| ROS2 | **Jazzy** (Ubuntu 24.04) |
| Python | `odrive` `pyusb` `PyQt5` `matplotlib` |
| ROS pkg | `sensor_msgs` `std_msgs` `std_srvs` |

```bash
pip install odrive pyusb matplotlib
sudo apt install python3-pyqt5
```

---

## ディレクトリ構成

```
src/
├── gim8108_driver/          # ROS2 モータードライバーノード
│   ├── motor_node_usb.py    # USB Type-C (ODrive SDK) 用ノード
│   ├── motor_node.py        # USB-CAN (SLCAN/SocketCAN) 用ノード
│   └── launch/
│       ├── start.launch.py        # ドライバー + GUI 一発起動
│       ├── driver_usb.launch.py   # USB ドライバーのみ
│       └── driver.launch.py       # CAN ドライバーのみ
├── gim8108_gui/             # マルチモーター制御 GUI
│   └── control_gui.py
└── gim8108_motion/          # モーション再生スタンドアロン GUI
    └── motion_gui.py

~/gim8108_motions/           # 記録したモーションファイル (*.json)
~/gim8108_sequences/         # 保存したシーケンスファイル (*.json)
~/.config/gim8108/           # GUI 設定の永続化ファイル
```

---

## ビルド

```bash
cd ~/gim8108-8_test
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 起動方法

 cd ~/gim8108-8_test

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 launch gim8108_driver start.launch.py

### USB Type-C（推奨）— ODrive を直接 USB 接続する場合

```bash
source /opt/ros/jazzy/setup.bash
source ~/gim8108-8_test/install/setup.bash

# ドライバー + GUI を同時起動（推奨）
ros2 launch gim8108_driver start.launch.py

# オプション引数の例
ros2 launch gim8108_driver start.launch.py gear_ratio:=8.0 traj_vel:=20.0 traj_accel:=5.0
```

引数一覧：

| 引数 | デフォルト | 説明 |
|---|---|---|
| `axis` | `0` | ODrive 軸番号 |
| `gear_ratio` | `8.0` | 減速比 |
| `traj_vel` | `10.0` | トラジェクトリ速度制限 [turns/s] |
| `traj_accel` | `5.0` | 加速度制限 [turns/s²] |
| `traj_decel` | `5.0` | 減速度制限 [turns/s²] |
| `connect_timeout` | `60.0` | ODrive 接続タイムアウト [s] |

GUI 単体起動（ドライバーは別ターミナルで起動済みの場合）：

```bash
ros2 launch gim8108_gui gui.launch.py
```

---

### USB-CAN アダプター（SLCAN）接続

```bash
# ターミナル 1: CAN ドライバー起動
ros2 launch gim8108_driver driver.launch.py \
    can_channel:=/dev/ttyACM0 \
    can_bustype:=slcan \
    can_bitrate:=500000

# ターミナル 2: GUI 起動
ros2 launch gim8108_gui gui.launch.py
```

---

## WSL2 での起動方法（Windows + usbipd）

WSL2 では USB デバイスを `usbipd-win` で転送してから使用します。

### 1. usbipd-win のインストール（Windows 側・初回のみ）

```powershell
# Windows PowerShell（管理者）
winget install usbipd
```

### 2. ODrive を WSL に転送する

```powershell
# Windows PowerShell（管理者）で実行
usbipd list                          # CyberBeast の BUSID を確認（例: 2-3）
usbipd bind --busid <BUSID>          # 初回のみ
usbipd attach --wsl --busid <BUSID>  # WSL に接続
```

### 3. WSL 側のセットアップ（初回のみ）

```bash
# WSL ターミナル
bash ~/gim8108-8_test/setup_usb_wsl.sh
```

このスクリプトが実行する内容：
- ODrive 用 udev ルール作成（`/etc/udev/rules.d/91-odrive.rules`）
- ODrive デバイスの検出確認

### 4. ROS2 GUI の起動

```bash
# WSL ターミナル（毎回）
source /opt/ros/jazzy/setup.bash
source ~/gim8108-8_test/install/setup.bash
ros2 launch gim8108_driver start.launch.py
```

### 5. USB 転送の切断（使用後）

```powershell
# Windows PowerShell（管理者）
usbipd detach --busid <BUSID>
```

### ディスプレイ設定（GUI が表示されない場合）

WSL2 では DISPLAY 環境変数が必要な場合があります：

```bash
# ~/.bashrc に追加
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
export LIBGL_ALWAYS_INDIRECT=1
```

Windows 側で **VcXsrv** または **X410** などの X サーバーを起動しておく必要があります。  
**Windows 11 + WSLg** を使用している場合は上記設定は不要です（GUI が自動的に表示されます）。

---

## GUI の使い方

### モーターの追加

1. 左パネルの **Add Motor** ボタンをクリック
2. ODrive の軸番号・減速比・シリアル番号（またはUSBアドレス）を設定
3. **OK** → ドライバーノードが自動起動し接続を待機

### モーターの制御

- **Enable Motor** → クローズドループ制御を開始
- **Control Mode** → Position / Velocity / Torque を切り替え
- **スライダー** → 0–360° の範囲で位置指令
- **Abs. Position** → 多回転対応の絶対位置入力（例: -3600°〜+3600°）
- **+1°/-1° など** → 相対移動ボタン
- **Soft Limit** → ソフトウェア角度リミット（有効時は自動クランプ）

### モーションの記録と再生

1. **Motion タブ** → Record パネルで **Free (Torque OFF)** → 手動でモーターを動かす
2. **Record** ボタンで記録開始 → **Stop** → **Save Motion** で保存
3. Playback パネルの一覧からモーションを選択 → **Play**

### シーケンサー

1. **Sequencer タブ** → Motion Library から再生したいモーションをダブルクリックして Playlist に追加
2. 各アイテムの "Wait after" でモーション間のウェイト時間を設定
3. **Save Playlist** で名前をつけて保存、次回 **Load** で呼び出し可能
4. **Play Sequence** → 順番に再生（Loop チェックで繰り返し）

### ステータスグラフ

**Status タブ** で角度・速度・電流・温度・電圧のリアルタイムグラフを表示します。  
更新レート（1/2/5/10 Hz）と Clear ボタンで管理できます。

---

## ROS2 トピック・サービス一覧

ノード名前空間 `<ns>` はデフォルト `/gim8108_motor_node`（GUI から追加した場合は `/motor_N/gim8108_motor_node`）。

| トピック | 型 | 方向 | 説明 |
|---|---|---|---|
| `<ns>/joint_state` | `JointState` | 出力 | 位置[rad] / 速度[rad/s] / 電流[A] |
| `<ns>/is_connected` | `Bool` | 出力 | ODrive 接続状態（latched） |
| `<ns>/bus_voltage` | `Float64` | 出力 | バス電圧 [V] |
| `<ns>/temperature` | `Float64` | 出力 | FET 温度 [°C] |
| `<ns>/serial_number` | `String` | 出力 | ODrive シリアル番号（latched） |
| `<ns>/cmd_pos_deg` | `Float64` | 入力 | 位置指令 [deg]（出力軸） |
| `<ns>/cmd_vel` | `Float64` | 入力 | 速度指令 [turns/s] |
| `<ns>/cmd_torque` | `Float64` | 入力 | トルク指令 [Nm] |
| `<ns>/traj_vel_limit` | `Float64` | 入力 | トラジェクトリ速度制限 |
| `<ns>/traj_accel_limit` | `Float64` | 入力 | 加速度制限 |
| `<ns>/traj_decel_limit` | `Float64` | 入力 | 減速度制限 |

| サービス | 型 | 説明 |
|---|---|---|
| `<ns>/enable` | `SetBool` | モーター有効化 / IDLE |
| `<ns>/calibrate` | `Trigger` | キャリブレーション実行 |
| `<ns>/clear_errors` | `Trigger` | エラークリア |
| `<ns>/save_config` | `Trigger` | ODrive フラッシュへ設定保存 |
| `<ns>/estop` | `Trigger` | 緊急停止 |
