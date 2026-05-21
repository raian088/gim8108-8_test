#!/bin/bash
# WSL で USB-CAN アダプター（AtoC）を使うためのセットアップスクリプト
# ============================================================
# 前提: Windows 側で usbipd-win がインストール済みであること
#
# Windows PowerShell (管理者) で実行:
#   usbipd list                        # USB デバイス一覧
#   usbipd bind --busid <BUSID>        # USB-CAN を共有設定
#   usbipd attach --wsl --busid <BUSID> # WSL へ接続
#
# その後、この WSL スクリプトを実行する
# ============================================================

set -e

echo "=== GIM8108-8 CAN セットアップ ==="

# USB デバイス確認
echo "[1] シリアルデバイス確認..."
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || {
    echo "  WARNING: /dev/ttyACM* または /dev/ttyUSB* が見つかりません"
    echo "  Windows 側で usbipd attach --wsl を実行してから再試行してください"
    exit 1
}

DEVICE=$(ls /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 2>/dev/null | head -1)
echo "  デバイス: $DEVICE"

# 権限付与
echo "[2] デバイス権限設定..."
sudo chmod 666 "$DEVICE"
echo "  OK: $DEVICE へのアクセス権付与"

echo ""
echo "=== 完了 ==="
echo "以下のコマンドでドライバーを起動してください:"
echo ""
echo "  # ターミナル 1: ドライバー起動"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/gim8108-8_test/install/setup.bash"
echo "  ros2 launch gim8108_driver driver.launch.py can_channel:=$DEVICE can_bustype:=slcan"
echo ""
echo "  # ターミナル 2: GUI 起動"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/gim8108-8_test/install/setup.bash"
echo "  ros2 launch gim8108_gui gui.launch.py"
