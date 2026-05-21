#!/bin/bash
# GIM8108-8 USB (Type-C) → usbipd → WSL セットアップスクリプト
# ==============================================================
# 前提: Windows 側で usbipd-win がインストール済みであること
# ==============================================================

set -e

echo "=== GIM8108-8 USB セットアップ ==="
echo ""

# --- 1. udev ルール設定 (ODrive 用) ---
UDEV_FILE="/etc/udev/rules.d/91-odrive.rules"
if [ ! -f "$UDEV_FILE" ]; then
    echo "[1] udev ルール設定中..."
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"' \
        | sudo tee "$UDEV_FILE" > /dev/null
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", MODE="0666"' \
        | sudo tee -a "$UDEV_FILE" > /dev/null
    sudo udevadm control --reload-rules 2>/dev/null || true
    echo "  OK: udev ルール作成"
else
    echo "[1] udev ルール: 設定済み"
fi

# --- 2. ODrive 検出確認 ---
echo "[2] ODrive デバイス確認..."
if python3 -c "import usb.core; d=usb.core.find(idVendor=0x1209); print('ODrive found' if d else 'ODrive not found')" 2>/dev/null | grep -q "found"; then
    echo "  OK: ODrive デバイスを検出しました"
else
    echo "  WARNING: ODrive デバイスが見つかりません"
    echo ""
    echo "  ── Windows PowerShell (管理者) で実行してください ──"
    echo "  1. usbipd list                      # CyberBeast の BUSID を確認"
    echo "  2. usbipd bind --busid <BUSID>       # (初回のみ)"
    echo "  3. usbipd attach --wsl --busid <BUSID>"
    echo "  ────────────────────────────────────────────────────"
    echo ""
    echo "  ※ CyberBeast が Connected リストに表示されない場合は"
    echo "     モーターの電源を入れ、USB-Cケーブルを接続してください"
fi

echo ""
echo "=== 起動コマンド ==="
echo ""
echo "  # ターミナル 1つだけ: ドライバー + GUI を一発起動"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/gim8108-8_test/install/setup.bash"
echo "  ros2 launch gim8108_driver start.launch.py"
echo ""
echo "  # オプション引数 (例)"
echo "  ros2 launch gim8108_driver start.launch.py gear_ratio:=8.0 traj_vel:=20.0"
echo ""
echo "  # 個別起動したい場合"
echo "  ros2 launch gim8108_driver driver_usb.launch.py   # ドライバーのみ"
echo "  ros2 launch gim8108_gui    gui.launch.py           # GUIのみ"
