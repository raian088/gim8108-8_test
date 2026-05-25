■WSLでのgimコントロールソフトの開き方

・windows側で（管理者権限）ターミナルで以下を打つ

usbipd attach --wsl --busid 2-4（2-4はusbipd listで検索）

WSL側で
source /opt/ros/jazzy/setup.bash
  source ~/gim8108-8_test/install/setup.bash
  ros2 launch gim8108_driver start.launch.py

