#!/bin/bash
# AMMR digital-twin DDS handshake gate (README STEP 0~3).
# Run on this Isaac PC while the ammr runs:
#   ros2 launch digital_twin_bridge digital_twin.launch.py
# Pass = /ammr/state streams at ~30 Hz. Only then launch the twin.
#
# Prereq (one-time, needs sudo):
#   sudo apt install ros-jazzy-rmw-cyclonedds-cpp
# ammr is reachable at one of these depending on which wifi this PC is on:
#   ammr20_test (PC 192.168.31.x) -> 192.168.31.56 ; AMMR20 (PC 192.168.10.x) -> 192.168.10.66
AMMR_IPS="192.168.31.56 192.168.10.66"
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=56
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"

if [ ! -e /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]; then
    echo "FAIL: system rmw_cyclonedds missing."
    echo "  -> sudo apt install ros-jazzy-rmw-cyclonedds-cpp"
    exit 1
fi

MYIP=$(ip -4 -o addr show wlp6s0 | awk '{print $4}')
SSID=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | grep '^yes' | cut -d: -f2)
echo "== STEP 0: this PC wlp6s0=$MYIP ssid=$SSID ; ping ammr ($AMMR_IPS)"
AMMR_IP=""
for ip in $AMMR_IPS; do
    ping -c 2 -W 2 $ip >/dev/null 2>&1 && { AMMR_IP=$ip; break; }
done
[ -n "$AMMR_IP" ] && echo "  OK: ammr at $AMMR_IP" || { echo "  FAIL: no route to ammr — robot on? PC on the robot wifi (AMMR20 / ammr20_test)?"; exit 1; }

echo "== STEP 3a: topic discovery (10 s)"
if timeout 10 ros2 topic list 2>/dev/null | grep -q "/ammr/state"; then
    echo "  OK: /ammr topics visible"
else
    echo "  FAIL: no /ammr topics."
    echo "  checklist: 1) ammr에서 digital_twin.launch.py 실행중인가"
    echo "             2) 양쪽 ROS_DOMAIN_ID=56 인가"
    echo "             3) 방화벽: sudo ufw allow from 192.168.31.0/24"
    exit 1
fi

echo "== STEP 3b: /ammr/state rate (expect ~30 Hz)"
timeout 8 ros2 topic hz /ammr/state 2>/dev/null | head -2

echo "== STEP 3c: one sample"
timeout 8 ros2 topic echo /ammr/state --once --field pose.pose 2>/dev/null | head -8

echo ""
echo "PASS. 이제 트윈 실행:"
echo "  scripts/run_isaac_ammr.sh twin --anchor 0.7365 0.7815 0.809"
echo "(로봇을 스캔 당시 자리에 세워둔 상태에서; 반대면 yaw 3.951)"
