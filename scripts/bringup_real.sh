#!/bin/bash
# Semantic mission stack on the REAL AMMR (test room) — no Gazebo / no local
# Nav2.  Everything runs in the robot's DDS domain (CycloneDDS / 56):
#   real_robot_nav_bridge (/ammr/state -> /amcl_pose, navigate_to_pose ->
#   /ammr/goal_pose) -> mediator (gated) -> mission_bt_cpp -> ui_server:8080
#   -> kg_to_usd --watch -> Isaac KG twin (run_isaac_ammr.sh kg, AMMR_REAL=1).
#
#   scripts/bringup_real.sh                      # offset = 2026-09-08 value
#   scripts/bringup_real.sh --anchor 0.7365 0.7815 3.951   # re-anchor first
#   NO_ISAAC=1 scripts/bringup_real.sh           # skip Isaac
# Prereq: robot on, digital_twin.launch.py + Nav2/AMCL running, same wifi
# (ammr20_test).  Logs in ~/bringup_logs.
set +u
LOG=${BRINGUP_LOG_DIR:-$HOME/bringup_logs}; mkdir -p "$LOG"
WS=/home/caselab/blk360_ros2_ws
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=56
export CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"

echo "== 0/5 stop the Gazebo stack (if any) =="
bash $WS/scripts/kill_all.sh >/dev/null 2>&1
for p in "ui[_]server" "mediator[_]server" "mission_bt[_]cpp" "kg_to[_]usd" "real_robot_nav[_]bridge" "isaacsim_kg[_]twin"; do
  pkill -f "$p" 2>/dev/null
done
sleep 3

echo "== 1/5 robot handshake =="
bash $WS/scripts/ammr_net_check.sh || { echo "HANDSHAKE FAILED — robot bridge running? same wifi?"; exit 1; }

echo "== 2/5 nav bridge (robot map <-> KG frame) =="
setsid nohup python3 $WS/scripts/real_robot_nav_bridge.py "$@" > $LOG/bridge.log 2>&1 &
sleep 4
timeout 6 ros2 topic echo /kg_robot_pose --once --field pose.pose.position >/dev/null 2>&1 \
  && echo "  /kg_robot_pose OK (KG frame)" || echo "  WARN: no /amcl_pose yet (robot pose not streaming?)"

echo "== 3/5 mediator + mission_bt_cpp (gated) =="
setsid nohup ros2 run semantic_nav_bt mediator_server --ros-args -p gated:=true > $LOG/mediator.log 2>&1 &
sleep 3
setsid nohup ros2 run semantic_nav_bt_cpp mission_bt_cpp --ros-args -r amcl_pose:=/kg_robot_pose > $LOG/btcpp.log 2>&1 &
sleep 4

echo "== 4/5 KG->USD watcher + ui_server =="
( cd /home/caselab/Downloads/Cyclone360_data/blk360_seg && setsid nohup .venv/bin/python scripts/kg_to_usd.py --watch > $LOG/kg2usd.log 2>&1 & )
setsid nohup ros2 run semantic_nav_bt ui_server --ros-args -r amcl_pose:=/kg_robot_pose -r goal_pose:=/kg_goal_pose -r initialpose:=/kg_initialpose > $LOG/ui.log 2>&1 &
sleep 4
curl -s -m 3 http://localhost:8080/api/state | head -c 120; echo

if [ -z "${NO_ISAAC:-}" ]; then
  echo "== 5/5 Isaac KG twin (real robot, pose from /amcl_pose) =="
  cd $WS && AMMR_REAL=1 DISPLAY=:1 setsid nohup scripts/run_isaac_ammr.sh kg --pose-source amcl --goal-topic /kg_goal_pose --amcl-topic /kg_robot_pose > $LOG/isaac_kg.log 2>&1 &
  echo "  Isaac starting (~80 s), log $LOG/isaac_kg.log"
fi
echo BRINGUP_REAL_DONE
