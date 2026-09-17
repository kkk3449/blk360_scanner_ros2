#!/bin/bash
# Restart bridge + mediator + BT + UI on the robot domain (keeps Isaac / KG watcher).
# Args are passed to the bridge, e.g.:
#   scripts/restart_real_ctrl.sh --map-offset X Y YAW_DEG
#   scripts/restart_real_ctrl.sh --anchor 0.7365 0.7815 0.809
# Run as a FILE so pkill cannot match this command line.
cd /home/caselab/blk360_ros2_ws
for p in "ui[_]server" "mediator[_]server" "mission_bt[_]cpp" "real_robot_nav[_]bridge"; do pkill -f "$p" 2>/dev/null; done
sleep 3
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=56 CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"
L=~/bringup_logs; mkdir -p $L
setsid nohup python3 scripts/real_robot_nav_bridge.py "$@" > $L/bridge.log 2>&1 &
sleep 3
setsid nohup ros2 run semantic_nav_bt mediator_server --ros-args -p gated:=true > $L/mediator.log 2>&1 &
sleep 3
setsid nohup ros2 run semantic_nav_bt_cpp mission_bt_cpp --ros-args -r amcl_pose:=/kg_robot_pose > $L/btcpp.log 2>&1 &
setsid nohup ros2 run semantic_nav_bt ui_server --ros-args -r amcl_pose:=/kg_robot_pose > $L/ui.log 2>&1 &
sleep 6
echo "procs: $(pgrep -af 'real_robot_nav|mediator_serv|mission_bt_cpp|ui_serv' | grep -v pgrep | wc -l)"
grep -E "bridge up|CALIBRATED|robot map" $L/bridge.log | tail -2 | cut -c1-150
curl -s -m 3 http://localhost:8080/api/state | python3 -c "import sys,json; d=json.load(sys.stdin); print('UI robot_pose', [round(v,2) for v in d['robot_pose']], '|', d['status_log'][-1])"
