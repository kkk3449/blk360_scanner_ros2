#!/bin/bash
# restart ONLY the robot nav bridge with the given args (e.g. --map-offset X Y YAW | --anchor X Y YAW_RAD)
cd /home/caselab/blk360_ros2_ws
pkill -f "real_robot_nav[_]bridge" 2>/dev/null; sleep 2
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=56 CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"
setsid nohup python3 scripts/real_robot_nav_bridge.py "$@" > ~/bringup_logs/bridge.log 2>&1 &
sleep 5
grep -E "bridge up|CALIBRATED|robot map" ~/bringup_logs/bridge.log | tail -2 | cut -c1-150
curl -s -m 3 http://localhost:8080/api/state | python3 -c "import sys,json; d=json.load(sys.stdin); print('UI robot_pose', [round(v,2) for v in d['robot_pose']])"
