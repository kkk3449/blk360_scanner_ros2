#!/bin/bash
# Send a direct goal in the KG frame through the bridge (/kg_goal_pose) and watch the robot for N seconds.
#   scripts/kg_goal_probe.sh X Y [yaw_rad] [watch_s]
X=$1; Y=$2; YAW=${3:-0}; W=${4:-30}
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=56 CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"
QZ=$(python3 -c "import math; print(math.sin($YAW/2))"); QW=$(python3 -c "import math; print(math.cos($YAW/2))")
S0=$(curl -s -m 2 http://localhost:8080/api/state | python3 -c "import sys,json; p=json.load(sys.stdin)['robot_pose']; print(f'{p[0]:.2f} {p[1]:.2f}')")
timeout 6 ros2 topic pub --once -w 1 /kg_goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}" >/dev/null 2>&1 && echo "KG goal ($X,$Y) sent; start=($S0)"
sleep 2; grep -E "DIRECT goal" ~/bringup_logs/bridge.log | tail -1 | cut -c40-140
for i in $(seq 1 $((W/3))); do sleep 3; S=$(curl -s -m 2 http://localhost:8080/api/state | python3 -c "import sys,json; p=json.load(sys.stdin)['robot_pose']; print(f'{p[0]:.2f} {p[1]:.2f}')"); python3 -c "
import math; a=[float(v) for v in '$S0'.split()]; b=[float(v) for v in '$S'.split()]
print(f't={$i*3}s KG=({b[0]:.2f},{b[1]:.2f}) moved={math.hypot(b[0]-a[0],b[1]-a[1]):.2f} m to_goal={math.hypot(b[0]-$X,b[1]-$Y):.2f} m')"; done
