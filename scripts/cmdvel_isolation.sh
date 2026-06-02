#!/usr/bin/env bash
# Isolate sim+bridge: bring up TB3 sim only, publish cmd_vel, check odom moves.
WS=/home/caselab/blk360_ros2_ws
LOG=$WS/scripts/last_test
mkdir -p "$LOG"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=77

cleanup(){ pkill -INT -f tb3_sim.launch 2>/dev/null; sleep 2; pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f ros_gz 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; }
trap cleanup EXIT

setsid ros2 launch blk360_bringup tb3_sim.launch.py gui:=false use_sim_time:=true > "$LOG/iso_sim.log" 2>&1 &
for i in $(seq 1 60); do timeout 3 ros2 topic echo /odom nav_msgs/msg/Odometry --once >/dev/null 2>&1 && break; sleep 1; done
echo "[iso] cmd_vel topic info:"; timeout 4 ros2 topic info /cmd_vel
X0=$(timeout 3 ros2 topic echo /odom nav_msgs/msg/Odometry --once 2>/dev/null | grep -m1 -A1 'position:' | grep 'x:' | awk '{print $2}')
echo "[iso] odom x before: $X0"
echo "[iso] publishing TwistStamped to /cmd_vel for 6s..."
timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.15}, angular: {z: 0.0}}}' >/dev/null 2>&1
sleep 1
X1=$(timeout 3 ros2 topic echo /odom nav_msgs/msg/Odometry --once 2>/dev/null | grep -m1 -A1 'position:' | grep 'x:' | awk '{print $2}')
echo "[iso] odom x after : $X1"
echo "[iso] DELTA: $(python3 -c "print(round(float('${X1:-0}')-float('${X0:-0}'),4))" 2>/dev/null)"
echo "[iso] DONE"
