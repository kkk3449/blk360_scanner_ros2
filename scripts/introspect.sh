#!/usr/bin/env bash
# Launch sim+carto+nav2 and introspect the cmd_vel chain wiring + types.
WS=/home/caselab/blk360_ros2_ws
L=$WS/scripts/last_test; mkdir -p "$L"
OUT=$L/introspect.log; : > "$OUT"
bash "$WS/scripts/kill_all.sh" >/dev/null 2>&1
sleep 2
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle ROS_DOMAIN_ID=77

setsid ros2 launch blk360_bringup tb3_sim.launch.py gui:=false use_sim_time:=true > "$L/i_sim.log" 2>&1 &
for i in $(seq 1 45); do timeout 3 ros2 topic echo /odom --once >/dev/null 2>&1 && { echo "/odom up ${i}s" >> "$OUT"; break; }; sleep 1; done
setsid ros2 launch blk360_bringup cartographer.launch.py use_sim_time:=true > "$L/i_carto.log" 2>&1 &
setsid ros2 launch blk360_bringup nav2.launch.py use_sim_time:=true > "$L/i_nav2.log" 2>&1 &
sleep 30
{
  echo "===== cmd_vel topics with TYPES ====="
  ros2 topic list -t 2>/dev/null | grep -iE "cmd_vel"
  echo "===== /velocity_smoother node info ====="
  timeout 8 ros2 node info /velocity_smoother 2>/dev/null
  echo "===== /collision_monitor node info ====="
  timeout 8 ros2 node info /collision_monitor 2>/dev/null
  echo "===== controller cmd_vel pub ====="
  timeout 8 ros2 node info /controller_server 2>/dev/null | grep -iE "cmd_vel|Twist"
} >> "$OUT" 2>&1
echo "INTROSPECT_DONE" >> "$OUT"
bash "$WS/scripts/kill_all.sh" >/dev/null 2>&1
