#!/usr/bin/env bash
# Decisive test: sim + cartographer + nav2 (NO frontier). Send ONE reachable
# navigate_to_pose goal and measure whether Nav2 actually drives the robot.
WS=/home/caselab/blk360_ros2_ws
L=$WS/scripts/last_test
mkdir -p "$L"; : > "$L/goal_test.log"
log(){ echo "$@" | tee -a "$L/goal_test.log"; }

for p in "gz sim" ros_gz parameter_bridge robot_state_publisher frontier_explorer \
         stop_scan_sequencer mock_blk360 cartographer "_server" bt_navigator \
         collision_monitor velocity_smoother lifecycle_manager route_server \
         smoother_server waypoint_follower docking_server image_bridge; do
  pkill -9 -f "$p" 2>/dev/null; done
sleep 3

source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle ROS_DOMAIN_ID=77

setsid ros2 launch blk360_bringup tb3_sim.launch.py gui:=false use_sim_time:=true > "$L/g_sim.log" 2>&1 &
for i in $(seq 1 45); do timeout 3 ros2 topic echo /odom --once >/dev/null 2>&1 && { log "[g] /odom up (${i}s)"; break; }; sleep 1; done
setsid ros2 launch blk360_bringup cartographer.launch.py use_sim_time:=true > "$L/g_carto.log" 2>&1 &
setsid ros2 launch blk360_bringup nav2.launch.py use_sim_time:=true > "$L/g_nav2.log" 2>&1 &
log "[g] waiting 30s for nav2 to activate + map..."
sleep 30
MW=$(timeout 4 ros2 topic echo /map --field info.width --once 2>/dev/null | head -1)
log "[g] map width=$MW"
X0=$(timeout 4 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null | head -1)
log "[g] odom.x before = $X0"

log "[g] measuring cmd_vel-chain rates for 12s during a goal..."
( timeout 12 ros2 topic hz /cmd_vel_nav    2>&1 | grep -m1 "average rate" | sed 's/^/[g] cmd_vel_nav: /'    >> "$L/goal_test.log" ) &
( timeout 12 ros2 topic hz /cmd_vel_smoothed 2>&1 | grep -m1 "average rate" | sed 's/^/[g] cmd_vel_smoothed: /' >> "$L/goal_test.log" ) &
( timeout 12 ros2 topic hz /cmd_vel         2>&1 | grep -m1 "average rate" | sed 's/^/[g] cmd_vel(final): /'   >> "$L/goal_test.log" ) &

# Goal ~1.2 m ahead of the start pose, in the map frame (origin ~ start).
timeout 20 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  > "$L/g_goal.log" 2>&1 &
sleep 14
X1=$(timeout 4 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null | head -1)
log "[g] odom.x after  = $X1"
log "[g] DELTA x = $(python3 -c "print(round(float('${X1:-0}')-float('${X0:-0}'),3))" 2>/dev/null) m"
log "[g] goal result tail:"; grep -iE "status|result|STATUS|Goal" "$L/g_goal.log" | tail -4 | sed 's/^/[g]   /' | tee -a "$L/goal_test.log"
log "[g] DONE"
for p in cartographer.launch nav2.launch tb3_sim.launch "gz sim" ros_gz "_server" \
         cartographer robot_state bt_navigator collision_monitor velocity_smoother \
         lifecycle_manager route_server smoother_server waypoint docking image_bridge; do
  pkill -9 -f "$p" 2>/dev/null; done
