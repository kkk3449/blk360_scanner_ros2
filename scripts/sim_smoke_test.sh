#!/usr/bin/env bash
# Headless smoke test for the full active-mapping stack on TurtleBot3 (Gazebo).
# Brings up the sim + active_mapping (mock scanner, reconnect path enabled),
# runs for a fixed duration, samples state, then tears everything down.
#
# Usage: sim_smoke_test.sh [DURATION_SEC]
# NOTE: no `set -u` — sourcing ROS setup.bash references unset vars and would abort.

WS=/home/caselab/blk360_ros2_ws
DURATION="${1:-150}"
LOGDIR="$WS/scripts/last_test"
mkdir -p "$LOGDIR"
rm -f "$LOGDIR"/*.log "$LOGDIR"/samples.txt

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle
# Isolate this test's ROS graph from anything else on the machine.
export ROS_DOMAIN_ID=77

# Pre-cleanup: kill any stragglers from previous runs so they can't thrash the graph.
for p in "gz sim" "ros_gz" "parameter_bridge" "robot_state_publisher" "image_bridge" \
         "frontier_explorer" "stop_scan_sequencer" "mock_blk360" "cartographer" \
         "controller_server" "planner_server" "bt_navigator" "behavior_server" \
         "collision_monitor" "velocity_smoother" "lifecycle_manager" "route_server" \
         "smoother_server" "waypoint_follower" "docking_server"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 3

cleanup() {
  echo "[test] cleanup..." | tee -a "$LOGDIR/test.log"
  [[ -n "${AM_PID:-}" ]] && kill -INT "-$AM_PID" 2>/dev/null
  [[ -n "${SIM_PID:-}" ]] && kill -INT "-$SIM_PID" 2>/dev/null
  sleep 3
  pkill -INT -f "active_mapping.launch" 2>/dev/null
  pkill -INT -f "tb3_sim.launch" 2>/dev/null
  sleep 2
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "ros_gz" 2>/dev/null
  pkill -9 -f "frontier_explorer" 2>/dev/null
  pkill -9 -f "stop_scan_sequencer" 2>/dev/null
  pkill -9 -f "mock_blk360_scanner" 2>/dev/null
  pkill -9 -f "cartographer" 2>/dev/null
  pkill -9 -f "controller_server\|planner_server\|bt_navigator\|behavior_server" 2>/dev/null
  echo "[test] done." | tee -a "$LOGDIR/test.log"
}
trap cleanup EXIT INT TERM

echo "[test] starting TB3 sim (headless)..." | tee -a "$LOGDIR/test.log"
setsid ros2 launch blk360_bringup tb3_sim.launch.py gui:=false use_sim_time:=true \
  > "$LOGDIR/sim.log" 2>&1 &
SIM_PID=$!

echo "[test] waiting for /odom and /scan..." | tee -a "$LOGDIR/test.log"
for i in $(seq 1 60); do
  if timeout 3 ros2 topic echo /odom nav_msgs/msg/Odometry --once >/dev/null 2>&1; then
    echo "[test] /odom is up after ${i}s." | tee -a "$LOGDIR/test.log"; break
  fi
  sleep 1
done
timeout 5 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once >/dev/null 2>&1 \
  && echo "[test] /scan is up." | tee -a "$LOGDIR/test.log" \
  || echo "[test] WARN: /scan not seen yet." | tee -a "$LOGDIR/test.log"

echo "[test] starting active_mapping (mock scanner, interval=1.0m, fail_first=1)..." | tee -a "$LOGDIR/test.log"
setsid ros2 launch blk360_bringup active_mapping.launch.py \
  use_sim_time:=true use_rviz:=false use_mock_scanner:=true \
  scan_interval_m:=2.0 fail_first_n_scans:=1 mock_scan_duration_s:=3.0 \
  > "$LOGDIR/active_mapping.log" 2>&1 &
AM_PID=$!

echo "[test] running for ${DURATION}s, sampling every 5s..." | tee -a "$LOGDIR/test.log"
for i in $(seq 1 $((DURATION/5))); do
  sleep 5
  T=$((i*5))
  STATE=$(timeout 3 ros2 topic echo /blk360_stop_scan/state --field data --once 2>/dev/null | head -1)
  OX=$(timeout 3 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null | head -1)
  OY=$(timeout 3 ros2 topic echo /odom --field pose.pose.position.y --once 2>/dev/null | head -1)
  VX=$(timeout 3 ros2 topic echo /cmd_vel --field twist.linear.x --once 2>/dev/null | head -1)
  MW=$(timeout 3 ros2 topic echo /map --field info.width --once 2>/dev/null | head -1)
  echo "t=${T}s state=${STATE:-?} odom=(${OX:-?},${OY:-?}) cmd_vel.x=${VX:-?} map_w=${MW:-?}" | tee -a "$LOGDIR/samples.txt"
done

echo "[test] === sequencer FSM transitions ===" | tee -a "$LOGDIR/test.log"
grep -E "\[FSM\]|scan triggered|scan #|reconnect|Scan #|complete|ACTION_" "$LOGDIR/active_mapping.log" | tee -a "$LOGDIR/test.log"
echo "[test] FINISHED" | tee -a "$LOGDIR/test.log"
