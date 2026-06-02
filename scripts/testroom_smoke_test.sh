#!/usr/bin/env bash
# Step 7: run the full active-mapping stack in the custom testroom world.
WS=/home/caselab/blk360_ros2_ws
DURATION="${1:-150}"
L="$WS/scripts/last_test"; mkdir -p "$L"
rm -f "$L"/tr_*.log "$L"/tr_samples.txt
bash "$WS/scripts/kill_all.sh" >/dev/null 2>&1; sleep 2

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle ROS_DOMAIN_ID=77

cleanup(){ bash "$WS/scripts/kill_all.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM

echo "[tr] launching testroom world (headless)..." | tee "$L/tr.log"
setsid ros2 launch blk360_bringup testroom_sim.launch.py gui:=false > "$L/tr_sim.log" 2>&1 &
for i in $(seq 1 60); do
  timeout 3 ros2 topic echo /odom nav_msgs/msg/Odometry --once >/dev/null 2>&1 && { echo "[tr] /odom up ${i}s" | tee -a "$L/tr.log"; break; }
  sleep 1
done
# Check the LiDAR actually sees the testroom walls (finite ranges present).
NR=$(timeout 5 ros2 topic echo /scan --field ranges --once 2>/dev/null | tr ',' '\n' | grep -E '^[ ]*[0-9]' | grep -vE 'inf|\.000000000' | wc -l)
echo "[tr] /scan finite returns (sees walls): $NR" | tee -a "$L/tr.log"

echo "[tr] launching active_mapping (mock scanner, interval=2.0m, fail_first=1)..." | tee -a "$L/tr.log"
setsid ros2 launch blk360_bringup active_mapping.launch.py \
  use_sim_time:=true use_rviz:=false use_mock_scanner:=true \
  scan_interval_m:=2.0 fail_first_n_scans:=1 mock_scan_duration_s:=3.0 \
  > "$L/tr_active.log" 2>&1 &

echo "[tr] running ${DURATION}s..." | tee -a "$L/tr.log"
for i in $(seq 1 $((DURATION/5))); do
  sleep 5; T=$((i*5))
  ST=$(timeout 3 ros2 topic echo /blk360_stop_scan/state --field data --once 2>/dev/null | head -1)
  OX=$(timeout 3 ros2 topic echo /odom --field pose.pose.position.x --once 2>/dev/null | head -1)
  OY=$(timeout 3 ros2 topic echo /odom --field pose.pose.position.y --once 2>/dev/null | head -1)
  MW=$(timeout 3 ros2 topic echo /map --field info.width --once 2>/dev/null | head -1)
  echo "t=${T}s state=${ST:-?} odom=(${OX:-?},${OY:-?}) map_w=${MW:-?}" | tee -a "$L/tr_samples.txt"
done

echo "[tr] === FSM transitions ===" | tee -a "$L/tr.log"
grep -E "\[FSM\]|Scan #|reconnect|Travelled|ACTION_" "$L/tr_active.log" | tee -a "$L/tr.log"
echo "[tr] FINISHED" | tee -a "$L/tr.log"
