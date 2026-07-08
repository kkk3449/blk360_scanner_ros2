#!/usr/bin/env bash
# Real Gazebo capture run for the test-room field experiment: TB3/Redbot base
# autonomously explores the testroom.world under frontier exploration + Nav2 +
# Cartographer, with the visibility stop-and-scan policy (R=6). Captures the
# real SLAM map, the real trajectory, and the real scan positions, which the
# offline disk-vs-visibility replay (render_testroom_field.py --gz) then uses.
set -u
WS=/home/caselab/blk360_ros2_ws
RUNS="$HOME/blk360_runs"
OUT="$HOME/blk360_fieldsim_gz"
LOG="$WS/scripts/last_test/run_field.log"
mkdir -p "$OUT" "$RUNS" "$(dirname "$LOG")"
: > "$LOG"

NAME=field
INTERVAL=2.0
R=6.0
SUPP=true
GUI=false
USE_RVIZ=false
MOCK_SCAN=1.0
MOCK_DL=2.0
STALL_TIMEOUT=90.0
RUN_TIMEOUT=600
SIM_WARMUP=18

log(){ echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
set -u
export TURTLEBOT3_MODEL=waffle ROS_DOMAIN_ID=77

kill_stack(){ bash "$WS/scripts/kill_all.sh" >>"$LOG" 2>&1; sleep 4; }
is_complete(){
  timeout 5 ros2 topic echo /exploration_complete std_msgs/msg/Bool --once \
    --qos-durability transient_local --qos-reliability reliable 2>/dev/null \
    | grep -q "data: true"
}

log "===== TESTROOM FIELD RUN  (interval=$INTERVAL R=$R visibility) ====="
kill_stack

log "launching testroom sim (headless)..."
nohup ros2 launch blk360_bringup testroom_sim.launch.py gui:=$GUI >>"$LOG" 2>&1 &
sleep "$SIM_WARMUP"

log "launching active_mapping (visibility policy)..."
nohup ros2 launch blk360_bringup active_mapping.launch.py \
    use_sim_time:=true use_rviz:=$USE_RVIZ \
    scan_interval_m:=$INTERVAL scan_coverage_radius_m:=$R \
    coverage_model:=visibility min_new_visible_ratio:=0.30 min_new_visible_area_m2:=5.0 \
    frontier_suppression_enabled:=$SUPP \
    mock_scan_duration_s:=$MOCK_SCAN mock_download_duration_s:=$MOCK_DL \
    stall_timeout_s:=$STALL_TIMEOUT >>"$LOG" 2>&1 &

sleep 3
log "starting trajectory logger..."
nohup python3 "$WS/scripts/traj_logger.py" --ros-args \
    -p use_sim_time:=true -p out:="$OUT/traj.csv" >>"$LOG" 2>&1 &
TRAJ_PID=$!

log "waiting for completion (max ${RUN_TIMEOUT}s)..."
t0=$SECONDS
while (( SECONDS - t0 < RUN_TIMEOUT )); do
  if is_complete; then log "  -> exploration_complete after $((SECONDS-t0))s"; sleep 6; break; fi
  sleep 12
done
(( SECONDS - t0 >= RUN_TIMEOUT )) && log "  -> TIMEOUT (using partial map)"

kill "$TRAJ_PID" 2>/dev/null
log "trajectory -> $OUT/traj.csv ($(wc -l < "$OUT/traj.csv" 2>/dev/null || echo 0) points)"

log "saving SLAM map..."
for a in 1 2 3; do
  if timeout 40 ros2 run nav2_map_server map_saver_cli -f "$OUT/map_${NAME}" \
      --ros-args -p map_subscribe_transient_local:=true -p use_sim_time:=true >>"$LOG" 2>&1; then
    log "  map -> $OUT/map_${NAME}.pgm (attempt $a)"; break
  fi
  log "  map save attempt $a failed, retry..."; sleep 5
done

pkill -INT -f stop_scan_sequencer 2>/dev/null
sleep 6
newest=$(ls -1t "$RUNS"/run_*.json 2>/dev/null | head -1)
if [ -n "$newest" ]; then
  cp "$newest" "$OUT/run_${NAME}.json"
  scans=$(python3 -c "import json;print(len(json.load(open('$OUT/run_${NAME}.json'))['scan_positions']))" 2>/dev/null)
  log "  run json -> $OUT/run_${NAME}.json (scans=$scans)"
else
  log "  WARNING: no run JSON produced"
fi
kill_stack
log "TESTROOM_FIELD_DONE"
