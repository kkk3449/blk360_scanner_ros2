#!/usr/bin/env bash
# N independent real Gazebo capture runs of the test-room field experiment on the
# new world (testroom_field.world), each from a different start pose, so the
# disk-vs-visibility numbers can be averaged (mean +/- sd) as in the paper.
set -u
WS=/home/caselab/blk360_ros2_ws
RUNS="$HOME/blk360_runs"
LOG="$WS/scripts/last_test/run_nruns.log"
mkdir -p "$RUNS" "$(dirname "$LOG")"
: > "$LOG"

# 5 spread start poses "x y"
SPAWNS=("1.97 0.28" "-4.03 0.62" "6.48 0.48" "-0.02 -2.52" "2.98 2.98")

INTERVAL=2.0; R=6.0; SUPP=false
MOCK_SCAN=1.0; MOCK_DL=2.0; STALL_TIMEOUT=90.0; RUN_TIMEOUT=750; SIM_WARMUP=18

log(){ echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
set -u
export TURTLEBOT3_MODEL=waffle ROS_DOMAIN_ID=77
kill_stack(){ bash "$WS/scripts/kill_all.sh" >>"$LOG" 2>&1; sleep 4; }
is_complete(){ timeout 5 ros2 topic echo /exploration_complete std_msgs/msg/Bool --once \
  --qos-durability transient_local --qos-reliability reliable 2>/dev/null | grep -q "data: true"; }

for i in "${!SPAWNS[@]}"; do
  n=$((i+1)); read -r X Y <<< "${SPAWNS[$i]}"
  OUT="$HOME/blk360_fieldsim_gz2_run${n}"; mkdir -p "$OUT"; rm -f "$OUT"/*
  log "===== RUN $n/5  spawn=($X,$Y) ====="
  kill_stack
  log "  launching sim..."
  nohup ros2 launch blk360_bringup testroom_sim.launch.py gui:=false \
    world_name:=testroom_field x_pose:=$X y_pose:=$Y >>"$LOG" 2>&1 &
  sleep "$SIM_WARMUP"
  log "  launching active_mapping (visibility R=$R)..."
  nohup ros2 launch blk360_bringup active_mapping.launch.py \
    use_sim_time:=true use_rviz:=false \
    scan_interval_m:=$INTERVAL scan_coverage_radius_m:=$R \
    coverage_model:=visibility min_new_visible_ratio:=0.30 min_new_visible_area_m2:=5.0 \
    frontier_suppression_enabled:=$SUPP \
    mock_scan_duration_s:=$MOCK_SCAN mock_download_duration_s:=$MOCK_DL \
    stall_timeout_s:=$STALL_TIMEOUT >>"$LOG" 2>&1 &
  sleep 3
  nohup python3 "$WS/scripts/traj_logger.py" --ros-args \
    -p use_sim_time:=true -p out:="$OUT/traj.csv" >>"$LOG" 2>&1 &
  TRAJ_PID=$!
  log "  exploring (max ${RUN_TIMEOUT}s)..."
  t0=$SECONDS
  while (( SECONDS - t0 < RUN_TIMEOUT )); do
    if is_complete; then log "  complete after $((SECONDS-t0))s"; sleep 6; break; fi
    sleep 12
  done
  (( SECONDS - t0 >= RUN_TIMEOUT )) && log "  TIMEOUT"
  kill "$TRAJ_PID" 2>/dev/null
  log "  traj $(wc -l < "$OUT/traj.csv" 2>/dev/null || echo 0) pts"
  for a in 1 2 3; do
    timeout 40 ros2 run nav2_map_server map_saver_cli -f "$OUT/map_field" \
      --ros-args -p map_subscribe_transient_local:=true -p use_sim_time:=true \
      >>"$LOG" 2>&1 && { log "  map saved"; break; }
    sleep 5
  done
  pkill -INT -f stop_scan_sequencer 2>/dev/null; sleep 6
  newest=$(ls -1t "$RUNS"/run_*.json 2>/dev/null | head -1)
  [ -n "$newest" ] && cp "$newest" "$OUT/run_field.json" && \
    log "  json saved (scans=$(python3 -c "import json;print(len(json.load(open('$OUT/run_field.json'))['scan_positions']))" 2>/dev/null))"
  kill_stack
done
log "ALL_NRUNS_DONE"
