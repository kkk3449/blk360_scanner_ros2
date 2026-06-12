#!/usr/bin/env bash
# Single coverage-aware (R=4) run, separate output dir, to capture a 4-scan run
# for the dissertation figure. Mirrors ablation.sh for one config.
set -u

WS=/home/caselab/blk360_ros2_ws
RUNS="$HOME/blk360_runs"
OUT="$HOME/blk360_visrun"
LOG="$WS/scripts/last_test/run_vis.log"
mkdir -p "$OUT" "$RUNS" "$(dirname "$LOG")"
: > "$LOG"

NAME=vis_R10
INTERVAL=2.0
R=10.0
SUPP=true

GUI=false
USE_RVIZ=false
MOCK_SCAN=1.0
MOCK_DL=2.0
STALL_TIMEOUT=90.0     # match the ablation table (R4 = 3 scans) for consistency
RUN_TIMEOUT=900
SIM_WARMUP=18

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
set -u

kill_stack() { bash "$WS/scripts/kill_all.sh" >>"$LOG" 2>&1; sleep 4; }

is_complete() {
  timeout 5 ros2 topic echo /exploration_complete std_msgs/msg/Bool --once \
    --qos-durability transient_local --qos-reliability reliable 2>/dev/null \
    | grep -q "data: true"
}

log "===== RUN $NAME  (interval=$INTERVAL  R=$R  stall=$STALL_TIMEOUT) ====="
kill_stack

log "launching sim (headless)..."
nohup ros2 launch blk360_bringup testroom_sim.launch.py gui:=$GUI >>"$LOG" 2>&1 &
sleep "$SIM_WARMUP"

log "launching active_mapping (mock scanner)..."
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
  if is_complete; then log "  -> exploration_complete=true after $((SECONDS-t0))s"; sleep 6; break; fi
  sleep 12
done
(( SECONDS - t0 >= RUN_TIMEOUT )) && log "  -> TIMEOUT"

kill "$TRAJ_PID" 2>/dev/null
log "trajectory logged -> $OUT/traj.csv ($(wc -l < "$OUT/traj.csv" 2>/dev/null || echo 0) points)"

log "saving SLAM map..."
for attempt in 1 2 3; do
  if timeout 40 ros2 run nav2_map_server map_saver_cli -f "$OUT/map_${NAME}" \
      --ros-args -p map_subscribe_transient_local:=true -p use_sim_time:=true \
      >>"$LOG" 2>&1; then
    log "  map -> $OUT/map_${NAME}.pgm (attempt $attempt)"; break
  fi
  log "  map save attempt $attempt failed, retrying..."; sleep 5
done

pkill -INT -f stop_scan_sequencer 2>/dev/null
sleep 6

newest=$(ls -1t "$RUNS"/run_*.json 2>/dev/null | head -1)
if [ -n "$newest" ]; then
  cp "$newest" "$OUT/run_${NAME}.json"
  "$WS/.e57venv/bin/python" -c "import json,sys; p=sys.argv[1]; d=json.load(open(p)); \
d['config_name']=sys.argv[2]; json.dump(d,open(p,'w'),indent=2)" "$OUT/run_${NAME}.json" "$NAME"
  scans=$("$WS/.e57venv/bin/python" -c "import json; print(len(json.load(open('$OUT/run_${NAME}.json'))['scan_positions']))")
  log "  collected -> $OUT/run_${NAME}.json  (scans=$scans)"
else
  log "  WARNING: no run JSON produced"
fi
kill_stack
log "RUN4SCAN_DONE"
