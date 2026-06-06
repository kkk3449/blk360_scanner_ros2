#!/usr/bin/env bash
# Ablation sweep for the coverage-aware scan-stop.
#
# Runs the testroom sim HEADLESS with the MOCK scanner for several scan-stop
# configs, waits for each run to finish (latched /exploration_complete == true),
# collects that run's JSON record, then aggregates everything with metrics.py.
#
# Each config is a full exploration run (minutes). Run it from a terminal with a
# sourced ROS env; it self-sources too:
#     bash scripts/ablation.sh
# Watch progress:  tail -f scripts/last_test/ablation.log
#
# Config columns:  name | scan_interval_m | scan_coverage_radius_m
#   - scan_coverage_radius_m = 0  -> coverage skip disabled = PURE DISTANCE trigger
set -u

WS=/home/caselab/blk360_ros2_ws
RUNS="$HOME/blk360_runs"
OUT="$HOME/blk360_ablation"
LOG="$WS/scripts/last_test/ablation.log"
mkdir -p "$OUT" "$RUNS" "$(dirname "$LOG")"
: > "$LOG"

# name | scan_interval_m | scan_coverage_radius_m | frontier_suppression_enabled
CONFIGS=(
  "dist_only_3m|3.0|0.0|true"
  "cov_R3|2.0|3.0|true"
  "cov_R4|2.0|4.0|true"
  "cov_R5|2.0|5.0|true"
  "cov_R6|2.0|6.0|true"
  "cov_R4_nosupp|2.0|4.0|false"
)

GUI=false
USE_RVIZ=false
MOCK_SCAN=1.0          # fast mock capture (s)
MOCK_DL=2.0            # fast mock download (s)
STALL_TIMEOUT=90.0     # end via stall if frontiers don't fully exhaust
RUN_TIMEOUT=600        # hard cap per config (s) -> fail fast if it won't finish
SIM_WARMUP=18          # s to let Gazebo + the robot come up before active_mapping

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

# ROS setup scripts reference unset vars; sourcing them under `set -u` exits the
# whole script. Disable -u just for the sourcing.
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
set -u

kill_stack() { bash "$WS/scripts/kill_all.sh" >>"$LOG" 2>&1; sleep 4; }

is_complete() {
  # Read the latched Bool with matching QoS; print 1 if data: true.
  timeout 5 ros2 topic echo /exploration_complete std_msgs/msg/Bool --once \
    --qos-durability transient_local --qos-reliability reliable 2>/dev/null \
    | grep -q "data: true"
}

for cfg in "${CONFIGS[@]}"; do
  IFS='|' read -r name interval R supp <<< "$cfg"
  log "===== CONFIG $name  (interval=$interval  R=$R  suppression=$supp) ====="
  kill_stack

  log "launching sim (headless)..."
  nohup ros2 launch blk360_bringup testroom_sim.launch.py gui:=$GUI >>"$LOG" 2>&1 &
  sleep "$SIM_WARMUP"

  log "launching active_mapping (mock scanner)..."
  nohup ros2 launch blk360_bringup active_mapping.launch.py \
      use_sim_time:=true use_rviz:=$USE_RVIZ \
      scan_interval_m:=$interval scan_coverage_radius_m:=$R \
      frontier_suppression_enabled:=$supp \
      mock_scan_duration_s:=$MOCK_SCAN mock_download_duration_s:=$MOCK_DL \
      stall_timeout_s:=$STALL_TIMEOUT >>"$LOG" 2>&1 &

  log "waiting for completion (max ${RUN_TIMEOUT}s)..."
  t0=$SECONDS
  while (( SECONDS - t0 < RUN_TIMEOUT )); do
    if is_complete; then log "  -> exploration_complete=true after $((SECONDS-t0))s"; sleep 6; break; fi
    sleep 12
  done
  (( SECONDS - t0 >= RUN_TIMEOUT )) && log "  -> TIMEOUT (collecting whatever was written)"

  # Save the final SLAM map (for GT comparison) while /map is still alive.
  log "saving SLAM map..."
  timeout 40 ros2 run nav2_map_server map_saver_cli -f "$OUT/map_${name}" \
      --ros-args -p map_subscribe_transient_local:=true -p use_sim_time:=true \
      >>"$LOG" 2>&1 && log "  map -> $OUT/map_${name}.pgm" || log "  map save failed"

  # Gracefully SIGINT the sequencer so it finalizes and writes its run JSON even
  # on timeout (a plain SIGKILL would lose the record). No-op if already DONE.
  pkill -INT -f stop_scan_sequencer 2>/dev/null
  sleep 6

  newest=$(ls -1t "$RUNS"/run_*.json 2>/dev/null | head -1)
  if [ -n "$newest" ]; then
    cp "$newest" "$OUT/run_${name}.json"
    # tag the record with the config so metrics.py can report it
    "$WS/.e57venv/bin/python" -c "import json,sys; p=sys.argv[1]; d=json.load(open(p)); \
d['config_name']=sys.argv[2]; d['frontier_suppression_enabled']=(sys.argv[3]=='true'); \
json.dump(d,open(p,'w'),indent=2)" "$OUT/run_${name}.json" "$name" "$supp"
    log "  collected -> $OUT/run_${name}.json ($(basename "$newest"))"
  else
    log "  WARNING: no run JSON produced for $name"
  fi
  kill_stack
done

log "===== aggregating with metrics.py ====="
"$WS/.e57venv/bin/python" "$WS/scripts/metrics.py" --runs-dir "$OUT" \
    --out "$OUT/metrics" --summary-log /dev/null 2>&1 | tee -a "$LOG"
log "ABLATION_DONE  (table: $OUT/metrics/runs_metrics.csv, figures: $OUT/metrics/*.png)"
