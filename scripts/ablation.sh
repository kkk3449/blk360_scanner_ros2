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

CONFIGS=(
  "dist_only_3m|3.0|0.0"
  "cov_R4|2.0|4.0"
  "cov_R5|2.0|5.0"
  "cov_R6|2.0|6.0"
)

GUI=false
USE_RVIZ=false
MOCK_SCAN=1.0          # fast mock capture (s)
MOCK_DL=2.0            # fast mock download (s)
STALL_TIMEOUT=90.0     # end via stall if frontiers don't fully exhaust
RUN_TIMEOUT=1500       # hard cap per config (s)
SIM_WARMUP=18          # s to let Gazebo + the robot come up before active_mapping

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null

kill_stack() { bash "$WS/scripts/kill_all.sh" >>"$LOG" 2>&1; sleep 4; }

is_complete() {
  # Read the latched Bool with matching QoS; print 1 if data: true.
  timeout 5 ros2 topic echo /exploration_complete std_msgs/msg/Bool --once \
    --qos-durability transient_local --qos-reliability reliable 2>/dev/null \
    | grep -q "data: true"
}

for cfg in "${CONFIGS[@]}"; do
  IFS='|' read -r name interval R <<< "$cfg"
  log "===== CONFIG $name  (scan_interval_m=$interval  scan_coverage_radius_m=$R) ====="
  kill_stack

  log "launching sim (headless)..."
  nohup ros2 launch blk360_bringup testroom_sim.launch.py gui:=$GUI >>"$LOG" 2>&1 &
  sleep "$SIM_WARMUP"

  log "launching active_mapping (mock scanner)..."
  nohup ros2 launch blk360_bringup active_mapping.launch.py \
      use_sim_time:=true use_rviz:=$USE_RVIZ \
      scan_interval_m:=$interval scan_coverage_radius_m:=$R \
      mock_scan_duration_s:=$MOCK_SCAN mock_download_duration_s:=$MOCK_DL \
      stall_timeout_s:=$STALL_TIMEOUT >>"$LOG" 2>&1 &

  log "waiting for completion (max ${RUN_TIMEOUT}s)..."
  t0=$SECONDS
  while (( SECONDS - t0 < RUN_TIMEOUT )); do
    if is_complete; then log "  -> exploration_complete=true after $((SECONDS-t0))s"; sleep 6; break; fi
    sleep 12
  done
  (( SECONDS - t0 >= RUN_TIMEOUT )) && log "  -> TIMEOUT (collecting whatever was written)"

  newest=$(ls -1t "$RUNS"/run_*.json 2>/dev/null | head -1)
  if [ -n "$newest" ]; then
    cp "$newest" "$OUT/run_${name}.json"
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
