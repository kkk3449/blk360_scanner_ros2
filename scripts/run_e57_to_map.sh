#!/usr/bin/env bash
WS=/home/caselab/blk360_ros2_ws
E57=/home/caselab/Downloads/Cyclone360_data/testroom260601.e57
OUTDIR=$WS/src/blk360_bringup/maps
LOG=$WS/scripts/last_test/e57_map.log; : > "$LOG"
source "$WS/.e57venv/bin/activate"
pip install --quiet pillow >>"$LOG" 2>&1
mkdir -p "$OUTDIR"
python3 "$WS/scripts/e57_to_map.py" "$E57" "$OUTDIR" testroom \
  --res 0.05 --band 0.4 1.8 --min-pts 3 --clip 0.5 99.5 --close 1 >>"$LOG" 2>&1
echo "RUN_DONE" >> "$LOG"
