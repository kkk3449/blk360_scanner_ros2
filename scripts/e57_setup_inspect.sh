#!/usr/bin/env bash
# Set up an isolated venv with pye57 and inspect the testroom e57 structure.
WS=/home/caselab/blk360_ros2_ws
E57=/home/caselab/Downloads/Cyclone360_data/testroom260601.e57
VENV=$WS/.e57venv
OUT=$WS/scripts/last_test/e57_inspect.log; mkdir -p "$(dirname "$OUT")"; : > "$OUT"
log(){ echo "$@" | tee -a "$OUT"; }

if [ ! -d "$VENV" ]; then
  log "[e57] creating venv..."
  python3 -m venv "$VENV" >>"$OUT" 2>&1
fi
source "$VENV/bin/activate"
log "[e57] installing pye57 + numpy (may take a minute)..."
pip install --quiet --upgrade pip >>"$OUT" 2>&1
pip install --quiet pye57 numpy >>"$OUT" 2>&1 && log "[e57] pip ok" || log "[e57] pip FAILED"

python3 - "$E57" >>"$OUT" 2>&1 <<'PY'
import sys, numpy as np, pye57
path = sys.argv[1]
e = pye57.E57(path)
n = e.scan_count
print(f"[e57] scan_count = {n}")
hdr = e.get_header(0)
print(f"[e57] scan0 point_count = {hdr.point_count}")
try:
    print(f"[e57] scan0 fields = {hdr.point_fields}")
except Exception as ex:
    print(f"[e57] fields err: {ex}")
# Read a downsampled subset to get bounds + z-histogram without loading everything.
data = e.read_scan(0, ignore_missing_fields=True, intensity=False, colors=False)
x = np.asarray(data['cartesianX']); y = np.asarray(data['cartesianY']); z = np.asarray(data['cartesianZ'])
m = np.isfinite(x)&np.isfinite(y)&np.isfinite(z)
x,y,z = x[m],y[m],z[m]
print(f"[e57] scan0 finite points = {x.size}")
print(f"[e57] X range [{x.min():.2f}, {x.max():.2f}]  span={x.max()-x.min():.2f} m")
print(f"[e57] Y range [{y.min():.2f}, {y.max():.2f}]  span={y.max()-y.min():.2f} m")
print(f"[e57] Z range [{z.min():.2f}, {z.max():.2f}]  span={z.max()-z.min():.2f} m")
hist,edges = np.histogram(z, bins=20)
print("[e57] Z histogram (height distribution):")
for c,(a,b) in zip(hist, zip(edges[:-1],edges[1:])):
    print(f"   z[{a:6.2f},{b:6.2f}] : {c}")
print("[e57] INSPECT_DONE")
PY
echo "[e57] script end" >> "$OUT"
