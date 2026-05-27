# blk360_scanner

ROS2 (Jazzy) package that triggers a **Leica BLK360** colorized point-cloud scan when a
specific message ("sequence") is published to a topic. The scan logic is ported from the
vendor sample `22-colorize-pc-ldr` and linked directly against the prebuilt `libBLK360.so`
(approach **B** — library integrated into the node, not a wrapper around the binary).

A full scan takes about **2 minutes 29 seconds** and saves a colorized point cloud as
`pointcloud_<scanId>.csv`.

## Interface

| Direction | Topic | Type | Meaning |
|-----------|-------|------|---------|
| sub | `/blk360/scan_trigger` | `std_msgs/String` | Starts a scan when `data` equals `trigger_command` (default `"scan"`) |
| pub | `/blk360/scan_status` | `std_msgs/String` | `IDLE` / `SCANNING` / `DONE` / `ERROR: <msg>` |
| pub | `/blk360/scan_progress` | `std_msgs/String` | Human-readable progress lines |

Overlapping triggers are ignored while a scan is running (the device serves one scan at a time).

### Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `device_address` | `192.168.10.90:8081` | BLK360 address:port |
| `trigger_command` | `scan` | The "sequence" string that starts a scan |
| `output_dir` | `.` | Where the CSV is written |
| `point_cloud_density` | `medium` | `low` / `medium` / `high` |
| `panorama_mode` | `ldr` | `ldr` / `hdr` |

## Requirements

- **x86-64 Linux** — `third_party/lib/libBLK360.so` is a prebuilt x86-64 binary; it will **not**
  run on ARM (Apple Silicon, Raspberry Pi, Jetson).
- ROS2 (developed on **Jazzy**).
- Network reachability to the BLK360 at `device_address`.

## Build

```bash
cd ~/blk360_ros2_ws
colcon build
source install/setup.bash
```

> No path editing is needed after cloning: the BLK360 headers and `.so` are vendored under
> `src/blk360_scanner/third_party/`, and the `.so` is resolved at runtime via `$ORIGIN`.

## Run

```bash
# Terminal 1 — start the node (optionally override params)
ros2 launch blk360_scanner scan.launch.py output_dir:=$HOME/scans
#   or: ros2 run blk360_scanner scan_node

# Terminal 2 — send the trigger "sequence"
ros2 topic pub --once /blk360/scan_trigger std_msgs/msg/String "{data: 'scan'}"

# Watch progress / result
ros2 topic echo /blk360/scan_progress
ros2 topic echo /blk360/scan_status
```

## Moving to another machine (git workflow)

`build/`, `install/`, `log/` and `*.csv` are git-ignored. On the laptop:

```bash
git clone <repo> ~/blk360_ros2_ws
cd ~/blk360_ros2_ws
colcon build && source install/setup.bash
```

That's it — no paths to fix.

> **Note:** `libBLK360.so` is proprietary Leica software vendored into this repo for
> portability. Keep the repository **private** unless you have permission to redistribute it.
