# BLK360 Active Mapping

ROS 2 (Jazzy) stack that **autonomously explores an unknown indoor space and periodically
takes a Leica BLK360 colorized scan along the way**. A simulated (or real) mobile robot
builds a 2D map with SLAM, a frontier planner drives it toward unexplored space, and a
stop-scan sequencer halts the robot every few metres to fire a high-fidelity BLK360 scan —
then resumes — until the map is complete.

It runs fully in simulation (TurtleBot3 / Gazebo) and in a **hybrid** mode (simulated robot +
the real BLK360 over WiFi).

---

## 1. System overview

```
                     ┌──────────────┐   /scan,/odom,TF   ┌───────────────────┐
                     │ Robot (TB3   │ ─────────────────▶ │ Cartographer SLAM │
                     │ sim or real) │ ◀───── /cmd_vel ─── │   → /map, map→odom │
                     └──────────────┘        ▲           └─────────┬─────────┘
                            ▲                │                     │ /map
                            │ navigate_to_   │ /cmd_vel            ▼
                            │ pose (Nav2)    │           ┌───────────────────┐
                     ┌──────┴───────┐        │           │ Frontier explorer │
                     │     Nav2     │ ◀──────┘           │  picks next goal  │
                     │ (MPPI ctrl)  │ ◀───── navigate_to_pose ── (frontier_   │
                     └──────────────┘                    │   exploration_ros2)│
                            ▲                             └─────────┬─────────┘
              control_exploration (STOP/START)                     │ exploration_complete_internal
                            │                                      ▼
                     ┌──────┴────────────┐  scan_trigger  ┌───────────────────┐
                     │ stop_scan         │ ─────────────▶ │ BLK360 scanner     │
                     │ sequencer (FSM)   │ ◀ scan_status  │ (real or mock)     │
                     └───────────────────┘                └───────────────────┘
                            │  exploration_remaining / exploration_complete(Bool) / summary
                            ▼
                     ┌───────────────────┐
                     │ exploration_      │  watches /map, stops on stall
                     │ monitor           │
                     └───────────────────┘
```

**Cycle:** explore → drive `scan_interval_m` → stop → scan → resume → … → *complete* → summary.

---

## 2. Algorithms / theory

### 2.1 Frontier-based exploration

The explorer (`frontier_exploration_ros2`) works on the live occupancy grid `/map`, whose
cells are **free**, **occupied**, or **unknown**.

- A **frontier** is a connected run of *free* cells that border *unknown* space — i.e. the
  edge of the known world. Detecting these (Wavefront Frontier Detection on a smoothed/
  dilated map) yields the set of places where driving would reveal new area.
- Tiny fragments are discarded: a cluster must have at least `min_frontier_size_cells`
  cells to count. Larger values → the robot ignores nooks and finishes *roughly* sooner.
- **Goal selection** scores candidate frontiers by a cost that trades **information gain**
  (large frontiers reveal more) against **travel cost** (distance + heading change), solved
  over a short horizon (a bounded MRTSP / "traveling-salesman-over-frontiers" formulation).
  The winning frontier is dispatched to Nav2 as a `navigate_to_pose` goal.
- **Suppression**: a frontier the robot repeatedly fails to reach or makes no progress
  toward (`frontier_suppression_no_progress_timeout_s`) is temporarily suppressed, so the
  robot abandons unreachable/too-narrow spots quickly and explores elsewhere.

### 2.2 When is exploration "done"? (two criteria)

Frontier exploration has **no notion of total area**, so there is no natural progress
*percentage* — only "are there frontiers left?". Completion is therefore **binary** and is
reached two ways:

1. **Frontier exhaustion** (native). When *no* qualifying, reachable frontier remains
   (after suppression + an "escape mode" retry), the explorer fires a completion event.
   Bigger `min_frontier_size_cells` makes this trigger once the space is *roughly* covered.

2. **Stall detection** (`exploration_monitor`, no ground truth needed). The honest "are we
   done?" signal in an unknown map is: *does more driving still reveal new area?* The
   monitor watches the count of **known** cells in `/map`; if it does not grow by
   `min_progress_cells` for `stall_timeout_s` **while actively exploring**, the map has
   converged and it stops the run — regardless of how many unreachable/tiny frontiers are
   still flagged. (The stall clock is frozen during scan/download pauses so a long scan is
   never mistaken for convergence.)

The monitor also publishes a live **remaining-frontier** readout (`/exploration_remaining` =
`[clusters, cells]`) which trends toward zero — the closest honest "how much is left".

### 2.3 Stop-scan sequencing (the FSM)

The sequencer coordinates "drive a bit, then stop and scan" on top of the explorer without
ever owning Nav2 goals or talking to the BLK360 SDK directly.

```
INIT ─▶ (optional first scan) ─▶ EXPLORING ─▶ STOPPING ─▶ SCANNING ─▶ RESUMING ─┐
          ▲                          │            │           │ (CAPTURED)        │
          └──────────────────────────┘            │           └─▶ RECONNECT ─▶ ───┘ (on ERROR)
                                                   ▼
                                      wait for prior download (gate)
```

- **Distance trigger.** A scan is taken when the robot's **straight-line displacement in the
  map frame** from the *last scan pose* reaches `scan_interval_m` (default 3 m). This is net
  displacement, **not** path length — wandering in place won't trigger a scan.

- **Capture/download decoupling.** A BLK360 scan has two phases: a **measurement** (the
  device must stay still) and a long **download/colorize** (pure data transfer). The scanner
  emits `CAPTURED` when the physical measurement is done; the sequencer then **resumes
  driving immediately** while the download finishes in the background, and the robot only
  blocks if it reaches the *next* scan point before that download completes.

- **Producer–consumer gate.** Because the device serves one scan at a time, the sequencer
  gates the next scan on the previous download's `DONE` (a race-safe boolean set on
  `CAPTURED`, cleared on `DONE`/`ERROR`, with a `download_wait_timeout_s` safety net so a
  missed `DONE` can never lock it forever). This overlaps download time with driving while
  guaranteeing scans never collide.

- **Reconnect/retry.** A capture-phase `ERROR` (e.g. dropped WiFi) triggers up to
  `max_scan_retries` re-triggers (each opens a fresh device session). A background download
  error just drops that scan's data and continues.

- **Settle pauses.** `post_capture_settle_s` holds the robot briefly after a capture before
  moving; `pre_scan_settle_s` waits after the device is free before the next scan — both let
  the BLK360 stabilize.

### 2.4 SLAM + navigation

- **Cartographer** provides 2D SLAM: `/map` (occupancy grid) and the `map → odom` transform.
- **Nav2** executes goals with the **MPPI** controller (sampling-based model-predictive
  control). The costmap `inflation_radius` is the keep-away buffer around obstacles; it is
  tuned as a trade-off between passing narrow gaps (small) and not wedging on walls (large).

---

## 3. Packages

| Package | Role |
|---------|------|
| `blk360_bringup` | Launch + config: Cartographer SLAM, Nav2, frontier exploration, the testroom world/maps, and the full `active_mapping` composition. |
| `blk360_stop_scan` | The stop-scan **sequencer** FSM, the **exploration_monitor** (remaining + stall-stop), and a **mock scanner** for sim. |
| `blk360_scanner` | C++ node driving the **real BLK360** colorized-scan workflow (links `libBLK360.so`). |
| `frontier_exploration_ros2` | Vendored frontier explorer (`mertgulerx/frontier_exploration_ros2`). |

---

## 4. Key topics & services

| Topic / service | Type | Meaning |
|-----------------|------|---------|
| `/map` | `nav_msgs/OccupancyGrid` | Live SLAM map (Cartographer). |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity to the robot (must be **stamped** for the TB3 bridge). |
| `/blk360/scan_trigger` | `std_msgs/String` | `"scan"` starts a scan. |
| `/blk360/scan_status` | `std_msgs/String` | `IDLE` → `SCANNING` → `CAPTURED` → `DONE` / `ERROR: …` |
| `/blk360_stop_scan/state` | `std_msgs/String` | Sequencer FSM state. |
| `/exploration_remaining` | `std_msgs/Int32MultiArray` | `[frontier_clusters, frontier_cells]`. |
| `/exploration_complete` | `std_msgs/Bool` | **Latched** completion flag: `false` while running, `true` when done. |
| `/exploration_complete_internal` | `std_msgs/Empty` | Internal completion trigger (explorer / monitor → sequencer). |
| `/control_exploration` | `frontier_exploration_ros2/ControlExploration` | STOP/START the explorer (used around each scan). |

The end-of-run **summary** (elapsed time, scans taken, downloads completed, scan timestamps)
is logged to the sequencer console **and** appended to `~/blk360_exploration_summary.log`.

---

## 5. Key parameters

**Sequencer** (`blk360_stop_scan/config/stop_scan.yaml`):

| Parameter | Default | Notes |
|-----------|---------|-------|
| `scan_interval_m` | `3.0` | Straight-line map displacement between scans. |
| `pre_scan_settle_s` | `4.0` | Settle after device free, before next scan. |
| `post_capture_settle_s` | `5.0` | Settle after capture, before resuming. |
| `download_wait_timeout_s` | `300.0` | Cap on waiting for a prior download. |
| `max_scan_retries` | `5` | Capture retries on link loss. |

**Exploration monitor** (launch args of `active_mapping.launch.py`):

| Parameter | Default | Notes |
|-----------|---------|-------|
| `auto_stop_on_stall` | `true` | Stop when the map stops growing. |
| `stall_timeout_s` | `300.0` | No-growth duration that declares convergence. |
| `min_progress_cells` | `80` | Known-cell gain below which counts as "no progress". |

**Explorer** (`blk360_bringup/config/frontier/frontier_params.yaml`):
`min_frontier_size_cells` (20, roughness), `frontier_suppression_enabled` (true),
`frontier_suppression_no_progress_timeout_s` (12).

**Nav2** (`blk360_bringup/config/nav2/nav2_params.yaml`):
MPPI `vx_max` (0.7), `inflation_radius` (0.25).

---

## 6. Build & run

```bash
cd ~/blk360_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Simulation (mock scanner)

```bash
# 1) world + TurtleBot3
ros2 launch blk360_bringup testroom_sim.launch.py gui:=true
# 2) SLAM + Nav2 + exploration + stop-scan + monitor
ros2 launch blk360_bringup active_mapping.launch.py use_sim_time:=true
```

### Hybrid (sim robot + real BLK360 over WiFi)

```bash
ros2 launch blk360_bringup testroom_sim.launch.py gui:=true
ros2 launch blk360_bringup active_mapping.launch.py \
    use_mock_scanner:=false device_address:=192.168.10.90:8081
#   use_sim_time defaults to true
```

Watch it:

```bash
ros2 topic echo /exploration_complete      # false → true when finished
ros2 topic echo /exploration_remaining     # [clusters, cells] → toward 0
cat ~/blk360_exploration_summary.log       # after it finishes
```

---

## 7. Test world from a real scan

The testroom world is generated from a real BLK360 `.e57` scan:

1. `scripts/e57_to_map.py` — point cloud → 2D occupancy grid (`maps/testroom.pgm` + meta).
2. `scripts/occ_to_world.py` — occupancy grid → Gazebo SDF world (merged wall boxes) + a
   collision-free spawn point.
3. `scripts/seal_testroom.py` — turns the *open* scan into an **enclosed** room (seals the
   perimeter so exploration can't leak out, optionally crops to a wide footprint) and emits a
   `testroom_gt.{pgm,yaml}` **ground-truth** map (free/occupied/unknown) for registering the
   live SLAM map against later. `*_open.*` are pristine backups it re-bases from.

---

## 8. The real BLK360 scanner (standalone)

`blk360_scanner` can also be used on its own. A full colorized scan takes ~**2 min 29 s** and
saves `pointcloud_<scanId>.csv`.

| Direction | Topic | Type | Meaning |
|-----------|-------|------|---------|
| sub | `/blk360/scan_trigger` | `std_msgs/String` | Start scan when `data == trigger_command`. |
| pub | `/blk360/scan_status` | `std_msgs/String` | `IDLE`/`SCANNING`/`CAPTURED`/`DONE`/`ERROR`. |
| pub | `/blk360/scan_progress` | `std_msgs/String` | Human-readable progress. |

Params: `device_address` (`192.168.10.90:8081`), `trigger_command` (`scan`), `output_dir`,
`point_cloud_density` (`low`/`medium`/`high`), `panorama_mode` (`ldr`/`hdr`).

```bash
ros2 launch blk360_scanner scan.launch.py output_dir:=$HOME/scans
ros2 topic pub --once /blk360/scan_trigger std_msgs/msg/String "{data: 'scan'}"
```

---

## 9. Requirements & notes

- **x86-64 Linux**, ROS 2 **Jazzy**. `third_party/lib/libBLK360.so` is a prebuilt x86-64
  binary (won't run on ARM); resolved at runtime via `$ORIGIN`, no path editing after clone.
- `build/`, `install/`, `log/`, `*.csv`, and `.e57venv/` are git-ignored — on a new machine
  just `git clone` then `colcon build`.
- **Known issue:** occasionally the real BLK360 reports
  `ERROR: PointCloudColorizer_AddPanorama: Panorama must be processed` — likely the colorize
  step disturbed by driving during download (WiFi); that scan's cloud is lost but exploration
  continues.

> `libBLK360.so` is proprietary Leica software vendored for portability. Keep the repository
> **private** unless you have permission to redistribute it.
