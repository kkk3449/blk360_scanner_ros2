# Phase 4 — TurtleBot3 digital twin in Isaac Sim + ROS 2 / Nav2

Goal: drive a TurtleBot3 in the testroom **inside Isaac Sim**, with our existing
ROS 2 mapping/navigation stack (Cartographer + Nav2 + frontier + BLK stop-scan)
unchanged — Isaac Sim simply replaces Gazebo as the simulator.

> Status: scaffold. The ROS 2 side is reused as-is; the Isaac Sim side
> (`occ_to_usd_world.py` collidable world + `isaacsim_phase4_setup.py` bridge)
> must be run on the user's Isaac Sim machine and may need version tweaks
> (targeting Isaac Sim 4.5 / 5.0). It was not run from the dev box (no GPU/Isaac).

## Why nothing changes on the ROS 2 side
`active_mapping.launch.py` / `exploration.launch.py` never launch the simulator —
they only consume `/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`. In Gazebo,
`tb3_sim.launch.py` provides those. In Phase 4, **Isaac Sim provides them**, so we
just don't launch `tb3_sim` / `testroom_sim`.

## The contract Isaac Sim must satisfy (matches the Cartographer/Nav2 config)
| Item | Value |
|---|---|
| `/clock` | sim time (use_sim_time:=true everywhere) |
| `/scan` | sensor_msgs/LaserScan, single 2D lidar |
| `/odom` | nav_msgs/Odometry |
| `/cmd_vel` | robot subscribes — **see TwistStamped note** |
| TF `odom -> base_footprint` | from Isaac odometry (`provide_odom_frame=false` in cartographer; sim owns it) |
| TF `base_footprint -> base_scan` | static, from the robot; lidar frame = `base_scan` |
| tracking_frame | `base_footprint` (cartographer_2d.lua) |

**TwistStamped gotcha (important).** Our Gazebo bridge used
`geometry_msgs/TwistStamped` on `/cmd_vel`. Isaac's ROS2 Subscribe-Twist node
defaults to `geometry_msgs/Twist`. Pick one:
- set Nav2 controller `enable_stamped_cmd_vel: false` (publish plain Twist), **or**
- run a `twist_stamper` relay, **or**
- use a TwistStamped-capable subscribe node in the OmniGraph.
Mismatch = robot receives no velocity and never moves.

## Run sequence
1. **Build the collidable world** (once, on the dev box or Isaac box):
   ```
   python3 scripts/occ_to_usd_world.py \
       src/blk360_bringup/maps/testroom.pgm src/blk360_bringup/maps/testroom.yaml \
       testroom_world.usda --wall-height 2.5
   ```
   → USD with 312 wall-box colliders + ground (navigable). Optionally also load
   `tosm_scene_with_walls.usda` as a *visual* semantic overlay (no collision).

2. **Isaac Sim**: run the setup script (loads world, spawns TurtleBot3, builds the
   ROS 2 OmniGraph bridge, presses Play):
   ```
   <isaac-sim>/python.sh scripts/isaacsim_phase4_setup.py --world testroom_world.usda
   ```
   (or paste the bridge-building part into the Script Editor over an already-open
   stage). Verify topics: `ros2 topic list` should show /scan /odom /clock /tf.

3. **ROS 2 stack** (separate terminal, `use_sim_time:=true`):
   ```
   ros2 launch blk360_bringup active_mapping.launch.py \
       use_sim_time:=true use_rviz:=true \
       coverage_model:=visibility scan_coverage_radius_m:=6.0 \
       min_new_visible_ratio:=0.30 min_new_visible_area_m2:=5.0 \
       stall_timeout_s:=120.0
   ```
   Cartographer builds the map live, frontier drives the robot, the stop-scan
   sequencer triggers (mock) BLK scans — identical to the Gazebo runs.

## Frame alignment (semantic overlay)
Navigation runs in the occupancy-map frame (Cartographer's runtime map). The
semantic point-cloud USD (`tosm_scene_*`) is in the E57 object frame. To overlay
them, apply the known static transform between the two as a parent Xform on the
overlay (translation/yaw); they need not be aligned for navigation itself.

## Validation milestones (do in order)
1. Topics present + `/clock` ticking, robot visible in Isaac.
2. Teleop: `ros2 run teleop_twist_keyboard ...` moves the robot (confirms cmd_vel).
3. RViz shows `/scan` hitting the walls + `/odom` moving.
4. Cartographer `/map` grows as the robot drives.
5. Full `active_mapping` run completes with a stop-scan summary.

## Next: real robot
Same ROS 2 stack; swap Isaac Sim for the real TurtleBot3 (bringup publishes
`/scan` `/odom` `/cmd_vel` `/tf`) and the real BLK360 driver for the mock scanner.
`use_sim_time:=false`.
