#!/bin/bash
# Full stack bring-up after reboot: Gazebo (visn2_room) -> AMCL localization
# -> nav2 -> mission_bt (rev6 gated) -> ui_server. All setsid nohup.
SCRATCH=${BRINGUP_LOG_DIR:-$HOME/bringup_logs}
mkdir -p "$SCRATCH"
source /opt/ros/jazzy/setup.bash
source /home/caselab/blk360_ros2_ws/install/setup.bash

echo "== 1/5 Gazebo (AMMR proxy) =="
WORLD=$(ros2 pkg prefix blk360_bringup)/share/blk360_bringup/worlds/visn2_room.world
DISPLAY=:1 setsid nohup ros2 launch blk360_bringup ammr_sim.launch.py \
  world:=$WORLD x_pose:=-2.0 y_pose:=0.5 gui:=false \
  > $SCRATCH/sim.log 2>&1 &
for i in $(seq 1 15); do
  timeout 4 ros2 topic echo /odom --once > /dev/null 2>&1 && break
  sleep 4
done
timeout 4 ros2 topic echo /odom --once > /dev/null 2>&1 && echo ODOM_OK || { echo ODOM_FAIL; exit 1; }

echo "== 2/5 localization =="
setsid nohup ros2 launch nav2_bringup localization_launch.py \
  map:=/home/caselab/ammr_twin/map_vis_n2_1.yaml use_sim_time:=true \
  params_file:=/home/caselab/blk360_ros2_ws/src/blk360_bringup/config/nav2/nav2_params.yaml \
  autostart:=true > $SCRATCH/loc.log 2>&1 &
sleep 14
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: 0.5}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"
sleep 4

echo "== 3/5 nav2 =="
setsid nohup ros2 launch blk360_bringup nav2.launch.py use_sim_time:=true \
  params_file:=/home/caselab/blk360_ros2_ws/src/blk360_bringup/config/nav2/nav2_params_ammr.yaml \
  > $SCRATCH/nav.log 2>&1 &
for i in $(seq 1 20); do
  grep -q "Managed nodes are active" $SCRATCH/nav.log && break
  sleep 3
done
grep -q "Managed nodes are active" $SCRATCH/nav.log && echo NAV_OK || echo NAV_PENDING

echo "== 3.5 overhead camera bridge =="
setsid nohup ros2 run ros_gz_image image_bridge /overhead/image_raw \
  > $SCRATCH/overhead_bridge.log 2>&1 &
sleep 2

echo "== 4/5 mediator + mission_bt_cpp (rev6 gated, Groot2 port 1667) =="
setsid nohup ros2 run semantic_nav_bt mediator_server --ros-args \
  -p gated:=true > $SCRATCH/mediator.log 2>&1 &
sleep 3
setsid nohup ros2 run semantic_nav_bt_cpp mission_bt_cpp \
  > $SCRATCH/btcpp.log 2>&1 &
sleep 5

echo "== 4.5 KG->USD watcher (Isaac scene follows the semantic DB) =="
cd /home/caselab/Downloads/Cyclone360_data/blk360_seg && setsid nohup .venv/bin/python scripts/kg_to_usd.py --watch > $SCRATCH/kg2usd.log 2>&1 &
cd /home/caselab/blk360_ros2_ws

echo "== 5/5 ui_server =="
setsid nohup ros2 run semantic_nav_bt ui_server > $SCRATCH/ui.log 2>&1 &
sleep 4
curl -s -m 3 http://localhost:8080/api/state | head -c 200
echo
echo BRINGUP_DONE
