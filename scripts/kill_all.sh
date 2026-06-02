#!/usr/bin/env bash
# Kill all sim/nav2/exploration processes. Run as a FILE so its own command line
# ('bash kill_all.sh') does not contain the patterns and thus cannot self-kill.
for p in "gz sim" "ros_gz" "parameter_bridge" "robot_state_publisher" "image_bridge" \
         "cartographer_node" "cartographer_occupancy" "controller_server" "planner_server" \
         "bt_navigator" "behavior_server" "collision_monitor" "velocity_smoother" \
         "lifecycle_manager" "route_server" "smoother_server" "waypoint_follower" \
         "docking_server" "frontier_explorer" "stop_scan_sequencer" "mock_blk360" \
         "blk360_scanner" "component_container"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 2
echo "remaining: $(pgrep -af 'gz sim|ros_gz|cartographer_node|controller_server|frontier_explorer' | grep -v pgrep | wc -l)"
