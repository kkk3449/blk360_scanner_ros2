#!/bin/bash
# stop the semantic stack (real or Gazebo) + Isaac KG twin. Run as a FILE (bash scripts/shutdown_all.sh): Isaac KG twin, bridge, mediator, BT, UI, KG watcher, mock, Gazebo leftovers
cd /home/caselab/blk360_ros2_ws
for p in "isaacsim_kg[_]twin" "real_robot_nav[_]bridge" "mission_bt[_]cpp" "mediator[_]server" "ui[_]server" "kg_to[_]usd" "ammr_pose[_]mock" "nav2[_]amcl" "nav2_map[_]server" "lifecycle[_]manager"; do
  pkill -f "$p" 2>/dev/null
done
bash scripts/kill_all.sh >/dev/null 2>&1
sleep 6
for p in "isaacsim_kg[_]twin" "real_robot_nav[_]bridge" "mission_bt[_]cpp" "mediator[_]server" "ui[_]server" "kg_to[_]usd"; do
  pgrep -f "$p" >/dev/null && pkill -9 -f "$p" 2>/dev/null
done
sleep 2
echo "remaining: $(pgrep -af 'isaacsim_kg[_]twin|real_robot_nav[_]bridge|mission_bt[_]cpp|mediator[_]server|ui[_]server|kg_to[_]usd|gz sim|nav2[_]amcl' | grep -v pgrep | wc -l)"
nvidia-smi --query-gpu=memory.used --format=csv,noheader
