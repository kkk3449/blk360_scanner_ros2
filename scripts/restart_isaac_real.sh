#!/bin/bash
# restart the Isaac KG twin in real-robot mode (pose from /amcl_pose, cone -> /kg_goal_pose)
cd /home/caselab/blk360_ros2_ws
pkill -f "isaacsim_kg[_]twin" 2>/dev/null; sleep 5
pgrep -f "isaacsim_kg[_]twin" >/dev/null && pkill -9 -f "isaacsim_kg[_]twin"; sleep 2
AMMR_REAL=1 DISPLAY=:1 setsid nohup scripts/run_isaac_ammr.sh kg --pose-source amcl --goal-topic /kg_goal_pose --amcl-topic /kg_robot_pose > ~/bringup_logs/isaac_kg.log 2>&1 < /dev/null &
echo "isaac restarting (~80 s)"
