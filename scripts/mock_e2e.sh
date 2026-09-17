#!/bin/bash
# mock end-to-end: bridge + mock robot + mediator + BT + UI on domain 0
cd /home/caselab/blk360_ros2_ws
bash scripts/kill_all.sh >/dev/null 2>&1
for p in "ui[_]server" "mediator[_]server" "mission_bt[_]cpp" "kg_to[_]usd" "isaacsim_kg[_]twin" "ammr_pose[_]mock" "real_robot_nav[_]bridge" "nav2[_]amcl" "nav2_map[_]server" "lifecycle[_]manager"; do pkill -9 -f "$p" 2>/dev/null; done
sleep 4
echo "leftover: $(pgrep -af 'gz sim|nav2_amcl|bt_navigator|ui[_]server|mediator[_]server|mission_bt[_]cpp|isaacsim_kg[_]twin' | grep -v pgrep | wc -l)"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; L=~/bringup_logs
setsid nohup python3 scripts/real_robot_nav_bridge.py --arrive-radius 0.35 > $L/mock_bridge.log 2>&1 &
setsid nohup python3 scripts/ammr_pose_mock.py > $L/mock_robot.log 2>&1 &
sleep 4
setsid nohup ros2 run semantic_nav_bt mediator_server --ros-args -p gated:=true > $L/mock_mediator.log 2>&1 &
sleep 3
setsid nohup ros2 run semantic_nav_bt_cpp mission_bt_cpp > $L/mock_btcpp.log 2>&1 &
setsid nohup ros2 run semantic_nav_bt ui_server > $L/mock_ui.log 2>&1 &
sleep 8
echo "--- amcl_pose (KG frame) from bridge:"; timeout 5 ros2 topic echo /amcl_pose --once --field pose.pose.position 2>/dev/null | head -2
echo "--- send goto_object tv via UI"; curl -s -m 3 -X POST http://localhost:8080/api/command -d '{"cmd":"goto_object","target":"tv"}'; echo
for i in $(seq 1 40); do sleep 3; ST=$(timeout 3 ros2 topic echo /semantic_status --once 2>/dev/null | grep -oE '"status":"[^"]*"'); echo "t=$((i*3))s $ST"; echo "$ST" | grep -qE "complete|idle|REFUSED|skip|unavailable" && [ $i -gt 2 ] && break; done
echo "--- bridge log"; grep -E "GOAL|ARRIVED|TIMEOUT|CALIB|canceled|Error|Traceback" $L/mock_bridge.log | tail -6
echo "--- mock log"; grep -E "goal received|arrived" $L/mock_robot.log | tail -3
echo "--- bt log"; grep -vE "snapshot" $L/mock_btcpp.log | tail -4 | cut -c1-140
