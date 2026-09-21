#!/bin/bash
# VDA 5050 stack bring-up: console (EMCS role) <-> MQTT broker <-> AGV adapter
#
#   scripts/bringup_vda5050.sh test            # local mock AGV, domain 77, console :8089
#   scripts/bringup_vda5050.sh real [--map-offset X Y YAW_DEG]   # real AMMR, domain 56, console :8080
#   NO_ISAAC=1 scripts/bringup_vda5050.sh test # skip the Isaac twin
#   scripts/bringup_vda5050.sh stop            # kill everything started here (+ Isaac)
#
# Processes (all logged to $LOG):
#   mosquitto (docker ammr-mqtt, :1883)
#   vda5050_agv_adapter   AGV side: order/instantActions -> /ammr/goal_pose, /ammr/state -> state
#   vda5050_master        console side: navigate_to_pose/kg_goal_pose/kg_initialpose/semantic_stop -> MQTT
#   mediator_server + mission_bt_cpp + ui_server (+ kg_to_usd watcher in real mode)
#   Isaac twin in VDA mode (mirrors the AGV from MQTT state, cone -> VDA order)
set +u
MODE="${1:-test}"; shift || true
WS=/home/caselab/blk360_ros2_ws
LOG=${BRINGUP_LOG_DIR:-$HOME/bringup_logs}; mkdir -p "$LOG"
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

stop_all() {
  # bracket patterns only: a pattern must never match this script's own command line
  for p in "isaacsim_kg_twi[n]" "ui_serve[r]" "mission_bt_cp[p]" "mediator_serve[r]" \
           "vda5050_maste[r]" "vda5050_agv_adapte[r]" "ammr_pose_moc[k]" "kg_to_us[d]"; do
    pkill -f "$p" 2>/dev/null
  done
}

if [ "$MODE" = stop ]; then stop_all; echo "stopped"; exit 0; fi

echo "== 0/6 stop previous stack =="
stop_all; sleep 2

echo "== 1/6 MQTT broker (docker ammr-mqtt :1883) =="
if ! docker ps --format '{{.Names}}' | grep -qx ammr-mqtt; then
  docker start ammr-mqtt >/dev/null 2>&1 || docker run -d --name ammr-mqtt --restart unless-stopped \
    -p 1883:1883 -v $HOME/ammr_twin/mosquitto/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro \
    eclipse-mosquitto:2 >/dev/null
  sleep 2
fi
docker ps --format '{{.Names}} {{.Status}}' | grep ammr-mqtt

if [ "$MODE" = real ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=56
  export CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"
  PORT=8080; HEAD=180; OFFSET_ARGS=("$@")          # default: ~/ammr_twin/robot_map_offset.json
  echo "== 2/6 robot handshake =="
  bash $WS/scripts/ammr_net_check.sh || { echo "HANDSHAKE FAILED"; exit 1; }
else
  export ROS_DOMAIN_ID=77
  PORT=8089; HEAD=0; OFFSET_ARGS=(--map-offset 0 0 0)
  echo "== 2/6 mock AGV (domain 77) =="
  setsid nohup python3 $WS/scripts/ammr_pose_mock.py --start 0.7365 0.7815 0.809 \
      --plan-map $HOME/ammr_twin/map_vis_n2_1.yaml --vmax 1.1 --accel 0.5 > $LOG/mock.log 2>&1 &
  rm -f /tmp/mapreg_test/offset.json; mkdir -p /tmp/mapreg_test
  UI_EXTRA="-p offset_path:=/tmp/mapreg_test/offset.json"
fi

echo "== 3/6 VDA 5050 AGV adapter + master control =="
setsid nohup ros2 run semantic_nav_bt vda5050_agv_adapter --serial ammr20 --map-id map --stop-mode hold > $LOG/agv_adapter.log 2>&1 &
sleep 2
setsid nohup ros2 run semantic_nav_bt vda5050_master --serial ammr20 --map-id map --heading-offset $HEAD "${OFFSET_ARGS[@]}" > $LOG/master.log 2>&1 &
sleep 3
timeout 6 ros2 topic echo /kg_robot_pose --once --field pose.pose.position >/dev/null 2>&1 \
  && echo "  /kg_robot_pose OK (AGV state via MQTT)" || echo "  WARN: no AGV state yet"

echo "== 4/6 mediator + mission_bt_cpp =="
setsid nohup ros2 run semantic_nav_bt mediator_server --ros-args -p gated:=true > $LOG/mediator.log 2>&1 &
sleep 3
setsid nohup ros2 run semantic_nav_bt_cpp mission_bt_cpp --ros-args -r amcl_pose:=/kg_robot_pose > $LOG/btcpp.log 2>&1 &
sleep 3

echo "== 5/6 console :$PORT =="
if [ "$MODE" = real ]; then
  ( cd /home/caselab/Downloads/Cyclone360_data/blk360_seg && setsid nohup .venv/bin/python scripts/kg_to_usd.py --watch > $LOG/kg2usd.log 2>&1 & )
fi
setsid nohup ros2 run semantic_nav_bt ui_server --ros-args -p port:=$PORT $UI_EXTRA \
  -r amcl_pose:=/kg_robot_pose -r goal_pose:=/kg_goal_pose -r initialpose:=/kg_initialpose > $LOG/ui.log 2>&1 &
sleep 4
curl -s -m 3 http://localhost:$PORT/api/state | python3 -c "import sys,json; d=json.load(sys.stdin); print('  console up; VDA:', (d.get('vda5050') or {}).get('agvs'))"

if [ -z "${NO_ISAAC:-}" ]; then
  echo "== 6/6 Isaac twin (VDA 5050 mode) =="
  ISAAC_ARGS=(kg --pose-source amcl --goal-topic /kg_goal_pose --amcl-topic /kg_robot_pose
              --vda5050-broker 127.0.0.1:1883 --vda-serial ammr20 --heading-offset $HEAD
              --sim-speed 1.1 --sim-accel 0.5)
  [ "$MODE" = test ] && ISAAC_ARGS+=(--kg-offset 0 0 0)
  ( cd $WS && unset PYTHONPATH && DISPLAY=${DISPLAY:-:1} setsid nohup scripts/run_isaac_ammr.sh "${ISAAC_ARGS[@]}" > $LOG/isaac_kg.log 2>&1 & )
fi
echo "done. console: http://localhost:$PORT   logs: $LOG   MQTT: uagv/v2/caselab/ammr20/{order,instantActions,state,visualization,connection}"
