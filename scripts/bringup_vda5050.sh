#!/bin/bash
# VDA 5050 stack bring-up: console (EMCS role) <-> MQTT broker <-> AGV adapter / Isaac twin
#
# single host:
#   scripts/bringup_vda5050.sh test            # local mock AGV, domain 77, console :8089 (+ Isaac)
#   scripts/bringup_vda5050.sh real [--map-offset X Y YAW_DEG]   # real AMMR, domain 56, console :8080
# split hosts (item 3: console PC <-wired LAN-> DT/GPU PC, robot on WiFi):
#   scripts/bringup_vda5050.sh console [real]  # broker + master + mediator + BT + console (+kg_to_usd)
#   BROKER=192.168.31.135 scripts/bringup_vda5050.sh dt [real]     # Isaac twin only (scene local);
#     SCENE_URL=http://<console>:8089/api/scene ...  dt        # or fetched from the console over HTTP
#   BROKER=192.168.31.135 scripts/bringup_vda5050.sh agv [real|mock]   # AGV adapter (+ mock AGV)
#   NO_ISAAC=1 ... test/real                   # skip the Isaac twin
#   scripts/bringup_vda5050.sh stop            # kill everything started here (+ Isaac)
#
# Env: BROKER (host[:port], default 127.0.0.1:1883), CONSOLE_URL (default http://127.0.0.1:$PORT),
#      KG_SOURCE_URL=http://<data host>:8090 + BLK_OUTPUTS=<local dir> + MAP_YAML=<local yaml>
#      (console role on another host: pull KG/map/scene from the data host, push KG edits back),
#      SCENE_URL (dt role: fetch the scene over HTTP), BRINGUP_LOG_DIR. Processes are logged to $LOG.
set +u
MODE="${1:-test}"; shift || true
WS=/home/caselab/blk360_ros2_ws
LOG=${BRINGUP_LOG_DIR:-$HOME/bringup_logs}; mkdir -p "$LOG"
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
BROKER="${BROKER:-127.0.0.1:1883}"; BH="${BROKER%%:*}"; BP="${BROKER##*:}"; [ "$BP" = "$BH" ] && BP=1883
ROLE=all
case "$MODE" in
  console|dt|agv) ROLE=$MODE; SUB="${1:-}"; shift || true
                  [ "$SUB" = real ] && MODE=real || MODE=test
                  [ "$ROLE" = agv ] && [ "$SUB" = mock ] && MODE=test ;;
esac
echo "== 0/6 stop previous $ROLE processes =="
stop_role $ROLE; sleep 2

# bracket patterns only: a pattern must never match this script's own command line
P_CONSOLE=("ui_serve[r]" "mission_bt_cp[p]" "mediator_serve[r]" "vda5050_maste[r]" "kg_to_us[d]")
P_AGV=("vda5050_agv_adapte[r]" "ammr_pose_moc[k]")
P_DT=("isaacsim_kg_twi[n]" "kg_data_serve[r]")
stop_role() {                       # $1 = all|console|agv|dt
  local pats=()
  case "$1" in
    all) pats=("${P_CONSOLE[@]}" "${P_AGV[@]}" "${P_DT[@]}") ;;
    console) pats=("${P_CONSOLE[@]}") ;;
    agv) pats=("${P_AGV[@]}") ;;
    dt) pats=("${P_DT[@]}") ;;
  esac
  for p in "${pats[@]}"; do pkill -f "$p" 2>/dev/null; done
}

if [ "$MODE" = stop ]; then stop_role all; echo "stopped"; exit 0; fi

if { [ "$ROLE" = all ] || [ "$ROLE" = console ]; } && { [ "$BH" = 127.0.0.1 ] || [ "$BH" = localhost ]; }; then
echo "== 1/6 MQTT broker (docker ammr-mqtt :1883) =="
if ! docker ps --format '{{.Names}}' | grep -qx ammr-mqtt; then
  docker start ammr-mqtt >/dev/null 2>&1 || docker run -d --name ammr-mqtt --restart unless-stopped \
    -p 1883:1883 -v $HOME/ammr_twin/mosquitto/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro \
    eclipse-mosquitto:2 >/dev/null
  sleep 2
fi
docker ps --format '{{.Names}} {{.Status}}' | grep ammr-mqtt
fi

if [ "$MODE" = real ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=56
  export CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"
  PORT=8080; HEAD=180; OFFSET_ARGS=("$@")          # default: ~/ammr_twin/robot_map_offset.json
  if [ "$ROLE" = all ] || [ "$ROLE" = agv ]; then
    echo "== 2/6 robot handshake =="
    bash $WS/scripts/ammr_net_check.sh || { echo "HANDSHAKE FAILED"; exit 1; }
  fi
else
  export ROS_DOMAIN_ID=77
  PORT=8089; HEAD=0; OFFSET_ARGS=(--map-offset 0 0 0)
  if [ "$ROLE" = all ] || [ "$ROLE" = agv ]; then
    echo "== 2/6 mock AGV (domain 77) =="
    setsid nohup python3 $WS/scripts/ammr_pose_mock.py --start 0.7365 0.7815 0.809 \
        --plan-map $HOME/ammr_twin/map_vis_n2_1.yaml --vmax 1.1 --accel 0.5 > $LOG/mock.log 2>&1 &
  fi
  rm -f /tmp/mapreg_test/offset.json; mkdir -p /tmp/mapreg_test
  UI_EXTRA="-p offset_path:=/tmp/mapreg_test/offset.json"
fi
CONSOLE_URL="${CONSOLE_URL:-http://127.0.0.1:$PORT}"

if [ "$ROLE" = all ] || [ "$ROLE" = agv ]; then
  echo "== 3/6 VDA 5050 AGV adapter (broker $BROKER) =="
  setsid nohup ros2 run semantic_nav_bt vda5050_agv_adapter --broker-host $BH --broker-port $BP \
      --serial ammr20 --map-id map --stop-mode hold > $LOG/agv_adapter.log 2>&1 &
  sleep 2
fi
if [ "$ROLE" = all ] || [ "$ROLE" = console ]; then
  echo "== 3/6 VDA 5050 master control (broker $BROKER) =="
  setsid nohup ros2 run semantic_nav_bt vda5050_master --broker-host $BH --broker-port $BP \
      --serial ammr20 --map-id map --heading-offset $HEAD "${OFFSET_ARGS[@]}" > $LOG/master.log 2>&1 &
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
fi

if [ "$ROLE" = dt ] || [ "$ROLE" = all ]; then
  # data host: serve KG/map/scene to a remote console (scripts/kg_data_server.py :8090)
  pkill -f "kg_data_serve[r]" 2>/dev/null; sleep 0.5
  setsid nohup python3 $WS/scripts/kg_data_server.py --port 8090 > $LOG/kg_data.log 2>&1 &
  echo "== KG data server :8090 (for a remote console) =="
fi

if { [ "$ROLE" = all ] && [ -z "${NO_ISAAC:-}" ]; } || [ "$ROLE" = dt ]; then
  echo "== 6/6 Isaac twin (VDA 5050 mode, broker $BROKER, scene $CONSOLE_URL/api/scene) =="
  ISAAC_ARGS=(kg --pose-source amcl --goal-topic /kg_goal_pose --amcl-topic /kg_robot_pose
              --vda5050-broker $BROKER --vda-serial ammr20 --heading-offset $HEAD
              --sim-speed 1.1 --sim-accel 0.5)
  # SCENE_URL=http://<console>:<port>/api/scene when the KG scene lives on the console
  # host; unset = the DT host has the scene locally (kg_to_usd runs here)
  [ -n "${SCENE_URL:-}" ] && ISAAC_ARGS+=(--scene "$SCENE_URL")
  [ "$MODE" = test ] && ISAAC_ARGS+=(--kg-offset 0 0 0)
  ( cd $WS && unset PYTHONPATH && DISPLAY=${DISPLAY:-:1} setsid nohup scripts/run_isaac_ammr.sh "${ISAAC_ARGS[@]}" > $LOG/isaac_kg.log 2>&1 & )
fi
echo "done ($ROLE). console: $CONSOLE_URL   broker: $BROKER   logs: $LOG"
