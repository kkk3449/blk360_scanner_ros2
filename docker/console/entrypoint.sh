#!/bin/bash
# Console container: KG sync (via ui_server's KG_SOURCE_URL) + master + mediator + BT + UI.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
mkdir -p "$BLK_OUTPUTS/vis_sota_det4" "$(dirname "$MAP_YAML")"
BH="${BROKER%%:*}"; BP="${BROKER##*:}"; [ "$BP" = "$BH" ] && BP=1883
export ROS_LOCALHOST_ONLY=1        # DDS never leaves the container

if [ -n "$KG_SOURCE_URL" ]; then
  echo "[console] pulling KG/map from $KG_SOURCE_URL"
  for i in $(seq 1 30); do curl -sf -m 3 "$KG_SOURCE_URL/manifest" >/dev/null && break; sleep 2; done
  for f in testroom_epochs_kg.json place_layer_T3_slic.json place_ring_naming.json t3_place_scoped_relations.json; do
    curl -sf -m 60 "$KG_SOURCE_URL/files/$f" -o "$BLK_OUTPUTS/$f" || echo "[console] WARN: $f not pulled"
  done
  curl -sf -m 60 "$KG_SOURCE_URL/files/map_vis_n2_1.yaml" -o "$MAP_YAML"
  curl -sf -m 60 "$KG_SOURCE_URL/files/map_vis_n2_1.pgm" -o "$(dirname "$MAP_YAML")/map_vis_n2_1.pgm"
  curl -sf -m 60 "$KG_SOURCE_URL/files/robot_map_offset.json" -o /data/robot_map_offset.json || true
fi
OFFSET_ARGS=()
[ -f /data/robot_map_offset.json ] || OFFSET_ARGS=(--map-offset 0 0 0)

ros2 run semantic_nav_bt vda5050_master --broker-host "$BH" --broker-port "$BP" --serial ammr20 --map-id map \
    --heading-offset "$HEADING_OFFSET" --map-offset-file /data/robot_map_offset.json "${OFFSET_ARGS[@]}" &
sleep 2
ros2 run semantic_nav_bt mediator_server --ros-args -p gated:=true \
    -p kg_path:="$BLK_OUTPUTS/testroom_epochs_kg.json" -p places_path:="$BLK_OUTPUTS/place_layer_T3_slic.json" \
    -p naming_path:="$BLK_OUTPUTS/place_ring_naming.json" -p map_yaml:="$MAP_YAML" &
sleep 2
ros2 run semantic_nav_bt_cpp mission_bt_cpp --ros-args -r amcl_pose:=/kg_robot_pose &
sleep 2
exec ros2 run semantic_nav_bt ui_server --ros-args -p port:="$PORT" \
    -p kg_path:="$BLK_OUTPUTS/testroom_epochs_kg.json" -p places_path:="$BLK_OUTPUTS/place_layer_T3_slic.json" \
    -p naming_path:="$BLK_OUTPUTS/place_ring_naming.json" -p map_yaml:="$MAP_YAML" \
    -p offset_path:=/data/robot_map_offset.json -p scene_path:="$BLK_OUTPUTS/vis_sota_det4/t4_kg_scene.usda" \
    -r amcl_pose:=/kg_robot_pose -r goal_pose:=/kg_goal_pose -r initialpose:=/kg_initialpose
