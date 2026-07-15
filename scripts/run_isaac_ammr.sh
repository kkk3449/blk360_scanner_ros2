#!/bin/bash
# Launch an Isaac Sim AMMR twin script with the bridge's INTERNAL ROS 2 jazzy
# libs (Isaac python is 3.11; the system /opt/ros/jazzy rclpy is 3.12 and must
# NOT leak in). DDS still talks to system-ROS nodes on the same domain.
#
#   scripts/run_isaac_ammr.sh teleop [extra args...]   # sim-only teleop stage
#   scripts/run_isaac_ammr.sh twin   [extra args...]   # live real-robot twin
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
ISAAC="$HOME/isaacsim"
BRIDGE="$ISAAC/exts/isaacsim.ros2.bridge/jazzy"

MODE="${1:-teleop}"; shift || true
case "$MODE" in
  teleop) SCRIPT="$WS/scripts/isaacsim_ammr_teleop.py"
          DEFAULTS=(--world "$HOME/ammr_twin/vis_n2_world.usda"
                    --overlay "$HOME/ammr_twin/vis_n2_tosm.usda") ;;
  twin)   SCRIPT="$WS/scripts/isaacsim_ammr_twin.py"
          DEFAULTS=(--world "$HOME/ammr_twin/vis_n2_world.usda"
                    --overlay "$HOME/ammr_twin/vis_n2_tosm.usda") ;;
  *) echo "usage: $0 {teleop|twin} [args]"; exit 1 ;;
esac

# scrub any sourced system ROS, then point at the internal jazzy libs
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
LD_SCRUBBED=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
export LD_LIBRARY_PATH="$LD_SCRUBBED${LD_SCRUBBED:+:}$BRIDGE/lib"
export ROS_DISTRO=jazzy
export PYTHONUNBUFFERED=1

if [ "$MODE" = twin ] && [ -z "${AMMR_LOCAL_TEST:-}" ]; then
  # real-robot bridge contract (~/Downloads/DataSend/README.md):
  # CycloneDDS + domain 56 + unicast peer to the ammr wifi IP
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=56
  export CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"
  echo "[run_isaac_ammr] twin mode: CycloneDDS, domain 56, $CYCLONEDDS_URI"
else
  # sim-only (teleop) or AMMR_LOCAL_TEST=1: local FastDDS, domain 0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
fi

cd "$WS"
exec "$ISAAC/python.sh" "$SCRIPT" "${DEFAULTS[@]}" "$@"
