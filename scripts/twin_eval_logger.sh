#!/bin/bash
# Run the twin-eval logger with the ammr DDS env (system ROS, CycloneDDS,
# domain 56 — same contract as ammr_net_check.sh).
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=56
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclonedds_isaac.xml"
exec python3 "$(dirname "$0")/twin_eval_logger.py" "$@"
