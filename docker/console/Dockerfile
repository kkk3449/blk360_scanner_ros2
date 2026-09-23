# Semantic console (EMCS role) for a laptop / any OS with Docker.
# Contains: mediator, mission BT (C++), console web UI, VDA 5050 master.
# All ROS 2 traffic stays inside the container; only MQTT (to the broker) and
# HTTP (console UI :8089 out, KG data server in) cross the network, so no DDS
# multicast/host networking is needed.
#
#   docker build -t ammr-console -f docker/console/Dockerfile .
#   docker run -d --name ammr-console -p 8089:8089 \
#       -e BROKER=192.168.31.135:1883 -e KG_SOURCE_URL=http://192.168.31.135:8090 \
#       ammr-console
FROM ros:jazzy-ros-base

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-jazzy-nav2-msgs ros-jazzy-behaviortree-cpp ros-jazzy-rmw-cyclonedds-cpp \
        python3-numpy python3-yaml python3-opencv python3-pil python3-pip curl \
    && pip3 install --no-cache-dir --break-system-packages paho-mqtt \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/semantic_nav_msgs src/semantic_nav_msgs
COPY src/semantic_nav_bt src/semantic_nav_bt
COPY src/semantic_nav_bt_cpp src/semantic_nav_bt_cpp
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select semantic_nav_msgs semantic_nav_bt semantic_nav_bt_cpp \
    && rm -rf build log

COPY docker/console/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENV BLK_OUTPUTS=/data/outputs MAP_YAML=/data/map/map_vis_n2_1.yaml \
    ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    BROKER=127.0.0.1:1883 KG_SOURCE_URL= PORT=8089 HEADING_OFFSET=180
EXPOSE 8089
ENTRYPOINT ["/entrypoint.sh"]
