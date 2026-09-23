# Split deployment (item 3): console laptop ↔ Isaac/data PC ↔ AMMR

All three on the same WiFi (lab AP 192.168.31.0/24). Console ↔ PC can also
be a wired Ethernet link; nothing in the software changes, only the address.

```
 laptop (console = EMCS role)          Isaac / data PC (192.168.31.135)         AMMR (192.168.31.56, WiFi)
 ┌───────────────────────────┐         ┌──────────────────────────────────┐     ┌──────────────────────┐
 │ ammr-console container    │  MQTT   │ mosquitto :1883 (docker)         │MQTT │ vda5050_agv_adapter* │
 │  ui_server :8089          │◀───────▶│ Isaac twin (ammr20-twin AGV)     │◀───▶│  ↔ Nav2 / AMCL       │
 │  mediator + mission BT    │  HTTP   │ kg_data_server :8090             │     └──────────────────────┘
 │  vda5050_master           │◀───────▶│  KG json / place layer / map /   │       *until the robot team ships
 │  (KG/map synced copy)     │  8090   │  scene  (authoritative copies)   │        a native VDA 5050 adapter
 └───────────────────────────┘         │ semantic pipeline, kg_to_usd     │        it runs on the PC on the
                                       └──────────────────────────────────┘        robot's DDS domain 56
```

* Only MQTT (VDA 5050) and HTTP cross machines. No DDS between hosts: the
  console container sets `ROS_LOCALHOST_ONLY=1`, the PC keeps DDS for
  Isaac/robot only.
* The knowledge graph has ONE authoritative copy (PC). The console pulls it
  (10 s poll on the data server manifest, hot-reload) and pushes owner edits
  back (`PUT /files/testroom_epochs_kg.json`).

## 1. PC (data + DT host)

```
scripts/bringup_vda5050.sh dt real          # Isaac twin + KG data server :8090 (+ broker if not running)
BROKER=127.0.0.1:1883 scripts/bringup_vda5050.sh agv real    # AGV adapter on the robot's domain 56
```
(`dt`/`agv` with `mock` instead of `real` for the local mock AGV.) The
broker container `ammr-mqtt` restarts with docker; `docker start ammr-mqtt`
if it is not up.

Firewall: ports 1883 (MQTT), 8090 (data), 8089 (console UI if the PC runs it)
must be reachable on the LAN (`ss -ltn` shows 0.0.0.0 binds).

## 2. Laptop (console) — Docker, any OS

Build once on the PC and copy the image, or build on the laptop from the repo:

```
docker build -t ammr-console -f docker/console/Dockerfile .      # on the PC
docker save ammr-console | gzip > ammr-console.tar.gz             # ~1 GB
# laptop:
docker load < ammr-console.tar.gz
docker run -d --name ammr-console --restart unless-stopped -p 8089:8089 \
    -e BROKER=192.168.31.135:1883 -e KG_SOURCE_URL=http://192.168.31.135:8090 \
    -e HEADING_OFFSET=180 ammr-console
```
Open http://localhost:8089 on the laptop. `HEADING_OFFSET=0` for the mock AGV.
Logs: `docker logs -f ammr-console`.

Laptop with Ubuntu 24.04 + ROS 2 jazzy instead of Docker: build the three
packages (`semantic_nav_msgs semantic_nav_bt semantic_nav_bt_cpp`, deps
`ros-jazzy-nav2-msgs ros-jazzy-behaviortree-cpp python3-opencv paho-mqtt`) and run

```
BROKER=192.168.31.135:1883 KG_SOURCE_URL=http://192.168.31.135:8090 \
BLK_OUTPUTS=$HOME/console_data/outputs MAP_YAML=$HOME/console_data/map/map_vis_n2_1.yaml \
scripts/bringup_vda5050.sh console real
```

## 3. What was verified (2026-09-22, single PC, LAN address 192.168.31.135)

* Three roles started separately (`console`, `agv mock`, `dt`) and connected
  through the LAN IP; Isaac fetched the scene over HTTP (12.4 MB) and later
  refreshes it when the console's copy changes.
* Mission through the split stack: real 5.1 s vs sim ETA 4.8 s, off-path 0.0 m.
* MQTT round trip via the LAN address: 20 ms median (10 ms on localhost).
* Second console instance (data dir `/tmp/console_data`, `KG_SOURCE_URL`) pulled
  7 files (46 objects, 7 places, 654-px map) and its owner edits were written
  back to the PC's KG (toggle → toggle back verified).

## 4. Laptop on the lab WiFi (2026-09-23, Ubuntu 22.04, 192.168.31.15)

`ammr-console_20260922b.tar.gz` loaded and run as above. The laptop console
pulled the 8 data files from the PC, showed the map, set the initial pose
(`initPosition` reached the PC adapter), sent a direct goal and a
`goto_place display_briefing_area` mission: orders arrived at the PC adapter
over the WiFi broker, the mock drove node by node, the Isaac twin planned
7.8 m / ETA 9.3 s and finished, the console showed both AGVs ONLINE and the
mission complete. Ping laptop↔PC 2–115 ms (WiFi jitter); MQTT is unaffected.

Gotchas seen: (1) the first image lacked `python3-pil` → empty map panel;
(2) two console containers (PC test + laptop) had identical MQTT client ids
(same in-container pid) and kicked each other off the broker → client ids
now include hostname + random; (3) after a PC reboot only the broker comes
back (docker restart policy) — run `bringup_vda5050.sh dt` and `agv ...` again
(or install them as systemd services); a console started while the data
server is down restarts until it can pull the KG.

Pending: latency/loss measurement over the real link (command in
`docs/ammr_twin_runbook.md` §13), KG edit push from the laptop (verified from a
second console on the PC only).
