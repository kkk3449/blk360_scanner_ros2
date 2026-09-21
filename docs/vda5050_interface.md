# VDA 5050 interface — console (EMCS role) ↔ AMMR / Isaac twin

Version 2026-09-21. Implements VDA 5050 **v2.0.0** message schemas (subset)
over MQTT. Written so that the EMCS control team can later replace our
console with their master control without touching the robot or the twin:
everything the console does on the AGV side goes through the topics below.

## 1. Transport

| item | value |
|---|---|
| broker | Mosquitto 2 (docker `ammr-mqtt`), TCP 1883, anonymous, no TLS (lab LAN) |
| topic | `uagv/v2/{manufacturer}/{serialNumber}/{topic}` |
| manufacturer | `caselab` |
| serialNumber | `ammr20` (real AMMR / its mock); twin AGV will be `ammr20-twin` |
| QoS | order, instantActions, state, connection: QoS 1; visualization: QoS 0 |
| retained | `connection` only (last will = `CONNECTIONBROKEN`) |
| network | robot ↔ broker over WiFi (192.168.31.x); console/Isaac ↔ broker over LAN or localhost |

Header on every message: `headerId` (per-topic counter), `timestamp`
(ISO 8601 UTC, ms), `version` `"2.0.0"`, `manufacturer`, `serialNumber`.

## 2. Master control → AGV

### `order`
Single-goal order as produced by the console (BT `navigate_to_pose`, map
click `/kg_goal_pose`, Isaac cone):

```json
{"orderId":"o1789970448041-1","orderUpdateId":0,
 "nodes":[
   {"nodeId":"o…-n0","sequenceId":0,"released":true,"nodeDescription":"current position",
    "nodePosition":{"x":1.72,"y":3.29,"theta":-1.41,"mapId":"map",
                    "allowedDeviationXY":1.0,"allowedDeviationTheta":3.14159},"actions":[]},
   {"nodeId":"o…-n1","sequenceId":2,"released":true,"nodeDescription":"goal",
    "nodePosition":{"x":0.0,"y":0.0,"theta":0.0,"mapId":"map",
                    "allowedDeviationXY":0.35,"allowedDeviationTheta":3.14159},"actions":[]}],
 "edges":[{"edgeId":"o…-e0","sequenceId":1,"released":true,
           "startNodeId":"o…-n0","endNodeId":"o…-n1","actions":[]}]}
```

* Coordinates are in the **AGV's map frame** (`mapId`); the master applies the
  KG→robot SE(2) offset (`~/ammr_twin/robot_map_offset.json`, live update on
  `/kg_map_offset`) and the heading offset (AMMR: 180°) before building the order.
* Node 0 is the AGV's current position (reached immediately); node 1 is the goal.
  Multi-node orders (place-graph routes) use the same schema with more nodes/edges.
* A new `orderId` preempts the running order; a higher `orderUpdateId` of the
  same order replaces it. Only `released` nodes are executed.

### `instantActions`
| actionType | parameters | effect on the AMMR adapter |
|---|---|---|
| `cancelOrder` | – | clears the order; `stop-mode hold` sends the current pose as goal (robot decelerates and holds) |
| `startPause` / `stopPause` | – | hold / resume the current node goal |
| `initPosition` | `x`, `y`, `theta`, `mapId`, `lastNodeId` | publishes `/ammr/initialpose` (AMCL re-init) |

Unsupported actionTypes are answered with `actionStatus: FAILED` and an
`actionError` warning in `state.errors`.

## 3. AGV → master control

### `state` (1 Hz, plus immediately on order accept / node reached / action)
Fields populated: `orderId`, `orderUpdateId`, `lastNodeId`, `lastNodeSequenceId`,
`nodeStates` (remaining nodes with positions), `edgeStates`, `agvPosition`
(`x,y,theta,mapId,positionInitialized`), `velocity` (`vx,vy,omega`), `driving`,
`paused`, `newBaseRequest` (false), `actionStates` (last 10), `batteryState`
(`batteryCharge` %, `charging`), `operatingMode` (`AUTOMATIC`), `errors`
(`orderError`, `actionError`, `noOrderToCancel`; `errorLevel` WARNING/FATAL),
`information` ([]), `safetyState` (`eStop: NONE`, `fieldViolation: false`).

Order completion as seen by the master: `orderId` matches, `nodeStates` empty,
`lastNodeId` == goal node, `driving` false (fallback: pose inside the goal
node's `allowedDeviationXY` and speed < 0.08 m/s for 1 s).

### `visualization` (20 Hz) — `agvPosition` + `velocity` only.
### `connection` (retained) — `ONLINE` on connect, `OFFLINE` on clean exit,
`CONNECTIONBROKEN` via MQTT last will.

## 4. Processes and ROS-side contract (unchanged)

| process | side | ROS in | ROS out |
|---|---|---|---|
| `vda5050_master` (console) | EMCS | `navigate_to_pose` action (BT), `/kg_goal_pose`, `/kg_initialpose`, `/semantic_stop`, `/kg_map_offset` | `/kg_robot_pose` (primary AGV, KG frame, latched), `/vda5050/agv_states` (JSON summary of every AGV, 2 Hz) |
| `vda5050_agv_adapter` (robot) | AGV | `/ammr/state` (Odometry), `/ammr/pose`, `/battery_state` | `/ammr/goal_pose`, `/ammr/initialpose` |
| Isaac twin `--vda5050-broker` | DT | – (mirrors `state`/`visualization` from MQTT; runs `ammr20-twin` orders) | cone release → `order`; twin `state`/`visualization`/`connection` |

The console shows the fleet link in the **VDA 5050 fleet link (MQTT)** card
(broker connection, per-AGV connection/order/last node/driving/position/battery/
actions/errors) from `/vda5050/agv_states`.

## 5. Bring-up

```
scripts/bringup_vda5050.sh test      # mock AGV, domain 77, console :8089 (+ Isaac)
scripts/bringup_vda5050.sh real      # AMMR on domain 56 (CycloneDDS), console :8080
scripts/bringup_vda5050.sh stop
```

Inspect the bus: `docker exec ammr-mqtt mosquitto_sub -t 'uagv/v2/#' -v`.

## 6. Verified 2026-09-21 (mock AGV, all local)

console goal → `order` (2 nodes) → adapter drives node by node → `state`
(`nodeStates` 2→1→0, `lastNodeId` n0→n1, `driving`) → mock arrives; BT
`goto_object tv` → order → `ARRIVED (state)` in 5.7 s; console STOP →
`instantActions cancelOrder` → hold goal, `driving:false`,
`actionStates cancelOrder:FINISHED`; console initial pose → `initPosition`
→ `/ammr/initialpose`; Isaac twin mirrors the AGV from MQTT and its cone
sends an `order`.

## 7. Digital-twin feedback (item 2, verified 2026-09-21)

The Isaac twin runs a **second AGV endpoint `ammr20-twin`** (`scripts/sim_agv.py`):
inflated A* on the KG occupancy grid (`--sim-map`, 0.10 m cells, 0.45 m
inflation, 1.2 m clearance around the start because the scan-derived map
contains the robot's own footprint), trapezoidal speed profile
(`--sim-speed 0.8`, `--sim-accel 0.5`), ghost robot + predicted-path curve in
the viewport. Its `state.information` carries
`infoType: simPrediction` (`pathLength`, `eta`, `nodeId`, `plannedAt`); a
planning failure is reported as `errors[]` `noPath` (FATAL).

Master (`--twin-serial ammr20-twin`):
* every order to the real AGV is mirrored to the twin after an `initPosition`
  sync onto the real pose, so both start together; the live comparison
  is published in `/vda5050/agv_states.twin.current`: **off-path distance**
  (cross-track distance of the real AGV to the twin's planned path, sent by
  the twin as `information[infoType=simPath]` waypoints — a route
  difference), the real–twin **gap** (pose-to-pose, mostly speed-profile
  difference), sim ETA vs elapsed, max off-path. Alarm when off-path exceeds
  `--deviation-alarm` (0.8 m) or the real run exceeds `--eta-slack`×ETA + 5 s;
  finished missions go to `twin.history`.
* **simulate-before-execute** (`--sim-first` or console toggle via
  `/vda5050/cmd {"cmd":"sim_first","value":true}`): the order goes to the twin
  only; the console shows the proposal (`simulating` → `sim ok` with path/ETA,
  or `no path`) with **Execute on robot** / **Discard**; on execute the real AGV
  and a re-synced twin run the order together under `orderId + "x"`.

Verified (mock with `--plan-map`, i.e. the mock plans like Nav2 and drives
at 1.1 m/s; twin at the same speed): `goto_object tv` / `goto_place` → real
11.3 s vs sim ETA 11.0 s, off-path 0.0 m, gap ≤ 0.17 m, no alarm; sim-first
proposal → `sim ok` → execute → history entry. On the real robot the off-path
value will be non-zero (Nav2 plans on its own costmap); tune
`--deviation-alarm` from the first real runs.

## 8. Not yet done (items 3–4 of the plan)

* Console and Isaac on separate hosts over wired Ethernet (only the broker
  address changes; DDS is no longer needed between them).
* Robot-side native VDA 5050 adapter (robot team) — until then
  `vda5050_agv_adapter` runs on this PC on the robot's DDS domain.
* Twin marker on the console map panel; deviation/ETA thresholds from real runs.
* TLS / authentication on the broker if it leaves the lab LAN.
