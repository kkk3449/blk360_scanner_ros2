#!/usr/bin/env python3
"""VDA 5050 master control on the console (EMCS) side.

Drop-in replacement for scripts/real_robot_nav_bridge.py: the ROS-facing
interface is identical (so mission_bt_cpp, the console and the Isaac twin
are untouched), but the AGV-facing side speaks VDA 5050 over MQTT instead
of ROS topics:

  ROS (KG frame)                          MQTT (VDA 5050, AGV map frame)
  navigate_to_pose action (from BT)  -->  order            (2 nodes + 1 edge)
  /kg_goal_pose  (console/Isaac)     -->  order
  /kg_initialpose (console)          -->  instantActions initPosition
  /semantic_stop (console STOP)      -->  instantActions cancelOrder
  /kg_robot_pose  (latched)          <--  state / visualization (primary AGV)
  /vda5050/agv_states (JSON, 2 Hz)   <--  state / connection of EVERY AGV
                                          under the manufacturer (real + twin)

KG <-> AGV-map SE(2) offset: --map-offset X Y YAW_DEG (default: the file
written by the console's map registration), updated live on /kg_map_offset.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from . import vda5050 as V


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def _cross_track(p, pts):
    """Distance from point p to the polyline pts (list of (x, y))."""
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


class Agv:
    """Last known VDA 5050 view of one AGV (serial)."""

    def __init__(self, serial):
        self.serial = serial
        self.connection = "UNKNOWN"
        self.state = {}
        self.pos = None          # (x, y, theta) in the AGV map frame
        self.vel = 0.0
        self.t_state = 0.0
        self.t_pos = 0.0


class Master(Node):
    def __init__(self, a):
        super().__init__("vda5050_master")
        self.a = a
        self.lock = threading.RLock()   # re-entrant: _agv() is called under the lock
        self.off = [a.map_offset[0], a.map_offset[1],
                    math.radians(a.map_offset[2])]
        self.h = math.radians(a.heading_offset)
        self.agvs = {}                                  # serial -> Agv
        self.hdr = V.Header(a.manufacturer, a.serial)
        self.active = None
        self.active_order = None                        # orderId being executed
        self.order_seq = 0
        # digital-twin feedback (twin AGV executes the same order in Isaac)
        self.twin = a.twin_serial or None
        self.sim_first = a.sim_first
        self.cmp = None            # live comparison of the current order
        self.history = []          # last missions: sim eta vs real time, max deviation
        self.proposal = None       # sim-first: order waiting for operator approval
        self.decision = None       # "execute" | "discard" from the console

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped,
                                              a.pose_out_topic, latched)
        self.pub_states = self.create_publisher(String, "/vda5050/agv_states",
                                                latched)
        self.create_subscription(String, "/semantic_stop", self._stop_cb, 10)
        self.create_subscription(String, "/kg_map_offset", self._offset_cb,
                                 latched)
        self.create_subscription(PoseStamped, "/kg_goal_pose", self._kg_goal_cb,
                                 10)
        self.create_subscription(PoseWithCovarianceStamped, "/kg_initialpose",
                                 self._kg_init_cb, 10)
        self.create_subscription(String, "/vda5050/cmd", self._cmd_cb, 10)
        self.server = ActionServer(
            self, NavigateToPose, "navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=lambda _r: GoalResponse.ACCEPT,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.create_timer(0.5, self._publish_states)

        # -- MQTT
        self.mq = V.Mqtt(a.broker_host, a.broker_port,
                         client_id=f"master-{a.serial}-{os.getpid()}",
                         log=lambda s: self.get_logger().info(s))
        base = f"{V.INTERFACE}/{V.MAJOR}/{a.manufacturer}/+/"
        self.mq.subscribe(base + "state", self._on_state)
        self.mq.subscribe(base + "visualization", self._on_vis)
        self.mq.subscribe(base + "connection", self._on_conn)
        self.mq.start()
        self.get_logger().info(
            f"master up: broker {a.broker_host}:{a.broker_port}, primary AGV "
            f"{a.manufacturer}/{a.serial}, mapId '{a.map_id}', offset "
            f"{self.off[0]:.3f} {self.off[1]:.3f} {math.degrees(self.off[2]):.2f} deg, "
            f"heading offset {a.heading_offset} deg")

    # ---------------------------------------------------------- transforms --
    def robot_to_kg(self, x, y, yaw):
        ox, oy, oyaw = self.off
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * x - s * y, oy + s * x + c * y, wrap(yaw + oyaw + self.h))

    def kg_to_robot(self, x, y, yaw):
        ox, oy, oyaw = self.off
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        return (c * dx - s * dy, s * dx + c * dy, wrap(yaw - oyaw - self.h))

    def _agv(self, serial):
        with self.lock:
            return self.agvs.setdefault(serial, Agv(serial))

    # ---------------------------------------------------------- MQTT in ----
    def _on_conn(self, pt, m):
        g = self._agv(pt[1])
        g.connection = m.get("connectionState", "UNKNOWN")
        self.get_logger().info(f"AGV {pt[1]} connection {g.connection}")

    def _on_vis(self, pt, m):
        self._take_pos(self._agv(pt[1]), m)

    def _on_state(self, pt, m):
        g = self._agv(pt[1])
        with self.lock:
            g.state = m
            g.t_state = time.time()
        self._take_pos(g, m)
        for e in m.get("errors", []):
            if e.get("errorLevel") == "FATAL":
                self.get_logger().error(
                    f"AGV {pt[1]} FATAL {e.get('errorType')}: {e.get('errorDescription')}")

    def _take_pos(self, g, m):
        p = m.get("agvPosition")
        if not p:
            return
        v = m.get("velocity") or {}
        with self.lock:
            g.pos = (float(p["x"]), float(p["y"]), float(p.get("theta", 0.0)))
            g.vel = math.hypot(float(v.get("vx", 0.0)), float(v.get("vy", 0.0)))
            g.t_pos = time.time()
        if g.serial == self.a.serial:
            kx, ky, kyaw = self.robot_to_kg(*g.pos)
            out = PoseWithCovarianceStamped()
            out.header.frame_id = "map"
            out.header.stamp = self.get_clock().now().to_msg()
            out.pose.pose.position.x, out.pose.pose.position.y = kx, ky
            out.pose.pose.orientation.z = math.sin(kyaw / 2)
            out.pose.pose.orientation.w = math.cos(kyaw / 2)
            self.pub_pose.publish(out)

    # ---------------------------------------------------------- MQTT out ---
    def _send_order(self, kx, ky, kyaw, serial=None, oid=None, cur=None):
        """Single-goal order: node 0 = where the AGV is now, node 1 = goal."""
        serial = serial or self.a.serial
        g = self._agv(serial)
        rx, ry, ryaw = self.kg_to_robot(kx, ky, kyaw)
        if cur is None:
            with self.lock:
                cur = g.pos
        if oid is None:
            self.order_seq += 1
            oid = f"o{int(time.time() * 1000)}-{self.order_seq}"
        nodes = []
        if cur is not None:
            nodes.append(V.node(f"{oid}-n0", 0, cur[0], cur[1], cur[2],
                                self.a.map_id, dev_xy=1.0,
                                description="current position"))
            nodes.append(V.node(f"{oid}-n1", 2, rx, ry, ryaw, self.a.map_id,
                                dev_xy=self.a.arrive_radius, description="goal"))
            edges = [V.edge(f"{oid}-e0", 1, nodes[0]["nodeId"],
                            nodes[1]["nodeId"])]
        else:                       # no position yet: single-node order
            nodes.append(V.node(f"{oid}-n1", 0, rx, ry, ryaw, self.a.map_id,
                                dev_xy=self.a.arrive_radius, description="goal"))
            edges = []
        hdr = V.Header(self.a.manufacturer, serial)
        self.mq.publish(V.topic(self.a.manufacturer, serial, "order"),
                        V.make_order(hdr, oid, 0, nodes, edges))
        self.get_logger().info(
            f"ORDER {oid} -> {serial}: KG=({kx:.2f},{ky:.2f}) -> map=({rx:.2f},{ry:.2f})")
        return oid, nodes[-1]["nodeId"]

    def _send_instant(self, act, serial=None):
        serial = serial or self.a.serial
        hdr = V.Header(self.a.manufacturer, serial)
        self.mq.publish(V.topic(self.a.manufacturer, serial, "instantActions"),
                        V.make_instant_actions(hdr, [act]))

    def _sync_twin(self):
        """Teleport the twin AGV onto the real AGV before a shared order."""
        with self.lock:
            cur = self._agv(self.a.serial).pos
        if cur is None or not self.twin:
            return
        self._send_instant(V.action(
            "initPosition", f"sync-{int(time.time()*1000)}",
            params={"x": cur[0], "y": cur[1], "theta": cur[2],
                    "mapId": self.a.map_id, "lastNodeId": ""}), serial=self.twin)

    def _dispatch(self, kx, ky, kyaw, wait_for_approval=False):
        """Order for the real AGV, mirrored to the twin AGV.
        sim-first: twin only, then wait for the console's execute/discard.
        Returns (orderId, goalNodeId) or (None, None) if discarded."""
        with self.lock:
            cur = self._agv(self.a.serial).pos
        if self.twin:
            self._sync_twin()
        if self.twin and self.sim_first:
            oid, gnode = self._send_order(kx, ky, kyaw, serial=self.twin, cur=cur)
            self.proposal = {"orderId": oid, "goal": [round(kx, 2), round(ky, 2)],
                             "t": time.time(), "status": "simulating",
                             "eta": None, "pathLength": None, "error": None}
            self.decision = None
            self.get_logger().info(f"SIM-FIRST: order {oid} sent to twin only; awaiting approval")
            if not wait_for_approval:
                return oid, gnode
            t0 = time.time()
            while rclpy.ok() and self.decision is None and time.time() - t0 < 300:
                time.sleep(0.1)
            if self.decision != "execute":
                self.proposal = None
                return None, None
            self.proposal = None
            return self._execute_approved(oid, kx, ky, kyaw)
        oid, gnode = self._send_order(kx, ky, kyaw, serial=self.a.serial, cur=cur)
        if self.twin:
            self._send_order(kx, ky, kyaw, serial=self.twin, oid=oid, cur=cur)
        self._start_cmp(oid, gnode, kx, ky, executed=True)
        return oid, gnode

    def _execute_approved(self, oid, kx, ky, kyaw):
        """Approved proposal: run real AGV and twin together (twin re-synced
        to the real pose) under a new orderId so the live comparison is fair."""
        with self.lock:
            cur = self._agv(self.a.serial).pos
        self._sync_twin()
        oid2 = oid + "x"
        _, gnode = self._send_order(kx, ky, kyaw, serial=self.a.serial, oid=oid2, cur=cur)
        self._send_order(kx, ky, kyaw, serial=self.twin, oid=oid2, cur=cur)
        self._start_cmp(oid2, gnode, kx, ky, executed=True)
        return oid2, gnode

    def _start_cmp(self, oid, gnode, kx, ky, executed):
        self.cmp = {"orderId": oid, "goalNode": gnode, "goal": [round(kx, 2), round(ky, 2)],
                    "t0": time.time(), "executed": executed,
                    "sim_eta": None, "sim_path": None, "sim_done_t": None,
                    "real_done_t": None, "deviation": None, "gap": None, "max_dev": 0.0,
                    "path_pts": None, "alarm": None, "sim_error": None}

    def _cmd_cb(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            return
        cmd = c.get("cmd")
        if cmd == "sim_first":
            self.sim_first = bool(c.get("value"))
            self.get_logger().info(f"sim-first mode {'ON' if self.sim_first else 'OFF'}")
        elif cmd in ("execute", "discard"):
            if self.proposal is None:
                return
            if cmd == "discard":
                self._send_instant(V.action("cancelOrder", f"cancel-{int(time.time()*1000)}"),
                                   serial=self.twin)
                self.get_logger().info(f"proposal {self.proposal['orderId']} discarded")
            self.decision = cmd
            # direct (non-action) proposals are executed here
            p = self.proposal
            if cmd == "execute" and p and p.get("direct"):
                kx, ky, kyaw = p["direct"]
                self.proposal = None
                self._execute_approved(p["orderId"], kx, ky, kyaw)
            elif cmd == "discard":
                self.proposal = None

    # ---------------------------------------------------------- ROS in ------
    def _kg_goal_cb(self, m):
        p, q = m.pose.position, m.pose.orientation
        oid, _ = self._dispatch(p.x, p.y, quat_yaw(q), wait_for_approval=False)
        if self.proposal is not None and self.proposal.get("orderId") == oid:
            self.proposal["direct"] = (p.x, p.y, quat_yaw(q))

    def _kg_init_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rx, ry, ryaw = self.kg_to_robot(p.x, p.y, quat_yaw(q))
        self._send_instant(V.action(
            "initPosition", f"init-{int(time.time()*1000)}",
            params={"x": rx, "y": ry, "theta": ryaw, "mapId": self.a.map_id,
                    "lastNodeId": ""}))
        self.get_logger().info(
            f"INITPOSITION KG=({p.x:.2f},{p.y:.2f}) -> map=({rx:.2f},{ry:.2f})")

    def _stop_cb(self, _m):
        self._send_instant(V.action("cancelOrder",
                                    f"cancel-{int(time.time()*1000)}"))
        if self.twin:
            self._send_instant(V.action("cancelOrder",
                                        f"cancel-{int(time.time()*1000)}"), serial=self.twin)
        if self.proposal is not None:
            self.decision = "discard"
            self.proposal = None
        h = self.active
        if h is not None and h.is_active:
            try:
                h.abort()
            except Exception:
                pass
        self.get_logger().warn("STOP -> instantAction cancelOrder")

    def _offset_cb(self, m):
        try:
            off = json.loads(m.data)["map_offset"]
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            self.off = [float(off[0]), float(off[1]), math.radians(float(off[2]))]
        self.get_logger().info(
            f"map offset updated from console: {off[0]:.4f} {off[1]:.4f} {off[2]:.3f} deg")

    # ---------------------------------------------------------- status out --
    def _update_cmp(self, now):
        """Real-vs-twin comparison of the current order (from the two states)
        and the status of a sim-first proposal."""
        with self.lock:
            real, twin = self.agvs.get(self.a.serial), self.agvs.get(self.twin)
            rs, rp = (real.state, real.pos) if real else ({}, None)
            ts, tp = (twin.state, twin.pos) if twin else ({}, None)
        # sim-first proposal status (twin-only run)
        p = self.proposal
        if p is not None and ts.get("orderId") == p["orderId"]:
            for inf in ts.get("information", []):
                if inf.get("infoType") == "simPrediction":
                    refs = {r["referenceKey"]: r["referenceValue"]
                            for r in inf.get("infoReferences", [])}
                    p["eta"], p["pathLength"] = float(refs.get("eta", 0)), float(refs.get("pathLength", 0))
            for e in ts.get("errors", []):
                if e.get("errorLevel") == "FATAL":
                    p["error"] = e.get("errorType")
            if p["error"]:
                p["status"] = "no path"
            elif not ts.get("nodeStates") and ts.get("lastNodeId", "").endswith("-n1"):
                p["status"] = "sim ok"
                if p["eta"] is None:
                    p["eta"], p["pathLength"] = 0.0, 0.0     # already at the goal
            else:
                p["status"] = "simulating"
        c = self.cmp
        if c is None:
            return
        # twin prediction
        if ts.get("orderId") == c["orderId"]:
            for inf in ts.get("information", []):
                refs = {r["referenceKey"]: r["referenceValue"]
                        for r in inf.get("infoReferences", [])}
                if inf.get("infoType") == "simPrediction":
                    if refs.get("nodeId") == c["goalNode"]:
                        c["sim_eta"] = float(refs.get("eta", 0))
                        c["sim_path"] = float(refs.get("pathLength", 0))
                elif inf.get("infoType") == "simPath" and refs.get("nodeId") == c["goalNode"]:
                    try:
                        c["path_pts"] = [tuple(float(v) for v in q.split(","))
                                         for q in refs.get("points", "").split(";") if q]
                    except ValueError:
                        pass
            for e in ts.get("errors", []):
                if e.get("errorLevel") == "FATAL":
                    c["sim_error"] = e.get("errorType")
            if (c["sim_done_t"] is None and not ts.get("nodeStates")
                    and ts.get("lastNodeId") == c["goalNode"]):
                c["sim_done_t"] = now
        if (c["executed"] and rs.get("orderId") == c["orderId"]
                and c["real_done_t"] is None and not rs.get("nodeStates")
                and rs.get("lastNodeId") == c["goalNode"]):
            c["real_done_t"] = now
        if c["executed"] and rp and tp and c["real_done_t"] is None:
            c["gap"] = round(math.hypot(rp[0] - tp[0], rp[1] - tp[1]), 2)
            pts = c.get("path_pts")
            if pts and len(pts) >= 2:      # off-path distance (route difference)
                c["deviation"] = round(_cross_track(rp, pts), 2)
            else:                          # no path yet: fall back to the gap
                c["deviation"] = c["gap"]
            c["max_dev"] = max(c["max_dev"], c["deviation"])
        elapsed = now - c["t0"]
        alarm = None
        if c["sim_error"]:
            alarm = f"twin: {c['sim_error']}"
        elif c["executed"] and c["deviation"] is not None and c["deviation"] > self.a.deviation_alarm:
            alarm = f"off predicted path by {c['deviation']} m (> {self.a.deviation_alarm} m)"
        elif (c["executed"] and c["sim_eta"] and c["real_done_t"] is None
              and elapsed > self.a.eta_slack * c["sim_eta"] + 5.0):
            alarm = f"real robot {elapsed:.0f} s vs sim eta {c['sim_eta']:.0f} s"
        c["alarm"] = alarm
        c["elapsed"] = round(elapsed, 1)
        if c["executed"] and c["real_done_t"] is not None:
            self.history = ([{"orderId": c["orderId"], "goal": c["goal"],
                              "sim_eta": c["sim_eta"], "sim_path": c["sim_path"],
                              "real_time": round(c["real_done_t"] - c["t0"], 1),
                              "max_dev": round(c["max_dev"], 2)}] + self.history)[:5]
            self.get_logger().info(
                f"MISSION {c['orderId']}: real {c['real_done_t']-c['t0']:.1f} s vs sim eta "
                f"{c['sim_eta']} s, max deviation {c['max_dev']:.2f} m")
            self.cmp = None

    def _publish_states(self):
        now = time.time()
        self._update_cmp(now)
        out = {"broker": f"{self.a.broker_host}:{self.a.broker_port}",
               "connected": self.mq.connected.is_set(),
               "primary": self.a.serial, "agvs": {},
               "twin": {"serial": self.twin, "sim_first": self.sim_first,
                        "current": self.cmp, "proposal": self.proposal,
                        "history": self.history}}
        with self.lock:
            agvs = list(self.agvs.values())
        for g in agvs:
            st = g.state
            kg = self.robot_to_kg(*g.pos) if g.pos else None
            out["agvs"][g.serial] = {
                "connection": g.connection,
                "orderId": st.get("orderId", ""),
                "orderUpdateId": st.get("orderUpdateId", 0),
                "lastNodeId": st.get("lastNodeId", ""),
                "nodesLeft": len(st.get("nodeStates", [])),
                "driving": st.get("driving", False),
                "paused": st.get("paused", False),
                "operatingMode": st.get("operatingMode", ""),
                "battery": (st.get("batteryState") or {}).get("batteryCharge"),
                "errors": [f"{e.get('errorLevel','')} {e.get('errorType','')}"
                           for e in st.get("errors", [])],
                "actions": [f"{x.get('actionType')}:{x.get('actionStatus')}"
                            for x in st.get("actionStates", [])][-3:],
                "pos_map": [round(v, 2) for v in g.pos] if g.pos else None,
                "pos_kg": [round(kg[0], 2), round(kg[1], 2),
                           round(math.degrees(kg[2]))] if kg else None,
                "state_age": round(now - g.t_state, 1) if g.t_state else None,
            }
        self.pub_states.publish(String(data=json.dumps(out)))

    # ---------------------------------------------------------- action ------
    def _execute(self, handle):
        p = handle.request.pose.pose
        gx, gy, gyaw = p.position.x, p.position.y, quat_yaw(p.orientation)
        if self.active is not None and self.active.is_active:
            self.active.abort()                     # preempt
        self.active = handle
        oid, goal_node = self._dispatch(gx, gy, gyaw, wait_for_approval=True)
        if oid is None:
            handle.abort()
            self.get_logger().warn("order discarded by operator (sim-first)")
            return NavigateToPose.Result()
        self.active_order = oid
        g = self._agv(self.a.serial)
        t0 = time.time()
        fb = NavigateToPose.Feedback()
        settled_since = None
        while rclpy.ok():
            if not handle.is_active:
                return NavigateToPose.Result()
            if handle.is_cancel_requested:
                self._send_instant(V.action("cancelOrder",
                                            f"cancel-{int(time.time()*1000)}"))
                handle.canceled()
                self.get_logger().info(f"order {oid} canceled")
                return NavigateToPose.Result()
            with self.lock:
                st, pos, vel = g.state, g.pos, g.vel
            kg = self.robot_to_kg(*pos) if pos else None
            if kg is not None:
                d = math.hypot(kg[0] - gx, kg[1] - gy)
                fb.distance_remaining = float(d)
                handle.publish_feedback(fb)
            done_by_state = (st.get("orderId") == oid and
                             not st.get("nodeStates") and
                             st.get("lastNodeId") == goal_node and
                             not st.get("driving", False))
            done_by_pos = (kg is not None and
                           math.hypot(kg[0] - gx, kg[1] - gy) < self.a.arrive_radius
                           and vel < 0.08)
            if done_by_state or done_by_pos:
                settled_since = settled_since or time.time()
                if time.time() - settled_since > 1.0:
                    handle.succeed()
                    self.get_logger().info(
                        f"ARRIVED order {oid} ({'state' if done_by_state else 'pose'}) "
                        f"after {time.time()-t0:.1f} s")
                    return NavigateToPose.Result()
            else:
                settled_since = None
            for e in st.get("errors", []) if st.get("orderId") == oid else []:
                if e.get("errorLevel") == "FATAL":
                    handle.abort()
                    self.get_logger().error(f"order {oid} FATAL: {e}")
                    return NavigateToPose.Result()
            if time.time() - t0 > self.a.timeout:
                self._send_instant(V.action("cancelOrder",
                                            f"cancel-{int(time.time()*1000)}"))
                handle.abort()
                self.get_logger().warn(f"TIMEOUT order {oid} after {self.a.timeout} s")
                return NavigateToPose.Result()
            time.sleep(0.1)
        return NavigateToPose.Result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker-host", default="127.0.0.1")
    ap.add_argument("--broker-port", type=int, default=1883)
    ap.add_argument("--manufacturer", default="caselab")
    ap.add_argument("--serial", default="ammr20",
                    help="primary AGV serialNumber (its pose becomes /kg_robot_pose)")
    ap.add_argument("--map-id", default="map",
                    help="mapId written into orders (the AGV's map)")
    ap.add_argument("--map-offset", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "YAW_DEG"),
                    help="AGV map origin in the KG frame (default: offset file, else 0 0 0)")
    ap.add_argument("--map-offset-file",
                    default=os.path.expanduser("~/ammr_twin/robot_map_offset.json"))
    ap.add_argument("--heading-offset", type=float, default=180.0,
                    help="deg added to the AGV's reported yaw to get its physical front (AMMR: 180)")
    ap.add_argument("--pose-out-topic", default="/kg_robot_pose")
    ap.add_argument("--arrive-radius", type=float, default=0.35)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--twin-serial", default="ammr20-twin",
                    help="serialNumber of the simulated twin AGV ('' = none)")
    ap.add_argument("--sim-first", action="store_true",
                    help="start in simulate-before-execute mode")
    ap.add_argument("--deviation-alarm", type=float, default=0.8,
                    help="alarm when the real AGV is farther than this from the twin's predicted path [m]")
    ap.add_argument("--eta-slack", type=float, default=1.5,
                    help="alarm when real time > slack * sim eta + 5 s")
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if a.map_offset is None:
        a.map_offset = [0.0, 0.0, 0.0]
        if os.path.exists(a.map_offset_file):
            try:
                a.map_offset = [float(v) for v in
                                json.load(open(a.map_offset_file))["map_offset"]]
            except (ValueError, KeyError, TypeError):
                pass
    rclpy.init()
    n = Master(a)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.mq.stop()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
