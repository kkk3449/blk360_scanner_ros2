#!/usr/bin/env python3
"""VDA 5050 AGV adapter for the AMMR (or its mock).

Runs next to the robot's ROS 2 interface (on the robot, or on this PC on the
robot's DDS domain) and makes the robot look like a VDA 5050 AGV:

  MQTT (VDA 5050)                       ROS (robot contract, DataSend/README)
  order            -->  node by node -->  /ammr/goal_pose   (PoseStamped, map)
  instantActions   -->  cancelOrder / startPause / stopPause / initPosition
                                     -->  /ammr/initialpose (PoseWithCovariance)
  state  (1 Hz + on change)   <--  /ammr/state (Odometry) | /ammr/pose
  visualization (20 Hz)       <--  same
  connection (retained, LWT)  <--  ONLINE at connect, OFFLINE on loss/exit

Order execution: nodes are driven in sequenceId order; a node counts as
reached when the AGV is inside its allowedDeviationXY and (nearly) still.
A new orderId preempts the current order; a higher orderUpdateId of the
same order replaces it (only released nodes are executed, base = horizon
here because Nav2 plans its own path between nodes).
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

from . import vda5050 as V


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Adapter(Node):
    def __init__(self, a):
        super().__init__("vda5050_agv_adapter")
        self.a = a
        self.lock = threading.Lock()
        self.hdr = V.Header(a.manufacturer, a.serial)
        self.pos = None                   # (x, y, yaw) map frame
        self.vel = (0.0, 0.0, 0.0)        # vx, vy, omega
        self.t_pos = 0.0
        self.battery = 100.0
        self.charging = False
        # order state
        self.order_id, self.order_update = "", 0
        self.nodes, self.edges = [], []   # remaining (released) nodes/edges
        self.last_node, self.last_seq = "", 0
        self.target = None                # node dict being driven to
        self.t_goal_sent = 0.0
        self.paused = False
        self.action_states = []
        self.errors = []
        self.dirty = threading.Event()    # publish state now

        self.pub_goal = self.create_publisher(PoseStamped, a.goal_topic, 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped,
                                              a.init_topic, 10)
        self.create_subscription(Odometry, a.state_topic, self._odom_cb, 10)
        self.create_subscription(PoseStamped, a.pose_topic, self._pose_cb, 10)
        self.create_subscription(BatteryState, a.battery_topic, self._batt_cb,
                                 10)
        self.create_timer(0.1, self._drive)
        self.create_timer(1.0 / a.state_rate, self._pub_state)
        self.create_timer(1.0 / a.vis_rate, self._pub_vis)
        self.create_timer(0.05, self._flush_dirty)

        conn_topic = V.topic(a.manufacturer, a.serial, "connection")
        self.mq = V.Mqtt(a.broker_host, a.broker_port,
                         client_id=V.client_id(f"agv-{a.serial}"),
                         will=(conn_topic, V.make_connection(
                             V.Header(a.manufacturer, a.serial), "CONNECTIONBROKEN")),
                         on_connect=self._announce,
                         log=lambda s: self.get_logger().info(s))
        self.mq.subscribe(V.topic(a.manufacturer, a.serial, "order"),
                          self._on_order)
        self.mq.subscribe(V.topic(a.manufacturer, a.serial, "instantActions"),
                          self._on_instant)
        self.mq.start()
        self.get_logger().info(
            f"AGV adapter {a.manufacturer}/{a.serial} mapId '{a.map_id}': "
            f"broker {a.broker_host}:{a.broker_port}; goals -> {a.goal_topic}, "
            f"pose <- {a.state_topic}|{a.pose_topic}; stop-mode {a.stop_mode}")

    def _announce(self):
        self.mq.publish(V.topic(self.a.manufacturer, self.a.serial, "connection"),
                        V.make_connection(self.hdr, "ONLINE"), retain=True)
        self.dirty.set()

    # ------------------------------------------------------------ ROS in ---
    def _set(self, x, y, yaw):
        with self.lock:
            self.pos = (x, y, yaw)
            self.t_pos = time.time()

    def _odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.twist.twist
        with self.lock:
            self.vel = (t.linear.x, t.linear.y, t.angular.z)
        self._set(p.x, p.y, quat_yaw(q))

    def _pose_cb(self, m):
        if self.pos is not None and time.time() - self.t_pos < 1.0:
            return                        # odometry has priority
        p, q = m.pose.position, m.pose.orientation
        self._set(p.x, p.y, quat_yaw(q))

    def _batt_cb(self, m):
        with self.lock:
            self.battery = float(m.percentage) * (100.0 if m.percentage <= 1.0 else 1.0)
            self.charging = m.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    # ------------------------------------------------------------ MQTT in --
    def _on_order(self, pt, m):
        oid, upd = m.get("orderId", ""), int(m.get("orderUpdateId", 0))
        nodes = sorted([n for n in m.get("nodes", []) if n.get("released", True)],
                       key=lambda n: n["sequenceId"])
        edges = sorted([e for e in m.get("edges", []) if e.get("released", True)],
                       key=lambda e: e["sequenceId"])
        bad = None
        if not nodes:
            bad = "order has no released nodes"
        for n in nodes:
            mid = (n.get("nodePosition") or {}).get("mapId")
            if mid not in (None, self.a.map_id):
                bad = f"mapId '{mid}' != '{self.a.map_id}'"
        with self.lock:
            if bad:
                self.errors = [V.error("orderError", "WARNING", bad,
                                       {"orderId": oid})]
                self.get_logger().warn(f"order {oid} rejected: {bad}")
                self.dirty.set()
                return
            if oid == self.order_id and upd <= self.order_update:
                self.get_logger().info(f"order {oid} update {upd} ignored (stale)")
                return
            self.errors = []
            preempt = self.order_id and oid != self.order_id and self.target is not None
            self.order_id, self.order_update = oid, upd
            self.nodes, self.edges = nodes, edges
            self.target = None
            self.paused = False
        self.get_logger().info(
            f"order {oid} upd {upd}: {len(nodes)} nodes"
            + (" (preempted previous order)" if preempt else ""))
        self.dirty.set()

    def _on_instant(self, pt, m):
        for act in m.get("actions", []):
            typ, aid = act.get("actionType"), act.get("actionId", "")
            prm = {p["key"]: p["value"] for p in act.get("actionParameters", [])}
            status, desc = "FINISHED", ""
            if typ == "cancelOrder":
                self._cancel()
            elif typ == "startPause":
                with self.lock:
                    self.paused = True
                self._hold()
            elif typ == "stopPause":
                with self.lock:
                    self.paused = False
                    self.t_goal_sent = 0.0          # re-send current target
            elif typ == "initPosition":
                self._init_position(prm)
            else:
                status, desc = "FAILED", f"unsupported actionType {typ}"
                with self.lock:
                    self.errors.append(V.error("actionError", "WARNING", desc,
                                               {"actionId": aid}))
            with self.lock:
                self.action_states = (self.action_states +
                                      [V.action_state(aid, typ, status, desc)])[-10:]
            self.get_logger().info(f"instantAction {typ} {aid}: {status} {desc}")
        self.dirty.set()

    # ------------------------------------------------------------ driving --
    def _goal(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(x), float(y)
        g.pose.orientation.z = math.sin(yaw / 2)
        g.pose.orientation.w = math.cos(yaw / 2)
        self.pub_goal.publish(g)

    def _hold(self):
        with self.lock:
            p = self.pos
        if p and self.a.stop_mode == "hold":
            self._goal(*p)

    def _cancel(self):
        with self.lock:
            had = self.target is not None or bool(self.nodes)
            self.nodes, self.edges, self.target = [], [], None
        if had:
            self._hold()
        else:
            with self.lock:
                self.errors.append(V.error("noOrderToCancel", "WARNING",
                                           "cancelOrder without active order"))

    def _init_position(self, prm):
        out = PoseWithCovarianceStamped()
        out.header.frame_id = "map"
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.pose.position.x = float(prm.get("x", 0.0))
        out.pose.pose.position.y = float(prm.get("y", 0.0))
        th = float(prm.get("theta", 0.0))
        out.pose.pose.orientation.z = math.sin(th / 2)
        out.pose.pose.orientation.w = math.cos(th / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.07
        out.pose.covariance = cov
        self.pub_init.publish(out)
        with self.lock:
            self.last_node = str(prm.get("lastNodeId", "") or "")

    def _drive(self):
        with self.lock:
            p, vel = self.pos, self.vel
            if p is None or self.paused:
                return
            if self.target is None and self.nodes:
                self.target = self.nodes[0]
                self.t_goal_sent = 0.0
            tgt = self.target
        if tgt is None:
            return
        np_ = tgt["nodePosition"]
        d = math.hypot(np_["x"] - p[0], np_["y"] - p[1])
        still = math.hypot(vel[0], vel[1]) < self.a.still_speed
        tol = float(np_.get("allowedDeviationXY", self.a.arrive_radius)) or self.a.arrive_radius
        # reached: inside tolerance and still (or never even had to move)
        if d < tol and (still or self.t_goal_sent == 0.0):
            with self.lock:
                self.last_node, self.last_seq = tgt["nodeId"], tgt["sequenceId"]
                if self.nodes and self.nodes[0]["nodeId"] == tgt["nodeId"]:
                    self.nodes.pop(0)
                self.edges = [e for e in self.edges
                              if e["sequenceId"] > tgt["sequenceId"]]
                self.target = None
            self.get_logger().info(
                f"node {tgt['nodeId']} reached (d={d:.2f} m); {len(self.nodes)} left")
            self.dirty.set()
            return
        if time.time() - self.t_goal_sent > self.a.resend_period:
            self._goal(np_["x"], np_["y"], float(np_.get("theta", 0.0)))
            with self.lock:
                self.t_goal_sent = time.time()

    # ------------------------------------------------------------ MQTT out -
    def _state_msg(self):
        with self.lock:
            p, v = self.pos, self.vel
            pos = ({"x": p[0], "y": p[1], "theta": p[2], "mapId": self.a.map_id,
                    "positionInitialized": True} if p else
                   {"x": 0.0, "y": 0.0, "theta": 0.0, "mapId": self.a.map_id,
                    "positionInitialized": False})
            driving = self.target is not None and not self.paused
            node_states = [{"nodeId": n["nodeId"], "sequenceId": n["sequenceId"],
                            "released": True, "nodePosition": n["nodePosition"]}
                           for n in self.nodes]
            edge_states = [{"edgeId": e["edgeId"], "sequenceId": e["sequenceId"],
                            "released": True} for e in self.edges]
            return V.make_state(
                self.hdr, order_id=self.order_id, order_update_id=self.order_update,
                last_node_id=self.last_node, last_node_seq=self.last_seq,
                node_states=node_states, edge_states=edge_states,
                agv_position=pos,
                velocity={"vx": v[0], "vy": v[1], "omega": v[2]},
                driving=driving, paused=self.paused,
                action_states=list(self.action_states),
                battery_charge=self.battery, charging=self.charging,
                errors=list(self.errors))

    def _pub_state(self):
        if self.mq.connected.is_set():
            self.mq.publish(V.topic(self.a.manufacturer, self.a.serial, "state"),
                            self._state_msg())

    def _flush_dirty(self):
        if self.dirty.is_set():
            self.dirty.clear()
            self._pub_state()

    def _pub_vis(self):
        with self.lock:
            p, v = self.pos, self.vel
        if p and self.mq.connected.is_set():
            self.mq.publish(V.topic(self.a.manufacturer, self.a.serial,
                                    "visualization"),
                            V.make_visualization(self.hdr, p[0], p[1], p[2],
                                                 self.a.map_id, *v), qos=0)

    def shutdown(self):
        try:
            self.mq.publish(V.topic(self.a.manufacturer, self.a.serial, "connection"),
                            V.make_connection(self.hdr, "OFFLINE"), retain=True)
            time.sleep(0.2)
        finally:
            self.mq.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker-host", default="127.0.0.1")
    ap.add_argument("--broker-port", type=int, default=1883)
    ap.add_argument("--manufacturer", default="caselab")
    ap.add_argument("--serial", default="ammr20")
    ap.add_argument("--map-id", default="map")
    ap.add_argument("--goal-topic", default="/ammr/goal_pose")
    ap.add_argument("--state-topic", default="/ammr/state")
    ap.add_argument("--pose-topic", default="/ammr/pose")
    ap.add_argument("--init-topic", default="/ammr/initialpose")
    ap.add_argument("--battery-topic", default="/battery_state")
    ap.add_argument("--arrive-radius", type=float, default=0.35)
    ap.add_argument("--still-speed", type=float, default=0.08)
    ap.add_argument("--resend-period", type=float, default=8.0,
                    help="re-publish the current node goal (the robot relay is edge-triggered)")
    ap.add_argument("--state-rate", type=float, default=1.0)
    ap.add_argument("--vis-rate", type=float, default=20.0)
    ap.add_argument("--stop-mode", choices=("hold", "none"), default="hold",
                    help="cancelOrder/startPause: hold = send current pose as goal")
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    n = Adapter(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.shutdown()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
