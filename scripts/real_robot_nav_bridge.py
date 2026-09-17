#!/usr/bin/env python3
"""Real-AMMR bridge for the semantic mission stack (replaces Gazebo + Nav2).

The mission BT (mission_bt_cpp) talks to a Nav2 `navigate_to_pose` action
server and reads `/amcl_pose`; the UI and the Isaac KG twin read `/amcl_pose`
too.  On the real robot we only have the digital_twin_bridge contract
(~/Downloads/DataSend/README.md):

    /ammr/state      nav_msgs/Odometry   robot pose in the ROBOT's map frame
    /ammr/goal_pose  geometry_msgs/PoseStamped  goal in the ROBOT's map frame
                     -> dt_goal_relay -> on-board Nav2

This node sits in between, in ONE domain (the robot's: CycloneDDS / 56):

  * /ammr/state -> SE(2) offset (robot map -> KG/vis_n2 map) -> /amcl_pose
    (latched QoS, like Nav2 AMCL) so BT / UI / Isaac see the robot in the
    semantic-DB frame;
  * a `navigate_to_pose` action server: goal (KG frame) -> inverse offset ->
    /ammr/goal_pose; SUCCEEDED when the robot is within --arrive-radius of
    the goal and has settled, ABORTED on --timeout; cancel/preempt re-sends
    the robot's current pose as goal so it stops.

Offset: same convention as isaacsim_ammr_twin.py (--map-offset X Y YAW_DEG,
or --anchor X Y YAW_RAD to calibrate from the first pose while the robot
stands on the scanned spot motor_035: 0.7365 0.7815 0.809 / 3.951).

    source /opt/ros/jazzy/setup.bash
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=56 \\
    CYCLONEDDS_URI=file://$HOME/cyclonedds_isaac.xml \\
    python3 scripts/real_robot_nav_bridge.py --map-offset -1.7625 -0.7828 -136.627
"""
import argparse
import math
import os
import json
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
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Bridge(Node):
    def __init__(self, a):
        super().__init__("real_robot_nav_bridge")
        self.a = a
        self.off = [a.map_offset[0], a.map_offset[1],
                    math.radians(a.map_offset[2])]
        self.anchor = tuple(a.anchor) if a.anchor else None
        # reported yaw vs physical front (AMMR reports its front 180 deg off)
        self.h = math.radians(a.heading_offset)
        self.robot = None            # (x, y, yaw) in the robot's map frame
        self.robot_t = 0.0
        self.speed = 0.0
        self._last = None
        self.lock = threading.Lock()
        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped,
                                              a.pose_out_topic, latched)
        self.pub_goal = self.create_publisher(PoseStamped, a.goal_topic, 10)
        self.create_subscription(Odometry, a.state_topic, self._state_cb, 10,
                                 callback_group=cb)
        self.create_subscription(PoseStamped, a.pose_topic, self._pose_cb, 10,
                                 callback_group=cb)
        from std_msgs.msg import String
        self.create_subscription(String, "/semantic_stop", self._stop_cb, 10,
                                 callback_group=cb)
        # live map-offset updates from the console's robot-map registration
        latched_in = QoSProfile(depth=1,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/kg_map_offset", self._offset_cb,
                                 latched_in, callback_group=cb)
        # direct goals in the KG frame (Isaac cone drag / console map) -> robot frame relay
        self.create_subscription(PoseStamped, "/kg_goal_pose", self._kg_goal_cb,
                                 10, callback_group=cb)
        # console-picked initial pose in the KG frame -> robot frame; the robot
        # side must relay /ammr/initialpose -> /initialpose (AMCL) like it
        # relays /ammr/goal_pose -> /goal_pose
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped,
                                              a.init_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/kg_initialpose",
                                 self._kg_init_cb, 10, callback_group=cb)
        self.server = ActionServer(
            self, NavigateToPose, "navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda h: CancelResponse.ACCEPT,
            callback_group=cb)
        self.active = None
        self.n = 0
        self.get_logger().info(
            f"bridge up: {a.state_topic} -> {a.pose_out_topic} (KG frame), "
            f"navigate_to_pose -> {a.goal_topic}; offset "
            f"{self.off[0]:.4f} {self.off[1]:.4f} {math.degrees(self.off[2]):.3f}"
            + (f"; ANCHOR pending {self.anchor}" if self.anchor else ""))

    # ---- frames ----
    def robot_to_kg(self, x, y, yaw):
        ox, oy, oyaw = self.off
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * x - s * y, oy + s * x + c * y, wrap(yaw + oyaw + self.h))

    def kg_to_robot(self, x, y, yaw):
        ox, oy, oyaw = self.off
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        return (c * dx - s * dy, s * dx + c * dy, wrap(yaw - oyaw - self.h))

    def _calibrate(self, rp):
        ax, ay, ayaw = self.anchor            # ayaw = PHYSICAL heading at the spot
        rx, ry, ryaw = rp
        oyaw = wrap(ayaw - self.h - ryaw)
        c, s = math.cos(oyaw), math.sin(oyaw)
        self.off = [ax - (c * rx - s * ry), ay - (s * rx + c * ry), oyaw]
        self.anchor = None
        self.get_logger().info(
            f"CALIBRATED map-offset: {self.off[0]:.4f} {self.off[1]:.4f} "
            f"{math.degrees(self.off[2]):.3f}  (reuse via --map-offset)")

    # ---- robot pose in ----
    def _set(self, x, y, yaw):
        now = time.time()
        with self.lock:
            if self.anchor is not None:
                self._calibrate((x, y, yaw))
            if self._last is not None:
                dt = max(now - self._last[3], 1e-3)
                self.speed = math.hypot(x - self._last[0], y - self._last[1]) / dt
            self._last = (x, y, yaw, now)
            self.robot = (x, y, yaw)
            self.robot_t = now
            kx, ky, kyaw = self.robot_to_kg(x, y, yaw)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = kx
        m.pose.pose.position.y = ky
        m.pose.pose.orientation.z = math.sin(kyaw / 2)
        m.pose.pose.orientation.w = math.cos(kyaw / 2)
        self.pub_amcl.publish(m)
        self.n += 1
        if self.n % 150 == 1:
            self.get_logger().info(
                f"robot map=({x:.2f},{y:.2f}) -> KG=({kx:.2f},{ky:.2f},"
                f"{math.degrees(kyaw):.0f}deg) v={self.speed:.2f}")

    def _state_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self._set(p.x, p.y, quat_yaw(q))

    def _pose_cb(self, m):
        if self.robot is not None and time.time() - self.robot_t < 1.0:
            return                                  # /ammr/state has priority
        p, q = m.pose.position, m.pose.orientation
        self._set(p.x, p.y, quat_yaw(q))

    # ---- goal out ----
    def _send_robot_goal(self, rx, ry, ryaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = rx
        g.pose.position.y = ry
        g.pose.orientation.z = math.sin(ryaw / 2)
        g.pose.orientation.w = math.cos(ryaw / 2)
        self.pub_goal.publish(g)

    def _kg_goal_cb(self, m):
        p, q = m.pose.position, m.pose.orientation
        rx, ry, ryaw = self.kg_to_robot(p.x, p.y, quat_yaw(q))
        self._send_robot_goal(rx, ry, ryaw)
        self.get_logger().info(
            f"DIRECT goal KG=({p.x:.2f},{p.y:.2f}) -> robot=({rx:.2f},{ry:.2f})")

    def _offset_cb(self, m):
        try:
            off = json.loads(m.data)["map_offset"]
        except (ValueError, KeyError, TypeError):
            return
        if self.anchor is not None:
            self.get_logger().warn("map offset from console ignored: --anchor calibration pending")
            return
        with self.lock:
            self.off = [float(off[0]), float(off[1]), math.radians(float(off[2]))]
        self.get_logger().info(
            f"map offset updated from console: {off[0]:.4f} {off[1]:.4f} {off[2]:.3f} deg")

    def _kg_init_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rx, ry, ryaw = self.kg_to_robot(p.x, p.y, quat_yaw(q))
        out = PoseWithCovarianceStamped()
        out.header.frame_id = self.a.robot_map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.pose.position.x, out.pose.pose.position.y = rx, ry
        out.pose.pose.orientation.z = math.sin(ryaw / 2)
        out.pose.pose.orientation.w = math.cos(ryaw / 2)
        out.pose.covariance = list(m.pose.covariance)
        self.pub_init.publish(out)
        self.get_logger().info(
            f"INITIALPOSE KG=({p.x:.2f},{p.y:.2f},{math.degrees(quat_yaw(q)):.0f}deg) "
            f"-> robot=({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}deg) on {self.a.init_topic}")

    def _stop_cb(self, _m):
        """Operator STOP: abort the active nav goal; --stop-mode hold re-sends
        the current pose as a goal (robot decelerates to it), none sends
        nothing to the robot (only the mission side is cancelled)."""
        with self.lock:
            r = self.robot
        if r and self.a.stop_mode == "hold":
            self._send_robot_goal(*r)
        h = self.active
        if h is not None and h.is_active:
            try:
                h.abort()
            except Exception:
                pass
        self.get_logger().warn(
            f"STOP received: nav goal aborted ({self.a.stop_mode} mode)")

    def _execute(self, handle):
        p = handle.request.pose.pose
        gx, gy, gyaw = p.position.x, p.position.y, quat_yaw(p.orientation)
        if self.active is not None and self.active.is_active:
            self.active.abort()                     # preempt
        self.active = handle
        rx, ry, ryaw = self.kg_to_robot(gx, gy, gyaw)
        self.get_logger().info(
            f"GOAL KG=({gx:.2f},{gy:.2f}) -> robot=({rx:.2f},{ry:.2f}) "
            f"radius {self.a.arrive_radius} m")
        self._send_robot_goal(rx, ry, ryaw)
        t0 = time.time()
        resend = t0
        settled_since = None
        fb = NavigateToPose.Feedback()
        rate = 10.0
        while rclpy.ok():
            if not handle.is_active:                # preempted by a newer goal
                return NavigateToPose.Result()
            if handle.is_cancel_requested:
                with self.lock:
                    r = self.robot
                if r:
                    self._send_robot_goal(*r)       # stop where you are
                handle.canceled()
                self.get_logger().info("goal canceled")
                return NavigateToPose.Result()
            with self.lock:
                r = self.robot
                kg = self.robot_to_kg(*r) if r else None
                spd = self.speed
            if kg is not None:
                d = math.hypot(kg[0] - gx, kg[1] - gy)
                fb.distance_remaining = float(d)
                handle.publish_feedback(fb)
                if d < self.a.arrive_radius and spd < 0.08:
                    settled_since = settled_since or time.time()
                    if time.time() - settled_since > 1.0:
                        handle.succeed()
                        self.get_logger().info(
                            f"ARRIVED d={d:.2f} m after {time.time()-t0:.1f} s")
                        return NavigateToPose.Result()
                else:
                    settled_since = None
            if time.time() - resend > 8.0 and (kg is None or
                                               math.hypot(kg[0]-gx, kg[1]-gy) > self.a.arrive_radius):
                self._send_robot_goal(rx, ry, ryaw)   # relay is edge-triggered
                resend = time.time()
            if time.time() - t0 > self.a.timeout:
                handle.abort()
                self.get_logger().warn(f"TIMEOUT after {self.a.timeout} s")
                return NavigateToPose.Result()
            time.sleep(1.0 / rate)
        return NavigateToPose.Result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-offset", nargs=3, type=float,
                    default=[-1.7625, -0.7828, -136.627],
                    metavar=("X", "Y", "YAW_DEG"),
                    help="robot map origin in the KG/vis_n2 frame "
                         "(default = 2026-09-08 re-anchor)")
    ap.add_argument("--anchor", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "YAW_RAD"))
    ap.add_argument("--state-topic", default="/ammr/state")
    ap.add_argument("--pose-topic", default="/ammr/pose")
    ap.add_argument("--goal-topic", default="/ammr/goal_pose")
    ap.add_argument("--pose-out-topic", default="/kg_robot_pose",
                    help="robot pose in the KG frame (latched). NOT /amcl_pose: the "
                         "robot's dt_state_publisher subscribes /amcl_pose on domain 56")
    ap.add_argument("--heading-offset", type=float, default=180.0,
                    help="deg added to the robot's reported yaw to get its physical "
                         "front (AMMR: 180); applied to poses and goals")
    ap.add_argument("--init-topic", default="/ammr/initialpose",
                    help="robot-frame initial pose relayed from /kg_initialpose")
    ap.add_argument("--robot-map-frame", default="map")
    ap.add_argument("--stop-mode", choices=("hold", "none"), default="hold",
                    help="what to send the robot on STOP: hold = current pose "
                         "as goal (default), none = nothing")
    ap.add_argument("--arrive-radius", type=float, default=0.35)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--map-offset-file",
                    default=os.path.expanduser("~/ammr_twin/robot_map_offset.json"),
                    help="offset json written by the console's robot-map registration; "
                         "used when --map-offset is not given explicitly")
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if "--map-offset" not in sys.argv and os.path.exists(a.map_offset_file):
        try:
            a.map_offset = json.load(open(a.map_offset_file))["map_offset"]
            print(f"map offset from {a.map_offset_file}: {a.map_offset}")
        except (OSError, ValueError, KeyError):
            pass
    rclpy.init()
    node = Bridge(a)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
