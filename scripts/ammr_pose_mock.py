#!/usr/bin/env python3
"""Mock AMMR emulating the digital_twin_bridge contract (no real robot).

Publishes /ammr/state (nav_msgs/Odometry, map frame, 30 Hz) + /ammr/pose
(PoseStamped) and drives toward any /ammr/goal_pose it receives (simple
point-and-shoot kinematics, 0.5 m/s / 1.0 rad/s). Run with the SYSTEM ROS 2
in the SAME rmw/domain as the twin (local test: AMMR_LOCAL_TEST=1 twin +
default fastdds/domain 0 here):

    python3 scripts/ammr_pose_mock.py [--start X Y YAW]
"""
import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class MockAmmr(Node):
    def __init__(self, start, plan_map=None, vmax=0.5, accel=0.5):
        super().__init__("ammr_mock")
        self.x, self.y, self.yaw = start
        self.goal = None
        self.v = 0.0
        self.vmax, self.accel = vmax, accel
        # optional Nav2 stand-in: plan on the occupancy grid (same A* as the
        # twin, scripts/sim_agv.py) and follow the path with a trapezoidal
        # speed profile instead of driving straight through walls
        self.planner = None
        self.path, self.s, self.L = None, 0.0, 0.0
        if plan_map:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            import sim_agv
            self.planner = sim_agv
            self.grid, self.origin, self.cell = sim_agv.load_grid(plan_map)
            self.get_logger().info(f"planning on {plan_map} (v {vmax} m/s, a {accel} m/s^2)")
        self.pub = self.create_publisher(PoseStamped, "/ammr/pose", 10)
        self.pub_state = self.create_publisher(Odometry, "/ammr/state", 10)
        self.create_subscription(PoseStamped, "/ammr/goal_pose",
                                 self._goal_cb, 10)
        self.dt = 1.0 / 30.0
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"mock AMMR at ({self.x:.2f}, {self.y:.2f})")

    def _goal_cb(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y)
        self.get_logger().info(f"goal received ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
        if self.planner is not None:
            path = self.planner.astar(self.grid, self.origin, self.cell,
                                      (self.x, self.y), self.goal)
            if path is None:
                self.get_logger().warn("no path on the map; falling back to straight line")
                self.path = None
            else:
                self.path, self.s = path, 0.0
                self.L = self.planner.path_length(path)
                self.get_logger().info(f"path {len(path)} pts, {self.L:.2f} m")

    def _tick(self):
        if self.goal and self.path is not None:
            remaining = self.L - self.s
            v_stop = math.sqrt(max(0.0, 2 * self.accel * remaining))
            self.v = min(self.vmax, self.v + self.accel * self.dt, v_stop)
            self.s = min(self.L, self.s + self.v * self.dt)
            self.x, self.y, self.yaw = self.planner.SimAgv._along(self.path, self.s)
            if self.L - self.s < 1e-3:
                self.goal, self.path, self.v = None, None, 0.0
        elif self.goal:
            dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                self.goal = None
            else:
                want = math.atan2(dy, dx)
                err = (want - self.yaw + math.pi) % (2 * math.pi) - math.pi
                self.yaw += max(-1.0, min(1.0, 3.0 * err)) * self.dt
                if abs(err) < 0.5:
                    self.v = min(self.vmax, dist)
                    self.x += self.v * math.cos(self.yaw) * self.dt
                    self.y += self.v * math.sin(self.yaw) * self.dt
        else:
            self.v = 0.0
        m = PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = self.x
        m.pose.position.y = self.y
        m.pose.orientation.z = math.sin(self.yaw / 2)
        m.pose.orientation.w = math.cos(self.yaw / 2)
        self.pub.publish(m)
        od = Odometry()
        od.header = m.header
        od.child_frame_id = "base_footprint"
        od.pose.pose = m.pose
        od.twist.twist.linear.x = self.v * math.cos(self.yaw)
        od.twist.twist.linear.y = self.v * math.sin(self.yaw)
        self.pub_state.publish(od)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", nargs=3, type=float, default=[-1.6, -0.1, 0.0])
    ap.add_argument("--plan-map", default=None,
                    help="map yaml: plan A* paths like Nav2 would instead of driving straight")
    ap.add_argument("--vmax", type=float, default=0.5, help="m/s (AMMR max 1.12)")
    ap.add_argument("--accel", type=float, default=0.5, help="m/s^2 (with --plan-map)")
    args = ap.parse_args()
    rclpy.init()
    n = MockAmmr(tuple(args.start), args.plan_map, args.vmax, args.accel)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
