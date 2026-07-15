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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class MockAmmr(Node):
    def __init__(self, start):
        super().__init__("ammr_mock")
        self.x, self.y, self.yaw = start
        self.goal = None
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

    def _tick(self):
        if self.goal:
            dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                self.goal = None
            else:
                want = math.atan2(dy, dx)
                err = (want - self.yaw + math.pi) % (2 * math.pi) - math.pi
                self.yaw += max(-1.0, min(1.0, 3.0 * err)) * self.dt
                if abs(err) < 0.5:
                    v = min(0.5, dist)
                    self.x += v * math.cos(self.yaw) * self.dt
                    self.y += v * math.sin(self.yaw) * self.dt
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
        self.pub_state.publish(od)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", nargs=3, type=float, default=[-1.6, -0.1, 0.0])
    args = ap.parse_args()
    rclpy.init()
    n = MockAmmr(tuple(args.start))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
