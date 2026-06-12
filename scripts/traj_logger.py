#!/usr/bin/env python3
"""Log the robot trajectory (map -> base_footprint) to CSV during a sim run.

  python3 traj_logger.py --ros-args -p use_sim_time:=true -p out:=/path/traj.csv
"""
import csv
import os
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TrajLogger(Node):
    def __init__(self):
        super().__init__("traj_logger")
        self.declare_parameter("out", os.path.expanduser("~/blk360_4scan/traj.csv"))
        out = self.get_parameter("out").value
        os.makedirs(os.path.dirname(out), exist_ok=True)
        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["x", "y"])
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.last = None
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(f"traj_logger writing {out}")

    def tick(self):
        try:
            t = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return
        x = t.transform.translation.x
        y = t.transform.translation.y
        if self.last is None or (x - self.last[0]) ** 2 + (y - self.last[1]) ** 2 > 0.01:
            self.w.writerow([round(x, 3), round(y, 3)])
            self.f.flush()
            self.last = (x, y)


def main():
    rclpy.init()
    n = TrajLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()


if __name__ == "__main__":
    main()
