"""Tiny CLI to publish semantic commands and echo status.

  ros2 run semantic_nav_bt semantic_cli goto_place display_briefing_area
  ros2 run semantic_nav_bt semantic_cli goto_object tv
  ros2 run semantic_nav_bt semantic_cli pick chair_011
  ros2 run semantic_nav_bt semantic_cli patrol
  ros2 run semantic_nav_bt semantic_cli return_home
"""
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    cmd = {"cmd": args[0]}
    if len(args) > 1:
        if args[0] == "patrol":
            cmd["targets"] = args[1:]
        else:
            cmd["target"] = " ".join(args[1:])
    rclpy.init()
    node = Node("semantic_cli")
    pub = node.create_publisher(String, "semantic_command", 10)
    got = []
    node.create_subscription(String, "semantic_status",
                             lambda m: got.append(m.data), 10)
    import time
    time.sleep(0.3)
    pub.publish(String(data=json.dumps(cmd)))
    print("sent:", cmd)
    t0 = time.time()
    last = None
    while time.time() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.5)
        if got and got[-1] != last:
            last = got[-1]
            s = json.loads(last)
            print("status:", s["status"])
            if s["command"] is None and "complete" in str(s["status"]) \
                    or "at home" in str(s["status"]):
                break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
