#!/usr/bin/env python3
"""Mock BLK360 scanner for simulation.

Mimics the topic interface of the real `blk360_scanner` node so the stop-scan
sequencer can be exercised end-to-end without BLK360 hardware:

  sub  /blk360/scan_trigger  (std_msgs/String)  -> start a (fake) scan on `scan` command
  pub  /blk360/scan_status   (std_msgs/String)  -> IDLE | SCANNING | DONE | ERROR: ...

It takes `scan_duration_s` to "scan", then reports DONE. To exercise the
sequencer's reconnect/retry path you can make the first `fail_first_n_scans`
scans fail with an ERROR (simulating a dropped BLK360 link); the scan that the
sequencer retries afterwards then succeeds.
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockBlk360Scanner(Node):
    def __init__(self):
        super().__init__("mock_blk360_scanner")
        self.trigger_command = self.declare_parameter("trigger_command", "scan").value
        self.scan_duration_s = self.declare_parameter("scan_duration_s", 4.0).value
        # Number of scan attempts that should fail before one succeeds (tests reconnect).
        self.fail_first_n_scans = self.declare_parameter("fail_first_n_scans", 0).value

        self.status_pub = self.create_publisher(String, "/blk360/scan_status", 10)
        self.create_subscription(String, "/blk360/scan_trigger", self._on_trigger, 10)

        self._busy = False
        self._attempts = 0
        self._lock = threading.Lock()

        self._publish("IDLE")
        self.get_logger().info(
            f"Mock BLK360 scanner ready (duration={self.scan_duration_s}s, "
            f"fail_first_n_scans={self.fail_first_n_scans}). "
            f"Send '{self.trigger_command}' to /blk360/scan_trigger.")

    def _publish(self, s):
        self.status_pub.publish(String(data=s))

    def _on_trigger(self, msg: String):
        if msg.data != self.trigger_command:
            return
        with self._lock:
            if self._busy:
                self.get_logger().warn("Mock scan already in progress, ignoring trigger.")
                return
            self._busy = True
        self._attempts += 1
        self.get_logger().info(f"Mock scan started (attempt #{self._attempts}).")
        self._publish("SCANNING")
        self._timer = self.create_timer(self.scan_duration_s, self._finish)

    def _finish(self):
        self._timer.cancel()
        if self._attempts <= self.fail_first_n_scans:
            self.get_logger().warn("Mock scan: simulating BLK360 connection loss (ERROR).")
            self._publish("ERROR: simulated connection loss")
        else:
            self.get_logger().info("Mock scan DONE.")
            self._publish("DONE")
        with self._lock:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MockBlk360Scanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
