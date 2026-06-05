#!/usr/bin/env python3
"""Mock BLK360 scanner for simulation.

Mimics the topic interface of the real `blk360_scanner` node so the stop-scan
sequencer can be exercised end-to-end without BLK360 hardware:

  sub  /blk360/scan_trigger  (std_msgs/String)  -> start a (fake) scan on `scan` command
  pub  /blk360/scan_status   (std_msgs/String)  -> IDLE | SCANNING | CAPTURED | DONE | ERROR: ...

Two-phase like the real BLK360: it takes `scan_duration_s` to "scan" (robot must
stay still), reports CAPTURED, then takes a further `download_duration_s` to
"download" (robot may move) before reporting DONE. To exercise the sequencer's
reconnect/retry path you can make the first `fail_first_n_scans` scans fail with
an ERROR at capture time (simulating a dropped BLK360 link); the scan that the
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
        # Time spent "downloading" after capture (robot may move meanwhile).
        self.download_duration_s = self.declare_parameter("download_duration_s", 10.0).value
        # Number of scan attempts that should fail before one succeeds (tests reconnect).
        self.fail_first_n_scans = self.declare_parameter("fail_first_n_scans", 0).value

        self.status_pub = self.create_publisher(String, "/blk360/scan_status", 10)
        self.create_subscription(String, "/blk360/scan_trigger", self._on_trigger, 10)

        self._busy = False
        self._attempts = 0
        self._lock = threading.Lock()

        self._publish("IDLE")
        self.get_logger().info(
            f"Mock BLK360 scanner ready (scan={self.scan_duration_s}s, "
            f"download={self.download_duration_s}s, "
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
        self._timer = self.create_timer(self.scan_duration_s, self._capture_done)

    def _capture_done(self):
        self._timer.cancel()
        # A dropped link is modelled as a capture-time failure: no CAPTURED/DONE.
        if self._attempts <= self.fail_first_n_scans:
            self.get_logger().warn("Mock scan: simulating BLK360 connection loss (ERROR).")
            self._publish("ERROR: simulated connection loss")
            with self._lock:
                self._busy = False
            return
        self.get_logger().info("Mock scan CAPTURED; downloading...")
        self._publish("CAPTURED")
        # Stay busy through the download phase, then report DONE.
        self._timer = self.create_timer(self.download_duration_s, self._download_done)

    def _download_done(self):
        self._timer.cancel()
        self.get_logger().info("Mock scan DONE (download complete).")
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
