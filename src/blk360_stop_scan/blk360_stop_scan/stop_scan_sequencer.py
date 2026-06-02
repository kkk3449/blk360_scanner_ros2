#!/usr/bin/env python3
"""BLK360 stop-scan sequencer.

Drives the high-level "drive a bit, then stop and scan" behaviour on top of the
frontier_exploration_ros2 + Nav2 + Cartographer stack:

  EXPLORING  -> frontier exploration drives the robot via Nav2. We watch how far
                the robot has travelled (map -> base TF). After `scan_interval_m`
                metres we pause.
  STOPPING   -> call the frontier `control_exploration` service (ACTION_STOP),
                which cancels the active Nav2 goal so the robot halts. Wait until
                the robot is actually stationary (odom twist ~ 0) or a settle
                timeout elapses.
  SCANNING   -> publish the scan trigger to the BLK360 scanner node and wait for
                its `scan_status` to report DONE or ERROR.
  RECONNECT  -> on ERROR / timeout (e.g. the BLK360 link dropped) wait a moment
                and re-trigger. The scanner opens a fresh session every trigger,
                so a re-trigger is itself a reconnect attempt. Retry up to
                `max_scan_retries` times.
  RESUMING   -> call ACTION_START to let frontier exploration continue, reset the
                travelled-distance accumulator, and go back to EXPLORING.

The node is intentionally decoupled: it never talks to the BLK360 SDK directly
(that is `blk360_scanner`) and it never owns Nav2 goals (frontier_explorer does).
It only coordinates via the control service, the scan trigger/status topics and
TF/odom.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped  # noqa: F401  (documentation)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, \
    ExtrapolationException

from frontier_exploration_ros2.srv import ControlExploration


# FSM states
INIT = "INIT"
EXPLORING = "EXPLORING"
STOPPING = "STOPPING"
SCANNING = "SCANNING"
RECONNECT = "RECONNECT"
RESUMING = "RESUMING"
ABORTED = "ABORTED"


class StopScanSequencer(Node):
    def __init__(self):
        super().__init__("blk360_stop_scan_sequencer")

        # --- Parameters ---
        self.scan_interval_m = self.declare_parameter("scan_interval_m", 2.0).value
        self.global_frame = self.declare_parameter("global_frame", "map").value
        self.robot_base_frame = self.declare_parameter("robot_base_frame", "base_footprint").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.scan_trigger_topic = self.declare_parameter(
            "scan_trigger_topic", "/blk360/scan_trigger").value
        self.scan_status_topic = self.declare_parameter(
            "scan_status_topic", "/blk360/scan_status").value
        self.scan_command = self.declare_parameter("scan_command", "scan").value
        self.control_service_name = self.declare_parameter(
            "control_service_name", "/control_exploration").value
        self.scan_at_start = self.declare_parameter("scan_at_start", True).value
        # How long to wait for the robot to come to rest after a stop request.
        self.stop_settle_timeout_s = self.declare_parameter("stop_settle_timeout_s", 8.0).value
        self.stopped_speed_eps = self.declare_parameter("stopped_speed_eps", 0.02).value
        # How long to wait for a scan to finish before treating it as a failure.
        self.scan_timeout_s = self.declare_parameter("scan_timeout_s", 240.0).value
        # Reconnect / retry behaviour on scan failure.
        self.max_scan_retries = self.declare_parameter("max_scan_retries", 5).value
        self.reconnect_delay_s = self.declare_parameter("reconnect_delay_s", 5.0).value
        # What to do when retries are exhausted: "resume" (keep exploring) or "abort".
        self.on_scan_failure = self.declare_parameter("on_scan_failure", "resume").value
        self.control_rate_hz = self.declare_parameter("control_rate_hz", 5.0).value

        # --- State ---
        self.state = INIT
        self.traveled = 0.0
        self.last_xy = None
        self.last_speed = 0.0
        self.scan_result = None          # None | "DONE" | "ERROR"
        self.scan_count = 0
        self.retry_count = 0
        self._stop_sent = False
        self._start_sent = False
        self._trigger_sent = False
        self._t_state_enter = self.get_clock().now()
        self._t_scan_sent = None
        self._t_retry_until = None
        self._control_pending = False

        # --- TF / IO ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trigger_pub = self.create_publisher(String, self.scan_trigger_topic, 10)
        self.state_pub = self.create_publisher(String, "/blk360_stop_scan/state", 10)
        self.create_subscription(String, self.scan_status_topic, self._on_scan_status, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)

        self.control_cli = self.create_client(ControlExploration, self.control_service_name)

        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f"BLK360 stop-scan sequencer up. interval={self.scan_interval_m} m, "
            f"scan_at_start={self.scan_at_start}, max_retries={self.max_scan_retries}, "
            f"control_service='{self.control_service_name}'")

    # ------------------------------------------------------------------ IO
    def _on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.last_speed = math.hypot(v.x, v.y) + abs(w.z) * 0.0  # linear speed; angular ignored
        # keep angular awareness separately if needed
        self._last_angular = abs(w.z)

    def _on_scan_status(self, msg: String):
        if self.state not in (SCANNING, RECONNECT):
            return
        data = msg.data.strip()
        if data == "DONE":
            self.scan_result = "DONE"
            self.get_logger().info("BLK360 scan reported DONE.")
        elif data.upper().startswith("ERROR"):
            self.scan_result = "ERROR"
            self.get_logger().warn(f"BLK360 scan reported failure: {data}")
        # IDLE / SCANNING are informational; ignore.

    def _publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def _set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f"[FSM] {self.state} -> {new_state}")
            self.state = new_state
            self._t_state_enter = self.get_clock().now()
        self._publish_state()

    def _elapsed_in_state(self):
        return (self.get_clock().now() - self._t_state_enter).nanoseconds * 1e-9

    # ------------------------------------------------- distance via TF
    def _current_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _accumulate_distance(self):
        xy = self._current_xy()
        if xy is None:
            return
        if self.last_xy is not None:
            self.traveled += math.dist(xy, self.last_xy)
        self.last_xy = xy

    def _reset_distance(self):
        self.traveled = 0.0
        self.last_xy = self._current_xy()

    # ------------------------------------------------- control service
    def _send_control(self, action):
        """Fire-and-forget control request; returns False if service not ready."""
        if not self.control_cli.service_is_ready():
            return False
        req = ControlExploration.Request()
        req.action = action
        req.delay_seconds = 0.0
        req.quit_after_stop = False
        self._control_pending = True
        future = self.control_cli.call_async(req)
        future.add_done_callback(self._on_control_response)
        return True

    def _on_control_response(self, future):
        self._control_pending = False
        try:
            resp = future.result()
            self.get_logger().info(
                f"control_exploration -> accepted={resp.accepted}, state={resp.state}, "
                f"'{resp.message}'")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"control_exploration call failed: {exc}")

    def _trigger_scan(self):
        self.scan_result = None
        self.trigger_pub.publish(String(data=self.scan_command))
        self._t_scan_sent = self.get_clock().now()
        self.get_logger().info(
            f"BLK360 scan triggered (attempt {self.retry_count + 1}/"
            f"{self.max_scan_retries + 1}).")

    def _scan_elapsed(self):
        if self._t_scan_sent is None:
            return 0.0
        return (self.get_clock().now() - self._t_scan_sent).nanoseconds * 1e-9

    # ------------------------------------------------------------ FSM tick
    def _tick(self):
        self._publish_state()

        if self.state == INIT:
            self._tick_init()
        elif self.state == EXPLORING:
            self._tick_exploring()
        elif self.state == STOPPING:
            self._tick_stopping()
        elif self.state == SCANNING:
            self._tick_scanning()
        elif self.state == RECONNECT:
            self._tick_reconnect()
        elif self.state == RESUMING:
            self._tick_resuming()
        elif self.state == ABORTED:
            pass  # terminal

    def _tick_init(self):
        # Wait until TF + control service are available before doing anything.
        if self._current_xy() is None:
            return
        if not self.control_cli.service_is_ready():
            self.control_cli.wait_for_service(timeout_sec=0.0)
            return
        self._reset_distance()
        if self.scan_at_start:
            self._begin_stop()
        else:
            self._set_state(EXPLORING)

    def _tick_exploring(self):
        self._accumulate_distance()
        if self.traveled >= self.scan_interval_m:
            self.get_logger().info(
                f"Travelled {self.traveled:.2f} m (>= {self.scan_interval_m} m): stopping to scan.")
            self._begin_stop()

    def _begin_stop(self):
        self._stop_sent = False
        self._set_state(STOPPING)

    def _tick_stopping(self):
        if not self._stop_sent:
            self._stop_sent = self._send_control(ControlExploration.Request.ACTION_STOP)
            if self._stop_sent:
                self.get_logger().info("Sent ACTION_STOP; waiting for robot to settle.")
            return
        # Proceed once the robot is at rest, or after the settle timeout.
        is_stopped = self.last_speed <= self.stopped_speed_eps
        if is_stopped or self._elapsed_in_state() >= self.stop_settle_timeout_s:
            if not is_stopped:
                self.get_logger().warn(
                    "Settle timeout reached before full stop; scanning anyway.")
            self.retry_count = 0
            self._begin_scan()

    def _begin_scan(self):
        self._set_state(SCANNING)
        self._trigger_scan()

    def _tick_scanning(self):
        if self.scan_result == "DONE":
            self.scan_count += 1
            self.get_logger().info(f"Scan #{self.scan_count} complete.")
            self._begin_resume()
            return
        if self.scan_result == "ERROR" or self._scan_elapsed() >= self.scan_timeout_s:
            if self.scan_result != "ERROR":
                self.get_logger().warn(
                    f"Scan timed out after {self.scan_timeout_s:.0f}s with no DONE.")
            self._enter_reconnect()

    def _enter_reconnect(self):
        if self.retry_count >= self.max_scan_retries:
            self.get_logger().error(
                f"BLK360 scan failed after {self.retry_count + 1} attempts.")
            if self.on_scan_failure == "abort":
                self.get_logger().error("on_scan_failure=abort: halting sequencer.")
                self._set_state(ABORTED)
            else:
                self.get_logger().warn(
                    "on_scan_failure=resume: skipping this scan and continuing exploration.")
                self._begin_resume()
            return
        self.retry_count += 1
        self._t_retry_until = self.get_clock().now() + Duration(seconds=self.reconnect_delay_s)
        self.get_logger().warn(
            f"BLK360 link lost; reconnect+retry {self.retry_count}/{self.max_scan_retries} "
            f"in {self.reconnect_delay_s:.0f}s.")
        self._set_state(RECONNECT)

    def _tick_reconnect(self):
        if self.get_clock().now() >= self._t_retry_until:
            self._set_state(SCANNING)
            self._trigger_scan()  # fresh session = reconnect attempt

    def _begin_resume(self):
        self._start_sent = False
        self._set_state(RESUMING)

    def _tick_resuming(self):
        if not self._start_sent:
            self._start_sent = self._send_control(ControlExploration.Request.ACTION_START)
            if self._start_sent:
                self.get_logger().info("Sent ACTION_START; resuming exploration.")
            return
        # Give the control call a tick to register, then resume distance tracking.
        self._reset_distance()
        self._set_state(EXPLORING)


def main(args=None):
    rclpy.init(args=args)
    node = StopScanSequencer()
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
