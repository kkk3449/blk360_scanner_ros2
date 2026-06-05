#!/usr/bin/env python3
"""BLK360 stop-scan sequencer.

Drives the high-level "drive a bit, then stop and scan" behaviour on top of the
frontier_exploration_ros2 + Nav2 + Cartographer stack:

  EXPLORING  -> frontier exploration drives the robot via Nav2. We watch the
                straight-line displacement (map -> base TF) from the last scan
                pose; once it reaches `scan_interval_m` we pause. This is net
                displacement, not path length -- wandering in place won't trigger.
  STOPPING   -> call the frontier `control_exploration` service (ACTION_STOP),
                which cancels the active Nav2 goal so the robot halts. Wait until
                the robot is stationary (odom twist ~ 0) or a settle timeout, then
                -- if the previous scan's download is still running -- wait for it
                to finish so two scans never overlap on the device.
  SCANNING   -> publish the scan trigger and wait for `scan_status` CAPTURED: the
                physical scan is then done and the robot may move while the BLK360
                downloads in the background. ERROR/timeout during capture ->
                RECONNECT. (A single-phase scanner that only emits DONE is handled
                too: DONE without CAPTURED is treated as an instant capture.)
  RECONNECT  -> on ERROR / timeout (e.g. the BLK360 link dropped) wait a moment
                and re-trigger. The scanner opens a fresh session every trigger,
                so a re-trigger is itself a reconnect attempt. Retry up to
                `max_scan_retries` times.
  RESUMING   -> call ACTION_START to let frontier exploration continue and go back
                to EXPLORING, leaving the BLK360 to finish downloading while the
                robot drives on toward the next scan point.

The node is intentionally decoupled: it never talks to the BLK360 SDK directly
(that is `blk360_scanner`) and it never owns Nav2 goals (frontier_explorer does).
It only coordinates via the control service, the scan trigger/status topics and
TF/odom.
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import TransformStamped  # noqa: F401  (documentation)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Empty, Bool
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
DONE = "DONE"


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
        # Internal trigger topic (std_msgs/Empty, transient_local) that
        # frontier_explorer / exploration_monitor fire on completion. Used to
        # finalize and print a summary. The public, human-friendly status is the
        # separate Bool topic below.
        self.completion_topic = self.declare_parameter(
            "completion_topic", "/exploration_complete_internal").value
        # Public completion flag: std_msgs/Bool, latched. false while running,
        # true once exploration is finished (echo-friendly).
        self.done_topic = self.declare_parameter("done_topic", "/exploration_complete").value
        # Max time to wait at a scan point for the previous scan's background
        # download to finish before giving up and scanning anyway. Safety net so
        # a missed/dropped DONE can never lock the sequencer forever; the scanner
        # itself still rejects a genuinely overlapping trigger.
        self.download_wait_timeout_s = self.declare_parameter(
            "download_wait_timeout_s", 300.0).value
        # Extra settle pause after the robot is stopped AND the device is free,
        # before triggering the scan -- lets the BLK360 stabilize after the
        # previous transfer instead of scanning the instant the gate clears.
        self.pre_scan_settle_s = self.declare_parameter("pre_scan_settle_s", 4.0).value
        # Settle pause right AFTER a capture (CAPTURED), before the robot starts
        # moving again -- gives the BLK360 a moment before download-while-driving.
        self.post_capture_settle_s = self.declare_parameter("post_capture_settle_s", 5.0).value
        # File the end-of-run summary is appended to (so it isn't lost in console
        # scroll). Empty -> ~/blk360_exploration_summary.log.
        self.summary_log_path = self.declare_parameter("summary_log_path", "").value
        if not self.summary_log_path:
            self.summary_log_path = os.path.join(
                os.path.expanduser("~"), "blk360_exploration_summary.log")

        # --- State ---
        self.state = INIT
        self._last_scan_xy = None        # map-frame pose of the last scan (#1 metric)
        self.last_speed = 0.0
        self.scan_count = 0              # physical scans taken (CAPTURED)
        self.retry_count = 0
        # Capture/download decoupling (#2).
        self._capture_done = False       # current scan reported CAPTURED
        self._capture_counted = False    # this capture already tallied + settle started
        self._t_capture_settle = None    # when the post-capture settle began
        self._scan_failed = False        # current scan reported ERROR during capture
        self._download_instant = False   # scanner emitted DONE without a CAPTURED
        # A single boolean (not a counter): only one scan is ever in flight
        # because the next scan is gated on this clearing. Set in the CAPTURED
        # handler (race-safe vs a fast DONE), cleared on DONE/ERROR.
        self._download_pending = False   # previous scan still downloading
        self._t_download_start = None    # when the pending download began
        self.downloads_done = 0          # downloads that reported DONE
        self._stop_settled = False       # robot has come to rest in STOPPING
        self._waiting_download_logged = False
        self._t_gate_cleared = None      # when robot+device became ready in STOPPING
        # Run accounting for the end-of-run summary.
        self._t_start = None             # set when exploration actually begins
        self._t_complete = None          # sim time of the completion event
        self._completion_pending = False
        self._done = False
        self.scan_times = []             # sim seconds-since-start of each DONE scan
        self.total_retries = 0
        self.failed_scans = 0            # scans abandoned after exhausting retries
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
        # frontier_explorer latches the completion event (transient_local), so a
        # matching QoS lets us catch it even if published before we subscribed.
        completion_qos = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Empty, self.completion_topic, self._on_completion, completion_qos)
        # Latched Bool: false now, true once finished.
        self.done_pub = self.create_publisher(Bool, self.done_topic, completion_qos)
        self.done_pub.publish(Bool(data=False))

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
        # DONE/ERROR are handled in ANY state because a download may finish (or
        # fail) in the background while the robot is already EXPLORING again.
        data = msg.data.strip()
        up = data.upper()
        if up == "CAPTURED":
            if self.state in (SCANNING, RECONNECT):
                self._capture_done = True
                # Mark the download in flight HERE (not in _tick_scanning) so a
                # fast DONE arriving moments later still clears it correctly.
                self._download_pending = True
                self._t_download_start = self.get_clock().now()
                self.get_logger().info("BLK360 capture done (CAPTURED); download starts.")
            return
        if up == "DONE":
            if self.state in (SCANNING, RECONNECT) and not self._capture_done:
                # Single-phase scanner: DONE with no prior CAPTURED == instant capture.
                self._capture_done = True
                self._download_instant = True
            if self._download_pending:
                self._download_pending = False
                self.downloads_done += 1
                self.get_logger().info(
                    f"BLK360 download complete (DONE). downloads={self.downloads_done}")
            return
        if up.startswith("ERROR"):
            if self.state in (SCANNING, RECONNECT) and not self._capture_done:
                # Capture-phase failure: retry path handles it.
                self._scan_failed = True
                self.get_logger().warn(f"BLK360 scan reported failure: {data}")
            elif self._download_pending:
                # Failure during the background download: free the gate so the
                # next scan isn't blocked (that scan's data may be lost).
                self._download_pending = False
                self.get_logger().warn(
                    f"BLK360 background download failed: {data} "
                    "(that scan's data may be lost; continuing).")
            return
        # IDLE / SCANNING are informational; ignore.

    def _on_completion(self, _msg: Empty):
        if self._done or self._completion_pending:
            return
        self._completion_pending = True
        self._t_complete = self.get_clock().now()
        self.get_logger().info(
            "Received exploration_complete event; finalizing once the robot is idle.")

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

    def _displacement_from_last_scan(self):
        """Straight-line map-frame distance from the last scan pose (#1)."""
        xy = self._current_xy()
        if xy is None or self._last_scan_xy is None:
            return 0.0
        return math.dist(xy, self._last_scan_xy)

    def _mark_scan_pose(self):
        """Record the current pose as the reference for the next interval."""
        xy = self._current_xy()
        if xy is not None:
            self._last_scan_xy = xy

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
        self._capture_done = False
        self._capture_counted = False
        self._t_capture_settle = None
        self._scan_failed = False
        self._download_instant = False
        self.trigger_pub.publish(String(data=self.scan_command))
        self._t_scan_sent = self.get_clock().now()
        self.get_logger().info(
            f"BLK360 scan triggered (attempt {self.retry_count + 1}/"
            f"{self.max_scan_retries + 1}).")

    def _scan_elapsed(self):
        if self._t_scan_sent is None:
            return 0.0
        return (self.get_clock().now() - self._t_scan_sent).nanoseconds * 1e-9

    def _elapsed_since_start(self):
        if self._t_start is None:
            return 0.0
        return (self.get_clock().now() - self._t_start).nanoseconds * 1e-9

    # --------------------------------------------------- end-of-run summary
    def _finalize(self, reason):
        """Print the run summary once and move to the terminal DONE state.
        Safe to call multiple times (e.g. completion event then Ctrl-C)."""
        if self._done:
            return
        self._done = True
        end = self._t_complete if self._t_complete is not None else self.get_clock().now()
        total = (end - self._t_start).nanoseconds * 1e-9 if self._t_start else 0.0
        mm, ss = int(total // 60), total % 60.0
        lines = [
            "=" * 56,
            f"BLK360 STOP-SCAN SUMMARY  ({reason})",
            f"  Scan-completion time : {total:.1f} s  ({mm}m {ss:04.1f}s, sim clock)",
            f"  BLK360 scans taken   : {self.scan_count}",
            f"  Downloads completed  : {self.downloads_done}",
        ]
        if self.scan_times:
            lines.append("  Scan times (s)       : ["
                         + ", ".join(f"{t:.1f}" for t in self.scan_times) + "]")
        if self._download_pending:
            lines.append("  Downloads pending    : 1 (still transferring at shutdown)")
        if self.total_retries or self.failed_scans:
            lines.append(f"  Retries / skipped    : {self.total_retries} / {self.failed_scans}")
        lines.append("=" * 56)
        log = self.get_logger()
        for ln in lines:
            log.info(ln)
        # Also persist to a file so the summary survives console scroll.
        try:
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(self.summary_log_path, "a") as f:
                f.write(f"\n[{stamp}] " + lines[1] + "\n")
                f.write("\n".join(lines[2:-1]) + "\n")
            log.info(f"Summary appended to {self.summary_log_path}")
        except Exception as exc:  # noqa: BLE001
            log.warn(f"Could not write summary file '{self.summary_log_path}': {exc}")
        # Flip the public completion flag to true (latched).
        self.done_pub.publish(Bool(data=True))
        self._set_state(DONE)

    # ------------------------------------------------------------ FSM tick
    def _tick(self):
        self._publish_state()

        # Completion can arrive in ANY state (frontier exhaustion or the
        # exploration_monitor's stall stop). Finalize as soon as we're not in the
        # middle of a capture, so the summary is never missed.
        if self._completion_pending and not self._done:
            if not (self.state == SCANNING and not self._capture_done):
                self._finalize("exploration complete")
                return

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
        elif self.state in (ABORTED, DONE):
            pass  # terminal

    def _tick_init(self):
        # Wait until TF + control service are available before doing anything.
        if self._current_xy() is None:
            return
        if not self.control_cli.service_is_ready():
            self.control_cli.wait_for_service(timeout_sec=0.0)
            return
        self._mark_scan_pose()
        self._t_start = self.get_clock().now()
        if self.scan_at_start:
            self._begin_stop()
        else:
            self._set_state(EXPLORING)

    def _tick_exploring(self):
        # Completion is handled centrally in _tick (works from any state).
        d = self._displacement_from_last_scan()
        if d >= self.scan_interval_m:
            self.get_logger().info(
                f"{d:.2f} m from last scan (>= {self.scan_interval_m} m): stopping to scan.")
            self._begin_stop()

    def _begin_stop(self):
        self._stop_sent = False
        self._stop_settled = False
        self._waiting_download_logged = False
        self._t_gate_cleared = None
        self._set_state(STOPPING)

    def _tick_stopping(self):
        if not self._stop_sent:
            self._stop_sent = self._send_control(ControlExploration.Request.ACTION_STOP)
            if self._stop_sent:
                self.get_logger().info("Sent ACTION_STOP; waiting for robot to settle.")
            return
        # Wait for the robot to come to rest (or the settle timeout).
        if not self._stop_settled:
            is_stopped = self.last_speed <= self.stopped_speed_eps
            if not (is_stopped or self._elapsed_in_state() >= self.stop_settle_timeout_s):
                return
            if not is_stopped:
                self.get_logger().warn(
                    "Settle timeout reached before full stop; proceeding anyway.")
            self._stop_settled = True
        # The device serves one scan at a time: if the previous scan is still
        # downloading, hold here until it finishes so the next scan can't overlap.
        if self._download_pending:
            waited = 0.0
            if self._t_download_start is not None:
                waited = (self.get_clock().now() - self._t_download_start).nanoseconds * 1e-9
            if waited >= self.download_wait_timeout_s:
                # Safety net: don't wait forever on a missed DONE. Proceed; the
                # scanner still rejects a genuinely overlapping trigger.
                self.get_logger().warn(
                    f"Previous download did not report DONE within "
                    f"{self.download_wait_timeout_s:.0f}s; assuming finished and scanning.")
                self._download_pending = False
            else:
                if not self._waiting_download_logged:
                    self.get_logger().info(
                        "At next scan point but previous BLK360 download is still "
                        "running; waiting for it to finish before scanning.")
                    self._waiting_download_logged = True
                return
        # Robot stopped and device free: give the BLK360 a moment to stabilize
        # before triggering, rather than scanning the instant the gate clears.
        if self._t_gate_cleared is None:
            self._t_gate_cleared = self.get_clock().now()
            if self.pre_scan_settle_s > 0.0:
                self.get_logger().info(
                    f"Device ready; settling {self.pre_scan_settle_s:.0f}s before scanning.")
        if (self.get_clock().now() - self._t_gate_cleared).nanoseconds * 1e-9 \
                < self.pre_scan_settle_s:
            return
        self.retry_count = 0
        self._begin_scan()

    def _begin_scan(self):
        self._mark_scan_pose()           # measure the next interval from here
        self._set_state(SCANNING)
        self._trigger_scan()

    def _tick_scanning(self):
        # Capture done -> tally once, settle a moment, then resume while the
        # download runs in the background.
        if self._capture_done:
            if not self._capture_counted:
                self._capture_counted = True
                self.scan_count += 1
                t = self._elapsed_since_start()
                self.scan_times.append(round(t, 1))
                self._t_capture_settle = self.get_clock().now()
                if self._download_instant:
                    self.get_logger().info(
                        f"Scan #{self.scan_count} done at t={t:.1f}s (single-phase scanner).")
                else:
                    self.get_logger().info(
                        f"Scan #{self.scan_count} captured at t={t:.1f}s; settling "
                        f"{self.post_capture_settle_s:.0f}s before moving (download in background).")
            # Hold the robot still briefly after capture before resuming.
            if (self.get_clock().now() - self._t_capture_settle).nanoseconds * 1e-9 \
                    < self.post_capture_settle_s:
                return
            self._begin_resume()
            return
        # The capture-phase timeout is short; the long download is not gated here.
        if self._scan_failed or self._scan_elapsed() >= self.scan_timeout_s:
            if not self._scan_failed:
                self.get_logger().warn(
                    f"Capture timed out after {self.scan_timeout_s:.0f}s with no CAPTURED.")
            self._enter_reconnect()

    def _enter_reconnect(self):
        if self.retry_count >= self.max_scan_retries:
            self.failed_scans += 1
            self.get_logger().error(
                f"BLK360 scan failed after {self.retry_count + 1} attempts.")
            if self.on_scan_failure == "abort":
                self.get_logger().error("on_scan_failure=abort: halting sequencer.")
                self._finalize("aborted on scan failure")
            else:
                self.get_logger().warn(
                    "on_scan_failure=resume: skipping this scan and continuing exploration.")
                self._begin_resume()
            return
        self.retry_count += 1
        self.total_retries += 1
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
        # The interval reference pose was set at scan time, so just resume.
        self._set_state(EXPLORING)


def main(args=None):
    rclpy.init(args=args)
    node = StopScanSequencer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Print/persist the summary on shutdown (Ctrl-C or launch teardown); a
        # no-op if the run already finalized. Guarded so a torn-down rclpy
        # context during teardown can't turn this into a non-zero exit.
        try:
            node._finalize("interrupted")
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
