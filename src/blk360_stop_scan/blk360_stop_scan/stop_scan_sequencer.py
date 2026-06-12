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
import json
import math
import os
import time

import colorsys

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, TransformStamped  # noqa: F401
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Empty, Bool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, \
    ExtrapolationException

from frontier_exploration_ros2.srv import ControlExploration

from blk360_stop_scan.visibility import (
    new_visible_ratio, union_visible_mask, visible_mask)


# FSM states
INIT = "INIT"
EXPLORING = "EXPLORING"
STOPPING = "STOPPING"
SCANNING = "SCANNING"
RECONNECT = "RECONNECT"
RESUMING = "RESUMING"
COVERING = "COVERING"   # coverage-completion: driving to an uncovered pocket
ABORTED = "ABORTED"
DONE = "DONE"


class StopScanSequencer(Node):
    def __init__(self):
        super().__init__("blk360_stop_scan_sequencer")

        # --- Parameters ---
        # Distance factor: how far the robot travels (straight line, map frame)
        # between *evaluating* whether to scan. A scan candidate is raised every
        # this many metres.
        self.scan_interval_m = self.declare_parameter("scan_interval_m", 2.0).value
        # Coverage factor: rough effective coverage radius of a BLK360-G1 scan for
        # registration purposes. A candidate whose position lies within this radius
        # of any previous scan is considered already covered and is SKIPPED -- so
        # scans end up ~this far apart (less overlap than pure distance), while
        # still close enough to overlap for registration. (BLK360-G1 ranges to
        # ~60 m but accuracy/occlusion limit the useful, dense, registerable area;
        # this is that rough effective radius, not the max laser range.)
        self.scan_coverage_radius_m = self.declare_parameter(
            "scan_coverage_radius_m", 4.0).value
        # Coverage model for the skip decision:
        #   "disk"       -- isotropic: candidate within R of any prior scan is
        #                   covered (ignores walls; legacy behaviour).
        #   "visibility" -- occlusion-aware: covered area is the union of
        #                   ray-cast visibility regions B(s_i, R) on the live
        #                   occupancy map; a candidate is skipped when the
        #                   NEW visible area its own region B(c, R) would add
        #                   is below `min_new_visible_ratio` of |B(c, R)|.
        self.coverage_model = self.declare_parameter("coverage_model", "disk").value
        # Rays per visibility region (angular resolution = 360/num_rays deg).
        self.visibility_num_rays = self.declare_parameter(
            "visibility_num_rays", 720).value
        # Marginal-gain thresholds: a candidate triggers a scan when EITHER
        # criterion says the scan is worthwhile --
        #   ratio: the fraction of B(c,R) that is new  >= min_new_visible_ratio
        #   area : the absolute new visible area (m^2) >= min_new_visible_area_m2
        # The area criterion keeps small occlusion pockets (shadows behind
        # obstacles) scannable even when they are a small fraction of a large
        # sensor footprint (with BLK360-scale R the ratio alone dilutes).
        self.min_new_visible_ratio = self.declare_parameter(
            "min_new_visible_ratio", 0.30).value
        self.min_new_visible_area_m2 = self.declare_parameter(
            "min_new_visible_area_m2", 5.0).value
        # Occupancy value (0-100) at/above which a cell blocks a ray.
        self.occupied_thresh = self.declare_parameter("occupied_thresh", 65).value
        # Unknown cells block rays (conservative: never claim coverage of
        # space the map has not confirmed free).
        self.unknown_blocks_ray = self.declare_parameter(
            "unknown_blocks_ray", True).value
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        # Coverage completion (visibility model only): when frontier exploration
        # finishes, do NOT finalize while uncovered free-space pockets remain.
        # Cluster the uncovered cells, drive into each pocket with Nav2, scan,
        # and repeat until every pocket smaller than `min_pocket_area_m2` --
        # i.e. no white left in the BLK coverage map.
        self.coverage_completion = self.declare_parameter(
            "coverage_completion", False).value
        self.min_pocket_area_m2 = self.declare_parameter(
            "min_pocket_area_m2", 1.0).value
        self.covering_nav_timeout_s = self.declare_parameter(
            "covering_nav_timeout_s", 90.0).value
        self.covering_max_scans = self.declare_parameter(
            "covering_max_scans", 8).value
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
        # Directory for per-run JSON records (scan positions + metrics) used by
        # scripts/metrics.py. Empty -> ~/blk360_runs.
        self.run_data_dir = self.declare_parameter("run_data_dir", "").value
        if not self.run_data_dir:
            self.run_data_dir = os.path.join(os.path.expanduser("~"), "blk360_runs")

        # --- State ---
        self.state = INIT
        self._last_scan_xy = None        # reference pose for the next candidate
        self.scan_positions = []         # (x,y) map-frame centres of taken scans
        self.scan_polygons = []          # per-scan visibility polygon (Nx2) or None
        self.skip_events = []            # per-candidate decision log (for analysis)
        self.scans_skipped = 0           # candidates skipped as already-covered
        # Latest occupancy map (visibility model input).
        self._map_grid = None            # int16 (H, W), -1 unknown / 0..100
        self._map_res = None
        self._map_origin = None
        # Coverage-completion phase state.
        self._covering = False           # in the coverage-completion phase
        self._cover_pending = False      # completion arrived; start when stable
        self._cover_done = False         # Nav2 reached the current pocket
        self._cover_failed = False       # Nav2 rejected/aborted the goal
        self._cover_goal_handle = None
        self._cover_target = None        # (x, y) of the current pocket goal
        self._failed_targets = []        # pocket goals Nav2 could not reach
        self.covering_scans = 0          # scans taken during coverage completion
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
        self._t_last_marker = None
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
        # Latched per-scan coverage markers (colored disks + centres) for RViz.
        self.coverage_pub = self.create_publisher(
            MarkerArray, "/blk360/scan_coverage", completion_qos)

        self.control_cli = self.create_client(ControlExploration, self.control_service_name)
        # Direct Nav2 access for the coverage-completion phase (the frontier
        # explorer owns Nav2 goals only while exploration is running).
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Live occupancy map for the visibility coverage model. Volatile sub is
        # compatible with both volatile and transient_local publishers.
        map_qos = QoSProfile(
            depth=2, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)

        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self._tick)

        cov_desc = (f"visibility B(s,R), tau={self.min_new_visible_ratio}, "
                    f"{self.visibility_num_rays} rays"
                    if self.coverage_model == "visibility" else "isotropic disk")
        self.get_logger().info(
            f"BLK360 stop-scan sequencer up. candidate every {self.scan_interval_m} m, "
            f"coverage model: {cov_desc}, R={self.scan_coverage_radius_m} m, "
            f"scan_at_start={self.scan_at_start}, max_retries={self.max_scan_retries}.")

    # ------------------------------------------------------------------ IO
    def _on_map(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0:
            return
        self._map_grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self._map_res = float(msg.info.resolution)
        self._map_origin = (float(msg.info.origin.position.x),
                            float(msg.info.origin.position.y))

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
            f"  Scans skipped (cover): {self.scans_skipped}",
            f"  Downloads completed  : {self.downloads_done}",
        ]
        if self.coverage_completion:
            lines.append(f"  Coverage-completion  : {self.covering_scans} extra "
                         f"scan(s), {len(self._failed_targets)} unreachable pocket(s)")
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
        # Structured per-run record (scan positions + metrics) for offline analysis.
        try:
            fstamp = time.strftime("%Y%m%d_%H%M%S")
            os.makedirs(self.run_data_dir, exist_ok=True)
            record = {
                "timestamp": stamp,
                "reason": reason,
                "completion_time_s": round(total, 2),
                "scan_count": self.scan_count,
                "scans_skipped": self.scans_skipped,
                "downloads_done": self.downloads_done,
                "total_retries": self.total_retries,
                "failed_scans": self.failed_scans,
                "scan_times_s": [round(t, 2) for t in self.scan_times],
                "scan_positions": [[round(float(x), 3), round(float(y), 3)]
                                   for (x, y) in self.scan_positions],
                "scan_interval_m": self.scan_interval_m,
                "scan_coverage_radius_m": self.scan_coverage_radius_m,
                "coverage_model": self.coverage_model,
            }
            if self.coverage_model == "visibility":
                record["min_new_visible_ratio"] = self.min_new_visible_ratio
                record["min_new_visible_area_m2"] = self.min_new_visible_area_m2
                record["visibility_num_rays"] = self.visibility_num_rays
                record["skip_events"] = self.skip_events
                record["coverage_completion"] = self.coverage_completion
                if self.coverage_completion:
                    record["covering_scans"] = self.covering_scans
                    record["min_pocket_area_m2"] = self.min_pocket_area_m2
                    record["unreachable_pockets"] = [
                        [round(float(x), 3), round(float(y), 3)]
                        for (x, y) in self._failed_targets]
                # Final-map polygons: the panoramic footprint, not the
                # capture-time wedge clipped by then-unknown space.
                self._refresh_scan_polygons()
                record["scan_visibility_polygons"] = [
                    None if poly is None else
                    [[round(float(px), 3), round(float(py), 3)]
                     for (px, py) in poly]
                    for poly in self.scan_polygons]
                # Union visible coverage on the final map (the figure-of-merit
                # the disk model overestimates behind walls).
                if self._map_grid is not None and self.scan_positions:
                    cov = union_visible_mask(
                        self._map_grid, self._map_res, self._map_origin,
                        self.scan_positions, self.scan_coverage_radius_m,
                        num_rays=self.visibility_num_rays,
                        occ_thresh=self.occupied_thresh,
                        unknown_blocks=self.unknown_blocks_ray)
                    cell = self._map_res * self._map_res
                    free = (self._map_grid >= 0) & (self._map_grid <= 25)
                    record["visible_covered_area_m2"] = round(
                        float(cov.sum()) * cell, 2)
                    record["map_free_area_m2"] = round(float(free.sum()) * cell, 2)
                    record["visible_room_coverage_pct"] = round(
                        100.0 * float((cov & free).sum()) / max(int(free.sum()), 1), 1)
            path = os.path.join(self.run_data_dir, f"run_{fstamp}.json")
            with open(path, "w") as f:
                json.dump(record, f, indent=2)
            log.info(f"Run record written to {path}")
        except Exception as exc:  # noqa: BLE001
            log.warn(f"Could not write run record: {exc}")
        # Flip the public completion flag to true (latched).
        self.done_pub.publish(Bool(data=True))
        self._set_state(DONE)

    # ------------------------------------------------------------ FSM tick
    def _tick(self):
        self._publish_state()

        # Periodically re-render coverage markers on the maturing map so the
        # visibility polygons grow from capture-time wedges to the full
        # panoramic footprint as unknown space gets mapped.
        if (self.coverage_model == "visibility" and self.scan_positions
                and not self._done):
            t_now = self.get_clock().now()
            if (self._t_last_marker is None
                    or (t_now - self._t_last_marker).nanoseconds * 1e-9 >= 5.0):
                self._t_last_marker = t_now
                self._publish_coverage_markers()

        # Completion can arrive in ANY state (frontier exhaustion or the
        # exploration_monitor's stall stop). Finalize as soon as we're not in the
        # middle of a capture, so the summary is never missed -- unless coverage
        # completion is enabled, in which case exploration completion only ends
        # the FRONTIER phase and uncovered pockets are visited next.
        if self._completion_pending and not self._done:
            if not (self.state == SCANNING and not self._capture_done):
                if (self.coverage_completion and not self._covering
                        and self.coverage_model == "visibility"
                        and self._map_grid is not None):
                    self._completion_pending = False
                    self._t_complete = None   # run ends after covering, not now
                    self._covering = True
                    self.get_logger().info(
                        "Frontier exploration complete; entering coverage-"
                        "completion phase (visiting uncovered pockets).")
                    self._covering_next()
                else:
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
        elif self.state == COVERING:
            self._tick_covering()
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
        # Distance factor: evaluate a scan candidate every scan_interval_m.
        d = self._displacement_from_last_scan()
        if d < self.scan_interval_m:
            return
        xy = self._current_xy()
        self._mark_scan_pose()   # re-measure the next candidate from here
        # Coverage factor: skip if this spot is already covered by prior scans.
        covered, why = self._candidate_covered(xy)
        if covered:
            self.scans_skipped += 1
            self.get_logger().info(
                f"Candidate at {d:.1f} m moved, but {why}: already covered, "
                f"skipping (skips={self.scans_skipped}).")
            return
        self.get_logger().info(
            f"Candidate at {d:.1f} m moved, {why}: stopping to scan.")
        self._begin_stop()

    def _candidate_covered(self, xy):
        """Decide whether a scan candidate is already covered.

        Returns (covered, reason). With the visibility model the criterion is
        the marginal-gain ratio |B(c,R) \\ C| / |B(c,R)| < tau, computed on the
        live occupancy map; the disk test is the fallback whenever the map (or
        a usable candidate region) is not available yet.
        """
        if xy is None:
            return False, "no pose available"
        if self.coverage_model == "visibility" and self._map_grid is not None:
            gain, cand_area, new_area = new_visible_ratio(
                self._map_grid, self._map_res, self._map_origin, xy,
                self.scan_positions, self.scan_coverage_radius_m,
                num_rays=self.visibility_num_rays,
                occ_thresh=self.occupied_thresh,
                unknown_blocks=self.unknown_blocks_ray)
            if gain is not None:
                covered = (gain < self.min_new_visible_ratio
                           and new_area < self.min_new_visible_area_m2)
                self.skip_events.append({
                    "xy": [round(float(xy[0]), 3), round(float(xy[1]), 3)],
                    "t_s": round(self._elapsed_since_start(), 1),
                    "gain": round(float(gain), 4),
                    "cand_area_m2": round(float(cand_area), 2),
                    "new_area_m2": round(float(new_area), 2),
                    "skipped": bool(covered),
                })
                return covered, (
                    f"new-visible gain {gain:.2f} (tau={self.min_new_visible_ratio:.2f}), "
                    f"new area {new_area:.1f} m^2 "
                    f"(A_min={self.min_new_visible_area_m2:.1f}, "
                    f"|B(c,R)|={cand_area:.1f} m^2)")
            # Degenerate candidate region: fall through to the distance test.
        d_near = self._dist_to_nearest_scan(xy)
        if d_near < self.scan_coverage_radius_m:
            return True, (f"{d_near:.1f} m from a previous scan "
                          f"(< R={self.scan_coverage_radius_m:.1f} m)")
        return False, f"{d_near:.1f} m from nearest scan (>= R)"

    def _dist_to_nearest_scan(self, xy):
        if xy is None or not self.scan_positions:
            return float("inf")
        return min(math.dist(xy, p) for p in self.scan_positions)

    def _compute_scan_polygon(self, xy, stride=4):
        """Visibility polygon at a taken scan pose (downsampled vertices), or
        None when no map has arrived yet / disk model is active."""
        if self.coverage_model != "visibility" or self._map_grid is None:
            return None
        _, endpoints = visible_mask(
            self._map_grid, self._map_res, self._map_origin,
            xy[0], xy[1], self.scan_coverage_radius_m,
            num_rays=self.visibility_num_rays,
            occ_thresh=self.occupied_thresh,
            unknown_blocks=self.unknown_blocks_ray)
        return endpoints[::stride]

    # ----------------------------------------------- coverage markers
    def _scan_color(self, i):
        """Distinct color per scan index (golden-ratio hue spacing)."""
        return colorsys.hsv_to_rgb((i * 0.6180339887) % 1.0, 0.85, 0.95)

    def _refresh_scan_polygons(self):
        """Recompute every scan's visibility polygon on the LATEST map.

        Capture-time polygons are clipped by then-unknown space; as the map
        matures the true panoramic (360 deg) footprint emerges, so the stored
        polygons are refreshed rather than frozen."""
        if self.coverage_model != "visibility" or self._map_grid is None:
            return
        self.scan_polygons = [self._compute_scan_polygon(p)
                              for p in self.scan_positions]

    def _publish_coverage_markers(self):
        self._refresh_scan_polygons()
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        R = self.scan_coverage_radius_m
        for i, (x, y) in enumerate(self.scan_positions):
            r, g, b = self._scan_color(i)
            poly = (self.scan_polygons[i]
                    if i < len(self.scan_polygons) else None)
            if poly is not None and len(poly) >= 3:
                # Occlusion-aware coverage: filled visibility polygon (triangle
                # fan from the scan pose) + outline.
                fan = Marker()
                fan.header.frame_id = self.global_frame
                fan.header.stamp = now
                fan.ns = "scan_coverage"
                fan.id = i
                fan.type = Marker.TRIANGLE_LIST
                fan.action = Marker.ADD
                fan.pose.orientation.w = 1.0
                fan.scale.x = fan.scale.y = fan.scale.z = 1.0
                fan.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.18)
                ctr_pt = Point(x=float(x), y=float(y), z=0.01)
                n = len(poly)
                for k in range(n):
                    p0, p1 = poly[k], poly[(k + 1) % n]
                    fan.points.append(ctr_pt)
                    fan.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=0.01))
                    fan.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=0.01))
                arr.markers.append(fan)

                edge = Marker()
                edge.header.frame_id = self.global_frame
                edge.header.stamp = now
                edge.ns = "scan_coverage_edge"
                edge.id = i
                edge.type = Marker.LINE_STRIP
                edge.action = Marker.ADD
                edge.pose.orientation.w = 1.0
                edge.scale.x = 0.04
                edge.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.8)
                edge.points = [Point(x=float(p[0]), y=float(p[1]), z=0.02)
                               for p in poly] + \
                              [Point(x=float(poly[0][0]), y=float(poly[0][1]), z=0.02)]
                arr.markers.append(edge)
            else:
                disk = Marker()
                disk.header.frame_id = self.global_frame
                disk.header.stamp = now
                disk.ns = "scan_coverage"
                disk.id = i
                disk.type = Marker.CYLINDER
                disk.action = Marker.ADD
                disk.pose.position.x = float(x)
                disk.pose.position.y = float(y)
                disk.pose.position.z = 0.01
                disk.pose.orientation.w = 1.0
                disk.scale.x = disk.scale.y = 2.0 * R
                disk.scale.z = 0.02
                disk.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.18)
                arr.markers.append(disk)

            ctr = Marker()
            ctr.header.frame_id = self.global_frame
            ctr.header.stamp = now
            ctr.ns = "scan_center"
            ctr.id = i
            ctr.type = Marker.SPHERE
            ctr.action = Marker.ADD
            ctr.pose.position.x = float(x)
            ctr.pose.position.y = float(y)
            ctr.pose.position.z = 0.15
            ctr.pose.orientation.w = 1.0
            ctr.scale.x = ctr.scale.y = ctr.scale.z = 0.3
            ctr.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.95)
            arr.markers.append(ctr)

            txt = Marker()
            txt.header.frame_id = self.global_frame
            txt.header.stamp = now
            txt.ns = "scan_label"
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(x)
            txt.pose.position.y = float(y)
            txt.pose.position.z = 0.5
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.4
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            txt.text = f"#{i + 1}"
            arr.markers.append(txt)
        self.coverage_pub.publish(arr)

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
                if self._covering:
                    self.covering_scans += 1
                t = self._elapsed_since_start()
                self.scan_times.append(round(t, 1))
                self._t_capture_settle = self.get_clock().now()
                # Remember + visualize where this scan was taken.
                pos = self._current_xy() or self._last_scan_xy
                if pos is not None:
                    self.scan_positions.append(pos)
                    self.scan_polygons.append(self._compute_scan_polygon(pos))
                    self._publish_coverage_markers()
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

    # ---------------------------------------------- coverage completion
    def _uncovered_pockets(self):
        """Cluster uncovered free cells on the live map.

        Returns a list of (area_m2, (x, y)) sorted largest-first, where (x, y)
        is the pocket cell nearest its centroid (the Nav2 goal: standing inside
        the pocket guarantees line-of-sight to it)."""
        grid = self._map_grid
        res, org = self._map_res, self._map_origin
        free = (grid >= 0) & (grid <= 25)
        cov = union_visible_mask(
            grid, res, org, self.scan_positions, self.scan_coverage_radius_m,
            num_rays=self.visibility_num_rays, occ_thresh=self.occupied_thresh,
            unknown_blocks=self.unknown_blocks_ray)
        uncovered = free & ~cov
        cell_area = res * res
        H, W = uncovered.shape
        seen = np.zeros_like(uncovered, dtype=bool)
        pockets = []
        ys, xs = np.where(uncovered)
        from collections import deque
        for y0, x0 in zip(ys.tolist(), xs.tolist()):
            if seen[y0, x0]:
                continue
            cells = []
            q = deque([(y0, x0)])
            seen[y0, x0] = True
            while q:
                y, x = q.popleft()
                cells.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < H and 0 <= nx < W
                                and uncovered[ny, nx] and not seen[ny, nx]):
                            seen[ny, nx] = True
                            q.append((ny, nx))
            area = len(cells) * cell_area
            if area < self.min_pocket_area_m2:
                continue
            arr = np.asarray(cells, dtype=float)
            cy, cx = arr.mean(axis=0)
            k = int(np.argmin((arr[:, 0] - cy) ** 2 + (arr[:, 1] - cx) ** 2))
            gy, gx = cells[k]
            wx = org[0] + (gx + 0.5) * res
            wy = org[1] + (gy + 0.5) * res
            if any(math.dist((wx, wy), f) < 0.6 for f in self._failed_targets):
                continue   # Nav2 already failed to reach this pocket
            pockets.append((area, (wx, wy)))
        pockets.sort(key=lambda p: -p[0])
        return pockets

    def _covering_next(self):
        """Pick the next uncovered pocket and drive there, or finish the run."""
        if self.covering_scans >= self.covering_max_scans:
            self.get_logger().warn(
                f"Coverage completion: scan budget ({self.covering_max_scans}) "
                "exhausted; finishing.")
            self._finalize("coverage completion (scan budget)")
            return
        pockets = self._uncovered_pockets()
        if not pockets:
            self.get_logger().info(
                "Coverage completion: no uncovered pocket >= "
                f"{self.min_pocket_area_m2:.1f} m^2 left -- full LOS coverage.")
            self._finalize("coverage complete")
            return
        area, (wx, wy) = pockets[0]
        self._cover_target = (wx, wy)
        self._cover_done = False
        self._cover_failed = False
        self._cover_goal_handle = None
        self.get_logger().info(
            f"Coverage completion: {len(pockets)} uncovered pocket(s) left; "
            f"driving into the largest ({area:.1f} m^2) at "
            f"({wx:.2f}, {wy:.2f}).")
        self._send_cover_goal((wx, wy))
        self._set_state(COVERING)

    def _send_cover_goal(self, xy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.w = 1.0
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose server not available.")
            self._cover_failed = True
            return
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_cover_goal_response)

    def _on_cover_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Coverage goal failed to send: {exc}")
            self._cover_failed = True
            return
        if not gh.accepted:
            self.get_logger().warn("Coverage goal rejected by Nav2.")
            self._cover_failed = True
            return
        self._cover_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_cover_result)

    def _on_cover_result(self, fut):
        try:
            status = fut.result().status
        except Exception:  # noqa: BLE001
            status = GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._cover_done = True
        else:
            self._cover_failed = True

    def _tick_covering(self):
        if self._cover_done:
            # Arrived inside the pocket; Nav2 has the robot stopped. Reuse the
            # STOPPING settle/download-gate/scan path, but the explorer is gone
            # so mark its stop request as already handled.
            self._stop_sent = True
            self._stop_settled = False
            self._waiting_download_logged = False
            self._t_gate_cleared = None
            self._set_state(STOPPING)
            return
        if self._cover_failed or self._elapsed_in_state() >= self.covering_nav_timeout_s:
            if not self._cover_failed and self._cover_goal_handle is not None:
                self._cover_goal_handle.cancel_goal_async()
            self.get_logger().warn(
                f"Coverage goal {self._cover_target} unreachable "
                "(failed or timed out); skipping this pocket.")
            if self._cover_target is not None:
                self._failed_targets.append(self._cover_target)
            self._covering_next()

    def _begin_resume(self):
        if self._covering:
            # Coverage-completion phase: after each pocket scan, go straight to
            # the next uncovered pocket (no frontier explorer to resume).
            self._covering_next()
            return
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
