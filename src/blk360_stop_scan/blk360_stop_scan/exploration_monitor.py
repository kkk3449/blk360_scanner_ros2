#!/usr/bin/env python3
"""Exploration progress monitor + stall-based early stop.

frontier_exploration_ros2 has no notion of "total area", so it can't report a
real progress percentage -- it only knows "are there frontiers left?". This node
adds two things that work WITHOUT any ground-truth map:

  1. Remaining-frontier readout. Each /map update it counts the frontier cells
     (mapped-free cells touching unknown space) and groups them into clusters,
     publishing both on /exploration_remaining (std_msgs/Int32MultiArray
     [clusters, cells]). As exploration proceeds these trend toward zero.

  2. Stall-based auto-stop. The honest "are we done?" signal in an unknown map
     is: does more driving still reveal new area? If the known map area stops
     growing (gains < `min_progress_cells` for `stall_timeout_s`), exploration
     has effectively converged -- no matter how many (unreachable/tiny) frontiers
     remain. We then STOP the explorer (control_exploration ACTION_STOP, quit)
     and fire the internal completion trigger so the stop-scan sequencer prints
     its summary, exactly like a natural frontier-exhaustion completion.

The stall threshold is NOT a fixed frontier count the user must guess -- it is
purely "the map stopped changing", i.e. it reflects the current state itself.
"""
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Empty, Int32MultiArray, String

from frontier_exploration_ros2.srv import ControlExploration


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__("exploration_monitor")

        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.remaining_topic = self.declare_parameter(
            "remaining_topic", "/exploration_remaining").value
        # Internal Empty trigger consumed by the stop-scan sequencer (the public
        # /exploration_complete Bool flag is published by the sequencer).
        self.completion_topic = self.declare_parameter(
            "completion_topic", "/exploration_complete_internal").value
        self.control_service_name = self.declare_parameter(
            "control_service_name", "/control_exploration").value
        # Occupancy value (0-100) at/below which a known cell counts as free.
        self.free_thresh = self.declare_parameter("free_thresh", 25).value
        # Min frontier-cluster size (cells) to count as a cluster in the readout.
        self.min_cluster_cells = self.declare_parameter("min_cluster_cells", 10).value
        # --- stall-based stop ---
        self.auto_stop = self.declare_parameter("auto_stop", True).value
        # Known-area gain (cells) below which a window counts as "no progress".
        # Big enough to ignore Cartographer's edge-refinement creep (tens of
        # cells) while real exploration (thousands of cells) still counts.
        self.min_progress_cells = self.declare_parameter("min_progress_cells", 400).value
        # No-progress duration that declares exploration converged.
        self.stall_timeout_s = self.declare_parameter("stall_timeout_s", 300.0).value
        # Ignore stalls during initial bring-up (map not flowing yet).
        self.startup_grace_s = self.declare_parameter("startup_grace_s", 20.0).value
        self.log_period_s = self.declare_parameter("log_period_s", 5.0).value
        # Sequencer FSM state topic: the stall clock only counts time while the
        # robot is actually EXPLORING, so a long scan/download pause is not
        # mistaken for convergence.
        self.sequencer_state_topic = self.declare_parameter(
            "sequencer_state_topic", "/blk360_stop_scan/state").value

        # Volatile sub is compatible with both volatile and transient_local
        # publishers, so we receive /map regardless of how Cartographer is
        # configured (it republishes periodically, so volatile loses nothing).
        map_qos = QoSProfile(
            depth=5, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        completion_qos = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.remaining_pub = self.create_publisher(Int32MultiArray, self.remaining_topic, 10)
        self.completion_pub = self.create_publisher(Empty, self.completion_topic, completion_qos)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(String, self.sequencer_state_topic, self._on_state, 10)
        self.control_cli = self.create_client(ControlExploration, self.control_service_name)

        self._baseline_known = 0
        self._t_start = self.get_clock().now()
        self._t_last_progress = self.get_clock().now()
        self._t_last_log = self.get_clock().now()
        self._stopped = False
        # Default active so the monitor still works if the sequencer isn't up.
        self._explorer_active = True

    def _on_state(self, msg: String):
        self._explorer_active = (msg.data.strip() == "EXPLORING")

        self.get_logger().info(
            f"Exploration monitor up (auto_stop={self.auto_stop}). DONE when the known "
            f"map grows <{self.min_progress_cells} cells for >{self.stall_timeout_s:.0f}s "
            f"while exploring (after {self.startup_grace_s:.0f}s). Watching /map...")

    # ------------------------------------------------------------------ map
    def _on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        unknown = grid < 0
        known = ~unknown
        free = known & (grid <= self.free_thresh)
        known_cells = int(known.sum())

        clusters, cells = self._frontier_stats(free, unknown)

        out = Int32MultiArray()
        out.data = [clusters, cells]
        self.remaining_pub.publish(out)

        now = self.get_clock().now()
        # Completion = the KNOWN MAP stops growing. min_progress_cells must be
        # large enough that Cartographer's slow edge-refinement creep (tens of
        # cells/cycle) counts as no-progress, while real exploration (thousands
        # of cells/cycle) resets the clock. (The /map-derived frontier count is
        # unreliable in a sealed room -- free is bounded by walls, not unknown --
        # so it is published as a readout only, NOT used to decide completion.)
        if known_cells > self._baseline_known + self.min_progress_cells:
            self._baseline_known = known_cells
            self._t_last_progress = now
        # Don't accumulate stall time during a scan/download pause (map frozen).
        if not self._explorer_active:
            self._t_last_progress = now

        stalled_for = (now - self._t_last_progress).nanoseconds * 1e-9
        if (now - self._t_last_log).nanoseconds * 1e-9 >= self.log_period_s:
            self._t_last_log = now
            self.get_logger().info(
                f"frontier(readout) clusters={clusters} cells={cells} | "
                f"known={known_cells} | no-growth {stalled_for:.0f}/{self.stall_timeout_s:.0f}s")

        if not self.auto_stop or self._stopped:
            return
        running_for = (now - self._t_start).nanoseconds * 1e-9
        if (self._explorer_active and running_for >= self.startup_grace_s
                and stalled_for >= self.stall_timeout_s):
            self._declare_converged(clusters, cells, known_cells, stalled_for,
                                    reason="known map stalled")

    def _frontier_stats(self, free, unknown):
        """Return (cluster_count, frontier_cell_count). A frontier cell is a free
        cell 4-adjacent to unknown space; clusters are 8-connected groups of them
        with >= min_cluster_cells members."""
        nbr_unknown = np.zeros_like(unknown)
        nbr_unknown[1:, :] |= unknown[:-1, :]
        nbr_unknown[:-1, :] |= unknown[1:, :]
        nbr_unknown[:, 1:] |= unknown[:, :-1]
        nbr_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & nbr_unknown
        cells = int(frontier.sum())
        if cells == 0:
            return 0, 0
        clusters = self._count_clusters(frontier)
        return clusters, cells

    def _count_clusters(self, mask):
        H, W = mask.shape
        seen = np.zeros_like(mask, dtype=bool)
        ys, xs = np.where(mask)
        count = 0
        for y0, x0 in zip(ys.tolist(), xs.tolist()):
            if seen[y0, x0]:
                continue
            size = 0
            q = deque([(y0, x0)])
            seen[y0, x0] = True
            while q:
                y, x = q.popleft()
                size += 1
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < H and 0 <= nx < W and mask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            q.append((ny, nx))
            if size >= self.min_cluster_cells:
                count += 1
        return count

    # --------------------------------------------------------------- stop
    def _declare_converged(self, clusters, cells, known_cells, elapsed_s,
                           reason="known map stalled"):
        self._stopped = True
        self.get_logger().warn(
            f"Exploration converged ({reason}) after {elapsed_s:.0f}s "
            f"({clusters} frontier clusters / {cells} cells still flagged). "
            "Stopping explorer and signalling complete.")
        # 1) halt the explorer for good
        if self.control_cli.service_is_ready():
            req = ControlExploration.Request()
            req.action = ControlExploration.Request.ACTION_STOP
            req.delay_seconds = 0.0
            req.quit_after_stop = True
            self.control_cli.call_async(req)
        else:
            self.get_logger().warn(
                f"control service '{self.control_service_name}' not ready; "
                "still publishing completion event.")
        # 2) trigger the stop-scan sequencer's summary, like a natural completion
        self.completion_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMonitor()
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
