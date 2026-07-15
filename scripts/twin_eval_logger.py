#!/usr/bin/env python3
"""Digital-twin quantitative-eval logger (paper experiment, this Isaac PC).

Records during a live twin session (robot + isaacsim_ammr_twin.py running):
  twin_log.csv     - continuous stream: /ammr/state (robot, map frame) and
                     /ammr/twin_pose (twin, Isaac frame)
  checkpoints.csv  - type a label (e.g. "A") + Enter while the robot is parked
                     on a marked floor point -> 2 s of samples are averaged
                     into one row (robot map pose + twin Isaac pose)
  goals.csv        - every /ammr/goal_pose is logged; arrival is auto-detected
                     (robot moved, then speed < 0.02 m/s for 3 s) and the
                     stop-pose error vs the goal is recorded

Run with the ammr DDS env (same as ammr_net_check.sh):
    scripts/twin_eval_logger.sh [--outdir DIR]
Afterwards: fill ground_truth.yaml and run scripts/twin_eval_report.py.
"""
import argparse
import csv
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

CHECKPOINT_WINDOW = 2.0   # s of samples averaged per checkpoint
ARRIVE_SPEED = 0.02       # m/s (and rad/s) considered "stopped"
ARRIVE_HOLD = 3.0         # s the robot must stay stopped
ARRIVE_MIN_MOVE = 0.3     # m the robot must move before a stop counts


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EvalLogger(Node):
    def __init__(self, outdir: Path):
        super().__init__("twin_eval_logger")
        self.outdir = outdir
        self.log_f = open(outdir / "twin_log.csv", "w", newline="")
        self.log = csv.writer(self.log_f)
        self.log.writerow(["t", "src", "x", "y", "yaw"])
        self.cp_f = open(outdir / "checkpoints.csv", "w", newline="")
        self.cp = csv.writer(self.cp_f)
        self.cp.writerow(["label", "t",
                          "robot_x", "robot_y", "robot_yaw",
                          "twin_x", "twin_y", "twin_yaw",
                          "n_robot", "n_twin", "spread_robot_m", "spread_twin_m"])
        self.goal_f = open(outdir / "goals.csv", "w", newline="")
        self.goal = csv.writer(self.goal_f)
        self.goal.writerow(["goal_id", "event", "t", "x", "y", "yaw",
                            "err_to_goal_m", "travel_s"])

        self.last_state = None          # (t, x, y, yaw)
        self.last_twin = None
        self.collect_until = 0.0        # checkpoint sampling window
        self.samples = {"state": [], "twin": []}
        self.pending_label = None

        self.goal_id = 0
        self.active_goal = None         # (id, t, x, y, start_pose)
        self.moved = False
        self.still_since = None

        self.create_subscription(Odometry, "/ammr/state", self._state_cb, 10)
        self.create_subscription(PoseStamped, "/ammr/twin_pose", self._twin_cb, 10)
        self.create_subscription(PoseStamped, "/ammr/goal_pose", self._goal_cb, 10)
        self.get_logger().info(f"logging to {outdir}")
        self.get_logger().info("체크포인트: 로봇을 기준점 위에 세우고  라벨 + Enter  (예: A)")

    # -- streams ------------------------------------------------------------
    def _state_cb(self, m):
        t = time.time()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.last_state = (t, p.x, p.y, yaw_of(q))
        self.log.writerow([f"{t:.3f}", "state", f"{p.x:.4f}", f"{p.y:.4f}",
                           f"{yaw_of(q):.4f}"])
        if t < self.collect_until:
            self.samples["state"].append((p.x, p.y, yaw_of(q)))
        self._track_arrival(t, m.twist.twist)

    def _twin_cb(self, m):
        t = time.time()
        p, q = m.pose.position, m.pose.orientation
        self.last_twin = (t, p.x, p.y, yaw_of(q))
        self.log.writerow([f"{t:.3f}", "twin", f"{p.x:.4f}", f"{p.y:.4f}",
                           f"{yaw_of(q):.4f}"])
        if t < self.collect_until:
            self.samples["twin"].append((p.x, p.y, yaw_of(q)))

    # -- checkpoints ---------------------------------------------------------
    def start_checkpoint(self, label):
        if self.last_state is None:
            print("!! /ammr/state 미수신 — 로봇/브리지 확인")
            return
        self.pending_label = label
        self.samples = {"state": [], "twin": []}
        self.collect_until = time.time() + CHECKPOINT_WINDOW
        print(f"[{label}] {CHECKPOINT_WINDOW:.0f}초 샘플링... 로봇을 움직이지 마세요")
        threading.Timer(CHECKPOINT_WINDOW + 0.3, self._finish_checkpoint).start()

    @staticmethod
    def _avg(rows):
        n = len(rows)
        if n == 0:
            return None
        xs, ys = [r[0] for r in rows], [r[1] for r in rows]
        cx, cy = sum(xs) / n, sum(ys) / n
        yaw = math.atan2(sum(math.sin(r[2]) for r in rows),
                         sum(math.cos(r[2]) for r in rows))
        spread = max(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
        return cx, cy, yaw, n, spread

    def _finish_checkpoint(self):
        label, self.pending_label = self.pending_label, None
        r = self._avg(self.samples["state"])
        w = self._avg(self.samples["twin"])
        if r is None:
            print(f"[{label}] 실패: 샘플 없음")
            return
        if w is None:
            w = (float("nan"),) * 3 + (0, float("nan"))
            print(f"[{label}] 경고: /ammr/twin_pose 없음 (트윈 미실행?) — robot pose만 기록")
        self.cp.writerow([label, f"{time.time():.3f}",
                          f"{r[0]:.4f}", f"{r[1]:.4f}", f"{r[2]:.4f}",
                          f"{w[0]:.4f}", f"{w[1]:.4f}", f"{w[2]:.4f}",
                          r[3], w[3], f"{r[4]:.4f}", f"{w[4]:.4f}"])
        self.cp_f.flush()
        print(f"[{label}] 기록: robot({r[0]:.3f}, {r[1]:.3f})  twin({w[0]:.3f}, {w[1]:.3f})"
              f"  흔들림 {r[4]*1000:.0f} mm")

    # -- goals ---------------------------------------------------------------
    def _goal_cb(self, m):
        t = time.time()
        self.goal_id += 1
        x, y, yaw = m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)
        start = self.last_state
        self.active_goal = (self.goal_id, t, x, y, start)
        self.moved = False
        self.still_since = None
        self.goal.writerow([self.goal_id, "goal", f"{t:.3f}",
                            f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}", "", ""])
        self.goal_f.flush()
        print(f"[goal {self.goal_id}] ({x:.2f}, {y:.2f}) — 도착 감지 대기")

    def _track_arrival(self, t, twist):
        if self.active_goal is None or self.last_state is None:
            return
        gid, t0, gx, gy, start = self.active_goal
        _, x, y, yaw = self.last_state
        if not self.moved:
            if start and math.hypot(x - start[1], y - start[2]) > ARRIVE_MIN_MOVE:
                self.moved = True
            return
        speed = math.hypot(twist.linear.x, twist.linear.y)
        if speed < ARRIVE_SPEED and abs(twist.angular.z) < ARRIVE_SPEED:
            if self.still_since is None:
                self.still_since = t
            elif t - self.still_since > ARRIVE_HOLD:
                err = math.hypot(x - gx, y - gy)
                self.goal.writerow([gid, "arrival", f"{t:.3f}",
                                    f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}",
                                    f"{err:.4f}", f"{t - t0:.1f}"])
                self.goal_f.flush()
                print(f"[goal {gid}] 도착: 오차 {err*100:.1f} cm, {t - t0:.0f} s")
                self.active_goal = None
        else:
            self.still_since = None

    def close(self):
        for f in (self.log_f, self.cp_f, self.goal_f):
            f.flush()
            f.close()


def stdin_loop(node):
    for line in sys.stdin:
        label = line.strip()
        if not label:
            continue
        if label.lower() in ("q", "quit", "exit"):
            rclpy.shutdown()
            return
        node.start_checkpoint(label)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default=str(Path.home() / "ammr_twin" / "eval"))
    args = ap.parse_args()
    session = Path(args.outdir) / time.strftime("session_%Y%m%d_%H%M%S")
    session.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = EvalLogger(session)
    threading.Thread(target=stdin_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        print(f"\n세션 저장: {session}")
        print("다음: ground_truth.yaml 작성 후 "
              f"python3 scripts/twin_eval_report.py {session}")


if __name__ == "__main__":
    main()
