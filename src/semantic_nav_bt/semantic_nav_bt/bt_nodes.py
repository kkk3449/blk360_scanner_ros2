"""py_trees behaviors for the TOSM semantic mission BT.

Tree architecture (hybrid deliberative/reactive, after the BT_ros1
reference tutorial, ported to ROS2/Nav2 and made semantic):

  Root: Fallback
   ├─ ReactiveSequence                    (reactive layer, checked every tick)
   │   ├─ BatteryOK        (condition; FAILURE => whole branch fails => dock)
   │   ├─ NoDockInterrupt  (condition; a {"cmd":"dock"} preempts the mission)
   │   └─ MissionExecutor  (deliberative layer)
   │       ├─ WaitForCommand      (pull next command; new command preempts)
   │       ├─ ResolveSemanticGoal (mediator: KG -> goal list on blackboard)
   │       └─ ExecuteGoals        (per goal: NavigateToPose [-> ArmPick])
   └─ GoDock                (dock subtree: resolve dock goal -> navigate)

Blackboard keys: command, goals, goal_idx, status_text.
"""
import json
import math
import queue

import py_trees
from py_trees.common import Status


def _quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Blackboard:
    """Plain shared state (py_trees blackboard client API varies across
    versions; a simple object shared by construction is version-proof)."""

    def __init__(self):
        self.command = None          # currently executing command (dict)
        self.pending = queue.Queue()  # commands waiting
        self.goals = []
        self.goal_idx = 0
        self.dock_requested = False
        self.docked = False
        self.battery_level = 1.0
        self.status = "idle"
        self.history = []

    def put_command(self, cmd):
        if cmd.get("cmd") == "dock":
            self.dock_requested = True
        else:
            self.pending.put(cmd)


class BatteryOK(py_trees.behaviour.Behaviour):
    def __init__(self, bb, threshold=0.2):
        super().__init__("BatteryOK")
        self.bb, self.threshold = bb, threshold

    def update(self):
        if self.bb.battery_level < self.threshold:
            if self.bb.docked:
                self.bb.status = (f"charging at dock "
                                  f"({self.bb.battery_level:.2f})")
                return Status.RUNNING
            self.bb.status = f"battery {self.bb.battery_level:.2f} < " \
                             f"{self.threshold} -> dock"
            return Status.FAILURE
        return Status.SUCCESS


class NoDockInterrupt(py_trees.behaviour.Behaviour):
    def __init__(self, bb):
        super().__init__("NoDockInterrupt")
        self.bb = bb

    def update(self):
        return Status.FAILURE if self.bb.dock_requested else Status.SUCCESS


class WaitForCommand(py_trees.behaviour.Behaviour):
    def __init__(self, bb):
        super().__init__("WaitForCommand")
        self.bb = bb

    def update(self):
        if self.bb.command is not None:
            return Status.SUCCESS
        try:
            self.bb.command = self.bb.pending.get_nowait()
            self.bb.docked = False
            self.bb.goals, self.bb.goal_idx = [], 0
            self.bb.status = f"command: {self.bb.command}"
            return Status.SUCCESS
        except queue.Empty:
            self.bb.status = "idle (waiting for /semantic_command)"
            return Status.RUNNING


class ResolveSemanticGoal(py_trees.behaviour.Behaviour):
    def __init__(self, bb, mediator):
        super().__init__("ResolveSemanticGoal")
        self.bb, self.mediator = bb, mediator

    def update(self):
        if self.bb.goals:
            return Status.SUCCESS
        r = self.mediator.resolve(self.bb.command)
        if "error" in r:
            self.bb.status = f"mediator REFUSED: {r['error']}"
            self.bb.history.append({"cmd": self.bb.command,
                                    "result": r["error"]})
            self.bb.command = None
            return Status.FAILURE
        self.bb.goals = r["goals"]
        self.bb.goal_idx = 0
        self.bb.status = f"resolved {len(self.bb.goals)} goal(s): " \
                         f"{[g['label'] for g in self.bb.goals]}"
        return Status.SUCCESS


class NavigateToGoal(py_trees.behaviour.Behaviour):
    """Nav2 NavigateToPose action client behavior (or a dry-run stub)."""

    def __init__(self, bb, node=None, dry_run=False, name="NavigateToGoal"):
        super().__init__(name)
        self.bb, self.node, self.dry_run = bb, node, dry_run
        self._handle = None
        self._result = None
        self._ticks = 0

    def _goal(self):
        return self.bb.goals[self.bb.goal_idx]

    def initialise(self):
        self._result, self._ticks = None, 0
        if self.dry_run or self.node is None:
            return
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        g = self._goal()
        if not hasattr(self.node, "_nav_client"):
            self.node._nav_client = ActionClient(self.node, NavigateToPose,
                                                 "navigate_to_pose")
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.pose.position.x = float(g["x"])
        msg.pose.pose.position.y = float(g["y"])
        qx, qy, qz, qw = _quat_from_yaw(g.get("yaw", 0.0))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.node._nav_client.wait_for_server(timeout_sec=5.0)
        fut = self.node._nav_client.send_goal_async(msg)
        fut.add_done_callback(self._on_accept)

    def _on_accept(self, fut):
        self._handle = fut.result()
        if self._handle and self._handle.accepted:
            self._handle.get_result_async().add_done_callback(self._on_result)
        else:
            self._result = "rejected"

    def _on_result(self, fut):
        code = fut.result().status          # 4 = SUCCEEDED
        self._result = "succeeded" if code == 4 else f"ended({code})"

    def update(self):
        g = self._goal()
        if self.dry_run or self.node is None:
            self._ticks += 1
            self.bb.status = f"[dry] navigating to {g['label']} " \
                             f"({g['x']:.2f},{g['y']:.2f}) t={self._ticks}"
            return Status.SUCCESS if self._ticks >= 3 else Status.RUNNING
        if self._result is None:
            self.bb.status = f"navigating to {g['label']} " \
                             f"({g['x']:.2f},{g['y']:.2f})"
            return Status.RUNNING
        ok = self._result == "succeeded"
        self.bb.status = f"nav {self._result}: {g['label']}"
        if not ok:
            # log + skip the failed goal so the mission loop moves on
            self.bb.history.append({"cmd": self.bb.command, "goal": g,
                                    "result": f"nav_{self._result}"})
            self.bb.goal_idx += 1
            if self.bb.goal_idx >= len(self.bb.goals):
                self.bb.command, self.bb.goals = None, []
                self.bb.goal_idx = 0
        return Status.SUCCESS if ok else Status.FAILURE

    def terminate(self, new_status):
        # preemption: cancel the active Nav2 goal
        if self._handle is not None and self._result is None \
                and new_status == Status.INVALID:
            self._handle.cancel_goal_async()


class ArmPick(py_trees.behaviour.Behaviour):
    """Manipulator pickup at a fixed target pose. Simulation stub: holds
    RUNNING for a few ticks, then reports success — the seam where the real
    AMMR arm action (MoveIt / vendor driver) plugs in for the thesis."""

    def __init__(self, bb, hold_ticks=5):
        super().__init__("ArmPick")
        self.bb, self.hold = bb, hold_ticks
        self._ticks = 0

    def initialise(self):
        self._ticks = 0

    def update(self):
        g = self.bb.goals[self.bb.goal_idx]
        if g.get("kind") != "pick":
            return Status.SUCCESS
        t = g["pick_target"]
        if t.get("isMovable") is False:
            self.bb.status = f"pick REFUSED: {t['name']} isMovable=false"
            self.bb.history.append({"cmd": self.bb.command, "goal": g,
                                    "result": "pick_refused_not_movable"})
            self.bb.goal_idx += 1
            if self.bb.goal_idx >= len(self.bb.goals):
                self.bb.command, self.bb.goals = None, []
                self.bb.goal_idx = 0
            return Status.FAILURE
        self._ticks += 1
        p = t["pose"]
        self.bb.status = (f"[arm stub] picking {t['name']} at "
                          f"({p['x']:.2f},{p['y']:.2f},{p.get('z', 0):.2f}) "
                          f"{self._ticks}/{self.hold}")
        return Status.SUCCESS if self._ticks >= self.hold else Status.RUNNING


class AdvanceGoal(py_trees.behaviour.Behaviour):
    def __init__(self, bb):
        super().__init__("AdvanceGoal")
        self.bb = bb

    def update(self):
        g = self.bb.goals[self.bb.goal_idx]
        self.bb.history.append({"cmd": self.bb.command, "goal": g["label"],
                                "result": "reached"})
        self.bb.goal_idx += 1
        if self.bb.goal_idx >= len(self.bb.goals):
            self.bb.status = f"mission complete: {self.bb.command}"
            self.bb.command, self.bb.goals, self.bb.goal_idx = None, [], 0
        # SUCCESS either way: the mission sequence completes per goal and the
        # next tick re-enters it for the remaining goals (or a new command)
        return Status.SUCCESS


class ResolveDock(py_trees.behaviour.Behaviour):
    def __init__(self, bb, mediator):
        super().__init__("ResolveDock")
        self.bb, self.mediator = bb, mediator

    def update(self):
        r = self.mediator.resolve({"cmd": "dock"})
        self.bb.goals = r["goals"]
        self.bb.goal_idx = 0
        self.bb.status = f"docking -> {r['goals'][0]['label']}"
        return Status.SUCCESS


class DockDone(py_trees.behaviour.Behaviour):
    def __init__(self, bb):
        super().__init__("DockDone")
        self.bb = bb

    def update(self):
        self.bb.dock_requested = False
        self.bb.docked = True
        self.bb.command, self.bb.goals, self.bb.goal_idx = None, [], 0
        self.bb.history.append({"cmd": {"cmd": "dock"}, "result": "docked"})
        self.bb.status = "docked"
        return Status.SUCCESS


def build_tree(bb, mediator, node=None, dry_run=False):
    execute = py_trees.composites.Sequence("ExecuteGoal", memory=True,
                                           children=[
        NavigateToGoal(bb, node, dry_run),
        ArmPick(bb),
        AdvanceGoal(bb),
    ])
    mission = py_trees.composites.Sequence("MissionExecutor", memory=True,
                                           children=[
        WaitForCommand(bb),
        ResolveSemanticGoal(bb, mediator),
        execute,
    ])
    # failures inside the mission (mediator refusal, nav abort, pick refusal)
    # are logged + skipped by the leaves themselves; this wrapper keeps them
    # from bubbling up and spuriously triggering the dock branch — docking is
    # reserved for the battery / interrupt conditions
    mission_safe = py_trees.decorators.FailureIsSuccess(
        name="MissionSafe", child=mission)
    reactive = py_trees.composites.Sequence("ReactiveLayer", memory=False,
                                            children=[
        BatteryOK(bb),
        NoDockInterrupt(bb),
        mission_safe,
    ])
    dock = py_trees.composites.Sequence("GoDock", memory=True, children=[
        ResolveDock(bb, mediator),
        NavigateToGoal(bb, node, dry_run, name="NavigateToDock"),
        DockDone(bb),
    ])
    root = py_trees.composites.Selector("Root", memory=False,
                                        children=[reactive, dock])
    return py_trees.trees.BehaviourTree(root)
