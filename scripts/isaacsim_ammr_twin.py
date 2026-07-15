#!/usr/bin/env python3
"""AMMR digital twin in Isaac Sim — LIVE stage (mirrors the real robot).

Direction 1 (real -> Isaac): subscribes the real AMMR's pose and moves the twin
robot prim kinematically (no physics fight). Direction 2 (Isaac -> real): the
user drags the red Target cone in the viewport; when it settles, its (x, y, yaw)
is published as a nav goal for the real robot's Nav2.

Contract with the AMMR digital_twin_bridge (see ~/Downloads/DataSend/README.md;
robot side runs `ros2 launch digital_twin_bridge digital_twin.launch.py`):
  in : /ammr/state     nav_msgs/Odometry            map-frame pose+twist, 30 Hz
       /ammr/pose      geometry_msgs/PoseStamped    (convenience fallback)
       /amcl_pose      geometry_msgs/PoseWithCovarianceStamped (fallback)
  out: /ammr/goal_pose geometry_msgs/PoseStamped    -> dt_goal_relay -> Nav2
DDS: CycloneDDS, ROS_DOMAIN_ID=56, CYCLONEDDS_URI=~/cyclonedds_isaac.xml —
all set by scripts/run_isaac_ammr.sh twin. Robot wifi 192.168.31.56, this PC
192.168.31.135.

Frames: Isaac world = vis_n2 e57 frame (floor z=0). If the robot's SLAM map is
a different frame, pass --map-offset X Y YAW_DEG = pose of the ROBOT map origin
expressed in the Isaac world (twin pose = offset ∘ robot pose; goals get the
inverse). Clock: real time — do NOT run other nodes with use_sim_time.

Run:
  ~/isaacsim/python.sh scripts/isaacsim_ammr_twin.py \
      --world ~/ammr_twin/vis_n2_world.usda --overlay ~/ammr_twin/vis_n2_tosm.usda \
      [--map-offset 0 0 0]

No real robot yet? Simulate one:
  python3 scripts/ammr_pose_mock.py     # drives a fake pose around + accepts goals
"""
import argparse
import math

from isaacsim import SimulationApp

_ap = argparse.ArgumentParser()
_ap.add_argument("--world", default=None)
_ap.add_argument("--overlay", default=None)
_ap.add_argument("--urdf", default="src/ammr_description/urdf/ammr.urdf")
_ap.add_argument("--map-offset", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                 metavar=("X", "Y", "YAW_DEG"),
                 help="robot map origin in the Isaac world frame")
_ap.add_argument("--anchor", nargs=3, type=float, default=None,
                 metavar=("X", "Y", "YAW_RAD"),
                 help="auto-calibrate: the robot is physically standing at this "
                      "Isaac-world pose right now (e.g. the scanned robot spot "
                      "motor_035: 0.7365 0.7815 0.809); the first received pose "
                      "solves --map-offset and prints it")
_ap.add_argument("--state-topic", default="/ammr/state")
_ap.add_argument("--pose-topic", default="/ammr/pose")
_ap.add_argument("--goal-topic", default="/ammr/goal_pose")
_ap.add_argument("--headless", action="store_true")
ARGS = _ap.parse_args()

sim_app = SimulationApp({"headless": ARGS.headless})

import os                                                     # noqa: E402
import omni.kit.commands                                      # noqa: E402
import omni.usd                                               # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdPhysics                  # noqa: E402
from isaacsim.core.api import World                           # noqa: E402
from isaacsim.core.utils.extensions import enable_extension   # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.ros2.bridge")
sim_app.update()

import rclpy                                                  # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from geometry_msgs.msg import (PoseStamped,                   # noqa: E402
                               PoseWithCovarianceStamped)
from nav_msgs.msg import Odometry                             # noqa: E402

OFF_X, OFF_Y = ARGS.map_offset[0], ARGS.map_offset[1]
OFF_YAW = math.radians(ARGS.map_offset[2])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def calibrate_offset(anchor, robot_pose):
    """Solve OFF so map_to_isaac(robot_pose) == anchor."""
    global OFF_X, OFF_Y, OFF_YAW
    ax, ay, ayaw = anchor
    rx, ry, ryaw = robot_pose
    OFF_YAW = wrap(ayaw - ryaw)
    c, s = math.cos(OFF_YAW), math.sin(OFF_YAW)
    OFF_X = ax - (c * rx - s * ry)
    OFF_Y = ay - (s * rx + c * ry)
    print(f"[twin] CALIBRATED map-offset: {OFF_X:.4f} {OFF_Y:.4f} "
          f"{math.degrees(OFF_YAW):.3f}  (reuse via --map-offset)")


def map_to_isaac(x, y, yaw):
    c, s = math.cos(OFF_YAW), math.sin(OFF_YAW)
    return (OFF_X + c * x - s * y, OFF_Y + s * x + c * y, yaw + OFF_YAW)


def isaac_to_map(x, y, yaw):
    c, s = math.cos(-OFF_YAW), math.sin(-OFF_YAW)
    dx, dy = x - OFF_X, y - OFF_Y
    return (c * dx - s * dy, s * dx + c * dy, yaw - OFF_YAW)


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


# ---------------- scene ----------------
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
if ARGS.world:
    add_reference_to_stage(usd_path=os.path.abspath(os.path.expanduser(ARGS.world)),
                           prim_path="/World/Env")
else:
    world.scene.add_default_ground_plane()
if ARGS.overlay:
    add_reference_to_stage(usd_path=os.path.abspath(os.path.expanduser(ARGS.overlay)),
                           prim_path="/World/Semantic")

# twin robot: URDF imported once, then physics stripped -> pure kinematic visual
status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = True
cfg.fix_base = True                      # no dynamics; we place it by hand
cfg.make_default_prim = False
status, ROBOT = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=os.path.abspath(os.path.expanduser(ARGS.urdf)),
    import_config=cfg, get_articulation_root=True)
# get_articulation_root hands back the physics root (a joint prim like
# /ammr/root_joint); xform ops on a joint never move the visual robot.
# The twin must drive the import root Xform, and the physics-disable
# traversal below must cover the WHOLE robot subtree.
ROBOT = "/" + ROBOT.strip("/").split("/")[0]
robot_prim = stage.GetPrimAtPath(ROBOT)
# disable articulation/rigid bodies so set_world_pose is authoritative
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if not p.startswith(ROBOT):
        continue
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        # RemoveAPI so PhysX never parses an articulation at all; a merely
        # "disabled" one still claimed the root xform every step (fabric
        # override), freezing the twin at its spawn pose in the viewport.
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        attr = prim.GetAttribute("physxArticulation:articulationEnabled")
        if not attr:
            attr = prim.CreateAttribute(
                "physxArticulation:articulationEnabled",
                Sdf.ValueTypeNames.Bool)
        attr.Set(False)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)

robot_xf = UsdGeom.Xformable(robot_prim)
robot_ops = {op.GetOpName(): op for op in robot_xf.GetOrderedXformOps()}
print(f"[twin] robot prim: {ROBOT}  xformOps: {list(robot_ops)}")
if "xformOp:translate" not in robot_ops:
    robot_ops["xformOp:translate"] = robot_xf.AddTranslateOp()
if "xformOp:orient" not in robot_ops:
    robot_ops["xformOp:orient"] = robot_xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)


def set_twin_pose(x, y, yaw, z=0.0):
    robot_ops["xformOp:translate"].Set(Gf.Vec3d(x, y, z))
    robot_ops["xformOp:orient"].Set(
        Gf.Quatd(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))


# target marker: red cone the user drags in the viewport
TGT = "/World/Target"
cone = UsdGeom.Cone.Define(stage, TGT + "/cone")
cone.GetHeightAttr().Set(0.6)
cone.GetRadiusAttr().Set(0.18)
cone.GetAxisAttr().Set("Z")
cone.GetDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.1, 0.1)])
UsdGeom.XformCommonAPI(cone.GetPrim()).SetTranslate(Gf.Vec3d(0, 0, 0.9))
tgt_xf = UsdGeom.Xformable(stage.GetPrimAtPath(TGT))
tgt_op = tgt_xf.AddTranslateOp()
tgt_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))


def target_xy():
    # world transform of the CONE, not the Target parent: viewport clicks
    # select the cone child, so a drag may move either prim — the cone's
    # world position reflects both
    m = UsdGeom.Xformable(cone.GetPrim()).ComputeLocalToWorldTransform(0)
    t = m.ExtractTranslation()
    return float(t[0]), float(t[1])


# ---------------- ROS ----------------
class TwinNode(Node):
    def __init__(self):
        super().__init__("ammr_isaac_live_twin")
        self.pose = None                      # (x, y, yaw) robot-map frame
        self.n_pose = 0
        self.create_subscription(Odometry, ARGS.state_topic, self._odom_cb, 10)
        self.create_subscription(PoseStamped, ARGS.pose_topic, self._ps_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._amcl_cb, 10)
        self.pub_goal = self.create_publisher(PoseStamped, ARGS.goal_topic, 10)
        # debug/monitoring: twin pose in the ISAAC world frame
        self.pub_twin = self.create_publisher(PoseStamped, "/ammr/twin_pose", 10)

    def publish_twin(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = "isaac_world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        self.pub_twin.publish(m)

    def _set(self, p, q):
        self.pose = (p.x, p.y, quat_yaw(q))
        self.n_pose += 1

    def _odom_cb(self, m):
        self._set(m.pose.pose.position, m.pose.pose.orientation)

    def _ps_cb(self, m):
        self._set(m.pose.position, m.pose.orientation)

    def _amcl_cb(self, m):
        self._set(m.pose.pose.position, m.pose.pose.orientation)

    def send_goal(self, mx, my, myaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = mx
        g.pose.position.y = my
        g.pose.orientation.z = math.sin(myaw / 2)
        g.pose.orientation.w = math.cos(myaw / 2)
        self.pub_goal.publish(g)
        self.get_logger().info(f"goal -> map ({mx:.2f}, {my:.2f})")


rclpy.init()
node = TwinNode()
world.reset()
set_twin_pose(*map_to_isaac(0.0, 0.0, 0.0))

last_tgt = target_xy()
settle_t = None
published_tgt = last_tgt
_dbg_t = [0.0]
print(f"[twin] live. Waiting for {ARGS.state_topic} (Odometry) / "
      f"{ARGS.pose_topic} / /amcl_pose; drag the red cone to send "
      f"{ARGS.goal_topic}")

try:
    while sim_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)

        if node.pose is not None:
            if ARGS.anchor is not None:
                calibrate_offset(tuple(ARGS.anchor), node.pose)
                ARGS.anchor = None            # once, on the first pose
            ix, iy, iyaw = map_to_isaac(*node.pose)
            set_twin_pose(ix, iy, iyaw)
            # publish the prim's ACTUAL transform, not the commanded one,
            # so /ammr/twin_pose exposes a stuck prim instead of masking it
            t_rb = UsdGeom.Xformable(robot_prim) \
                .ComputeLocalToWorldTransform(0).ExtractTranslation()
            node.publish_twin(float(t_rb[0]), float(t_rb[1]), iyaw)
            if world.current_time - _dbg_t[0] > 5.0:
                _dbg_t[0] = world.current_time
                print(f"[twin] cmd=({ix:.3f},{iy:.3f}) "
                      f"prim=({t_rb[0]:.3f},{t_rb[1]:.3f})", flush=True)

        # target-drag detection: moved, then stationary 0.5 s -> publish
        cur = target_xy()
        t_now = world.current_time
        if math.dist(cur, last_tgt) > 0.01:
            settle_t = t_now
            last_tgt = cur
        elif settle_t is not None and t_now - settle_t > 0.5:
            if math.dist(cur, published_tgt) > 0.05:
                rx, ry = cur
                # goal heading = from twin toward target
                if node.pose is not None:
                    tw = map_to_isaac(*node.pose)
                    gyaw = math.atan2(ry - tw[1], rx - tw[0])
                else:
                    gyaw = 0.0
                node.send_goal(*isaac_to_map(rx, ry, gyaw))
                published_tgt = cur
            settle_t = None

        world.step(render=not ARGS.headless)
finally:
    node.destroy_node()
    rclpy.shutdown()
    sim_app.close()
