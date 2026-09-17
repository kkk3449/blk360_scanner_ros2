"""Isaac Sim view of the SEMANTIC DB + the Gazebo/Nav2 robot, live.

Couples the semantic web UI stack (scripts/bringup_all.sh: Gazebo visn2_room
-> AMCL -> Nav2 -> mediator/BT -> ui_server) with Isaac Sim:

  * scene   = t4_kg_scene.usda, built from testroom_epochs_kg.json by
              blk360_seg/scripts/kg_to_usd.py --watch (map/vis_n2 frame,
              floor z=0).  This script watches the .usda mtime and RELOADS
              the layer in place, so an owner edit in the web UI (refute /
              type / movable / JSON editor) shows up in Isaac within seconds
              without File > Reload.
  * robot   = AMMR URDF as a kinematic twin, driven by the Nav2 localization
              of the Gazebo robot: map->odom from /tf (AMCL) composed with
              /odom (Gazebo), so the twin moves smoothly, not only on AMCL
              updates.  /amcl_pose (latched) is used as a fallback.
  * goal    = drag the red cone in the viewport -> /goal_pose (map frame)
              -> Nav2 drives the Gazebo robot; the web UI status log and the
              Isaac twin both follow.
  * inspect = click an object in the viewport -> "TOSM Object Info" panel
              shows the record (type / status / confidence / implicit /
              relations) read from the prim's customData.

Same frame everywhere (map == vis_n2 E57 XY, floor z=0), so no map-offset
is needed.  Run via scripts/run_isaac_ammr.sh kg (local FastDDS, domain 0).
"""
import argparse
import math
import os
import time

from isaacsim import SimulationApp

_ap = argparse.ArgumentParser()
_ap.add_argument("--scene", default=os.path.expanduser(
    "~/Downloads/Cyclone360_data/blk360_seg/outputs/vis_sota_det4/t4_kg_scene.usda"))
_ap.add_argument("--urdf", default="src/ammr_description/urdf/ammr.urdf")
_ap.add_argument("--goal-topic", default="/goal_pose")
_ap.add_argument("--map-offset", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                 metavar=("X", "Y", "YAW_DEG"))
_ap.add_argument("--pose-source", choices=["auto", "amcl"], default="auto",
                 help="auto: /tf map->odom + /odom (Gazebo); amcl: /amcl_pose only (real robot via bridge)")
_ap.add_argument("--amcl-topic", default="/amcl_pose",
                 help="latched robot pose topic (real robot: /kg_robot_pose from the bridge)")
_ap.add_argument("--headless", action="store_true")
ARGS = _ap.parse_args()

sim_app = SimulationApp({"headless": ARGS.headless})

import omni.kit.commands                                      # noqa: E402
import omni.usd                                               # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics          # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view    # noqa: E402
from isaacsim.core.api import World                           # noqa: E402
from isaacsim.core.utils.extensions import enable_extension   # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.ros2.bridge")
sim_app.update()

import rclpy                                                  # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,       # noqa: E402
                       QoSReliabilityPolicy)
from geometry_msgs.msg import (PoseStamped,                   # noqa: E402
                               PoseWithCovarianceStamped)
from nav_msgs.msg import Odometry                             # noqa: E402
from tf2_msgs.msg import TFMessage                            # noqa: E402

SCENE = os.path.abspath(os.path.expanduser(ARGS.scene))
OFF_X, OFF_Y = ARGS.map_offset[0], ARGS.map_offset[1]
OFF_YAW = math.radians(ARGS.map_offset[2])


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


def compose(a, b):
    """SE(2) a∘b: apply b in a's frame."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


# ---------------- scene ----------------
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
KG_PRIM = "/World/KG"
add_reference_to_stage(usd_path=SCENE, prim_path=KG_PRIM)
kg_layer = Sdf.Layer.FindOrOpen(SCENE)
scene_mtime = os.path.getmtime(SCENE)
print(f"[kg-twin] scene {SCENE} (rev mtime {time.ctime(scene_mtime)})",
      flush=True)
# lights (the KG scene carries none) + an overhead camera on the room
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(250.0)
_sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
_sun.GetIntensityAttr().Set(600.0)
UsdGeom.XformCommonAPI(_sun.GetPrim()).SetRotate(Gf.Vec3f(-35.0, 20.0, 0.0))
try:
    _g = __import__("json").load(open(os.path.expanduser(
        "~/Downloads/Cyclone360_data/blk360_seg/outputs/testroom_epochs_kg.json")))
    _pts = [(n["pose"]["x"], n["pose"]["y"]) for n in _g["nodes"]
            if n.get("presence") == "present"]
    _cx = sum(p[0] for p in _pts) / len(_pts)
    _cy = sum(p[1] for p in _pts) / len(_pts)
except Exception:
    _cx, _cy = -2.0, -1.5
if not ARGS.headless:
    set_camera_view(eye=[_cx + 0.5, _cy - 11.0, 9.5], target=[_cx, _cy, 0.0])

# twin robot: URDF imported once, physics stripped -> kinematic visual
status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = True
cfg.fix_base = True
cfg.make_default_prim = False
status, ROBOT = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=os.path.abspath(os.path.expanduser(ARGS.urdf)),
    import_config=cfg, get_articulation_root=True)
ROBOT = "/" + ROBOT.strip("/").split("/")[0]
robot_prim = stage.GetPrimAtPath(ROBOT)
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if not p.startswith(ROBOT):
        continue
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
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
if "xformOp:translate" not in robot_ops:
    robot_ops["xformOp:translate"] = robot_xf.AddTranslateOp()
if "xformOp:orient" not in robot_ops:
    robot_ops["xformOp:orient"] = robot_xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)


def set_twin_pose(x, y, yaw, z=0.0):
    robot_ops["xformOp:translate"].Set(Gf.Vec3d(x, y, z))
    robot_ops["xformOp:orient"].Set(
        Gf.Quatd(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))


# red cone = navigation goal handle
TGT = "/World/Target"
UsdGeom.Xform.Define(stage, TGT)          # must be a real Xform, else its translate op is ignored
cone = UsdGeom.Cone.Define(stage, TGT + "/cone")
cone.GetHeightAttr().Set(0.6)
cone.GetRadiusAttr().Set(0.18)
cone.GetAxisAttr().Set("Z")
cone.GetDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.1, 0.1)])
UsdGeom.XformCommonAPI(cone.GetPrim()).SetTranslate(Gf.Vec3d(0, 0, 0.9))
tgt_op = UsdGeom.Xformable(stage.GetPrimAtPath(TGT)).AddTranslateOp()
tgt_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))     # parked at the origin until the first robot pose


def target_xy():
    m = UsdGeom.Xformable(cone.GetPrim()).ComputeLocalToWorldTransform(0)
    t = m.ExtractTranslation()
    return float(t[0]), float(t[1])


# ---------------- TOSM inspector panel ----------------
_KEYS = ["name", "type", "verificationStatus", "confidence", "isKeyObject",
         "isMovable", "heightLevel", "pose_xyz", "dimensions_lwh",
         "rel_isInsideOf", "rel_isOn", "rel_isNextTo", "symbolicReason"]
inspector = None
if not ARGS.headless:
    import omni.ui as ui

    class TosmInspector:
        def __init__(self):
            self._win = ui.Window("TOSM Object Info", width=380, height=460)
            # park the panel over the Property pane (bottom-right), off the viewport
            self._win.position_x = 985
            self._win.position_y = 470
            self._labels = {}
            with self._win.frame:
                with ui.VStack(spacing=4):
                    self._title = ui.Label("(click an object)",
                                           style={"font_size": 18})
                    ui.Separator()
                    for k in _KEYS:
                        with ui.HStack(height=0):
                            ui.Label(k, width=140, style={"color": 0xFF9AA0A6})
                            self._labels[k] = ui.Label("", word_wrap=True)
            self._sub = (omni.usd.get_context().get_stage_event_stream()
                         .create_subscription_to_pop(self._on_event,
                                                     name="tosm_inspector"))

        def _on_event(self, e):
            if e.type != int(omni.usd.StageEventType.SELECTION_CHANGED):
                return
            sel = omni.usd.get_context().get_selection().get_selected_prim_paths()
            if not sel:
                return
            prim = stage.GetPrimAtPath(sel[0])
            # climb to the object prim (the one carrying customData)
            while prim and prim.IsValid() and not prim.GetCustomData():
                prim = prim.GetParent()
            if not prim or not prim.IsValid():
                return
            cd = prim.GetCustomData()
            self._title.text = f"{cd.get('name', prim.GetName())}"
            for k in _KEYS:
                v = cd.get(k, "")
                self._labels[k].text = str(v) if v != "" else "-"

    inspector = TosmInspector()


# ---------------- ROS ----------------
class KgTwinNode(Node):
    def __init__(self):
        super().__init__("isaac_kg_twin")
        self.map_odom = None      # (x, y, yaw) map->odom from AMCL via /tf
        self.odom_base = None     # (x, y, yaw) odom->base from Gazebo /odom
        self.amcl = None          # latched AMCL pose (fallback)
        self.n_pose = 0
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        if ARGS.pose_source == "auto":
            self.create_subscription(TFMessage, "/tf", self._tf_cb, 50)
            self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(PoseWithCovarianceStamped, ARGS.amcl_topic,
                                 self._amcl_cb, latched)
        self.pub_goal = self.create_publisher(PoseStamped, ARGS.goal_topic, 10)

    def _tf_cb(self, m):
        for t in m.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                tr, q = t.transform.translation, t.transform.rotation
                self.map_odom = (tr.x, tr.y, quat_yaw(q))

    def _odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom_base = (p.x, p.y, quat_yaw(q))
        self.n_pose += 1

    def _amcl_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.amcl = (p.x, p.y, quat_yaw(q))
        self.n_pose += 1

    def pose(self):
        if ARGS.pose_source == "auto" and self.map_odom is not None \
                and self.odom_base is not None:
            return compose(self.map_odom, self.odom_base)
        return self.amcl

    def send_goal(self, mx, my, myaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = mx
        g.pose.position.y = my
        g.pose.orientation.z = math.sin(myaw / 2)
        g.pose.orientation.w = math.cos(myaw / 2)
        self.pub_goal.publish(g)
        print(f"[kg-twin] goal -> {ARGS.goal_topic} map ({mx:.2f}, {my:.2f})",
              flush=True)


rclpy.init()
node = KgTwinNode()
world.reset()
set_twin_pose(*map_to_isaac(-2.0, 0.5, 0.0))
if not ARGS.headless:
    set_camera_view(eye=[_cx + 0.5, _cy - 11.0, 9.5], target=[_cx, _cy, 0.0])
    omni.usd.get_context().get_selection().clear_selected_prim_paths()

last_tgt = target_xy()
published_tgt = last_tgt
settle_t = None
cone_parked = [False]
_dbg_t = [0.0]
_chk_t = [0.0]
print("[kg-twin] live. Mirroring Gazebo/Nav2 robot (/tf map->odom + /odom, "
      "fallback /amcl_pose); drag the red cone to send /goal_pose; "
      "KG scene reloads when t4_kg_scene.usda changes.", flush=True)

try:
    while sim_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)

        p = node.pose()
        if p is not None:
            ix, iy, iyaw = map_to_isaac(*p)
            set_twin_pose(ix, iy, iyaw)
            if not cone_parked[0]:            # first pose: put the cone on the robot
                tgt_op.Set(Gf.Vec3d(ix, iy, 0.0))
                cone_parked[0] = True
                last_tgt = published_tgt = target_xy()
                settle_t = None
            if world.current_time - _dbg_t[0] > 5.0:
                _dbg_t[0] = world.current_time
                src = ("tf+odom" if ARGS.pose_source == "auto"
                       and node.map_odom is not None else "amcl")
                t_rb = UsdGeom.Xformable(robot_prim) \
                    .ComputeLocalToWorldTransform(0).ExtractTranslation()
                cx, cy = target_xy()
                print(f"[kg-twin] robot map=({p[0]:.2f},{p[1]:.2f},"
                      f"{math.degrees(p[2]):.0f}deg) via {src} | prim=("
                      f"{t_rb[0]:.2f},{t_rb[1]:.2f}) cone=({cx:.2f},{cy:.2f})",
                      flush=True)

        # scene hot-reload (kg_to_usd --watch rewrote the file)
        if time.time() - _chk_t[0] > 1.0:
            _chk_t[0] = time.time()
            try:
                mt = os.path.getmtime(SCENE)
            except OSError:
                mt = scene_mtime
            if mt != scene_mtime and time.time() - mt > 1.5:   # write settled
                scene_mtime = mt
                kg_layer.Reload(force=True)
                print(f"[kg-twin] KG scene reloaded ({time.ctime(mt)})",
                      flush=True)

        # cone drag -> goal (moved, then still for 0.5 s)
        cur = target_xy()
        t_now = world.current_time
        if math.dist(cur, last_tgt) > 0.01:
            settle_t = t_now
            last_tgt = cur
        elif settle_t is not None and t_now - settle_t > 0.5:
            if math.dist(cur, published_tgt) > 0.05:
                rx, ry = cur
                if p is not None:
                    tw = map_to_isaac(*p)
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
