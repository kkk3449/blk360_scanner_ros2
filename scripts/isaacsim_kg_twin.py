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
_ap.add_argument("--fps", type=float, default=30.0,
                 help="cap the render loop (0 = uncapped). Uncapped, the twin pins the GPU "
                      "at 100%% and starves gnome-shell (frozen desktop, Xid faults)")
# VDA 5050 mode: mirror the AGV from its MQTT state/visualization and send the
# cone goal as a VDA 5050 order (instead of ROS /amcl_pose + /goal_pose)
_ap.add_argument("--vda5050-broker", default=None, metavar="HOST[:PORT]")
_ap.add_argument("--vda-manufacturer", default="caselab")
_ap.add_argument("--vda-serial", default="ammr20")
_ap.add_argument("--vda-map-id", default="map")
_ap.add_argument("--kg-offset", nargs=3, type=float, default=None,
                 metavar=("X", "Y", "YAW_DEG"),
                 help="AGV map origin in the KG frame (VDA mode; default: "
                      "~/ammr_twin/robot_map_offset.json, else 0 0 0)")
_ap.add_argument("--heading-offset", type=float, default=180.0,
                 help="deg added to the AGV's reported yaw (AMMR: 180)")
# simulated twin AGV (second VDA 5050 endpoint executing the same orders)
_ap.add_argument("--twin-serial", default="ammr20-twin",
                 help="serialNumber of the simulated AGV ('' = disable)")
_ap.add_argument("--sim-map", default=os.path.expanduser("~/ammr_twin/map_vis_n2_1.yaml"),
                 help="KG-frame occupancy map the simulated AGV plans on")
_ap.add_argument("--sim-speed", type=float, default=0.8, help="m/s (AMMR max 1.12)")
_ap.add_argument("--sim-accel", type=float, default=0.5, help="m/s^2")
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

SCENE_URL = None
if ARGS.scene.startswith(("http://", "https://")):
    # remote DT host: fetch the scene from the console and re-fetch when its
    # mtime changes (polled via /api/scene/meta); local file = cache
    import json as _sj
    import urllib.request as _ur
    SCENE_URL = ARGS.scene.rstrip("/")
    _cache = os.path.expanduser("~/ammr_twin/cache")
    os.makedirs(_cache, exist_ok=True)
    SCENE = os.path.join(_cache, "kg_scene.usda")
    _remote_mtime = [0.0]

    def fetch_scene():
        with _ur.urlopen(SCENE_URL, timeout=30) as r:
            data = r.read()
            mt = float(r.headers.get("X-Scene-Mtime", "0") or 0)
        tmp = SCENE + ".tmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, SCENE)
        _remote_mtime[0] = mt
        print(f"[kg-twin] scene fetched from {SCENE_URL} ({len(data)//1024} kB, "
              f"remote mtime {time.ctime(mt)})", flush=True)

    def remote_scene_changed():
        try:
            with _ur.urlopen(SCENE_URL + "/meta", timeout=5) as r:
                mt = float(_sj.loads(r.read().decode())["mtime"])
        except Exception:  # noqa: BLE001
            return False
        return mt > _remote_mtime[0] + 0.5

    fetch_scene()
else:
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

# ---------------- VDA 5050 (MQTT) mode ----------------
VDA = None
if ARGS.vda5050_broker:
    import sys as _sys
    import json as _json
    _sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "..", "src", "semantic_nav_bt"))
    from semantic_nav_bt import vda5050 as V   # pure-python (no ROS) helpers

    _koff = ARGS.kg_offset
    if _koff is None:
        _koff = [0.0, 0.0, 0.0]
        _f = os.path.expanduser("~/ammr_twin/robot_map_offset.json")
        if os.path.exists(_f):
            try:
                _koff = [float(v) for v in _json.load(open(_f))["map_offset"]]
            except (ValueError, KeyError, TypeError):
                pass
    _KO = (_koff[0], _koff[1], math.radians(_koff[2]))
    _H = math.radians(ARGS.heading_offset)

    def agv_to_kg(x, y, yaw):
        ox, oy, oyaw = _KO
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * x - s * y, oy + s * x + c * y, yaw + oyaw + _H)

    def kg_to_agv(x, y, yaw):
        ox, oy, oyaw = _KO
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        return (c * dx - s * dy, s * dx + c * dy, yaw - oyaw - _H)

    _agv_pos = [None]                       # (x, y, theta) in the AGV map

    def _on_agv(pt, m):
        p = m.get("agvPosition")
        if p:
            _agv_pos[0] = (float(p["x"]), float(p["y"]), float(p.get("theta", 0.0)))
            node.amcl = agv_to_kg(*_agv_pos[0])
            node.n_pose += 1

    _hp = ARGS.vda5050_broker.split(":")
    VDA = V.Mqtt(_hp[0], int(_hp[1]) if len(_hp) > 1 else 1883,
                 client_id=V.client_id("isaac-twin"),
                 log=lambda s: print(f"[kg-twin] {s}", flush=True))
    VDA.subscribe(V.topic(ARGS.vda_manufacturer, ARGS.vda_serial, "state"), _on_agv)
    VDA.subscribe(V.topic(ARGS.vda_manufacturer, ARGS.vda_serial, "visualization"), _on_agv)
    VDA.start()
    _order_n = [0]

    def _send_order(mx, my, myaw):
        """cone goal (KG/map frame) -> single-goal VDA 5050 order to the AGV."""
        rx, ry, ryaw = kg_to_agv(mx, my, myaw)
        _order_n[0] += 1
        oid = f"twin{int(time.time() * 1000)}-{_order_n[0]}"
        cur = _agv_pos[0]
        nodes, edges = [], []
        if cur is not None:
            nodes.append(V.node(f"{oid}-n0", 0, cur[0], cur[1], cur[2],
                                ARGS.vda_map_id, dev_xy=1.0, description="current position"))
            nodes.append(V.node(f"{oid}-n1", 2, rx, ry, ryaw, ARGS.vda_map_id,
                                dev_xy=0.35, description="goal (Isaac cone)"))
            edges.append(V.edge(f"{oid}-e0", 1, nodes[0]["nodeId"], nodes[1]["nodeId"]))
        else:
            nodes.append(V.node(f"{oid}-n1", 0, rx, ry, ryaw, ARGS.vda_map_id,
                                dev_xy=0.35, description="goal (Isaac cone)"))
        VDA.publish(V.topic(ARGS.vda_manufacturer, ARGS.vda_serial, "order"),
                    V.make_order(V.Header(ARGS.vda_manufacturer, ARGS.vda_serial),
                                 oid, 0, nodes, edges))
        print(f"[kg-twin] VDA order {oid} -> {ARGS.vda_serial}: KG ({mx:.2f}, {my:.2f}) "
              f"-> AGV map ({rx:.2f}, {ry:.2f})", flush=True)

    node.send_goal = _send_order
    print(f"[kg-twin] VDA 5050 mode: broker {ARGS.vda5050_broker}, AGV "
          f"{ARGS.vda_manufacturer}/{ARGS.vda_serial}, KG offset {_koff}, "
          f"heading offset {ARGS.heading_offset}", flush=True)

    # ---- simulated twin AGV: ghost robot + predicted path ----
    SIM = None
    if ARGS.twin_serial:
        _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import sim_agv
        _grid, _gorig, _gcell = sim_agv.load_grid(ARGS.sim_map)
        GHOST = "/World/TwinAGV"
        UsdGeom.Xform.Define(stage, GHOST)
        _gb = UsdGeom.Cube.Define(stage, GHOST + "/body")
        _gb.GetSizeAttr().Set(1.0)
        UsdGeom.XformCommonAPI(_gb.GetPrim()).SetScale(Gf.Vec3f(1.244, 0.794, 0.45))
        UsdGeom.XformCommonAPI(_gb.GetPrim()).SetTranslate(Gf.Vec3d(0, 0, 0.30))
        _gb.GetDisplayColorAttr().Set([Gf.Vec3f(0.15, 0.45, 0.95)])
        _gb.GetDisplayOpacityAttr().Set([0.35])
        _gn = UsdGeom.Cone.Define(stage, GHOST + "/nose")     # heading marker
        _gn.GetHeightAttr().Set(0.35)
        _gn.GetRadiusAttr().Set(0.12)
        _gn.GetAxisAttr().Set("X")
        _gn.GetDisplayColorAttr().Set([Gf.Vec3f(0.15, 0.45, 0.95)])
        _gn.GetDisplayOpacityAttr().Set([0.6])
        UsdGeom.XformCommonAPI(_gn.GetPrim()).SetTranslate(Gf.Vec3d(0.75, 0, 0.30))
        _gxf = UsdGeom.Xformable(stage.GetPrimAtPath(GHOST))
        _g_tr = _gxf.AddTranslateOp()
        _g_or = _gxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        _g_tr.Set(Gf.Vec3d(0, 0, -5.0))                     # hidden until first pose
        _path_curve = UsdGeom.BasisCurves.Define(stage, "/World/TwinPath")
        _path_curve.GetTypeAttr().Set("linear")
        _path_curve.GetWidthsAttr().Set([0.06])
        _path_curve.GetDisplayColorAttr().Set([Gf.Vec3f(0.15, 0.45, 0.95)])
        _path_curve.GetCurveVertexCountsAttr().Set([])
        _path_curve.GetPointsAttr().Set([])
        _pending_path = [None, False]                        # (path, changed)

        def _on_path(path):
            _pending_path[0], _pending_path[1] = path, True   # applied on the render thread

        SIM = sim_agv.SimAgv(V, VDA, ARGS.vda_manufacturer, ARGS.twin_serial,
                             ARGS.vda_map_id, _grid, _gorig, _gcell,
                             agv_to_kg, kg_to_agv, vmax=ARGS.sim_speed,
                             acc=ARGS.sim_accel, log=lambda m: print(m, flush=True))
        SIM.on_path = _on_path
        if VDA.connected.is_set():
            SIM.announce()
        else:
            VDA._user_on_connect = SIM.announce
        _sim_last_t = [None]

        def sim_tick():
            now = time.time()
            dt = 0.0 if _sim_last_t[0] is None else min(0.1, now - _sim_last_t[0])
            _sim_last_t[0] = now
            if SIM.pose is None and node.amcl is not None:   # start on the real robot
                SIM.pose = node.amcl
            SIM.step(dt, now)
            pose, _v, _busy = SIM.snapshot()
            if pose is not None:
                ix, iy, iyaw = map_to_isaac(*pose)
                _g_tr.Set(Gf.Vec3d(ix, iy, 0.0))
                _g_or.Set(Gf.Quatd(math.cos(iyaw / 2), 0, 0, math.sin(iyaw / 2)))
            if _pending_path[1]:
                _pending_path[1] = False
                pth = _pending_path[0]
                if pth:
                    pts = [Gf.Vec3f(*map_to_isaac(x, y, 0.0)[:2], 0.12) for x, y in pth]
                    _path_curve.GetPointsAttr().Set(pts)
                    _path_curve.GetCurveVertexCountsAttr().Set([len(pts)])
                else:
                    _path_curve.GetPointsAttr().Set([])
                    _path_curve.GetCurveVertexCountsAttr().Set([])
        print(f"[kg-twin] simulated twin AGV {ARGS.twin_serial}: map {ARGS.sim_map} "
              f"grid {_grid.shape}, v {ARGS.sim_speed} m/s", flush=True)

world.reset()
set_twin_pose(*map_to_isaac(-2.0, 0.5, 0.0))
if not ARGS.headless:
    set_camera_view(eye=[_cx + 0.5, _cy - 11.0, 9.5], target=[_cx, _cy, 0.0])
    omni.usd.get_context().get_selection().clear_selected_prim_paths()

last_tgt = target_xy()
published_tgt = last_tgt
settle_t = None
cone_parked = [False]
# goal is sent when the LEFT mouse button is RELEASED after a cone drag (not
# while dragging). Headless (no mouse): fall back to "still for 0.5 s".
mouse_released = [False]
left_was_down = [False]
_mouse_sub = None
if not ARGS.headless:
    try:
        import carb.input
        import omni.appwindow
        _mouse = omni.appwindow.get_default_app_window().get_mouse()
        _inp = carb.input.acquire_input_interface()

        def _on_mouse(ev, *_):
            if ev.type == carb.input.MouseEventType.LEFT_BUTTON_UP:
                mouse_released[0] = True
            return True
        _mouse_sub = _inp.subscribe_to_mouse_events(_mouse, _on_mouse)

        def _left_down():
            # polled every frame: falling edge = release (works even when the
            # viewport gizmo consumes the button-up event)
            return _inp.get_mouse_value(
                _mouse, carb.input.MouseInput.LEFT_BUTTON) > 0.5
        print("[kg-twin] goal sends on left-button release", flush=True)
    except Exception as _e:  # noqa: BLE001
        print(f"[kg-twin] mouse hook unavailable ({_e}); using settle mode",
              flush=True)
_dbg_t = [0.0]
_chk_t = [0.0]
print("[kg-twin] live. Mirroring Gazebo/Nav2 robot (/tf map->odom + /odom, "
      "fallback /amcl_pose); drag the red cone to send /goal_pose; "
      "KG scene reloads when t4_kg_scene.usda changes.", flush=True)

_frame_budget = (1.0 / ARGS.fps) if ARGS.fps > 0 else 0.0
try:
    while sim_app.is_running():
        _frame_t0 = time.perf_counter()
        rclpy.spin_once(node, timeout_sec=0.0)
        if VDA is not None and SIM is not None:
            sim_tick()

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

        # scene hot-reload (kg_to_usd --watch rewrote the file; remote host:
        # the console's copy changed -> re-fetch into the local cache first)
        if time.time() - _chk_t[0] > 1.0:
            _chk_t[0] = time.time()
            if SCENE_URL and int(time.time()) % 5 == 0 and remote_scene_changed():
                try:
                    fetch_scene()
                except Exception as _e:  # noqa: BLE001
                    print(f"[kg-twin] scene re-fetch failed: {_e}", flush=True)
            try:
                mt = os.path.getmtime(SCENE)
            except OSError:
                mt = scene_mtime
            if mt != scene_mtime and time.time() - mt > 1.5:   # write settled
                scene_mtime = mt
                kg_layer.Reload(force=True)
                print(f"[kg-twin] KG scene reloaded ({time.ctime(mt)})",
                      flush=True)

        # cone drag -> goal: on left-button release (or, without a mouse hook,
        # once the cone has been still for 0.5 s)
        cur = target_xy()
        t_now = world.current_time
        fire = False
        if _mouse_sub is not None:
            down = _left_down()
            if left_was_down[0] and not down:
                mouse_released[0] = True
            left_was_down[0] = down
            if mouse_released[0]:
                mouse_released[0] = False
                fire = True
        else:
            if math.dist(cur, last_tgt) > 0.01:
                settle_t = t_now
                last_tgt = cur
            elif settle_t is not None and t_now - settle_t > 0.5:
                fire = True
        if fire:
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
        if _frame_budget:
            _left = _frame_budget - (time.perf_counter() - _frame_t0)
            if _left > 0:
                time.sleep(_left)
finally:
    node.destroy_node()
    rclpy.shutdown()
    sim_app.close()
