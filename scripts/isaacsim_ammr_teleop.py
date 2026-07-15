#!/usr/bin/env python3
"""AMMR digital twin in Isaac Sim — teleop stage (sim-only).

Loads the vis_n2 collidable world + TOSM semantic overlay, imports the AMMR
URDF (swerve, 2 diagonal modules), and runs a swerve-drive controller fed by
ROS 2 /cmd_vel. Publishes /odom, /tf (odom->base_footprint), /clock and
/joint_states so the motion can be watched from RViz too.

Teleop test (user):
  terminal 1:
    ~/isaacsim/python.sh scripts/isaacsim_ammr_teleop.py \
        --world ~/ammr_twin/vis_n2_world.usda --overlay ~/ammr_twin/vis_n2_tosm.usda
  terminal 2 (system ROS 2 Jazzy, same domain):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # holonomic strafe: hold shift keys (J/L etc.) — swerve supports vy

Kinematics (ammr기구학): wheel r=0.075 m, modules right_front [0.364,-0.137] /
left_rear [-0.364,0.137], max drive 1.12 m/s, max steer rate 1.24 rad/s.
"""
import argparse
import math

from isaacsim import SimulationApp

_ap = argparse.ArgumentParser()
_ap.add_argument("--world", default=None, help="collidable world .usda")
_ap.add_argument("--overlay", default=None, help="TOSM semantic .usda (visual)")
_ap.add_argument("--urdf", default="src/ammr_description/urdf/ammr.urdf")
_ap.add_argument("--spawn", nargs=3, type=float, default=[-1.6, -0.1, 0.0],
                 help="x y yaw (map frame; default = big room centroid)")
_ap.add_argument("--headless", action="store_true")
_ap.add_argument("--physics-hz", type=float, default=120.0)
ARGS = _ap.parse_args()

sim_app = SimulationApp({"headless": ARGS.headless})

import os                                                     # noqa: E402
import numpy as np                                            # noqa: E402
import omni.kit.commands                                      # noqa: E402
import omni.usd                                               # noqa: E402
from pxr import UsdGeom, UsdPhysics, UsdShade, Gf             # noqa: E402
from isaacsim.core.api import World                           # noqa: E402
from isaacsim.core.utils.extensions import enable_extension   # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.ros2.bridge")
sim_app.update()

import rclpy                                                  # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry                             # noqa: E402
from sensor_msgs.msg import JointState                        # noqa: E402
from rosgraph_msgs.msg import Clock                           # noqa: E402
from tf2_msgs.msg import TFMessage                            # noqa: E402

# ---------------- swerve geometry ----------------
WHEEL_R = 0.075
MODULES = {                       # name -> (px, py) in base frame
    "right_front": (0.364, -0.137),
    "left_rear": (-0.364, 0.137),
}
V_MAX = 1.12                      # m/s   (spec)
W_MAX = 2.0                       # rad/s (limited by V_MAX at module radius)
CMD_TIMEOUT = 0.5                 # s without cmd_vel -> stop


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------- scene ----------------
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / ARGS.physics_hz, rendering_dt=1.0 / 60.0)
stage = omni.usd.get_context().get_stage()

if ARGS.world:
    add_reference_to_stage(usd_path=os.path.abspath(os.path.expanduser(ARGS.world)),
                           prim_path="/World/Env")
else:
    world.scene.add_default_ground_plane()
if ARGS.overlay:
    add_reference_to_stage(usd_path=os.path.abspath(os.path.expanduser(ARGS.overlay)),
                           prim_path="/World/Semantic")
    # raw-e57 frame -> floor at z=0 (matches build_usd --floor-offset, which
    # already shifted embedded points; overlay needs no extra z shift)

# ---------------- import AMMR URDF ----------------
urdf_path = os.path.abspath(os.path.expanduser(ARGS.urdf))
status, import_cfg = omni.kit.commands.execute("URDFCreateImportConfig")
import_cfg.merge_fixed_joints = True      # casters/lidar fold into base_link
import_cfg.fix_base = False
import_cfg.make_default_prim = False
import_cfg.self_collision = False
import_cfg.distance_scale = 1.0
import_cfg.density = 0.0                  # use URDF masses
status, robot_prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile", urdf_path=urdf_path,
    import_config=import_cfg, get_articulation_root=True)
print(f"[ammr] URDF imported at {robot_prim_path}")

# spawn pose
xf = UsdGeom.Xformable(stage.GetPrimAtPath(robot_prim_path))
sx, sy, syaw = ARGS.spawn
ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
if "xformOp:translate" in ops:
    ops["xformOp:translate"].Set(Gf.Vec3d(sx, sy, 0.02))
else:
    xf.AddTranslateOp().Set(Gf.Vec3d(sx, sy, 0.02))
if "xformOp:orient" in ops:
    ops["xformOp:orient"].Set(Gf.Quatd(math.cos(syaw / 2), 0, 0, math.sin(syaw / 2)))
else:
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(math.cos(syaw / 2), 0, 0, math.sin(syaw / 2)))

# frictionless caster spheres / grippy wheels
caster_mat = UsdShade.Material.Define(stage, "/World/PhysicsMat/caster")
capi = UsdPhysics.MaterialAPI.Apply(caster_mat.GetPrim())
capi.CreateStaticFrictionAttr(0.0)
capi.CreateDynamicFrictionAttr(0.0)
wheel_mat = UsdShade.Material.Define(stage, "/World/PhysicsMat/wheel")
wapi = UsdPhysics.MaterialAPI.Apply(wheel_mat.GetPrim())
wapi.CreateStaticFrictionAttr(1.1)
wapi.CreateDynamicFrictionAttr(1.0)
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if not p.startswith(robot_prim_path):
        continue
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    mat = caster_mat if "caster" in p else \
        (wheel_mat if "wheel_link" in p else None)
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            mat, materialPurpose="physics")

from isaacsim.core.prims import SingleArticulation            # noqa: E402
robot = world.scene.add(
    SingleArticulation(prim_path=robot_prim_path, name="ammr"))


# ---------------- ROS 2 node ----------------
class AmmrTeleopNode(Node):
    def __init__(self):
        super().__init__("ammr_isaac_twin")
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = -1e9
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb, 10)
        self.create_subscription(TwistStamped, "/cmd_vel_stamped",
                                 self._twist_stamped_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.now = 0.0

    def _store(self, t):
        vx = max(-V_MAX, min(V_MAX, t.linear.x))
        vy = max(-V_MAX, min(V_MAX, t.linear.y))
        wz = max(-W_MAX, min(W_MAX, t.angular.z))
        self.cmd = (vx, vy, wz)
        self.cmd_time = self.now

    def _twist_cb(self, m):
        self._store(m)

    def _twist_stamped_cb(self, m):
        self._store(m.twist)

    def active_cmd(self):
        if self.now - self.cmd_time > CMD_TIMEOUT:
            return (0.0, 0.0, 0.0)
        return self.cmd

    def _stamp(self):
        from builtin_interfaces.msg import Time as TimeMsg
        s = TimeMsg()
        s.sec = int(self.now)
        s.nanosec = int((self.now - int(self.now)) * 1e9)
        return s

    def publish_state(self, sim_t, pose_xyyaw, vel_xyw, joints):
        self.now = sim_t
        st = self._stamp()
        c = Clock()
        c.clock = st
        self.pub_clock.publish(c)

        x, y, yaw = pose_xyyaw
        od = Odometry()
        od.header.stamp = st
        od.header.frame_id = "odom"
        od.child_frame_id = "base_footprint"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = math.sin(yaw / 2)
        od.pose.pose.orientation.w = math.cos(yaw / 2)
        od.twist.twist.linear.x = vel_xyw[0]
        od.twist.twist.linear.y = vel_xyw[1]
        od.twist.twist.angular.z = vel_xyw[2]
        self.pub_odom.publish(od)

        tr = TransformStamped()
        tr.header.stamp = st
        tr.header.frame_id = "odom"
        tr.child_frame_id = "base_footprint"
        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.rotation.z = math.sin(yaw / 2)
        tr.transform.rotation.w = math.cos(yaw / 2)
        tf = TFMessage()
        tf.transforms = [tr]
        self.pub_tf.publish(tf)

        js = JointState()
        js.header.stamp = st
        js.name = list(joints.keys())
        js.position = [v[0] for v in joints.values()]
        js.velocity = [v[1] for v in joints.values()]
        self.pub_js.publish(js)


rclpy.init()
node = AmmrTeleopNode()

# ---------------- start sim, resolve DOFs ----------------
world.reset()
robot.initialize()
dof_names = list(robot.dof_names)
print("[ammr] DOFs:", dof_names)
idx = {}
for m in MODULES:
    idx[m + "_steer"] = dof_names.index(m + "_steer_joint")
    idx[m + "_wheel"] = dof_names.index(m + "_wheel_joint")

# gains: steer = stiff position drive, wheel = velocity drive
ndof = len(dof_names)
kps = np.zeros(ndof)
kds = np.zeros(ndof)
for m in MODULES:
    kps[idx[m + "_steer"]] = 8000.0
    kds[idx[m + "_steer"]] = 600.0
    kds[idx[m + "_wheel"]] = 1500.0
robot.get_articulation_controller().set_gains(kps=kps, kds=kds)

from isaacsim.core.utils.types import ArticulationAction      # noqa: E402

steer_target = {m: 0.0 for m in MODULES}
flip_state = {m: False for m in MODULES}
FLIP_HYST = 0.12          # rad; kills ±180°/±90° boundary oscillation
print("[ammr] ready — publish geometry_msgs/Twist on /cmd_vel "
      "(teleop_twist_keyboard). Ctrl-C here to quit.")

# ---------------- main loop ----------------
try:
    while sim_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        vx, vy, wz = node.active_cmd()

        q = robot.get_joint_positions()
        dq = robot.get_joint_velocities()

        steer_idx, steer_pos, wheel_idx, wheel_vel = [], [], [], []
        for m, (px, py) in MODULES.items():
            vix = vx - wz * py
            viy = vy + wz * px
            speed = math.hypot(vix, viy)
            cur = float(q[idx[m + "_steer"]])
            if speed > 0.02:
                ang = math.atan2(viy, vix)
                d_direct = wrap(ang - cur)
                d_flip = wrap(ang + math.pi - cur)
                if abs(d_flip) < abs(d_direct) - FLIP_HYST:
                    use_flip = True
                elif abs(d_direct) < abs(d_flip) - FLIP_HYST:
                    use_flip = False
                else:                             # near-tie: keep previous choice
                    use_flip = flip_state[m]
                flip_state[m] = use_flip
                delta = d_flip if use_flip else d_direct
                if use_flip:
                    speed = -speed
                steer_target[m] = cur + delta
                # don't drive across a large steering error (cosine gate)
                speed *= max(0.0, math.cos(delta))
            steer_idx.append(idx[m + "_steer"])
            steer_pos.append(steer_target[m])
            wheel_idx.append(idx[m + "_wheel"])
            wheel_vel.append(speed / WHEEL_R)
        ctrl = robot.get_articulation_controller()
        ctrl.apply_action(ArticulationAction(
            joint_positions=np.array(steer_pos), joint_indices=np.array(steer_idx)))
        ctrl.apply_action(ArticulationAction(
            joint_velocities=np.array(wheel_vel), joint_indices=np.array(wheel_idx)))

        world.step(render=not ARGS.headless)

        # pose -> odom (root x,y,yaw; base_footprint offset is pure z)
        p, quat = robot.get_world_pose()          # quat = (w,x,y,z)
        w_, x_, y_, z_ = [float(v) for v in quat]
        yaw = math.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
        lin = robot.get_linear_velocity()
        angv = robot.get_angular_velocity()
        joints = {n: (float(q[i]), float(dq[i])) for i, n in enumerate(dof_names)}
        node.publish_state(world.current_time,
                           (float(p[0]), float(p[1]), yaw),
                           (float(lin[0]), float(lin[1]), float(angv[2])),
                           joints)
finally:
    node.destroy_node()
    rclpy.shutdown()
    sim_app.close()
