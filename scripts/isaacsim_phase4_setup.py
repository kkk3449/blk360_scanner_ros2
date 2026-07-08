"""Isaac Sim Phase-4 setup: TurtleBot3 digital twin + ROS 2 bridge (SCAFFOLD).

Loads the collidable testroom world, spawns a TurtleBot3, and builds the ROS 2
OmniGraph bridge so our existing ROS 2 stack (Cartographer + Nav2 + frontier +
stop-scan) drives the robot — Isaac Sim replacing Gazebo.

NOT RUN/VERIFIED on the dev box (needs Isaac Sim + GPU). Targets Isaac Sim
4.5 / 5.0 (isaacsim.* namespace). On 4.2 the modules are omni.isaac.*. Treat as
a scaffold; adjust asset paths / node names to your install. See
docs/isaac_phase4_runbook.md for the contract (topics, TF, TwistStamped).

Run:
    <isaac>/python.sh scripts/isaacsim_phase4_setup.py --world testroom_world.usda
"""
import argparse

# --- 1. boot the app BEFORE importing isaac/omni modules ---
from isaacsim import SimulationApp  # noqa: E402

_ap = argparse.ArgumentParser()
_ap.add_argument("--world", required=True, help="collidable world .usda")
_ap.add_argument("--overlay", default=None, help="optional semantic point-cloud USD")
_ap.add_argument("--headless", action="store_true")
_ap.add_argument("--spawn", nargs=2, type=float, default=[1.83, -0.62],
                 help="robot spawn x y (free cell in the map frame)")
ARGS = _ap.parse_args()

sim_app = SimulationApp({"headless": ARGS.headless})

# isaac/omni imports must come after SimulationApp()
import omni.usd                                              # noqa: E402
from isaacsim.core.api import World                          # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path    # noqa: E402
import omni.graph.core as og                                # noqa: E402

# --- 2. scene: collidable world (+ optional semantic overlay) ---
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
add_reference_to_stage(usd_path=ARGS.world, prim_path="/World/Testroom")
if ARGS.overlay:
    add_reference_to_stage(usd_path=ARGS.overlay, prim_path="/World/Semantic")

# --- 3. robot: TurtleBot3 (Waffle). Asset path varies by Isaac version/install;
#     if the bundled TB3 is absent, import its URDF once via the URDF Importer
#     and reference the resulting USD here. Carter is the always-present fallback.
assets = get_assets_root_path()
TB3 = assets + "/Isaac/Robots/Turtlebot/turtlebot3_waffle.usd"   # verify path
ROBOT_PRIM = "/World/turtlebot3"
add_reference_to_stage(usd_path=TB3, prim_path=ROBOT_PRIM)
# set spawn pose
from isaacsim.core.prims import SingleXFormPrim              # noqa: E402
SingleXFormPrim(ROBOT_PRIM).set_world_pose(
    position=[ARGS.spawn[0], ARGS.spawn[1], 0.05])

# --- 4. ROS 2 bridge as an OmniGraph (clock / tf / odom / lidar / cmd_vel) ---
# Enable the ROS 2 bridge extension first:
#   from isaacsim.core.utils.extensions import enable_extension
#   enable_extension("isaacsim.ros2.bridge")
#
# Build a graph equivalent to the official "ROS2 Nav2" sample, wired to TB3:
#   OnPlaybackTick ─► ROS2Context
#                 ├─► ROS2PublishClock            (/clock)
#                 ├─► IsaacComputeOdometry ─► ROS2PublishOdometry   (/odom)
#                 │                        └─► ROS2PublishRawTransformTree (odom->base_footprint)
#                 ├─► ROS2PublishTransformTree    (TF: base_footprint->base_scan, ...)
#                 ├─► RTXLidar ─► ROS2RtxLidarHelper(type=laser_scan)  (/scan, frame=base_scan)
#                 └─► ROS2SubscribeTwist(/cmd_vel) ─► (scale) ─► ArticulationController(TB3 wheels)
#
# IMPORTANT (see runbook): match /cmd_vel message type. Our Nav2 publishes
# TwistStamped; either set Nav2 enable_stamped_cmd_vel:=false, run a twist_stamper
# relay, or use a TwistStamped subscribe node here. Lidar frame MUST be
# "base_scan" and TF must provide odom->base_footprint->base_scan so Cartographer
# (tracking_frame=base_footprint, scan topic /scan) is satisfied.
GRAPH = "/World/ROS2_Bridge"
try:
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                # add Odometry / TF / Lidar / SubscribeTwist nodes + their
                # ATTRIBUTE/CONNECT entries here per your TB3 prim paths.
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("Ctx.outputs:context", "Clock.inputs:context"),
            ],
        },
    )
    print("[phase4] ROS2 bridge graph stub created at", GRAPH,
          "- extend with Odometry/TF/Lidar/Twist nodes (see comments).")
except Exception as exc:  # noqa: BLE001
    print("[phase4] graph creation needs the ros2 bridge extension enabled "
          "and node names matching your Isaac version:", exc)

# --- 5. run ---
world.reset()
print("[phase4] Playing. In another terminal:\n"
      "  ros2 topic list   # expect /clock /scan /odom /tf\n"
      "  ros2 launch blk360_bringup active_mapping.launch.py use_sim_time:=true ...")
while sim_app.is_running():
    world.step(render=not ARGS.headless)
sim_app.close()
