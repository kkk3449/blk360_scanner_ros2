"""ROS2 mission node: ticks the semantic BT, bridges topics.

Subscribes  /semantic_command   std_msgs/String (JSON command)
            /battery_state      sensor_msgs/BatteryState (optional)
Publishes   /semantic_status    std_msgs/String (JSON: status + mission state)

  ros2 run semantic_nav_bt mission_bt --ros-args \
      -p kg_path:=/path/testroom_epochs_kg.json \
      -p places_path:=/path/place_layer_T3_slic.json \
      -p dry_run:=true
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

from .mediator import Mediator
from .bt_nodes import Blackboard, build_tree

BLK = "/home/caselab/Downloads/Cyclone360_data/blk360_seg/outputs"


class MissionNode(Node):
    def __init__(self):
        super().__init__("semantic_mission_bt")
        p = self.declare_parameter
        kg = p("kg_path", f"{BLK}/testroom_epochs_kg.json").value
        places = p("places_path", f"{BLK}/place_layer_T3_slic.json").value
        naming = p("naming_path", f"{BLK}/place_ring_naming.json").value
        map_yaml = p("map_yaml",
                     "/home/caselab/ammr_twin/map_vis_n2_1.yaml").value
        gated = p("gated", True).value
        self.dry_run = p("dry_run", False).value
        tick_hz = p("tick_hz", 2.0).value

        self.mediator = Mediator(kg, places, naming, map_yaml, gated=gated)
        self.bb = Blackboard()
        self.tree = build_tree(self.bb, self.mediator, node=self,
                               dry_run=self.dry_run)
        self.tree.setup()

        self.create_subscription(String, "semantic_command", self._on_cmd, 10)
        self.create_subscription(BatteryState, "battery_state",
                                 self._on_batt, 10)
        self.status_pub = self.create_publisher(String, "semantic_status", 10)
        self._last_status = None
        self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info(
            f"semantic mission BT up (gated={gated}, dry_run={self.dry_run})")

    def _on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            # bare-word convenience: "dock", or "goto <target>"
            w = msg.data.strip().split()
            cmd = {"cmd": "dock"} if w[0] == "dock" else \
                {"cmd": "goto_object", "target": " ".join(w[1:])} \
                if w[0] == "goto" else None
        if not cmd:
            self.get_logger().warning(f"bad command: {msg.data}")
            return
        self.get_logger().info(f"command: {cmd}")
        self.bb.put_command(cmd)

    def _on_batt(self, msg):
        if msg.percentage:
            self.bb.battery_level = (msg.percentage / 100.0
                                     if msg.percentage > 1.0
                                     else msg.percentage)

    def _tick(self):
        self.tree.root.tick_once()
        if self.bb.status != self._last_status:
            self._last_status = self.bb.status
            self.get_logger().info(self.bb.status)
        out = {"status": self.bb.status,
               "command": self.bb.command,
               "goal_idx": self.bb.goal_idx,
               "n_goals": len(self.bb.goals),
               "battery": self.bb.battery_level,
               "dock_requested": self.bb.dock_requested}
        self.status_pub.publish(String(data=json.dumps(out)))


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
