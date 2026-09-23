"""ROS2 service wrapper around the TOSM semantic mediator.

Exposes /resolve_semantic (semantic_nav_msgs/ResolveSemantic) so the C++
BT executor (mission_bt_cpp) can resolve semantic commands against the
knowledge graph while the KG layer — including owner-edit hot-reload —
stays in Python.

  ros2 run semantic_nav_bt mediator_server --ros-args -p gated:=true
"""
import json
import os

import rclpy
from rclpy.node import Node
from semantic_nav_msgs.srv import ResolveSemantic

from .mediator import Mediator

# data dir; a remote console host (laptop/container) points this at its synced copy
BLK = os.environ.get("BLK_OUTPUTS", "/home/caselab/Downloads/Cyclone360_data/blk360_seg/outputs")
MAP_YAML_DEFAULT = os.environ.get("MAP_YAML", "/home/caselab/ammr_twin/map_vis_n2_1.yaml")


class MediatorServer(Node):
    def __init__(self):
        super().__init__("semantic_mediator")
        p = self.declare_parameter
        kg = p("kg_path", f"{BLK}/testroom_epochs_kg.json").value
        places = p("places_path", f"{BLK}/place_layer_T3_slic.json").value
        naming = p("naming_path", f"{BLK}/place_ring_naming.json").value
        map_yaml = p("map_yaml", MAP_YAML_DEFAULT).value
        gated = p("gated", True).value
        self.mediator = Mediator(kg, places, naming, map_yaml, gated=gated)
        self.create_service(ResolveSemantic, "resolve_semantic", self._on_req)
        self.get_logger().info(f"mediator service up (gated={gated})")

    def _on_req(self, req, res):
        try:
            cmd = json.loads(req.command_json)
            out = self.mediator.resolve(cmd)
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
            out = {"error": f"bad command: {e}"}
        res.result_json = json.dumps(out)
        return res


def main():
    rclpy.init()
    node = MediatorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
