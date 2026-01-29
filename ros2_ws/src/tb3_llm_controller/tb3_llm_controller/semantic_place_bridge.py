#!/usr/bin/env python3
import os
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml


class SemanticPlaceBridge(Node):
    """
    把语义地点名 (door/window) 翻译成 Nav2 用的 (x, y, yaw).

    订阅:
      /llm_place (std_msgs/String)
        例如: "door", "window"

    发布:
      /llm_nav_json (std_msgs/String)
        例如: '{"x": 1.0, "y": 0.0, "yaw": 0.0}'
    """

    def __init__(self):
        super().__init__('semantic_place_bridge')

        # 载入 semantic_places.yaml
        package_share = get_package_share_directory('tb3_llm_controller')
        yaml_path = os.path.join(package_share, 'config', 'semantic_places.yaml')
        self.get_logger().info(f'Loading semantic places from: {yaml_path}')

        self.places = {}  # name -> (x, y, yaw)
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            for item in data.get('places', []):
                name = str(item.get('name', '')).strip().lower()
                x = float(item.get('x', 0.0))
                y = float(item.get('y', 0.0))
                yaw = float(item.get('yaw', 0.0))
                self.places[name] = (x, y, yaw)
            self.get_logger().info(f'Loaded places: {list(self.places.keys())}')
        except Exception as e:
            self.get_logger().error(f'Failed to load semantic_places.yaml: {e}')

        # 订阅 LLM 发来的地点名
        self.sub = self.create_subscription(
            String,
            'llm_place',
            self.place_callback,
            10
        )

        # 发布 给 llm_nav_bridge 的 JSON 指令
        self.pub = self.create_publisher(
            String,
            'llm_nav_json',
            10
        )

        self.get_logger().info(
            'SemanticPlaceBridge started. '
            'Listening on [/llm_place], publishing JSON to [/llm_nav_json].'
        )

    def place_callback(self, msg: String):
        name_raw = msg.data.strip()
        name = name_raw.lower()
        self.get_logger().info(f'Received place name: "{name_raw}"')

        if name not in self.places:
            self.get_logger().warn(
                f'Unknown place "{name_raw}". Known places: {list(self.places.keys())}'
            )
            return

        x, y, yaw = self.places[name]
        cmd = {
            "x": x,
            "y": y,
            "yaw": yaw,
        }
        out = String()
        out.data = json.dumps(cmd)
        self.pub.publish(out)
        self.get_logger().info(
            f'Published to /llm_nav_json: {out.data}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPlaceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
