#!/usr/bin/env python3
import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class LlmNavBridge(Node):
    """
    LLM → Nav2 桥接节点.

    订阅:
      /llm_nav_json (std_msgs/String)
        JSON 例:
          {"x": 1.0, "y": 0.5, "yaw": 0.0}

    发布:
      /go_to_pose (geometry_msgs/PoseStamped)
    """

    def __init__(self):
        super().__init__('llm_nav_bridge')

        self._sub = self.create_subscription(
            String,
            'llm_nav_json',
            self.command_callback,
            10
        )

        self._pub = self.create_publisher(
            PoseStamped,
            'go_to_pose',
            10
        )

        self.get_logger().info(
            'LLM Nav Bridge started. '
            'Listening on [/llm_nav_json], publishing Pose to [/go_to_pose].'
        )

    def command_callback(self, msg: String):
        self.get_logger().info(f"Received LLM nav JSON: {msg.data}")

        try:
            data = json.loads(msg.data)
            x = float(data.get('x', 0.0))
            y = float(data.get('y', 0.0))
            yaw = float(data.get('yaw', 0.0))
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._pub.publish(pose)
        self.get_logger().info(
            f"Published /go_to_pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LlmNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
