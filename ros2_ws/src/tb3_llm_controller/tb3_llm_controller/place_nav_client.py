#!/usr/bin/env python3
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# 这里定义语义地点 → 地图坐标（你后面可以自己调）
PLACE_MAP = {
    "red_box":   (2.0, 0.0, 0.0),
    "green_box": (0.0, 2.0, 0.0),
    "home":      (0.0, 0.0, 0.0),
}


class PlaceNavClient(Node):
    """Nav2 client that navigates to a named place (semantic place)."""

    def __init__(self):
        super().__init__('place_nav_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info(
            "PlaceNavClient ready. Usage: ros2 run tb3_llm_controller place_nav_client -- <place_name>"
        )

    def go_to_place(self, place_name: str):
        if place_name not in PLACE_MAP:
            self.get_logger().error(f"Unknown place: {place_name}. Known places: {list(PLACE_MAP.keys())}")
            return

        x, y, yaw = PLACE_MAP[place_name]
        self.get_logger().info(f"Navigating to place '{place_name}' -> (x={x}, y={y}, yaw={yaw})")

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(x, y, yaw)

        self._client.wait_for_server()
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)

    def _build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Nav2 finished with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = PlaceNavClient()

    # 从命令行拿地点名：ros2 run ... place_nav_client -- red_box
    if len(sys.argv) < 2:
        node.get_logger().error("No place name provided. Example: ros2 run tb3_llm_controller place_nav_client -- red_box")
    else:
        place = sys.argv[1]
        node.go_to_place(place)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
