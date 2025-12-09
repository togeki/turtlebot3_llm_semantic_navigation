#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class Nav2Client(Node):
    """Nav2 NavigateToPose client + /go_to_pose 订阅者."""

    def __init__(self):
        super().__init__('nav2_client')

        # Nav2 Action Client
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 Client Ready. Call go_to(x, y, yaw) to move.')

        # 订阅 LLM / bridge 发来的 Pose 目标
        self._pose_sub = self.create_subscription(
            PoseStamped,
            'go_to_pose',           # 话题名：/go_to_pose
            self.pose_callback,
            10
        )
        self.get_logger().info('Topic [/go_to_pose] subscriber ready.')

    # 外部可直接调用的接口
    def go_to(self, x: float, y: float, yaw: float = 0.0) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(x, y, yaw)

        self.get_logger().info(f'Navigating to: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self._client.wait_for_server()
        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    # /go_to_pose 回调：从 PoseStamped 里取出 x, y, yaw，再调用 Nav2
    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y

        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        yaw = 2.0 * math.atan2(z, w)

        self.get_logger().info(
            f"Received /go_to_pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )
        self.go_to(x, y, yaw)

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
    node = Nav2Client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

