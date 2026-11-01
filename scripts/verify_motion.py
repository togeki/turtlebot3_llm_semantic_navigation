#!/usr/bin/env python3
"""Exercise the real controller and check odometry, without calling an LLM."""
import argparse
import json
import math
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tb3_llm_controller.llm_controller import LLMController


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gemini", action="store_true", help="Call real Gemini instead of a fixed plan")
    options = parser.parse_args()
    plan = [{"action": "FORWARD", "value": 0.2},
            {"action": "TURN_RIGHT", "value": 90},
            {"action": "STOP", "value": 0}]
    if options.gemini:
        from tb3_llm_controller.gemini_planner import get_plan
        command = "0.2メートル前進して、右に90度回転して、停止してください"
        plan = get_plan(command)
        print("Gemini input:", command, flush=True)
        print("Gemini plan:", json.dumps(plan, ensure_ascii=False), flush=True)
    rclpy.init()
    executor = SingleThreadedExecutor()
    observer = Node('demo_motion_check')
    samples = []
    observer.create_subscription(Odometry, '/odom', lambda msg: samples.append(msg), 10)
    executor.add_node(observer)
    deadline = time.monotonic() + 15
    while not samples and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    if not samples:
        raise RuntimeError('No /odom received')
    initial = samples[-1].pose.pose
    node = LLMController(plan)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    executor.add_node(node)
    deadline = time.monotonic() + 60
    try:
        while not node.finished and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not node.finished:
            raise RuntimeError('Motion timeout')
        end = time.monotonic() + 1
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        final = samples[-1]
        distance = math.hypot(final.pose.pose.position.x - initial.position.x,
                              final.pose.pose.position.y - initial.position.y)
        def yaw(q):
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        turn = math.atan2(math.sin(yaw(final.pose.pose.orientation)-yaw(initial.orientation)),
                          math.cos(yaw(final.pose.pose.orientation)-yaw(initial.orientation)))
        result = {'finished': node.finished, 'distance_m': distance, 'turn_degrees': math.degrees(turn),
                  'final_linear_mps': final.twist.twist.linear.x,
                  'final_angular_radps': final.twist.twist.angular.z}
        print(json.dumps(result, indent=2))
        assert 0.10 < distance < 0.35, result
        assert -115 < math.degrees(turn) < -65, result
        assert abs(result['final_linear_mps']) < 0.01, result
        assert abs(result['final_angular_radps']) < 0.02, result
    finally:
        from geometry_msgs.msg import Twist
        node.publish_velocity(Twist())
        node.destroy_node()
        observer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
