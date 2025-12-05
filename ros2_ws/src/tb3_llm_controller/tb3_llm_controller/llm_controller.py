# tb3_llm_controller/llm_controller.py

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

from tb3_llm_controller.gemini_planner import get_plan


class LLMController(Node):
    """
    LLM から受け取った plan（FORWARD / TURN_LEFT / TURN_RIGHT / STOP）を
    順番に実行するノード。
    1 つの plan を実行し終わると finished フラグが立つ。
    """

    def __init__(self, plan):
        super().__init__('llm_controller')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.plan = plan
        self.current_step_index = 0
        self.ticks_remaining = 0

        # 制御パラメータ
        self.dt = 0.05            # timer period (s)
        self.linear_speed = 0.1   # m/s
        self.angular_speed = 0.5  # rad/s

        self.current_mode = "stop"  # "forward" / "turn_left" / "turn_right" / "stop"
        self.finished = False       # この plan が終わったかどうか

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(f"LLMController started with {len(self.plan)} steps.")
        self._start_next_step()

    # ---- 内部ロジック ----

    def _start_next_step(self):
        """次のステップへ進み，必要な tick 数を計算する。"""
        if self.current_step_index >= len(self.plan):
            self.get_logger().info("Plan finished. Robot will stop.")
            self.current_mode = "stop"
            self.ticks_remaining = -1
            self.finished = True
            return

        step = self.plan[self.current_step_index]
        action = str(step.get("action", "")).upper()
        value = float(step.get("value", 0.0))

        min_ticks = 1

        if action == "FORWARD":
            duration = abs(value) / self.linear_speed if self.linear_speed > 0 else 0.0
            ticks = max(min_ticks, int(duration / self.dt))
            self.current_mode = "forward"
            self.ticks_remaining = ticks
            self.get_logger().info(f"[Step {self.current_step_index}] FORWARD {value} m, ticks={ticks}")

        elif action == "TURN_LEFT":
            angle_rad = abs(value) * math.pi / 180.0
            duration = angle_rad / self.angular_speed if self.angular_speed > 0 else 0.0
            ticks = max(min_ticks, int(duration / self.dt))
            self.current_mode = "turn_left"
            self.ticks_remaining = ticks
            self.get_logger().info(f"[Step {self.current_step_index}] TURN_LEFT {value} deg, ticks={ticks}")

        elif action == "TURN_RIGHT":
            angle_rad = abs(value) * math.pi / 180.0
            duration = angle_rad / self.angular_speed if self.angular_speed > 0 else 0.0
            ticks = max(min_ticks, int(duration / self.dt))
            self.current_mode = "turn_right"
            self.ticks_remaining = ticks
            self.get_logger().info(f"[Step {self.current_step_index}] TURN_RIGHT {value} deg, ticks={ticks}")

        elif action == "STOP":
            self.current_mode = "stop"
            self.ticks_remaining = min_ticks
            self.get_logger().info(f"[Step {self.current_step_index}] STOP")
        else:
            self.get_logger().warn(f"Unknown action: {action}, skipping.")
            self.current_step_index += 1
            self._start_next_step()

    def timer_callback(self):
        msg = Twist()

        if self.ticks_remaining == -1:
            # 完全終了：停止状態を維持
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        if self.current_mode == "forward":
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
        elif self.current_mode == "turn_left":
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed
        elif self.current_mode == "turn_right":
            msg.linear.x = 0.0
            msg.angular.z = -self.angular_speed
        else:  # stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

        self.ticks_remaining -= 1
        if self.ticks_remaining <= 0:
            self.current_step_index += 1
            self._start_next_step()

    def is_finished(self):
        """この plan の実行が終わったかどうかを返す。"""
        return self.finished


# ---- 1 コマンド分を実行するヘルパ関数 ----

def run_one_command(executor: SingleThreadedExecutor, user_cmd: str):
    """
    1つの日本語指示を受け取り：
      1. LLM から plan を生成
      2. LLMController ノードを作成
      3. plan が終わるまで実行
    """
    print("LLM に計画生成を依頼中...")

    try:
        plan = get_plan(user_cmd)
    except Exception as e:
        print(f"LLM からの計画生成に失敗しました: {e}")
        return

    node = LLMController(plan)
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.is_finished():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        node.destroy_node()


# ---- main：多輪指令版 ----

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()

    try:
        while rclpy.ok():
            user_cmd = input("\nロボットへの日本語指示を入力してください（q で終了）：\n> ").strip()
            if user_cmd.lower() in ("q", "quit", "exit"):
                print("LLM コントローラを終了します。")
                break
            if not user_cmd:
                continue

            run_one_command(executor, user_cmd)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

