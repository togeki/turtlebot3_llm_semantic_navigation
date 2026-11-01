# tb3_llm_controller/llm_controller.py

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
import os,yaml

from tb3_llm_controller.gemini_planner import get_plan


class LLMController(Node):
    """
    LLM から受け取った plan（FORWARD / TURN_LEFT / TURN_RIGHT / STOP）を
    順番に実行するノード。
    1 つの plan を実行し終わると finished フラグが立つ。
    """

    def __init__(self, plan):
        super().__init__('llm_controller')

        self.semantic_places = self._load_semantic_places()

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.plan = plan
        self.current_step_index = 0
        self.ticks_remaining = 0

        self.place_pub = self.create_publisher(String, 'llm_place', 10)
        self.nav_pending = False
        self.nav_result_sub = self.create_subscription(
            String, 'tb3_nav_result', self.nav_result_callback, 10
        )

        # 制御パラメータ
        self.dt = 0.05            # timer period (s)
        self.linear_speed = 0.1   # m/s
        self.angular_speed = 0.5  # rad/s

        self.current_mode = "stop"  # "forward" / "turn_left" / "turn_right" / "stop"
        self.finished = False       # この plan が終わったかどうか

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(f"LLMController started with {len(self.plan)} steps.")
        self.plan_started = False
        self.startup_timer = self.create_timer(0.2, self.start_when_ready)

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

        elif action == "GO_TO_PLACE":
            target = str(step.get("target", "")).strip().lower()

            if target not in self.semantic_places:
                self.get_logger().error(f"Unknown place: {target}")
                self.finished = True
                self.current_mode = "stop"
                self.ticks_remaining = -1
                return

            self.nav_pending = True
            self.current_mode = "nav"
            self.ticks_remaining = 0

            msg = String()
            msg.data = target
            self.place_pub.publish(msg)

            self.get_logger().info(
                f"[NAV] Going to {target}. Waiting for Nav2 result..."
            )
            return

        else:
            self.get_logger().warn(f"Unknown action: {action}, skipping.")
            self.current_step_index += 1
            self._start_next_step()

    def nav_result_callback(self, msg: String):
        if not self.nav_pending:
            return

        result = msg.data.strip().upper()
        self.nav_pending = False

        if result == "SUCCEEDED":
            self.get_logger().info("[NAV] Arrived. Next step.")
            self.current_step_index += 1
            self._start_next_step()
        else:
            self.get_logger().error(
                f"[NAV] Navigation failed: {result}. Aborting plan."
            )
            self.current_mode = "stop"
            self.ticks_remaining = -1
            self.finished = True

    def publish_velocity(self, twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist = twist
        self.cmd_pub.publish(msg)

    def start_when_ready(self):
        needs_nav = any(step.get("action") == "GO_TO_PLACE" for step in self.plan)
        if needs_nav and self.place_pub.get_subscription_count() == 0:
            return

        self.startup_timer.cancel()
        self.plan_started = True
        self._start_next_step()

    def timer_callback(self):
        if not self.plan_started or self.nav_pending:
            return

        msg = Twist()

        if self.ticks_remaining == -1:
            # 完全終了：停止状態を維持
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publish_velocity(msg)
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

        self.publish_velocity(msg)

        self.ticks_remaining -= 1
        if self.ticks_remaining <= 0:
            self.current_step_index += 1
            self._start_next_step()

    def is_finished(self):
        """この plan の実行が終わったかどうかを返す。"""
        return self.finished


    def _load_semantic_places(self):
        """
        semantic_places.yaml を読み込み，
        {place_name: (x, y, theta)} の辞書として返す。

        YAML は 2 つの形式どちらでも対応:
        1) 古い形式:
           places:
             door: {x: ..., y: ..., theta: ...}
        2) 新しい形式（LV1.5 で使用中）:
           places:
             - name: "door"
               x: ...
               y: ...
               yaw: ...  # または theta
        """
        try:
            pkg_share = get_package_share_directory('tb3_llm_controller')
            yaml_path = os.path.join(pkg_share, 'config', 'semantic_places.yaml')

            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}

            raw_places = data.get('places', {})

            places = {}

            # ① 古い dict 形式: places: {door: {...}, window: {...}}
            if isinstance(raw_places, dict):
                for name, info in raw_places.items():
                    name = str(name).strip().lower()
                    x = float(info.get('x', 0.0))
                    y = float(info.get('y', 0.0))
                    theta = float(info.get('theta', info.get('yaw', 0.0)))
                    places[name] = (x, y, theta)

            # ② 新しい list 形式: places: [ {name: "door", x:..., y:..., yaw:...}, ... ]
            elif isinstance(raw_places, list):
                for item in raw_places:
                    if not isinstance(item, dict):
                        continue
                    name = str(item.get('name', '')).strip().lower()
                    if not name:
                        continue
                    x = float(item.get('x', 0.0))
                    y = float(item.get('y', 0.0))
                    theta = float(item.get('theta', item.get('yaw', 0.0)))
                    places[name] = (x, y, theta)

            self.get_logger().info(f"Loaded semantic places: {list(places.keys())}")
            return places

        except Exception as e:
            self.get_logger().error(f"Failed to load semantic places: {e}")
            return {}




# ---- 1 コマンド分を実行するヘルパ関数 ----

def run_one_command(executor: SingleThreadedExecutor, user_cmd: str):
    """
    1つの日本語指示を受け取り：
      1. LLM から plan を生成
      2. LLMController ノードを作成
      3. plan が終わるまで実行
    """

    if user_cmd == "テスト窓":
        print("[TEST] Semantic GO_TO_PLACE for 'window'")
        plan = [{"action": "GO_TO_PLACE", "target": "window"}]
        
    else:
        plan = get_plan(user_cmd)
        print("LLM に計画生成を依頼中...")

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

