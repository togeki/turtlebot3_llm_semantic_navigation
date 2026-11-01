import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class SimpleScriptController(Node):
    def __init__(self):
        super().__init__('simple_script_controller')
        self.stamped = self.declare_parameter('stamped_cmd_vel', True).value
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist, 'cmd_vel', 10)

        # 状态：0=前进，1=左转，2=再前进，3=停止
        self.state = 0
        self.step_count = 0

        # 50ms 调用一次 timer_callback
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('SimpleScriptController started.')

    def publish_velocity(self, twist):
        if self.stamped:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.twist = twist
        else:
            message = twist
        self.cmd_pub.publish(message)

    def timer_callback(self):
        msg = Twist()

        if self.state == 0:
            # 前进
            msg.linear.x = 0.1   # 向前，速度可以改
            msg.angular.z = 0.0
            self.step_count += 1
            if self.step_count > 40:  # 40 * 0.05s ≒ 2秒
                self.state = 1
                self.step_count = 0
                self.get_logger().info('Step 1 done: forward -> now turn left.')

        elif self.state == 1:
            # 左转
            msg.linear.x = 0.0
            msg.angular.z = 0.5   # 向左转
            self.step_count += 1
            if self.step_count > 30:  # 30 * 0.05s ≒ 1.5秒
                self.state = 2
                self.step_count = 0
                self.get_logger().info('Step 2 done: turn left -> forward again.')

        elif self.state == 2:
            # 再前进
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.step_count += 1
            if self.step_count > 40:
                self.state = 3
                self.step_count = 0
                self.get_logger().info('Step 3 done: forward -> stop.')

        elif self.state == 3:
            # 停止
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # 停止后可以选择不再发消息，也可以继续发 0
            # 这里保持发 0，防止残留速度
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publish_velocity(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimpleScriptController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
