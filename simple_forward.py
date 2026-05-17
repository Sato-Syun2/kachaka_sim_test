import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

class SimpleForwarder(Node):
    def __init__(self):
        super().__init__('simple_forwarder')
        # カチャカのマニュアル操作用トピックへパブリッシュ
        self.publisher_ = self.create_publisher(Twist, '/kachaka/manual_control/cmd_vel', 10)
        # 0.1秒ごとに実行
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('低速前進を開始します...')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = -0.2  # 秒速0.2m (時速0.72km) の低速
        msg.angular.z = 0.0 # 回転なし
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimpleForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 終了時に停止信号を送る
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0

        for _ in range(20):
            node.publisher_.publish(stop_msg)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()