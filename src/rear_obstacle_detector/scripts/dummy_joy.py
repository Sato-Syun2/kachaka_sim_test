#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class DummyJoyTeleop(Node):
    def __init__(self):
        super().__init__('dummy_joy_teleop')
        
        # 1. コントローラーの生データを受信
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 2. ダミー人間へ「直接」速度を送信（リマップ不要！）
        self.publisher = self.create_publisher(Twist, '/model/human_dummy/cmd_vel', 10)
        
        # --- カスタムキーマップ設定（ここで自由に書き換え可能！） ---
        self.axis_x = 1       # 前後（左スティックの上下）
        self.axis_y = 0       # 左右（左スティックの左右：カニ歩き）
        self.axis_yaw = 3     # 旋回（右スティックの左右）
        self.enable_button = 0 # 押しっぱなしにする安全スイッチ（例: L1やAボタン）
        
        # 最高速度の設定 (m/s)
        self.speed_x = 1.5
        self.speed_y = 1.5
        self.speed_yaw = 2.0
        
        self.get_logger().info("🎮 専用コントローラーノード起動！安全スイッチを押しながらスティックを倒してください。")

    def joy_callback(self, msg):
        twist = Twist()
        
        try:
            # 安全スイッチが押されているかチェック
            if msg.buttons[self.enable_button] == 1:
                # スティックの傾き(-1.0 ~ 1.0) × 最高速度
                twist.linear.x = msg.axes[self.axis_x] * self.speed_x
                twist.linear.y = msg.axes[self.axis_y] * self.speed_y
                twist.angular.z = msg.axes[self.axis_yaw] * self.speed_yaw
            else:
                # ボタンを離したらピタッと止まるようにゼロを送信
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                
            # 決定した速度を直接送信！
            self.publisher.publish(twist)
            
        except IndexError:
            # 万が一設定した番号がコントローラーのボタン数を超えていたときのエラー回避
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DummyJoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()