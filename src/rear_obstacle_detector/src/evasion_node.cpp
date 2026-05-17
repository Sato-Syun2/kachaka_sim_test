#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class EvasionNode : public rclcpp::Node
{
public:
    EvasionNode() : Node("evasion_node"), last_x_(0.0), last_y_(0.0), first_frame_(true)
    {
        // パラメーターの設定（ここで回避の挙動を調整できます）
        this->declare_parameter("avoidance_distance", 2.0); // 距離での危険判定 (m)
        this->declare_parameter("ttc_threshold", 3.0);      // TTC（衝突余裕時間）の危険判定 (秒)
        this->declare_parameter("evasion_speed_x", 0.2);    // 退避時の前進速度 (m/s)
        this->declare_parameter("evasion_speed_yaw", 0.4);  // 退避時の旋回速度 (rad/s) 左へ

        // Subscriber & Publisher
        sub_tracker_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/dynamic_tracker", 10, std::bind(&EvasionNode::trackerCallback, this, std::placeholders::_1));
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "🚀 回避ノード起動！人の接近を監視中...");
    }

private:
    void trackerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // 最初のマーカー（重心）の位置を取得すると仮定
        if (msg->markers.empty()) return;

        double current_x = msg->markers[0].pose.position.x;
        double current_y = msg->markers[0].pose.position.y;
        auto current_time = this->now();

        if (first_frame_) {
            last_x_ = current_x;
            last_y_ = current_y;
            last_time_ = current_time;
            first_frame_ = false;
            return;
        }

        // 1. 相対速度の計算 (dt)
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        double vx = (current_x - last_x_) / dt;
        double vy = (current_y - last_y_) / dt;

        // 2. 距離とTTCの計算
        // カチャカから見て後ろ (x < 0) かつ、カチャカに向かってきている (vx > 0.1) かを判定
        double distance = std::hypot(current_x, current_y);
        double ttc = 999.0; 

        if (current_x < 0.0 && vx > 0.1) {
            ttc = std::abs(current_x) / vx; // $TTC = \frac{|x|}{v_x}$
        }

        // 3. 回避アクションの決定
        geometry_msgs::msg::Twist cmd;
        double avoidance_distance = this->get_parameter("avoidance_distance").as_double();
        double ttc_threshold = this->get_parameter("ttc_threshold").as_double();

        if (distance < avoidance_distance || ttc < ttc_threshold) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 接近検知！ TTC: %.1f秒, 距離: %.1fm -> 左へ退避！", ttc, distance);
            cmd.linear.x = this->get_parameter("evasion_speed_x").as_double();
            cmd.angular.z = this->get_parameter("evasion_speed_yaw").as_double();
        } else {
            // 安全な場合は停止（今回はNav2を使わないため停止指令を出す）
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        pub_cmd_vel_->publish(cmd);

        // 次のループのための更新
        last_x_ = current_x;
        last_y_ = current_y;
        last_time_ = current_time;
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_tracker_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    double last_x_, last_y_;
    rclcpp::Time last_time_;
    bool first_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EvasionNode>());
    rclcpp::shutdown();
    return 0;
}