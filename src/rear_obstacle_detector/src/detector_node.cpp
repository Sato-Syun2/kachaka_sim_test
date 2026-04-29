#include <memory>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MapDifferenceDetector : public rclcpp::Node {
public:
    MapDifferenceDetector() : Node("map_difference_detector") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&MapDifferenceDetector::map_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/kachaka/lidar/scan", rclcpp::SensorDataQoS(),
            std::bind(&MapDifferenceDetector::scan_callback, this, std::placeholders::_1));

        dynamic_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/dynamic_scan", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/dynamic_tracker", 10);

        // 速度計算用の初期化
        has_last_pos_ = false;
        smoothed_vel_x_ = 0.0;
        smoothed_vel_y_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "ノードを起動しました。/map と TF の受信を待機中...");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = msg;
        RCLCPP_INFO(this->get_logger(), "マップを受信しました。");
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // マップがない場合は警告を出して終了
        if (!map_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "マップがまだ届いていません。Nav2を起動してください。");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF待ち: %s", ex.what());
            return;
        }

        auto dynamic_scan = *msg;
        double sum_x = 0.0, sum_y = 0.0;
        int dynamic_count = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double range = msg->ranges[i];
            if (std::isinf(range) || std::isnan(range) || range < msg->range_min || range > msg->range_max) {
                dynamic_scan.ranges[i] = std::numeric_limits<float>::infinity();
                continue;
            }

            // 座標変換
            double angle = msg->angle_min + i * msg->angle_increment;
            geometry_msgs::msg::PointStamped pt_laser, pt_map;
            pt_laser.point.x = range * std::cos(angle);
            pt_laser.point.y = range * std::sin(angle);
            tf2::doTransform(pt_laser, pt_map, transform);

            // マップ座標からグリッド座標へ
            int mx = std::floor((pt_map.point.x - map_->info.origin.position.x) / map_->info.resolution);
            int my = std::floor((pt_map.point.y - map_->info.origin.position.y) / map_->info.resolution);

            // 背景差分フィルター
            bool is_dynamic = true;
            int radius = 2; // ノイズが多い場合はここを3に
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    int cx = mx + dx, cy = my + dy;
                    if (cx >= 0 && cx < (int)map_->info.width && cy >= 0 && cy < (int)map_->info.height) {
                        if (map_->data[cy * map_->info.width + cx] > 50) { is_dynamic = false; break; }
                    }
                }
                if (!is_dynamic) break;
            }

            if (!is_dynamic) {
                dynamic_scan.ranges[i] = std::numeric_limits<float>::infinity();
            } else {
                sum_x += pt_map.point.x;
                sum_y += pt_map.point.y;
                dynamic_count++;
            }
        }

        // 動的スキャンのパブリッシュ（点がなくても空のスキャンを送ることで更新を維持）
        dynamic_pub_->publish(dynamic_scan);

        // --- 速度計算セクション ---
        if (dynamic_count > 5) {
            double current_x = sum_x / dynamic_count;
            double current_y = sum_y / dynamic_count;
            rclcpp::Time current_time = msg->header.stamp;

            if (has_last_pos_) {
                double dt = (current_time - last_time_).seconds();
                if (dt > 0.01 && dt < 1.0) { // 極端な時間は除外
                    double raw_vx = (current_x - last_x_) / dt;
                    double raw_vy = (current_y - last_y_) / dt;

                    // 指数移動平均フィルタ（0.15が新しい値の採用率）
                    smoothed_vel_x_ = 0.85 * smoothed_vel_x_ + 0.15 * raw_vx;
                    smoothed_vel_y_ = 0.85 * smoothed_vel_y_ + 0.15 * raw_vy;

                    double speed = std::sqrt(smoothed_vel_x_ * smoothed_vel_x_ + smoothed_vel_y_ * smoothed_vel_y_);
                    double angle = std::atan2(smoothed_vel_y_, smoothed_vel_x_);

                    publish_markers(current_x, current_y, angle, speed, current_time);
                }
            }
            last_x_ = current_x; last_y_ = current_y;
            last_time_ = current_time; has_last_pos_ = true;
        } else {
            // 物体を見失った場合はリセット
            has_last_pos_ = false;
            smoothed_vel_x_ = 0.0; smoothed_vel_y_ = 0.0;
        }
    }

    void publish_markers(double x, double y, double angle, double speed, rclcpp::Time stamp) {
        // 重心球体
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map"; m.header.stamp = stamp;
        m.ns = "centroid"; m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.5;
        m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3;
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0;
        marker_pub_->publish(m);

        // 速度矢印
        if (speed > 0.1) {
            visualization_msgs::msg::Marker a;
            a.header.frame_id = "map"; a.header.stamp = stamp;
            a.ns = "velocity"; a.id = 1;
            a.type = visualization_msgs::msg::Marker::ARROW;
            a.pose.position.x = x; a.pose.position.y = y; a.pose.position.z = 0.5;
            tf2::Quaternion q; q.setRPY(0, 0, angle);
            a.pose.orientation = tf2::toMsg(q);
            a.scale.x = speed; a.scale.y = 0.1; a.scale.z = 0.1;
            a.color.a = 1.0; a.color.r = 1.0; a.color.g = 0.0; a.color.b = 0.0;
            marker_pub_->publish(a);
        }
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr dynamic_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double last_x_, last_y_, smoothed_vel_x_, smoothed_vel_y_;
    rclcpp::Time last_time_;
    bool has_last_pos_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapDifferenceDetector>());
    rclcpp::shutdown();
    return 0;
}