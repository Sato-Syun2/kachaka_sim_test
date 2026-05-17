#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

// ==========================================
// 1. 全パラメーターの一元管理 (ここをいじるだけで挙動が変わります)
// ==========================================
#define front 0
#define back 1

struct SimConfig {
  // エージェントの基本性能
  double robot_max_speed = 0.5;   // ロボットの最高速度 (m/s)
  double human_max_speed = 1.2;   // 人間の最高速度 (m/s)
  double tau = 0.2;               // 目標速度への緩和時間（小さいほど急加速）

  double A_classic = 1.6; // 論文5.2節の基本相互作用力振幅
  double B_classic = 1.0; // 論文5.2節の有効範囲係数

  // TTC（衝突予測）に基づく斥力のパラメーター
  double T_c_limit = 5.0;         // 何秒前の衝突予測から避け始めるか
  double A_cp = 3.0;              // 人やロボットから受ける斥力の最大振幅
  double B_cp = 1.5;              // 時間的余裕に対する減衰係数（大きいほど早くから避ける）
  double human_size = 0.5;
  double robot_size = 0.5;

  // 安全距離のしきい値（論文では 0.5m などを基準にしている）
  double d_safe = 0.5;

  // 力のブレンド重み
  double weight_rep = 0.2;        // 真正面に押し返される力（ブレーキ）の重み
  double weight_oss = 1.8;        // 横にスライドして避ける力（ステアリング）の重み

  // 壁のパラメーター
  double A_wall = 5.0;            // 壁からの斥力の強さ
  double B_wall = 0.5;            // 壁からの斥力が及ぶ距離
  double wall_left_x = -1.25;      // 左壁のX座標
  double wall_right_x = 1.25;      // 右壁のX座標

  int situation_id = back;             // シチュエーションID（将来の拡張用）
};

// エージェント（ロボット・人）の状態を保持する構造体
struct Agent {
  double x, y;
  double vx, vy;
  double goal_x, goal_y;
  double max_speed;
};

class MutualSFMNode : public rclcpp::Node
{
public:
  MutualSFMNode()
  : Node("mutual_sfm_node")
  {
    robot_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);
    human_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("human_marker", 10);

    reset_simulation();

    timer_ = this->create_wall_timer(
      100ms, std::bind(&MutualSFMNode::timer_callback, this));
  }

private:
  SimConfig config_; // パラメーター設定
  Agent robot_;
  Agent human_;

  void reset_simulation() {
    switch (config_.situation_id) {
      case front:
    // ロボットの初期配置 (下から上へ)
    robot_ = {0.0, 0.0, 0.0, 0.0, 0.0, 200.0, config_.robot_max_speed};
    // 人の初期配置 (上から下へ)
    human_ = {0.0, 10.0, 0.0, 0.0, 0.0, -200.0, config_.human_max_speed};
        break;
      case back:
    // ロボットの初期配置 (下から上へ)
    robot_ = {0.0, 0.0, 0.0, 0.0, 0.0, 200.0, config_.robot_max_speed};
    // 人の初期配置 (上から下へ)
    human_ = {0.0, -5.0, 0.0, 0.0, 0.0, 200.0, config_.human_max_speed};
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "未知のシチュエーションIDです。デフォルト値を使用します。");
    }

  }

  // SFMの力を計算し、速度と位置を更新する共通関数
  void update_agent(Agent &self, const Agent &other, double dt)
  {
    // --- (A) 目的地に向かう引力 ---
    double dist_to_goal = std::hypot(self.goal_x - self.x, self.goal_y - self.y);
    double dir_goal_x = (self.goal_x - self.x) / dist_to_goal;
    double dir_goal_y = (self.goal_y - self.y) / dist_to_goal;
    
    double f_att_x = (self.max_speed * dir_goal_x - self.vx) / config_.tau;
    double f_att_y = (self.max_speed * dir_goal_y - self.vy) / config_.tau;

    // --- (B) 相手からの斥力 f_ij---
    double d_rp_x = other.x - self.x;
    double d_rp_y = other.y - self.y;
    double dist_rp = std::hypot(d_rp_x, d_rp_y);

    double n_rp_x = -d_rp_x / dist_rp; // 相手から遠ざかる単位ベクトル
    double n_rp_y = -d_rp_y / dist_rp;

    double f_classic_mag = config_.A_classic * std::exp((config_.robot_size + config_.human_size - dist_rp) / config_.B_classic);
    
    double f_ij_x = f_classic_mag * n_rp_x;
    double f_ij_y = f_classic_mag * n_rp_y;

    // --- (C) TTCベースの予測力 ＋ OSS (f_cp) ---
    double v_rp_x = other.vx - self.vx;
    double v_rp_y = other.vy - self.vy;
    double v_rp_sq = v_rp_x * v_rp_x + v_rp_y * v_rp_y;

    double T_c = 100.0;
    double dot_product = (d_rp_x * v_rp_x) + (d_rp_y * v_rp_y);
    if (v_rp_sq > 0.001) {
      T_c = -dot_product / v_rp_sq;
    }

    double f_cp_x = 0.0;
    double f_cp_y = 0.0;

    // T_c が正（将来、確実に接近するタイミングがある）場合のみチェック
    if (T_c > 0.0) {
      // 1. T_c秒後の予測位置を計算 (論文 式9, 式10)
      double pred_self_x = self.x + self.vx * T_c;
      double pred_self_y = self.y + self.vy * T_c;
      double pred_other_x = other.x + other.vx * T_c;
      double pred_other_y = other.y + other.vy * T_c;

      // 2. 予測される最小接近距離 d_min を計算 (論文 式8)
      double d_min = std::hypot(pred_other_x - pred_self_x, pred_other_y - pred_self_y);

      // 3. d_min が安全距離未満の場合のみ、回避アクション(f_cp)を発動！ (Algorithm 1 の 6行目)
      if (d_min < config_.human_size + config_.robot_size + config_.d_safe) {
        // 力の強さは T_c を使って計算（時間が短いほど強く避ける）
        double f_cp_mag = config_.A_cp * std::exp(-T_c / config_.B_cp);
        
        // OSS（直交ステアリング戦略）の適用
        f_cp_x = f_cp_mag * n_rp_y; 
        f_cp_y = f_cp_mag * (-n_rp_x);
      }
    }

    // --- (D) 壁からの斥力 ---
    double dist_wall_left = self.x - config_.wall_left_x;
    double dist_wall_right = config_.wall_right_x - self.x;
    double f_wall_x = config_.A_wall * std::exp(-dist_wall_left / config_.B_wall)
                    - config_.A_wall * std::exp(-dist_wall_right / config_.B_wall);

    // --- (E) 力の合算と更新 ---
    double f_total_x = f_att_x + (f_ij_x * config_.weight_rep) + (f_cp_x * config_.weight_oss) + f_wall_x;
    double f_total_y = f_att_y + (f_ij_y * config_.weight_rep) + (f_cp_y * config_.weight_oss);

    self.vx += f_total_x * dt;
    self.vy += f_total_y * dt;

    // 速度制限 (エージェントの最高速度を超えないようにクリップ)
    double speed = std::hypot(self.vx, self.vy);
    if (speed > self.max_speed) {
      self.vx = (self.vx / speed) * self.max_speed;
      self.vy = (self.vy / speed) * self.max_speed;
    }

    self.x += self.vx * dt;
    self.y += self.vy * dt;
  }

  void timer_callback()
  {
    double dt = 0.1;

    // お互いを障害物として認識し、それぞれの状態を更新
    // (計算中に互いの状態が変わらないよう、一時的なコピーを使うのがより厳密ですが、簡略化のため直接更新します)
    Agent prev_robot = robot_;
    update_agent(robot_, human_, dt);
    update_agent(human_, prev_robot, dt);

    // 人がロボットの遥か後ろに行ったらリセット
    if (human_.y < -10.0 || human_.y > 10.0 ) {
      reset_simulation();
      RCLCPP_INFO(this->get_logger(), "=== シミュレーションをリセットしました ===");
    }

    // 描画処理
    publish_marker(robot_pub_, robot_.x, robot_.y, 1, 0.0, 0.0, 0.8); // 青(ロボット)
    publish_marker(human_pub_, human_.x, human_.y, 2, 0.0, 0.8, 0.0); // 緑(人)
    publish_wall(robot_pub_, config_.wall_right_x, 10);
    publish_wall(robot_pub_, config_.wall_left_x, 11);
  }

  // --- 描画用関数群 (前回と同じ) ---
  void publish_wall(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher, double x_pos, int id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map"; marker.header.stamp = this->now();
    marker.ns = "environment"; marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE; marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x_pos; marker.pose.position.y = 0.0; marker.pose.position.z = 0.5;
    marker.scale.x = 0.1; marker.scale.y = 40.0; marker.scale.z = 1.0;
    marker.color.r = 0.7; marker.color.g = 0.7; marker.color.b = 0.7; marker.color.a = 0.5;
    publisher->publish(marker);
  }

  void publish_marker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
    double x, double y, int marker_id, double r, double g, double b)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map"; marker.header.stamp = this->now();
    marker.ns = "simulation"; marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER; marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x; marker.pose.position.y = y; marker.pose.position.z = 0.5;
    marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 1.0;
    marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0;
    publisher->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr human_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MutualSFMNode>());
  rclcpp::shutdown();
  return 0;
}