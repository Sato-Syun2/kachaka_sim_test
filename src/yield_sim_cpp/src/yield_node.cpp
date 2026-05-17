#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

#define front 0
#define back 1

// ==========================================
// 1. 全パラメーターの一元管理
// ==========================================
struct SimConfig {
  bool fast_mode = true;          // true=爆速(Optuna用), false=視覚化(RViz用)
  
  double robot_max_speed = 0.5;
  double human_max_speed = 1.2;
  double tau = 0.5;               
  
  double human_size = 0.25;
  double robot_size = 0.25;

  double A_classic = 1.6;
  double B_classic = 1.0;

  double T_c_limit = 5.0;
  double A_cp = 3.0;
  double B_cp = 1.5;
  double d_safe = 0.5;

  double weight_rep = 0.5;
  double weight_oss = 1.5;

  double A_wall = 5.0;
  double B_wall = 0.5;
  double wall_left_x = -1.25;
  double wall_right_x = 1.25;

  int situation_id = 1;
};

struct Agent {
  double x, y;
  double vx, vy;
  double goal_x, goal_y;
  double max_speed;
};

class YieldNode : public rclcpp::Node
{
public:
  YieldNode()
  : Node("yield_node") // ノード名を yield_node に統一
  {
    // パラメータの宣言
    this->declare_parameter("fast_mode", true);
    this->declare_parameter("robot_max_speed", 0.5);
    this->declare_parameter("human_max_speed", 1.2);
    this->declare_parameter("tau", 0.5);
    this->declare_parameter("A_classic", 1.6);
    this->declare_parameter("B_classic", 1.0);
    this->declare_parameter("T_c_limit", 5.0);
    this->declare_parameter("A_cp", 3.0);
    this->declare_parameter("B_cp", 1.5);
    this->declare_parameter("human_size", 0.25);
    this->declare_parameter("robot_size", 0.25);
    this->declare_parameter("d_safe", 0.5);
    this->declare_parameter("weight_rep", 0.5);
    this->declare_parameter("weight_oss", 1.5);
    this->declare_parameter("A_wall", 5.0);
    this->declare_parameter("B_wall", 0.5);
    this->declare_parameter("wall_left_x", -1.25);
    this->declare_parameter("wall_right_x", 1.25);
    this->declare_parameter("situation_id", 1);

    update_parameters();

    // パラメータ変更コールバックの登録
    param_subscriber_ = this->add_on_set_parameters_callback(
      std::bind(&YieldNode::parameters_callback, this, std::placeholders::_1));

    robot_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);
    human_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("human_marker", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("simulation_result", 10);

    reset_simulation();

    timer_ = this->create_wall_timer(
      100ms, std::bind(&YieldNode::timer_callback, this));
  }

private:
  SimConfig config_;
  Agent robot_;
  Agent human_;
  
  double sim_time_;
  double min_dist_to_human_;
  double min_dist_to_left_wall_;
  double min_dist_to_right_wall_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr human_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void update_parameters() {
    config_.fast_mode = this->get_parameter("fast_mode").as_bool();
    config_.robot_max_speed = this->get_parameter("robot_max_speed").as_double();
    config_.human_max_speed = this->get_parameter("human_max_speed").as_double();
    config_.tau = this->get_parameter("tau").as_double();
    config_.A_classic = this->get_parameter("A_classic").as_double();
    config_.B_classic = this->get_parameter("B_classic").as_double();
    config_.T_c_limit = this->get_parameter("T_c_limit").as_double();
    config_.A_cp = this->get_parameter("A_cp").as_double();
    config_.B_cp = this->get_parameter("B_cp").as_double();
    config_.human_size = this->get_parameter("human_size").as_double();
    config_.robot_size = this->get_parameter("robot_size").as_double();
    config_.d_safe = this->get_parameter("d_safe").as_double();
    config_.weight_rep = this->get_parameter("weight_rep").as_double();
    config_.weight_oss = this->get_parameter("weight_oss").as_double();
    config_.A_wall = this->get_parameter("A_wall").as_double();
    config_.B_wall = this->get_parameter("B_wall").as_double();
    config_.wall_left_x = this->get_parameter("wall_left_x").as_double();
    config_.wall_right_x = this->get_parameter("wall_right_x").as_double();
    
    int new_situation = this->get_parameter("situation_id").as_int();
    if (new_situation != config_.situation_id) {
      config_.situation_id = new_situation;
      reset_simulation();
    }
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto & param : parameters) {
      if (param.get_name() == "fast_mode") config_.fast_mode = param.as_bool();
      else if (param.get_name() == "A_classic") config_.A_classic = param.as_double();
      else if (param.get_name() == "B_classic") config_.B_classic = param.as_double();
      else if (param.get_name() == "T_c_limit") config_.T_c_limit = param.as_double();
      else if (param.get_name() == "A_cp") config_.A_cp = param.as_double();
      else if (param.get_name() == "B_cp") config_.B_cp = param.as_double();
      else if (param.get_name() == "weight_rep") config_.weight_rep = param.as_double();
      else if (param.get_name() == "weight_oss") config_.weight_oss = param.as_double();
      else if (param.get_name() == "A_wall") config_.A_wall = param.as_double();
      else if (param.get_name() == "B_wall") config_.B_wall = param.as_double();
      else if (param.get_name() == "d_safe") config_.d_safe = param.as_double();
    }
    return result;
  }

  void reset_simulation() {
    switch (config_.situation_id) {
      case front:
        robot_ = {0.0, 0.0, 0.0, 0.0, 0.0, 20.0, config_.robot_max_speed};
        human_ = {0.0, 10.0, 0.0, 0.0, 0.0, -20.0, config_.human_max_speed};
        break;
      case back:
        robot_ = {0.0, 0.0, 0.0, 0.0, 0.0, 20.0, config_.robot_max_speed};
        human_ = {0.0, -5.0, 0.0, 0.0, 0.0, 20.0, config_.human_max_speed};
        break;
      default:
        robot_ = {0.0, 0.0, 0.0, 0.0, 0.0, 20.0, config_.robot_max_speed};
        human_ = {0.0, -5.0, 0.0, 0.0, 0.0, 20.0, config_.human_max_speed};
    }
    sim_time_ = 0.0;
    min_dist_to_human_ = 100.0;
    min_dist_to_left_wall_ = 100.0;
    min_dist_to_right_wall_ = 100.0;
  }

  void update_agent(Agent &self, const Agent &other, double dt)
  {
    double dist_to_goal = std::hypot(self.goal_x - self.x, self.goal_y - self.y);
    double dir_goal_x = (self.goal_x - self.x) / dist_to_goal;
    double dir_goal_y = (self.goal_y - self.y) / dist_to_goal;
    
    double f_att_x = (self.max_speed * dir_goal_x - self.vx) / config_.tau;
    double f_att_y = (self.max_speed * dir_goal_y - self.vy) / config_.tau;

    double d_rp_x = other.x - self.x;
    double d_rp_y = other.y - self.y;
    double dist_rp = std::hypot(d_rp_x, d_rp_y);

    double n_rp_x = -d_rp_x / dist_rp;
    double n_rp_y = -d_rp_y / dist_rp;

    double f_classic_mag = config_.A_classic * std::exp((config_.robot_size + config_.human_size - dist_rp) / config_.B_classic);
    double f_ij_x = f_classic_mag * n_rp_x;
    double f_ij_y = f_classic_mag * n_rp_y;

    double v_rp_x = other.vx - self.vx;
    double v_rp_y = other.vy - self.vy;
    double v_rp_sq = v_rp_x * v_rp_x + v_rp_y * v_rp_y;

    double T_c = 100.0;
    double dot_product = (d_rp_x * v_rp_x) + (d_rp_y * v_rp_y);
    if (v_rp_sq > 0.001) T_c = -dot_product / v_rp_sq;

    double f_cp_x = 0.0;
    double f_cp_y = 0.0;

    if (T_c > 0.0 && T_c < config_.T_c_limit) {
      double pred_self_x = self.x + self.vx * T_c;
      double pred_self_y = self.y + self.vy * T_c;
      double pred_other_x = other.x + other.vx * T_c;
      double pred_other_y = other.y + other.vy * T_c;
      double d_min = std::hypot(pred_other_x - pred_self_x, pred_other_y - pred_self_y);

      if (d_min < config_.human_size + config_.robot_size + config_.d_safe) {
        double f_cp_mag = config_.A_cp * std::exp(-T_c / config_.B_cp);
        f_cp_x = f_cp_mag * n_rp_y; 
        f_cp_y = f_cp_mag * (-n_rp_x);
      }
    }

    double dist_wall_left = self.x - config_.wall_left_x;
    double dist_wall_right = config_.wall_right_x - self.x;
    double f_wall_x = config_.A_wall * std::exp(-dist_wall_left / config_.B_wall)
                    - config_.A_wall * std::exp(-dist_wall_right / config_.B_wall);

    double f_total_x = f_att_x + (f_ij_x * config_.weight_rep) + (f_cp_x * config_.weight_oss) + f_wall_x;
    double f_total_y = f_att_y + (f_ij_y * config_.weight_rep) + (f_cp_y * config_.weight_oss);

    self.vx += f_total_x * dt;
    self.vy += f_total_y * dt;

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
    if (human_.y < -10.0 || human_.y > 10.0) return;

    double dt = 0.1;

    if (config_.fast_mode) {
      // 爆速モード（Optuna用）
      while (!(human_.y < -10.0 || human_.y > 10.0)) {
        Agent prev_robot = robot_;
        update_agent(robot_, human_, dt);
        update_agent(human_, prev_robot, dt);

        sim_time_ += dt;
        double dist = std::hypot(robot_.x - human_.x, robot_.y - human_.y);
        if (dist < min_dist_to_human_) min_dist_to_human_ = dist;
        double left_dist = robot_.x - config_.wall_left_x;
        if (left_dist < min_dist_to_left_wall_) min_dist_to_left_wall_ = left_dist;
        double right_dist = config_.wall_right_x - robot_.x;
        if (right_dist < min_dist_to_right_wall_) min_dist_to_right_wall_ = right_dist;
      }
      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data = {sim_time_, min_dist_to_human_, min_dist_to_left_wall_, min_dist_to_right_wall_};
      result_pub_->publish(msg);
      reset_simulation();
    } else {
      // 視覚化モード（RViz用）
      Agent prev_robot = robot_;
      update_agent(robot_, human_, dt);
      update_agent(human_, prev_robot, dt);

      sim_time_ += dt;
      double dist = std::hypot(robot_.x - human_.x, robot_.y - human_.y);
      if (dist < min_dist_to_human_) min_dist_to_human_ = dist;
      double left_dist = robot_.x - config_.wall_left_x;
      if (left_dist < min_dist_to_left_wall_) min_dist_to_left_wall_ = left_dist;
      double right_dist = config_.wall_right_x - robot_.x;
      if (right_dist < min_dist_to_right_wall_) min_dist_to_right_wall_ = right_dist;

      if (human_.y < -10.0 || human_.y > 10.0) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {sim_time_, min_dist_to_human_, min_dist_to_left_wall_, min_dist_to_right_wall_};
        result_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "=== 軌道終了：シミュレーションをリセットします ===");
        reset_simulation();
      }

      publish_marker(robot_pub_, robot_.x, robot_.y, 1, 0.0, 0.0, 0.8);
      publish_marker(human_pub_, human_.x, human_.y, 2, 0.0, 0.8, 0.0);
      publish_wall(robot_pub_, config_.wall_right_x, 10);
      publish_wall(robot_pub_, config_.wall_left_x, 11);
    }
  }

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
    marker.scale.x = config_.robot_size * 2.0; 
    marker.scale.y = config_.robot_size * 2.0; 
    marker.scale.z = 1.0;
    marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0;
    publisher->publish(marker);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YieldNode>());
  rclcpp::shutdown();
  return 0;
}