#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import optuna
import time
import subprocess

class OptimizerNode(Node):
    def __init__(self):
        super().__init__('optimizer_node')
        self.subscription = self.create_subscription(
            Float64MultiArray, 'simulation_result', self.result_callback, 10)
        self.current_result = None
        self.trial_completed = False
        
        # ターゲットノード名
        self.target_node_name = "/yield_node" 

    def result_callback(self, msg):
        self.current_result = msg.data
        self.trial_completed = True

    def set_simulation_params(self, params_dict):
        # 辞書に入っているすべてのパラメータを順番にセットする
        for key, value in params_dict.items():
            subprocess.run(["ros2", "param", "set", self.target_node_name, key, str(value)], 
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def run_optimization(self):
        sampler = optuna.samplers.NSGAIISampler(population_size=30)
        study = optuna.create_study(direction='minimize', sampler=sampler)
        
        self.get_logger().info('=== 多次元パラメータのGA最適化を開始します ===')
        study.optimize(self.objective, n_trials=300) 
        
        self.get_logger().info('=== 最適化完了 ===')
        self.get_logger().info(f'Best Cost: {study.best_value}')
        self.get_logger().info(f'Best params: {study.best_params}')

    def objective(self, trial):
        # すべてのパラメータの探索範囲を定義
        params_dict = {
            'A_classic': trial.suggest_float('A_classic', 0.5, 3.0),
            'B_classic': trial.suggest_float('B_classic', 0.2, 2.0),
            'T_c_limit': trial.suggest_float('T_c_limit', 2.0, 8.0),
            'A_cp': trial.suggest_float('A_cp', 1.0, 5.0),
            'B_cp': trial.suggest_float('B_cp', 0.5, 3.0),
            'weight_rep': trial.suggest_float('weight_rep', 0.1, 2.0),
            'weight_oss': trial.suggest_float('weight_oss', 0.5, 3.0),
            'A_wall': trial.suggest_float('A_wall', 1.0, 8.0),
            'B_wall': trial.suggest_float('B_wall', 0.1, 1.0),
            'd_safe': trial.suggest_float('d_safe', 0.1, 1.0)
        }

        self.get_logger().info(f'Trial {trial.number}: Testing {len(params_dict)} parameters...')

        # C++にまとめて送信
        self.set_simulation_params(params_dict)
        
        # シミュレーションの完了を待つ
        self.trial_completed = False
        while not self.trial_completed:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 評価計算
        elapsed_time = self.current_result[0]
        min_dist_human = self.current_result[1]
        
        human_radius = 0.25
        robot_radius = 0.25
        physical_limit = human_radius + robot_radius 

        # 評価1: 時間
        F1_efficiency = elapsed_time

        # 評価2: 物理的衝突
        F2_collision_penalty = 0.0
        if min_dist_human < physical_limit:
            penetration_depth = physical_limit - min_dist_human
            F2_collision_penalty = penetration_depth * 2000.0 

        # 評価3: 心理的圧迫感
        psychological_penalty = 0.0
        if min_dist_human < physical_limit + 0.3:
            psychological_penalty = (physical_limit + 0.3 - min_dist_human) * 50.0

        total_cost = F1_efficiency + F2_collision_penalty + psychological_penalty

        self.get_logger().info(f'-> Time: {elapsed_time:.1f}s, MinDist: {min_dist_human:.2f}m, Cost: {total_cost:.1f}')
        return total_cost

def main(args=None):
    rclpy.init(args=args)
    node = OptimizerNode()
    time.sleep(2)
    node.run_optimization()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()