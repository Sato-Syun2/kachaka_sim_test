#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import optuna
import time

class OptimizerNode(Node):
    def __init__(self):
        super().__init__('optimizer_node')
        
        self.subscription = self.create_subscription(
            Float64MultiArray, 'simulation_result', self.result_callback, 10)
        self.current_result = None
        self.trial_completed = False
        
        self.target_node_name = "/yield_node" 

        self.param_client = self.create_client(SetParameters, f'{self.target_node_name}/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.target_node_name} のパラメータサービスを待機中...')

    def result_callback(self, msg):
        self.current_result = msg.data
        self.trial_completed = True

    def set_simulation_params(self, params_dict):
        request = SetParameters.Request()
        for key, value in params_dict.items():
            param = Parameter(name=key, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
            request.parameters.append(param)
        
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def run_optimization(self):
        sampler = optuna.samplers.NSGAIISampler(population_size=30)
        study = optuna.create_study(direction='minimize', sampler=sampler)
        
        self.get_logger().info('=== 9次元パラメータの爆速GA最適化を開始します ===')
        study.optimize(self.objective, n_trials=300) 
        
        self.get_logger().info('=== 最適化完了 ===')
        self.get_logger().info(f'Best Cost: {study.best_value}')
        self.get_logger().info(f'Best params: {study.best_params}')

    def objective(self, trial):
        params_dict = {
            'A_classic': trial.suggest_float('A_classic', 0.5, 3.0),
            'B_classic': trial.suggest_float('B_classic', 0.2, 2.0),
            'T_c_limit': trial.suggest_float('T_c_limit', 2.0, 8.0),
            'A_cp': trial.suggest_float('A_cp', 1.0, 5.0),
            'B_cp': trial.suggest_float('B_cp', 0.5, 3.0),
            'weight_rep': trial.suggest_float('weight_rep', 0.1, 2.0),
            'weight_oss': trial.suggest_float('weight_oss', 0.5, 3.0),
            'A_wall': trial.suggest_float('A_wall', 1.0, 8.0),
            'B_wall': trial.suggest_float('B_wall', 0.1, 1.0)
        }

        self.get_logger().info(f'Trial {trial.number}: Testing...')

        self.set_simulation_params(params_dict)
        
        self.trial_completed = False
        while not self.trial_completed:
            rclpy.spin_once(self, timeout_sec=0.01)

        # 評価計算
        elapsed_time = self.current_result[0]
        min_dist_human = self.current_result[1]
        
        human_radius = 0.25
        robot_radius = 0.25
        physical_limit = human_radius + robot_radius 

        # ====================================================
        # 【修正】評価関数の重みバランスの最適化
        # ====================================================
        
        # 評価1: 人間の遅延（イライラ）ペナルティ
        # 人間(-5.0から10.0へ15m移動)が最高速1.2m/sで歩けた場合の理想時間は12.5秒。
        # これより遅れた秒数に対して、超特大のペナルティを与える！
        ideal_time = 15.0 / 1.2
        delay = max(0.0, elapsed_time - ideal_time)
        F1_efficiency = delay * 5000.0  # 重みを 1.0 -> 500.0 に激増！
        
        # 評価2: 物理的衝突へのペナルティ (変わらず重く)
        F2_collision_penalty = 0.0
        if min_dist_human < physical_limit:
            penetration_depth = physical_limit - min_dist_human
            F2_collision_penalty = penetration_depth * 2000.0 

        # 評価3: 心理的圧迫感へのペナルティ
        psychological_penalty = 0.0
        if min_dist_human < physical_limit + 0.3:
            psychological_penalty = (physical_limit + 0.3 - min_dist_human) * 50.0

        # トータルコスト
        total_cost = F1_efficiency + F2_collision_penalty + psychological_penalty

        self.get_logger().info(f'-> Delay: {delay:.1f}s, MinDist: {min_dist_human:.2f}m, Cost: {total_cost:.1f}')
        return total_cost

def main(args=None):
    rclpy.init(args=args)
    node = OptimizerNode()
    time.sleep(1)
    node.run_optimization()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()