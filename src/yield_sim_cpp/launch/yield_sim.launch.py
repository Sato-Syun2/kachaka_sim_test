import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージのパスとrvizファイルのパスを取得
    package_dir = get_package_share_directory('yield_sim_cpp')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'yield_sim.rviz')

    return LaunchDescription([
        # シミュレーションノードの起動
        Node(
            package='yield_sim_cpp',
            executable='yield_node',
            name='yield_node',
            output='screen'
        ),
        # 保存した設定ファイルを読み込んでRViz2を起動
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])