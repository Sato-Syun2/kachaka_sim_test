import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. それぞれのファイルが所属するパッケージのパスを個別に取得
    # あなたが作ったパッケージ
    my_pkg_dir = get_package_share_directory('rear_obstacle_detector')
    
    # カチャカ公式や他のパッケージ（パッケージ名は実際の環境に合わせてください）
    # 例：Nav2の起動ファイルが入っているパッケージ
    nav2_pkg_dir = get_package_share_directory('kachaka_gazebo') 
    
    # 例：SDFファイルが入っているパッケージ
    world_pkg_dir = get_package_share_directory('kachaka_gazebo')

    # 2. Gazeboシミュレータの起動
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('kachaka_gazebo'), 'launch', 'simulation.launch.py')
        ),
        # world_pkg_dir 内の worlds/corridor.sdf を指定
        launch_arguments={'world': os.path.join(world_pkg_dir, 'worlds', 'corridor.sdf')}.items()
    )

    # 3. Nav2の起動
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'kachaka_nav2.launch.py')
        )
    )

    # 4. 自作ノード群（これは自分のパッケージ my_pkg_dir から呼ぶ）
    detector_node = Node(
        package='rear_obstacle_detector',
        executable='detector_node',
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/human_dummy/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/human_dummy/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose'
        ],
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    custom_teleop = Node(
        package='rear_obstacle_detector',
        executable='dummy_joy.py',
        output='screen'
    )

    rviz_config_path = os.path.join(my_pkg_dir, 'rviz', 'kachaka_experiment.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path], 
        output='screen'
    )

    # すべてを束ねて起動！
    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        detector_node,
        gz_bridge,
        joy_node,
        custom_teleop,
        rviz_node
    ])