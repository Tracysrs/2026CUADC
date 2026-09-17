"""CUADC 任务状态机启动文件。

单独启动状态机（前提：MAVROS 已在跑、飞控/SITL 已连接）：

    ros2 launch cuadc_mission mission.launch.py

SITL 仿真（配合 08_参考资料/外部仓库/hgd_cudac/sim/cuadc_sim 时使用仿真时钟）：

    ros2 launch cuadc_mission mission.launch.py use_sim_time:=true

完整联跑顺序见 03_机载软件/README.md。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('cuadc_mission'),
        'config', 'mission_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='cuadc_mission',
            executable='cuadc_mission_node',
            name='cuadc_mission',  # 必须与 mission_params.yaml 顶层键一致
            output='screen',
            parameters=[
                default_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
