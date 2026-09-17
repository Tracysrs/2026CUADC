"""假感知节点启动文件（契约演示 / 联调 / P0.4 延迟注入测试）。

    ros2 launch cuadc_perception fake_perception.launch.py
    # 配合契约校验（另开终端）：
    ros2 run cuadc_perception check_vision_contract --ros-args -p duration_s:=10.0
    # P0.4 时间同步验收：注入 200ms 人工延迟
    ros2 launch cuadc_perception fake_perception.launch.py pipeline_delay_s:=0.2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('cuadc_perception'),
        'config', 'fake_perception_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('pipeline_delay_s', default_value='0.0'),
        Node(
            package='cuadc_perception',
            executable='fake_perception_node',
            name='fake_perception',  # 必须与 fake_perception_params.yaml 顶层键一致
            output='screen',
            parameters=[
                default_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'pipeline_delay_s': LaunchConfiguration('pipeline_delay_s'),
                },
            ],
        ),
    ])
