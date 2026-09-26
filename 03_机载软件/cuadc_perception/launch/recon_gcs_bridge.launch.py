"""判读→数传填单桥 launch（参数默认取 recon_gcs_bridge_params.yaml）。

前置：mavros 已起（Jetson↔FC TELEM3 串口 /dev/ttyTHS1:921600，09-26 起）、
判读服务在跑（systemd cuadc-perception，本桥 09-26 已并入其启动脚本自启）。
    ros2 launch cuadc_perception recon_gcs_bridge.launch.py
地面侧核对：Mission Planner 消息栏出现「侦察V…」行 = 全链通（04 册 §2.4）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('cuadc_perception'),
                          'config', 'recon_gcs_bridge_params.yaml')
    return LaunchDescription([
        Node(
            package='cuadc_perception',
            executable='recon_gcs_bridge_node',
            name='recon_gcs_bridge',
            output='screen',
            parameters=[params],
        ),
    ])
