"""H 圆感知与精准降落节点 launch（参数默认取 h_circle_params.yaml）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('cuadc_perception'),
                          'config', 'h_circle_params.yaml')
    return LaunchDescription([
        Node(
            package='cuadc_perception',
            executable='h_circle_node',
            name='h_circle_perception',
            output='screen',
            parameters=[params],
        ),
    ])
