"""真侦察判读节点 launch（参数默认取 hazard_recon_params.yaml）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('cuadc_perception'),
                          'config', 'hazard_recon_params.yaml')
    return LaunchDescription([
        Node(
            package='cuadc_perception',
            executable='hazard_recon_node',
            name='hazard_recon',
            output='screen',
            parameters=[params],
        ),
    ])
