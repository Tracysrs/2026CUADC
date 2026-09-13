"""真机白桶感知节点 launch（参数默认取 bucket_perception_params.yaml）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('cuadc_perception'),
                          'config', 'bucket_perception_params.yaml')
    return LaunchDescription([
        Node(
            package='cuadc_perception',
            executable='bucket_perception_node',
            name='bucket_perception',
            output='screen',
            parameters=[params],
        ),
    ])
