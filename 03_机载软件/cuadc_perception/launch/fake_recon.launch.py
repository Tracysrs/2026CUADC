"""假判读 + 地面站查看器 一键联调（不需要真模型/飞控）。

    ros2 launch cuadc_perception fake_recon.launch.py
    # 场景切换（normal / ambiguous / blank）：
    ros2 launch cuadc_perception fake_recon.launch.py scenario:=ambiguous
    # 与状态机 SITL 联跑（握手由状态机驱动）：
    ros2 launch cuadc_perception fake_recon.launch.py use_sim_time:=true

单独手测握手语义（不起状态机，手工发一次请求）：
    ros2 topic pub --once /cuadc/recon/capture_request geometry_msgs/msg/PointStamped \
        "{point: {x: 0}}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scenario', default_value='normal'),
        DeclareLaunchArgument('verdict_delay_s', default_value='2.0'),
        Node(
            package='cuadc_perception',
            executable='fake_recon_node',
            name='fake_recon',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scenario': LaunchConfiguration('scenario'),
                'verdict_delay_s': LaunchConfiguration('verdict_delay_s'),
            }],
        ),
        Node(
            package='cuadc_perception',
            executable='recon_viewer_node',
            name='recon_viewer',
            output='screen',
        ),
    ])
