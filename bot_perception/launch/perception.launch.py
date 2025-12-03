#!/usr/bin/env python3
"""
感知节点启动文件
所有感知相关节点在虚拟环境中运行（支持NumPy 1.x）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    # 获取虚拟环境启动脚本路径
    perception_share = get_package_share_directory('bot_perception')
    venv_wrapper = os.path.join(perception_share, 'scripts', 'run_with_venv.sh')
    
    # 深度图转激光雷达（在虚拟环境中运行）
    depth_to_laserscan = Node(
        package='bot_perception',
        executable='depth_to_laserscan',
        name='depth_to_laserscan',
        output='screen',
        prefix=f'bash {venv_wrapper}',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_height': 10,
            'range_min': 0.1,  # 降低最小距离
            'range_max': 8.0,
            'angle_min': -0.5236,  # -30度，匹配相机 FOV
            'angle_max': 0.5236,   # +30度
            'output_frame': 'base_link'  # LaserScan 在 base_link 平面
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        depth_to_laserscan,
    ])
