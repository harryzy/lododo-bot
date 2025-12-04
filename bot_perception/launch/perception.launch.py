#!/usr/bin/env python3
"""
Perception Node Launch File / 感知节点启动文件
All perception related nodes run in virtual environment (supporting NumPy 1.x) / 所有感知相关节点在虚拟环境中运行（支持NumPy 1.x）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Parameters / 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    # Get virtual environment startup script path / 获取虚拟环境启动脚本路径
    perception_share = get_package_share_directory('bot_perception')
    venv_wrapper = os.path.join(perception_share, 'scripts', 'run_with_venv.sh')
    
    # Depth to LaserScan (running in virtual environment) / 深度图转激光雷达（在虚拟环境中运行）
    # Publishes both /scan (BEST_EFFORT) and /scan_reliable (RELIABLE)
    # 同时发布 /scan (BEST_EFFORT) 和 /scan_reliable (RELIABLE)
    depth_to_laserscan = Node(
        package='bot_perception',
        executable='depth_to_laserscan',
        name='depth_to_laserscan',
        output='screen',
        prefix=f'bash {venv_wrapper}',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_height': 10,
            # 最小距离0.05m(5cm)支持近距离检测 / Min range 0.05m(5cm) for close-range detection
            'range_min': 0.05,
            'range_max': 8.0,
            'angle_min': -0.5236,  # -30度，匹配相机 FOV / -30 deg, match camera FOV
            'angle_max': 0.5236,   # +30度 / +30 deg
            'output_frame': 'base_link'  # LaserScan 在 base_link 平面 / LaserScan in base_link plane
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        depth_to_laserscan,
    ])
