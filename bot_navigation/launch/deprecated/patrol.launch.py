#!/usr/bin/env python3
"""
Launch文件: patrol.launch.py

功能 / Features:
  - 启动PatrolNode巡航节点
  - 加载巡航参数配置
  - 支持不同巡航模式
  
使用方法 / Usage:
  ros2 launch bot_navigation patrol.launch.py
  ros2 launch bot_navigation patrol.launch.py patrol_mode:=ping_pong max_loops:=2
  
Author: LeKiwi Bot Development Team
Date: 2025-12-24
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare('bot_navigation')
    
    # Launch参数
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'patrol_waypoints.yaml']),
        description='Path to waypoint configuration file'
    )
    
    patrol_mode_arg = DeclareLaunchArgument(
        'patrol_mode',
        default_value='loop',
        description='Patrol mode: loop, ping_pong, once, random'
    )
    
    max_loops_arg = DeclareLaunchArgument(
        'max_loops',
        default_value='-1',
        description='Maximum number of patrol loops (-1 for infinite)'
    )
    
    default_dwell_time_arg = DeclareLaunchArgument(
        'default_dwell_time',
        default_value='2.0',
        description='Default dwell time at waypoints (seconds)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto start patrol after loading waypoints'
    )
    
    # PatrolNode节点
    patrol_node = Node(
        package='bot_navigation',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
        parameters=[{
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'patrol_mode': LaunchConfiguration('patrol_mode'),
            'max_loops': LaunchConfiguration('max_loops'),
            'default_dwell_time': LaunchConfiguration('default_dwell_time'),
            'auto_start': LaunchConfiguration('auto_start'),
            'arrival_tolerance': 0.3,
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        waypoint_file_arg,
        patrol_mode_arg,
        max_loops_arg,
        default_dwell_time_arg,
        auto_start_arg,
        patrol_node,
    ])
