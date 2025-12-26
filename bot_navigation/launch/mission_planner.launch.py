#!/usr/bin/env python3
"""
mission_planner.launch.py - 启动任务规划器

启动 MissionPlanner 节点，提供统一的任务管理接口
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""
    
    # 声明参数
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='MissionPlanner update rate (Hz)'
    )
    
    task_timeout_arg = DeclareLaunchArgument(
        'task_timeout',
        default_value='300.0',
        description='Task timeout in seconds'
    )
    
    enable_auto_recovery_arg = DeclareLaunchArgument(
        'enable_auto_recovery',
        default_value='true',
        description='Enable automatic task recovery'
    )
    
    # MissionPlanner 节点
    mission_planner_node = Node(
        package='bot_navigation',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'task_timeout': LaunchConfiguration('task_timeout'),
            'enable_auto_recovery': LaunchConfiguration('enable_auto_recovery'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        update_rate_arg,
        task_timeout_arg,
        enable_auto_recovery_arg,
        LogInfo(msg='Starting MissionPlanner...'),
        mission_planner_node,
    ])
