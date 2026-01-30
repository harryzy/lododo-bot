#!/usr/bin/env python3
"""
remote_control.launch.py - PC端远程监控和控制接口

功能 / Features:
  - PC端轻量级监控界面（树莓派运行完整系统）
  - Mission Planner任务管理
  - Web界面控制
  - RViz可视化（可选）
  - 语音控制（未来扩展）
  
架构 / Architecture:
  树莓派（Edge Computing）          PC（Remote Control）
  ├─ 硬件 + SLAM + Nav2          →  ├─ Mission Planner
  ├─ 所有计算在本地完成            ←  ├─ Web界面
  └─ 发布状态话题                    ├─ RViz（可选）
                                    └─ 语音控制（TODO）
  
网络流量（轻量级）:
  - PC → 树莓派: /goal_pose, /cmd_vel, 任务指令 (<1KB/s)
  - 树莓派 → PC: /tf, /map (压缩), /robot_state (<50KB/s)

使用方法 / Usage:
  # 基础监控（Mission Planner）
  ros2 launch bot_bringup remote_control.launch.py
  
  # 启用RViz可视化
  ros2 launch bot_bringup remote_control.launch.py use_rviz:=true
  
  # 启用Web界面（未来）
  ros2 launch bot_bringup remote_control.launch.py enable_web:=true

注意事项 / Notes:
  - 确保树莓派已启动 real_robot_bringup.launch.py
  - ROS_DOMAIN_ID 必须一致
  - 网络延迟 <1000ms 即可（不再要求低延迟）
  - 地图话题使用压缩传输（map_saver订阅）

Author: LeKiwi Bot Development Team
Date: 2026-01-30
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate PC-side remote control launch description
    生成PC端远程控制启动描述
    """
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    # ==========================================================================
    # Declare Launch Arguments / 声明启动参数
    # ==========================================================================
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for visualization / 启动RViz2可视化'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_bot_navigation, 'config', 'rviz', 'nav2_rtabmap.rviz'),
        description='RViz config file / RViz配置文件'
    )
    
    declare_enable_mission_planner = DeclareLaunchArgument(
        'enable_mission_planner',
        default_value='true',
        description='Enable Mission Planner service interface / 启用Mission Planner服务接口'
    )
    
    declare_enable_web = DeclareLaunchArgument(
        'enable_web',
        default_value='false',
        description='Enable Web control interface (TODO) / 启用Web控制界面（待实现）'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error) / 日志级别'
    )
    
    # ==========================================================================
    # Get Launch Configurations / 获取启动配置
    # ==========================================================================
    
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_mission_planner = LaunchConfiguration('enable_mission_planner')
    log_level = LaunchConfiguration('log_level')
    
    # ==========================================================================
    # RViz Visualization / RViz可视化
    # ==========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz)
    )
    
    # ==========================================================================
    # Mission Planner (Service Interface) / Mission Planner（服务接口）
    # ==========================================================================
    
    mission_planner_node = Node(
        package='bot_navigation',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        condition=IfCondition(enable_mission_planner)
    )
    
    # ==========================================================================
    # Map Saver (保存远程地图) / Map Saver (Save remote maps)
    # ==========================================================================
    # TODO: 添加定时保存地图的节点，从树莓派接收压缩地图数据
    
    # ==========================================================================
    # Web Interface (未来扩展) / Web Interface (Future)
    # ==========================================================================
    # TODO: bot_web节点，提供HTTP接口控制机器人
    
    # ==========================================================================
    # Launch Description / 启动描述
    # ==========================================================================
    
    return LaunchDescription([
        # Declare arguments / 声明参数
        declare_use_rviz,
        declare_rviz_config,
        declare_enable_mission_planner,
        declare_enable_web,
        declare_log_level,
        
        # Startup info / 启动信息
        LogInfo(msg='='*70),
        LogInfo(msg='Remote Control Interface (PC Side)'),
        LogInfo(msg='远程控制接口（PC端）'),
        LogInfo(msg='='*70),
        LogInfo(msg='Architecture / 架构:'),
        LogInfo(msg='  - Raspberry Pi: Hardware + SLAM + Nav2 (Edge Computing)'),
        LogInfo(msg='  - PC: Mission Planner + RViz (Remote Monitoring)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Network Traffic / 网络流量:'),
        LogInfo(msg='  - Control commands: /goal_pose, /cmd_vel (<1KB/s)'),
        LogInfo(msg='  - Status feedback: /tf, /map (compressed), /odom (<50KB/s)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Prerequisites / 前置条件:'),
        LogInfo(msg='  1. Raspberry Pi running real_robot_bringup.launch.py'),
        LogInfo(msg='  2. ROS_DOMAIN_ID consistent on both sides'),
        LogInfo(msg='  3. Network latency <1000ms acceptable'),
        LogInfo(msg='='*70),
        
        # Launch nodes / 启动节点
        rviz_node,
        mission_planner_node,
        
        LogInfo(msg='Remote control interface started!'),
        LogInfo(msg='远程控制接口已启动！'),
    ])
