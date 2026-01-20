#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
hardware_bringup.launch.py - 硬件控制节点启动文件
Hardware Control Node Launch File

⚠️ 架构决策 (2026-01-19): 不使用ros2_control框架
Architecture Decision: Not using ros2_control framework

功能 / Features:
- 启动OmniHardwareNode（独立ROS2节点）
- 可选启动IMU滤波节点
- 可选启动相机节点

设计参考 / Design Reference:
- HARDWARE_DEPLOYMENT_DESIGN.md §3.2.5 Launch文件集成
- ARCHITECTURE_DECISION_STANDALONE_NODE.md
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成Launch描述 / Generate launch description"""
    
    # ============ Launch参数 / Launch Arguments ============
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('bot_hardware'),
            'config',
            'hardware_config.yaml'
        ]),
        description='Path to hardware configuration file'
    )
    
    start_imu_arg = DeclareLaunchArgument(
        'start_imu',
        default_value='false',
        description='Whether to start IMU filter node'
    )
    
    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='false',
        description='Whether to start camera node'
    )
    
    # ============ 节点定义 / Node Definitions ============
    
    # OmniHardwareNode - 核心硬件控制节点
    omni_hardware_node = Node(
        package='bot_hardware',
        executable='omni_hardware_node',
        name='omni_hardware_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'use_sim_time': False  # 真机环境
        }],
        emulate_tty=True,
        respawn=False,  # 初期调试不自动重启
        # respawn=True,  # 生产环境建议启用
        # respawn_delay=2.0
    )
    
    # IMU Filter Node - 可选
    imu_filter_node = Node(
        package='bot_hardware',
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('start_imu'))
    )
    
    # 信息日志
    info_log = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            'Lododo Robot Hardware Bringup\n',
            '=' * 60, '\n',
            'Architecture: Standalone ROS2 Node (Not using ros2_control)\n',
            'Config file: ', LaunchConfiguration('config_file'), '\n',
            'Start IMU: ', LaunchConfiguration('start_imu'), '\n',
            'Start Camera: ', LaunchConfiguration('start_camera'), '\n',
            '=' * 60, '\n'
        ]
    )
    
    return LaunchDescription([
        # Launch参数
        config_file_arg,
        start_imu_arg,
        start_camera_arg,
        
        # 信息输出
        info_log,
        
        # 启动节点
        omni_hardware_node,
        imu_filter_node,
    ])
