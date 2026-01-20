#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
servo_bringup.launch.py - 舵机驱动启动文件
Servo Driver Bringup Launch File

功能 / Features:
- 仅启动OmniHardwareNode（舵机驱动 + 里程计）
- 不包含传感器，便于独立测试和调试

设计参考 / Design Reference:
- HARDWARE_DEPLOYMENT_DESIGN.md §3.2.5
- IMPLEMENTATION_ROADMAP.md P4.1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (for real hardware, should be false)'
    )
    
    # ============ 节点定义 / Node Definitions ============
    
    # OmniHardwareNode - 舵机驱动核心节点
    omni_hardware_node = Node(
        package='bot_hardware',
        executable='omni_hardware_node',
        name='omni_hardware_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        emulate_tty=True,
        respawn=False,  # 初期调试不自动重启 / No auto-restart during debugging
        # respawn=True,  # 生产环境建议启用 / Enable in production
        # respawn_delay=2.0
    )
    
    # ============ 信息日志 / Info Log ============
    info_log = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            '         Servo Driver Bringup (Standalone)\n',
            '=' * 60, '\n',
            'Module: OmniHardwareNode\n',
            '  - Servo control (ST3215)\n',
            '  - Odometry publishing (/wheel/odom)\n',
            '  - Control frequency: 50Hz\n',
            '\n',
            'Config: ', LaunchConfiguration('config_file'), '\n',
            'Use sim time: ', LaunchConfiguration('use_sim_time'), '\n',
            '\n',
            'Topics:\n',
            '  /cmd_vel       → Input (command velocity)\n',
            '  /wheel/odom    → Output (wheel odometry)\n',
            '=' * 60, '\n'
        ]
    )
    
    return LaunchDescription([
        # Launch参数
        config_file_arg,
        use_sim_time_arg,
        
        # 信息输出
        info_log,
        
        # 核心节点
        omni_hardware_node,
    ])
