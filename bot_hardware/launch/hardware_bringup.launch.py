#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
hardware_bringup.launch.py - 完整硬件系统启动文件
Complete Hardware System Bringup Launch File

⚠️ 架构决策 (2026-01-19): 不使用ros2_control框架
Architecture Decision: Not using ros2_control framework

功能 / Features:
- 启动OmniHardwareNode（舵机驱动，50Hz控制循环）
- 包含sensor_bringup.launch.py（IMU + 相机传感器）
- 松耦合模块化设计，便于维护

模块组成 / Module Composition:
1. servo_bringup.launch.py: 舵机驱动 + 编码器里程计
2. sensor_bringup.launch.py: IMU驱动/滤波 + Astra Pro相机
3. 静态TF: base_link → imu_link/camera_link (由sensor_bringup发布)

设计参考 / Design Reference:
- HARDWARE_DEPLOYMENT_DESIGN.md §3.4 Launch文件设计
- ARCHITECTURE_DECISION_STANDALONE_NODE.md
- IMPLEMENTATION_ROADMAP.md P4.1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
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
    
    enable_sensors_arg = DeclareLaunchArgument(
        'enable_sensors',
        default_value='true',
        description='Whether to start sensor nodes (IMU + Camera)'
    )
    
    enable_servo_arg = DeclareLaunchArgument(
        'enable_servo',
        default_value='true',
        description='Whether to start servo driver node'
    )
    
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Whether to enable IMU (passed to sensor_bringup)'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Whether to enable camera (passed to sensor_bringup)'
    )
    
    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_tf',
        default_value='true',
        description='Whether to publish static TF (passed to sensor_bringup)'
    )
    
    # ============ 包含模块启动 / Include Module Bringup ============
    
    # 包含servo_bringup.launch.py（舵机驱动）
    servo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_hardware'),
                'launch',
                'servo_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_servo'))
    )
    
    # 包含sensor_bringup.launch.py（IMU + Camera）
    sensor_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_hardware'),
                'launch',
                'sensor_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_imu': LaunchConfiguration('enable_imu'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'publish_static_tf': LaunchConfiguration('publish_static_tf'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_sensors'))
    )
    
    # ============ 信息日志 / Info Log ============
    info_log = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            '           Lododo Robot Hardware Bringup\n',
            '=' * 70, '\n',
            'Architecture: Standalone ROS2 Node (Not using ros2_control)\n',
            'Config file: ', LaunchConfiguration('config_file'), '\n',
            '\n',
            'Modules:\n',
            '  [1] Servo Bringup    - Servo control + Odometry (', LaunchConfiguration('enable_servo'), ')\n',
            '  [2] Sensor Bringup   - IMU (', LaunchConfiguration('enable_imu'), 
            ') + Camera (', LaunchConfiguration('enable_camera'), ')\n',
            '\n',
            'Topics:\n',
            '  /cmd_vel       → OmniHardwareNode (input)\n',
            '  /wheel/odom    ← OmniHardwareNode (output)\n',
            '  /imu/data      ← IMU Filter Node\n',
            '  /camera/*      ← Astra Pro Camera\n',
            '=' * 70, '\n'
        ]
    )
    
    return LaunchDescription([
        # Launch参数
        config_file_arg,
        enable_servo_arg,
        enable_sensors_arg,
        enable_imu_arg,
        enable_camera_arg,
        publish_static_tf_arg,
        
        # 信息输出
        info_log,
        
        # 舵机模块（包含）
        servo_bringup_launch,
        
        # 传感器模块（包含）
        sensor_bringup_launch,
    ])
