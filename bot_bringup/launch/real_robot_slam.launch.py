#!/usr/bin/env python3
"""
real_robot_slam.launch.py - 真机SLAM建图专用启动文件

功能 / Features:
  - 真机SLAM建图模式启动（简化版）
  - 自动设置slam:=true
  - 包含完整硬件层 + EKF融合 + RTABMap SLAM + Nav2
  - 用于首次建图或更新地图
  
与 real_robot_bringup.launch.py 的区别:
  - real_robot_bringup.launch.py: 通用启动文件，支持SLAM和定位两种模式
  - real_robot_slam.launch.py: SLAM专用，固定slam:=true，简化参数
  
使用方法 / Usage:
  # 标准SLAM建图
  ros2 launch bot_bringup real_robot_slam.launch.py
  
  # 启动RViz可视化
  ros2 launch bot_bringup real_robot_slam.launch.py use_rviz:=true
  
  # 调整日志级别
  ros2 launch bot_bringup real_robot_slam.launch.py log_level:=info

保存地图 / Save Map:
  # 方法1: RTABMap自动保存（关闭节点时）
  # 方法2: 手动保存
  ros2 service call /rtabmap/save_map std_srvs/srv/Empty
  
  # 导出2D栅格地图
  ros2 run nav2_map_server map_saver_cli -f ~/lododo_bot/maps/<map_name>/map

注意事项 / Notes:
  - SLAM模式会清空已有数据库（--delete_db_on_start）
  - 建议首次建图时关闭Nav2（enable_nav:=false），提高SLAM性能
  - 地图自动保存到RTABMap默认位置

Author: LeKiwi Bot Development Team
Date: 2026-01-21
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_bot_hardware = get_package_share_directory('bot_hardware')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    # ==========================================================================
    # Declare Launch Arguments / 声明启动参数
    # ==========================================================================
    
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav',
        default_value='false',
        description='Enable Nav2 navigation during SLAM (may affect mapping quality) / SLAM期间启用Nav2（可能影响建图质量）'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for SLAM visualization / 启动RViz2用于SLAM可视化'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_bot_navigation, 'config', 'rviz', 'nav2_rtabmap.rviz'),
        description='RViz config file / RViz配置文件'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(pkg_bot_navigation, 'config', 'nav2', 'nav2_params_imu.yaml'),
        description='Nav2 parameters file / Nav2参数文件'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Log level (debug, info, warn, error, fatal) / 日志级别'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_bot_hardware, 'config', 'hardware_config.yaml'),
        description='Hardware configuration file / 硬件配置文件'
    )
    
    # ==========================================================================
    # Launch Configurations / 启动配置
    # ==========================================================================
    enable_nav = LaunchConfiguration('enable_nav')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    log_level = LaunchConfiguration('log_level')
    config_file = LaunchConfiguration('config_file')
    
    # ==========================================================================
    # Include real_robot_bringup with SLAM mode / 包含SLAM模式的真机启动
    # ==========================================================================
    
    real_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bot_bringup, 'launch', 'real_robot_bringup.launch.py')
        ),
        launch_arguments={
            'slam': 'true',              # ← 固定SLAM模式
            'map_name': '',              # SLAM模式不需要地图
            'enable_nav': enable_nav,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'nav2_params': nav2_params,
            'log_level': log_level,
            'config_file': config_file,
        }.items()
    )
    
    # ==========================================================================
    # Launch Description / 启动描述
    # ==========================================================================
    
    return LaunchDescription([
        # 启动信息
        LogInfo(msg='='*70),
        LogInfo(msg='Real Robot SLAM Mapping Mode'),
        LogInfo(msg='真机SLAM建图模式'),
        LogInfo(msg='='*70),
        LogInfo(msg='Purpose / 用途: Building new map or updating existing map'),
        LogInfo(msg='            首次建图或更新现有地图'),
        LogInfo(msg='='*70),
        LogInfo(msg='Tips / 提示:'),
        LogInfo(msg='  1. Drive robot slowly for better feature tracking'),
        LogInfo(msg='     缓慢驾驶机器人以获得更好的特征跟踪'),
        LogInfo(msg='  2. Avoid fast rotation to prevent motion blur'),
        LogInfo(msg='     避免快速旋转以防止运动模糊'),
        LogInfo(msg='  3. Cover all areas you want to map'),
        LogInfo(msg='     覆盖所有需要建图的区域'),
        LogInfo(msg='  4. Close loop by revisiting starting point'),
        LogInfo(msg='     通过重访起点来闭合回环'),
        LogInfo(msg='='*70),
        
        # 参数声明
        declare_enable_nav,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        declare_log_level,
        declare_config_file,
        
        # 包含真机启动
        real_robot_bringup,
    ])
