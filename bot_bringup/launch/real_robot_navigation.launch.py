#!/usr/bin/env python3
"""
real_robot_navigation.launch.py - 真机定位导航专用启动文件

功能 / Features:
  - 真机定位导航模式启动（简化版）
  - 自动设置slam:=false, enable_nav:=true
  - 需要已有地图文件
  - 用于自主导航、巡逻任务
  
与 real_robot_bringup.launch.py 的区别:
  - real_robot_bringup.launch.py: 通用启动文件，支持SLAM和定位两种模式
  - real_robot_navigation.launch.py: 导航专用，固定slam:=false + enable_nav:=true
  
使用方法 / Usage:
  # 标准导航模式（需要指定地图）
  ros2 launch bot_bringup real_robot_navigation.launch.py map_name:=office_floor1
  
  # 启动RViz可视化
  ros2 launch bot_bringup real_robot_navigation.launch.py \
    map_name:=office_floor1 \
    use_rviz:=true
  
  # 调整日志级别
  ros2 launch bot_bringup real_robot_navigation.launch.py \
    map_name:=office_floor1 \
    log_level:=info

导航测试 / Navigation Test:
  # 发送导航目标（通过RViz或命令行）
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"
  
  # 取消导航
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose --cancel

巡逻任务 / Patrol Task:
  # 启动巡逻（需要预先录制的航点）
  ros2 service call /mission/start_patrol bot_navigation_msgs/srv/StartPatrol \
    "{waypoint_file: 'route1.yaml', mode: 'loop'}"

注意事项 / Notes:
  - 必须提供map_name参数，指向已存在的地图
  - 地图路径：workspace/maps/<map_name>/
  - 首次定位可能需要手动在RViz中设置初始位姿（2D Pose Estimate）
  - RTABMap会在启动时尝试重定位

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
    
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='Map name for localization (required, e.g., office_floor1) / 定位使用的地图名称（必填，如office_floor1）'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for navigation visualization / 启动RViz2用于导航可视化'
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
    map_name = LaunchConfiguration('map_name')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    log_level = LaunchConfiguration('log_level')
    config_file = LaunchConfiguration('config_file')
    
    # ==========================================================================
    # Include real_robot_bringup with Navigation mode / 包含导航模式的真机启动
    # ==========================================================================
    
    real_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bot_bringup, 'launch', 'real_robot_bringup.launch.py')
        ),
        launch_arguments={
            'slam': 'false',             # ← 固定定位模式
            'map_name': map_name,        # 必须提供地图名称
            'enable_nav': 'true',        # ← 固定启用Nav2
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
        LogInfo(msg='Real Robot Navigation Mode'),
        LogInfo(msg='真机定位导航模式'),
        LogInfo(msg='='*70),
        LogInfo(msg='Purpose / 用途: Autonomous navigation with existing map'),
        LogInfo(msg='            使用已有地图进行自主导航'),
        LogInfo(msg='='*70),
        LogInfo(msg='Requirements / 要求:'),
        LogInfo(msg='  1. Map file must exist: workspace/maps/<map_name>/'),
        LogInfo(msg='     地图文件必须存在：workspace/maps/<map_name>/'),
        LogInfo(msg='  2. Set initial pose in RViz (2D Pose Estimate)'),
        LogInfo(msg='     在RViz中设置初始位姿（2D Pose Estimate）'),
        LogInfo(msg='='*70),
        LogInfo(msg='Navigation Commands / 导航命令:'),
        LogInfo(msg='  - Set goal in RViz: 2D Goal Pose'),
        LogInfo(msg='    在RViz中设置目标：2D Goal Pose'),
        LogInfo(msg='  - Or use topic: /goal_pose'),
        LogInfo(msg='    或使用话题：/goal_pose'),
        LogInfo(msg='  - Or use MissionPlanner services'),
        LogInfo(msg='    或使用MissionPlanner服务'),
        LogInfo(msg='='*70),
        
        # 参数声明
        declare_map_name,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        declare_log_level,
        declare_config_file,
        
        # 包含真机启动
        real_robot_bringup,
    ])
