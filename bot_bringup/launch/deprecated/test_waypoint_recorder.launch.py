#!/usr/bin/env python3
"""
test_waypoint_recorder.launch.py - 路点录制器测试启动文件

功能 / Features:
  - 启动仿真环境 + RTABMap定位 + Nav2导航系统
  - 为路点录制准备环境
  - 需要在另一个终端单独启动waypoint_recorder
  
使用方法 / Usage:
  # 终端1: 启动仿真+定位系统
  ros2 launch bot_bringup test_waypoint_recorder.launch.py \
    rtabmap_db_path:=~/lododo_bot/maps/test_map/rtabmap.db
  
  # 终端2: 启动路点录制器（等待终端1完全启动后）
  # ⚠️ 必须使用ros2 run，不能用launch文件（CLI交互需要）
  ros2 run bot_navigation waypoint_recorder \
    --ros-args \
    -p persistence_dir:=~/lododo_bot/waypoints \
    -p pose_topic:=/rtabmap/localization_pose \
    -p use_odom_backup:=true
  
  然后:
    - 在RViz中使用"Nav2 Goal"工具导航机器人
    - 在waypoint_recorder终端中使用命令:
      r - 录制当前位置
      d - 删除最后路点
      s - 保存到文件
      l - 列出所有路点
      q - 退出
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # ===== 包路径 =====
    bot_bringup_dir = FindPackageShare('bot_bringup')
    bot_navigation_dir = FindPackageShare('bot_navigation')
    
    # ===== 启动参数 =====
    
    # RTABMap数据库路径（必须指定）
    rtabmap_db_path_arg = DeclareLaunchArgument(
        'rtabmap_db_path',
        default_value=os.path.expanduser('~/lododo_bot/maps/test_map/rtabmap.db'),
        description='Custom RTABMap database path (auto-detected if using map library)'
    )
    
    # 注意：路点录制器需要在另一个终端单独启动
    # 使用: ros2 launch bot_navigation waypoint_recorder.launch.py
    
    # 仿真参数
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2'
    )
    
    # ===== 获取配置 =====
    rtabmap_db_path = LaunchConfiguration('rtabmap_db_path')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # ===== 启动定位导航系统 =====
    # 使用RTABMap Localization模式（纯定位，不建图）
    simulation_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bot_bringup_dir,
                'launch',
                'simple_simulation_nav2_rtabmap_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'rtabmap_db_path': rtabmap_db_path,
            'gui': gui,
            'use_rviz': use_rviz,
            'log_level': 'warn',
        }.items(),
    )
    
    # 路点录制器需要在另一个终端单独启动
    # 参见顶部的使用说明
    
    return LaunchDescription([
        # 参数声明
        rtabmap_db_path_arg,
        gui_arg,
        use_rviz_arg,
        
        # 启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Waypoint Recorder Test Environment'),
        LogInfo(msg='路点录制测试环境'),
        LogInfo(msg='========================================'),
        LogInfo(msg=''),
        LogInfo(msg='步骤:'),
        LogInfo(msg='  1. 等待系统完全启动（看到地图和机器人）'),
        LogInfo(msg='  2. 在另一个终端启动路点录制器:'),
        LogInfo(msg='     ros2 run bot_navigation waypoint_recorder \\'),
        LogInfo(msg='       --ros-args -p persistence_dir:=~/lododo_bot/waypoints'),
        LogInfo(msg='     ⚠️  不要用launch文件（会导致CLI无法交互）'),
        LogInfo(msg='  3. 在RViz中使用"Nav2 Goal"工具导航机器人'),
        LogInfo(msg='  4. 在waypoint_recorder终端使用命令录制路点'),
        LogInfo(msg=''),
        LogInfo(msg='========================================'),
        
        # 启动定位导航系统
        simulation_nav2_launch,
    ])
