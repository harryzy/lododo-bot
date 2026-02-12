#!/usr/bin/env python3
"""
remote_web_full.launch.py - PC端远程完整Web控制环境启动文件

功能 / Features:
  - PC端完整Web控制环境（用于分布式部署）
  - 硬件层在树莓派运行，PC运行所有计算和任务调度
  - 支持SLAM建图和定位导航两种模式
  - 包含Web接口、任务调度、命令适配器
  
组件（PC端）/ Components (PC Side):
  1. EKF + RTABMap + Nav2 - 通过remote_slam.launch.py或remote_navigation.launch.py启动
  2. MissionPlanner - 任务调度和管理
  3. CommandAdapter - 统一命令接口
  4. rosbridge_server - Web前端连接桥
  5. RViz（可选）- 可视化
  
架构 / Architecture:
  [Web Frontend / Voice / CLI]
           ↓
  rosbridge_server (WebSocket :9090)
           ↓
  CommandAdapter (/cmd/request → /cmd/response)
           ↓
  MissionPlanner (Task Scheduling)
           ↓
  remote_slam.launch.py / remote_navigation.launch.py
  (包含: Nav2 + RTABMap + EKF)
           ↓
  [Network] ← Hardware Layer (Raspberry Pi)

前置条件 / Prerequisites:
  1. 树莓派硬件层已启动（需要先运行）
  2. ROS_DOMAIN_ID 在PC和树莓派端必须一致
  3. 网络配置正确（Cyclone DDS配置、socket缓冲区、防火墙）
  4. （可选）已有地图文件用于定位模式

使用方法 / Usage:
  # SLAM建图模式 + Web控制
  ros2 launch bot_bringup remote_web_full.launch.py slam:=true
  
  # 定位导航模式 + Web控制（需要已有地图）
  ros2 launch bot_bringup remote_web_full.launch.py \
    slam:=false \
    map_name:=office_floor1
  
  # 启动RViz可视化
  ros2 launch bot_bringup remote_web_full.launch.py \
    map_name:=office_floor1 \
    use_rviz:=true

Web前端访问 / Web Frontend Access:
  1. 启动Web服务器（需要在另一个终端）:
     cd ~/lododo_bot/src/bot_teleop
     bash scripts/start_web_server.sh
  
  2. 浏览器访问:
     http://<pc_ip>:8000
  
  3. WebSocket连接:
     ws://<pc_ip>:9090

命令接口测试 / Command Interface Test:
  # 发送导航请求
  ros2 topic pub --once /cmd/request std_msgs/msg/String \
    'data: "{\"header\":{\"request_id\":\"nav-001\",\"timestamp\":\"2026-01-29T12:00:00\",\"priority\":3},\"body\":{\"action\":\"navigate_to_pose\",\"params\":{\"x\":2.0,\"y\":3.0,\"yaw\":0.0},\"timeout\":300.0}}"'
  
  # 监听响应
  ros2 topic echo /cmd/response

注意事项 / Notes:
  - 必须先启动树莓派端的硬件驱动
  - use_sim_time固定为false（真机模式）
  - rosbridge_server监听所有网络接口（0.0.0.0:9090）
  - 确保防火墙允许9090端口（WebSocket）和8000端口（Web服务器）
  - Web服务器需要手动启动（不在launch文件中）
  - 本launch文件复用remote_slam.launch.py和remote_navigation.launch.py，避免重复配置

Author: LeKiwi Bot Development Team
Date: 2026-02-12 (Refactored)
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, *args, **kwargs):
    """
    Dynamic launch setup function / 动态启动设置函数
    """
    
    # ==========================================================================
    # Get Launch Configurations / 获取启动配置
    # ==========================================================================
    slam_str = LaunchConfiguration('slam').perform(context)
    map_name_str = LaunchConfiguration('map_name').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    log_level = LaunchConfiguration('log_level')
    rosbridge_port_str = LaunchConfiguration('rosbridge_port').perform(context)
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    
    # ==========================================================================
    # Base Navigation System / 基础导航系统
    # ==========================================================================
    
    # 根据模式选择合适的launch文件（这些文件已包含EKF、RTABMap、Nav2）
    if slam_str.lower() == 'true':
        # SLAM模式：使用remote_slam.launch.py（已测试通过）
        base_system = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bot_bringup, 'launch', 'remote_slam.launch.py')
            ),
            launch_arguments={
                'enable_nav': 'true',    # Web控制需要Nav2导航功能
                'use_rviz': use_rviz,
                'rviz_config': rviz_config,
                'nav2_params': nav2_params,
                'log_level': log_level,
            }.items()
        )
    else:
        # 定位模式：使用remote_navigation.launch.py（已测试通过）
        # 注意：remote_navigation.launch.py已经默认启用Nav2，不需要额外参数
        base_system = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bot_bringup, 'launch', 'remote_navigation.launch.py')
            ),
            launch_arguments={
                'map_name': map_name_str,
                'use_rviz': use_rviz,
                'rviz_config': rviz_config,
                'nav2_params': nav2_params,
                'log_level': log_level,
            }.items()
        )
    
    # ==========================================================================
    # Mission Planner / 任务调度器
    # ==========================================================================
    # 注意：MissionPlanner不在remote_slam/remote_navigation中，需要在这里启动
    
    mission_planner = Node(
        package='bot_navigation',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'update_rate': 10.0,
            'task_timeout': 600.0,
            'enable_auto_recovery': True,
            'persistence_dir': os.path.expanduser('~/lododo_bot/mission_data'),
        }],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=False,
    )
    
    # ==========================================================================
    # Command Adapter / 统一命令接口
    # ==========================================================================
    # 注意：使用bot_cmd_interface包的独立launch文件
    
    cmd_adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bot_cmd_interface'),
                'launch',
                'cmd_adapter.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'log_level': log_level,
            'use_mock': 'false',
            'service_timeout': '10.0',
        }.items()
    )
    
    # ==========================================================================
    # rosbridge_server / Web前端连接桥
    # ==========================================================================
    # 注意：监听所有网络接口，确保Web前端可以连接
    
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': int(rosbridge_port_str),
            'address': '0.0.0.0',  # 监听所有网络接口
            'use_sim_time': False,
            'authenticate': False,  # 生产环境建议启用认证
            'retry_startup_delay': 5.0,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,  # rosbridge可能因网络问题崩溃，启用自动重启
        respawn_delay=5.0,
    )
    
    # ==========================================================================
    # Return Launch Actions / 返回启动动作
    # ==========================================================================
    return [
        # 启动信息
        LogInfo(msg='='*70),
        LogInfo(msg='Remote Web Full Control Environment (PC Side)'),
        LogInfo(msg='远程完整Web控制环境（PC端）'),
        LogInfo(msg='='*70),
        LogInfo(msg=f'Mode / 模式: {"SLAM (建图)" if slam_str.lower() == "true" else "Localization (定位)"}'),
        LogInfo(msg=f'Map / 地图: {map_name_str if map_name_str else "N/A (SLAM mode)"}'),
        LogInfo(msg=f'rosbridge Port / 端口: {rosbridge_port_str}'),
        LogInfo(msg='='*70),
        LogInfo(msg='Launch Composition / 启动组合:'),
        LogInfo(msg=f'  [Base] {"remote_slam.launch.py" if slam_str.lower() == "true" else "remote_navigation.launch.py"}'),
        LogInfo(msg='         (包含: EKF + RTABMap + Nav2)'),
        LogInfo(msg='  [+] MissionPlanner (任务调度)'),
        LogInfo(msg='  [+] CommandAdapter (命令接口)'),
        LogInfo(msg='  [+] rosbridge_server (Web桥接)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Prerequisites / 前置条件:'),
        LogInfo(msg='  1. Hardware layer running on Raspberry Pi'),
        LogInfo(msg='     硬件层在树莓派上运行'),
        LogInfo(msg='  2. ROS_DOMAIN_ID matches between PC and Pi'),
        LogInfo(msg='     ROS_DOMAIN_ID在PC和树莓派上匹配'),
        LogInfo(msg='='*70),
        LogInfo(msg='Web Access / Web访问:'),
        LogInfo(msg=f'  WebSocket: ws://<pc_ip>:{rosbridge_port_str}'),
        LogInfo(msg='  Web UI: http://<pc_ip>:8000 (需要手动启动web服务器)'),
        LogInfo(msg='='*70),
        
        # 启动组件（按依赖顺序）
        base_system,           # 1. 基础导航系统（EKF + RTABMap + Nav2）
        mission_planner,       # 2. 任务调度器（依赖Nav2）
        cmd_adapter_launch,    # 3. 命令接口（依赖MissionPlanner）
        rosbridge_server,      # 4. Web桥接（独立组件）
    ]


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    # ==========================================================================
    # Declare Launch Arguments / 声明启动参数
    # ==========================================================================
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Use SLAM mode (true) or localization mode (false) / 使用SLAM建图模式（true）或定位模式（false）'
    )
    
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='Map name for localization mode (required if slam:=false) / 定位模式使用的地图名称（slam:=false时必需）'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz2 visualization / 启动RViz2可视化'
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
    
    declare_rosbridge_port = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9091',
        description='rosbridge_server WebSocket port / rosbridge服务器WebSocket端口'
    )
    
    # ==========================================================================
    # Launch Description / 启动描述
    # ==========================================================================
    
    return LaunchDescription([
        declare_slam,
        declare_map_name,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        declare_log_level,
        declare_rosbridge_port,
        OpaqueFunction(function=launch_setup)
    ])
