#!/usr/bin/env python3
"""
real_robot_web_full.launch.py - 真机完整Web控制环境启动文件

功能 / Features:
  - 启动完整的真机Web控制环境
  - 包含硬件层 + 导航栈 + 任务调度 + Web接口
  - 支持SLAM建图和定位导航两种模式
  - 支持Web前端、语音控制、命令行控制
  
组件 / Components:
  1. 硬件层（Hardware Layer）- 舵机 + IMU + 相机
  2. EKF融合（EKF Fusion）- 传感器融合
  3. RTABMap（SLAM/Localization）- 地图构建/定位
  4. Nav2导航（Navigation）- 路径规划
  5. MissionPlanner（Task Scheduler）- 任务调度
  6. CommandAdapter（Command Interface）- 统一命令接口
  7. rosbridge_server（Web Bridge）- Web前端连接
  
架构 / Architecture:
  [Web Frontend / Voice / CLI]
           ↓
    rosbridge_server (WebSocket :9090)
           ↓
    CommandAdapter (/cmd/request → /cmd/response)
           ↓
    MissionPlanner (Task Scheduling)
           ↓
    Nav2 + RTABMap (Navigation + Mapping)
           ↓
    Hardware Layer (Servo + Sensors)

使用方法 / Usage:
  # SLAM建图模式 + Web控制
  ros2 launch bot_bringup real_robot_web_full.launch.py slam:=true
  
  # 定位导航模式 + Web控制（需要已有地图）
  ros2 launch bot_bringup real_robot_web_full.launch.py \
    slam:=false \
    map_name:=office_floor1
  
  # 启动RViz可视化
  ros2 launch bot_bringup real_robot_web_full.launch.py \
    map_name:=office_floor1 \
    use_rviz:=true

Web前端访问 / Web Frontend Access:
  1. 启动Web服务器（需要在另一个终端）:
     cd ~/lododo_bot/src/bot_teleop
     bash scripts/start_web_server.sh
  
  2. 浏览器访问:
     http://<robot_ip>:8000
  
  3. WebSocket连接:
     ws://<robot_ip>:9090

命令接口测试 / Command Interface Test:
  # 发送导航请求
  ros2 topic pub --once /cmd/request std_msgs/msg/String \
    'data: "{\"header\":{\"request_id\":\"nav-001\",\"timestamp\":\"2026-01-21T12:00:00\",\"priority\":3},\"body\":{\"action\":\"navigate_to_pose\",\"params\":{\"x\":2.0,\"y\":3.0,\"yaw\":0.0},\"timeout\":300.0}}"'
  
  # 监听响应
  ros2 topic echo /cmd/response

注意事项 / Notes:
  - rosbridge_server监听所有网络接口（0.0.0.0:9090）
  - 确保防火墙允许9090端口（WebSocket）和8000端口（Web服务器）
  - MissionPlanner和CommandAdapter延迟启动，等待Nav2就绪
  - Web服务器需要手动启动（不在launch文件中）

Author: LeKiwi Bot Development Team
Date: 2026-01-21
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
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
    slam = LaunchConfiguration('slam')
    slam_str = LaunchConfiguration('slam').perform(context)
    map_name = LaunchConfiguration('map_name')
    map_name_str = LaunchConfiguration('map_name').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    log_level = LaunchConfiguration('log_level')
    config_file = LaunchConfiguration('config_file')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    rosbridge_port_str = LaunchConfiguration('rosbridge_port').perform(context)
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    pkg_bot_cmd_interface = get_package_share_directory('bot_cmd_interface')
    
    # ==========================================================================
    # Base Robot System / 基础机器人系统
    # ==========================================================================
    
    # 包含真机完整系统启动
    real_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bot_bringup, 'launch', 'real_robot_bringup.launch.py')
        ),
        launch_arguments={
            'slam': slam,
            'map_name': map_name,
            'enable_nav': 'true',        # Web控制需要Nav2
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'nav2_params': nav2_params,
            'log_level': log_level,
            'config_file': config_file,
        }.items()
    )
    
    # ==========================================================================
    # Mission Planner / 任务调度器
    # ==========================================================================
    
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
            'persistence_dir': '~/lododo_bot/mission_data',
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        respawn=False,
    )
    
    # ==========================================================================
    # Command Adapter / 统一命令接口
    # ==========================================================================
    
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
            'log_level': 'info',
            'use_mock': 'false',
            'service_timeout': '10.0',
        }.items()
    )
    
    # ==========================================================================
    # rosbridge_server / Web前端连接桥
    # ==========================================================================
    
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
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    # ==========================================================================
    # Return Launch Actions / 返回启动动作
    # 注意：real_robot_bringup是IncludeLaunchDescription，无法用OnProcessStart监听
    # 因此直接顺序启动所有组件，依赖内部的事件驱动机制
    # ==========================================================================
    return [
        # 启动信息
        LogInfo(msg='='*70),
        LogInfo(msg='Real Robot Web Full Control Environment'),
        LogInfo(msg='真机完整Web控制环境'),
        LogInfo(msg='='*70),
        LogInfo(msg=f'Mode / 模式: {"SLAM (Building Map)" if slam_str == "true" else "Localization (Navigation)"}'),
        LogInfo(msg=f'Map / 地图: {map_name_str if map_name_str else "N/A (SLAM mode)"}'),
        LogInfo(msg=f'rosbridge Port / 端口: {rosbridge_port_str}'),
        LogInfo(msg='='*70),
        LogInfo(msg='Components / 组件:'),
        LogInfo(msg='  [1] Hardware Layer (Servo + IMU + Camera)'),
        LogInfo(msg='  [2] EKF Fusion (Wheel + IMU)'),
        LogInfo(msg='  [3] RTABMap (SLAM/Localization)'),
        LogInfo(msg='  [4] Nav2 Navigation'),
        LogInfo(msg='  [5] MissionPlanner (Task Scheduler)'),
        LogInfo(msg='  [6] CommandAdapter (Command Interface)'),
        LogInfo(msg='  [7] rosbridge_server (Web Bridge)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Web Frontend / Web前端:'),
        LogInfo(msg='  1. Start web server manually:'),
        LogInfo(msg='     cd ~/lododo_bot/src/bot_teleop && bash scripts/start_web_server.sh'),
        LogInfo(msg='  2. Access in browser:'),
        LogInfo(msg='     http://<robot_ip>:8000'),
        LogInfo(msg='  3. WebSocket endpoint:'),
        LogInfo(msg=f'     ws://<robot_ip>:{rosbridge_port_str}'),
        LogInfo(msg='='*70),
        LogInfo(msg='Command Interface Topics / 命令接口话题:'),
        LogInfo(msg='  Request:  /cmd/request'),
        LogInfo(msg='  Response: /cmd/response'),
        LogInfo(msg='='*70),
        
        # 启动组件（顺序启动）
        real_robot_bringup,      # 基础系统（内部事件驱动）
        mission_planner,         # 任务调度
        cmd_adapter_launch,      # 命令接口
        rosbridge_server,        # Web桥接
    ]


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_hardware = get_package_share_directory('bot_hardware')
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
        description='Map name for localization mode / 定位模式使用的地图名称'
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
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_bot_hardware, 'config', 'hardware_config.yaml'),
        description='Hardware configuration file / 硬件配置文件'
    )
    
    declare_rosbridge_port = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
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
        declare_config_file,
        declare_rosbridge_port,
        OpaqueFunction(function=launch_setup)
    ])
