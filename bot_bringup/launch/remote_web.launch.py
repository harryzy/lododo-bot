#!/usr/bin/env python3
"""
remote_web.launch.py - Web层补充启动文件（配合remote_slam/remote_navigation使用）

功能 / Features:
  - 轻量级Web层启动文件，仅启动Web控制相关组件
  - 设计为在remote_slam.launch.py或remote_navigation.launch.py之后启动
  - 避免节点冲突和同时启动导致的资源竞争
  
组件（仅Web层）/ Components (Web Layer Only):
  1. MissionPlanner - 任务调度和管理
  2. CommandAdapter - 统一命令接口（/cmd/request → /cmd/response）
  3. rosbridge_server - Web前端WebSocket桥接
  
架构 / Architecture:
  [必须先运行] remote_slam.launch.py 或 remote_navigation.launch.py
       ↓ (提供: EKF + RTABMap + Nav2 + RViz)
  [然后运行] remote_web.launch.py (本文件)
       ↓ (添加: MissionPlanner + CommandAdapter + rosbridge)
  [Web Frontend / Voice / CLI]

使用方法 / Usage:
  
  步骤1：启动基础系统（SLAM或定位模式）
  # SLAM模式
  ros2 launch bot_bringup remote_slam.launch.py enable_nav:=true
  
  # 或定位模式
  ros2 launch bot_bringup remote_navigation.launch.py map_name:=office_floor1
  
  步骤2：等待基础系统稳定（约10-15秒）
  # 验证基础系统运行正常
  ros2 node list | grep -E "ekf|rtabmap|bt_navigator"
  ros2 topic hz /rtabmap_slam/info  # 或 /rtabmap/info（定位模式）
  
  步骤3：启动Web层（本文件）
  ros2 launch bot_bringup remote_web.launch.py
  
  # 可选：自定义rosbridge端口
  ros2 launch bot_bringup remote_web.launch.py rosbridge_port:=9091

Web前端访问 / Web Frontend Access:
  1. 启动Web服务器（需要在另一个终端）:
     cd ~/lododo_bot/src/bot_teleop
     bash scripts/start_web_server.sh
  
  2. 浏览器访问:
     http://<pc_ip>:8000
  
  3. WebSocket连接:
     ws://<pc_ip>:9091 (默认端口)

命令接口测试 / Command Interface Test:
  # 发送导航请求
  ros2 topic pub --once /cmd/request std_msgs/msg/String \
    'data: "{\"header\":{\"request_id\":\"nav-001\",\"timestamp\":\"2026-02-12T12:00:00\",\"priority\":3},\"body\":{\"action\":\"navigate_to_pose\",\"params\":{\"x\":2.0,\"y\":3.0,\"yaw\":0.0},\"timeout\":300.0}}"'
  
  # 监听响应
  ros2 topic echo /cmd/response

前置条件 / Prerequisites:
  1. 硬件层在树莓派运行
  2. remote_slam.launch.py 或 remote_navigation.launch.py 已启动
  3. ROS_DOMAIN_ID 在PC和树莓派端必须一致
  4. EKF、RTABMap、Nav2已经正常运行

注意事项 / Notes:
  - 本文件**不启动**EKF、RTABMap、Nav2、robot_state_publisher等
  - 这些基础组件必须通过remote_slam/remote_navigation启动
  - use_sim_time固定为false（真机模式）
  - 如果基础系统未运行，MissionPlanner的某些功能可能不可用
  - rosbridge_server可以单独重启而不影响基础系统

Author: LeKiwi Bot Development Team
Date: 2026-02-12
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    # ==========================================================================
    # Launch Arguments / 启动参数
    # ==========================================================================
    
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
    
    # Get launch configurations
    log_level = LaunchConfiguration('log_level')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    
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
            'persistence_dir': os.path.expanduser('~/lododo_bot/mission_data'),
        }],
        arguments=['--ros-args', '--log-level', log_level],
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
            'log_level': log_level,
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
            'port': rosbridge_port,
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
    # Launch Description / 启动描述
    # ==========================================================================
    
    return LaunchDescription([
        # 声明参数
        declare_log_level,
        declare_rosbridge_port,
        
        # 启动信息
        LogInfo(msg='='*70),
        LogInfo(msg='Remote Web Layer (PC Side)'),
        LogInfo(msg='远程Web控制层（PC端）- 补充性启动文件'),
        LogInfo(msg='='*70),
        LogInfo(msg='⚠️  IMPORTANT / 重要提示:'),
        LogInfo(msg='   This launch file assumes base system is already running!'),
        LogInfo(msg='   本launch文件假设基础系统已经运行！'),
        LogInfo(msg=''),
        LogInfo(msg='   You should have started ONE of:'),
        LogInfo(msg='   你应该已经启动了以下之一：'),
        LogInfo(msg='     - remote_slam.launch.py (SLAM模式)'),
        LogInfo(msg='     - remote_navigation.launch.py (定位模式)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Components Starting / 启动组件:'),
        LogInfo(msg='  [1] MissionPlanner (任务调度器)'),
        LogInfo(msg='  [2] CommandAdapter (命令接口)'),
        LogInfo(msg='  [3] rosbridge_server (Web桥接)'),
        LogInfo(msg='='*70),
        LogInfo(msg='NOT Starting / 不启动:'),
        LogInfo(msg='  ✗ EKF (应该已在remote_slam/navigation中运行)'),
        LogInfo(msg='  ✗ RTABMap (应该已在remote_slam/navigation中运行)'),
        LogInfo(msg='  ✗ Nav2 (应该已在remote_slam/navigation中运行)'),
        LogInfo(msg='  ✗ robot_state_publisher (应该已在remote_slam/navigation中运行)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Web Access / Web访问:'),
        LogInfo(msg='  WebSocket: ws://<pc_ip>:9091 (默认)'),
        LogInfo(msg='  Web UI: http://<pc_ip>:8000 (需手动启动web服务器)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Verification / 验证:'),
        LogInfo(msg='  检查基础系统: ros2 node list | grep -E "ekf|rtabmap"'),
        LogInfo(msg='  检查任务系统: ros2 node list | grep -E "mission|command"'),
        LogInfo(msg='  测试命令接口: ros2 topic echo /cmd/response'),
        LogInfo(msg='='*70),
        
        # 启动Web层组件
        mission_planner,
        cmd_adapter_launch,
        rosbridge_server,
        
        # 完成信息
        LogInfo(msg=''),
        LogInfo(msg='✓ Web layer started successfully!'),
        LogInfo(msg='✓ Web控制层启动成功！'),
        LogInfo(msg=''),
        LogInfo(msg='Next steps / 下一步:'),
        LogInfo(msg='  1. 启动Web服务器: bash src/bot_teleop/scripts/start_web_server.sh'),
        LogInfo(msg='  2. 浏览器访问: http://<pc_ip>:8000'),
        LogInfo(msg='  3. 开始使用Web界面控制机器人'),
        LogInfo(msg='='*70),
    ])
