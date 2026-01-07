#!/usr/bin/env python3
"""
simulation_cmd_interface_test.launch.py - 统一命令接口集成测试环境

功能 / Features:
  - 启动完整的仿真环境 + Nav2导航系统 + MissionPlanner
  - 启动CommandAdapter（统一命令接口）
  - 支持SLAM建图模式和定位模式
  - 用于集成测试

组件 / Components:
  1. Gazebo仿真环境
  2. RTABMap SLAM/定位
  3. Nav2导航系统
  4. MissionPlanner任务调度
  5. CommandAdapter命令适配器
  
使用方法 / Usage:
  # SLAM建图模式（用于探索测试）
  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=true
  
  # 定位模式（用于导航/巡航测试，需要已有地图）
  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
    slam:=false \
    map_name:=office_floor1
  
  # 指定世界文件
  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
    world:=office.world \
    slam:=false \
    map_name:=office_floor1

测试命令接口 / Test Command Interface:
  # 发送导航请求
  ros2 topic pub --once /cmd/request std_msgs/msg/String \
    'data: "{\"header\":{\"request_id\":\"test-001\",\"timestamp\":\"2026-01-07T12:00:00\",\"priority\":3},\"body\":{\"action\":\"navigate_to_pose\",\"params\":{\"x\":2.0,\"y\":3.0,\"yaw\":0.0},\"timeout\":300.0}}"'
  
  # 监听响应
  ros2 topic echo /cmd/response

运行集成测试 / Run Integration Tests:
  # 在另一个终端运行测试脚本
  python3 ~/lododo_bot/src/bot_cmd_interface/test/test_integration.py

Author: LeKiwi Bot Development Team
Date: 2026-01-07
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成启动描述"""
    
    # ===== 参数声明 =====
    
    # 世界文件
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='navigation_5x5_rgbd.world',
        description='Gazebo world file name (e.g., office.world, empty_world.world)'
    )
    
    # SLAM/定位模式
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Use SLAM mode (true) or localization mode (false)'
    )
    
    # 地图名称（定位模式使用）
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='exploration_test',
        description='Map name for localization mode (ignored in SLAM mode)'
    )
    
    # 仿真时间
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # 日志级别
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug/info/warn/error)'
    )
    
    # CommandAdapter Mock模式
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock mode for CommandAdapter (true=no real services, false=real services)'
    )
    
    # 服务超时
    service_timeout_arg = DeclareLaunchArgument(
        'service_timeout',
        default_value='10.0',
        description='Service call timeout in seconds for CommandAdapter'
    )
    
    # ===== 包路径 =====
    bot_bringup_dir = FindPackageShare('bot_bringup')
    cmd_interface_dir = FindPackageShare('bot_cmd_interface')
    
    # ===== 启动配置 =====
    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_mock = LaunchConfiguration('use_mock')
    service_timeout = LaunchConfiguration('service_timeout')
    
    # ===== 1. 启动MissionPlanner环境（SLAM模式） =====
    # 当slam=true时启动
    mission_planner_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bot_bringup_dir,
                'launch',
                'simulation_mission_planner_exploration.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(slam)
    )
    
    # ===== 2. 启动MissionPlanner环境（定位模式） =====
    # 当slam=false时启动
    mission_planner_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bot_bringup_dir,
                'launch',
                'simulation_mission_planner_localization.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'map_name': map_name,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(PythonExpression([
            'not ', slam
        ]))
    )
    
    # ===== 3. 启动CommandAdapter =====
    cmd_adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                cmd_interface_dir,
                'launch',
                'cmd_adapter.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': log_level,
            'use_mock': use_mock,
            'service_timeout': service_timeout,
        }.items()
    )
    
    # ===== 启动信息 =====
    launch_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'Command Interface Integration Test Environment\n',
            '统一命令接口集成测试环境\n',
            '='*60, '\n',
            'Mode / 模式: ', slam, ' (true=SLAM建图, false=定位导航)\n',
            'World / 世界: ', world, '\n',
            'Map / 地图: ', map_name, ' (定位模式使用)\n',
            'Mock Mode / Mock模式: ', use_mock, '\n',
            '='*60, '\n',
            '\n',
            'Test Topics / 测试话题:\n',
            '  - Request:  /cmd/request\n',
            '  - Response: /cmd/response\n',
            '\n',
            'Run Integration Tests / 运行集成测试:\n',
            '  python3 ~/lododo_bot/src/bot_cmd_interface/test/test_integration.py\n',
            '\n',
            'Quick Test / 快速测试:\n',
            '  # 发送导航请求\n',
            '  ros2 topic pub --once /cmd/request std_msgs/msg/String \\\n',
            '    \'data: "{...}"\'\n',
            '\n',
            '  # 监听响应\n',
            '  ros2 topic echo /cmd/response\n',
            '\n',
            '='*60, '\n'
        ]
    )
    
    # ===== 返回LaunchDescription =====
    return LaunchDescription([
        # 参数声明
        world_arg,
        slam_arg,
        map_name_arg,
        use_sim_time_arg,
        log_level_arg,
        use_mock_arg,
        service_timeout_arg,
        
        # 启动信息
        launch_info,
        
        # 启动组件（根据slam参数选择）
        mission_planner_slam,
        mission_planner_localization,
        cmd_adapter_launch,
    ])
