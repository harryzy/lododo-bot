#!/usr/bin/env python3
"""
simulation_mission_planner_exploration.launch.py - Mission Planner 探索建图测试环境

功能 / Features:
  - 启动完整的仿真环境 + Nav2导航系统
  - **SLAM建图模式**（实时建图，适合探索任务）
  - 启动 MissionPlanner 节点
  - 支持自动化探索建图测试
  
与 simulation_mission_planner.launch.py 的区别：
  - simulation_mission_planner.launch.py: 使用定位模式（需要已有地图），适合导航和巡航测试
  - **simulation_mission_planner_exploration.launch.py**: 使用 SLAM 建图模式，适合探索建图测试
  
使用方法 / Usage:
  # 启动探索建图环境
  ros2 launch bot_bringup simulation_mission_planner_exploration.launch.py
  
  # 指定世界文件
  ros2 launch bot_bringup simulation_mission_planner_exploration.launch.py \
    world:=empty_world.world
  
  # 启动自动化探索测试
  ros2 launch bot_bringup simulation_mission_planner_exploration.launch.py \
    run_tests:=true

测试探索建图 / Test Exploration Mapping:
  # 1. 开始探索（自动建图）
  ros2 service call /mission/start_exploration bot_navigation_msgs/srv/StartExploration \
    "{map_name: 'new_explore', save_map: true, max_duration: 600.0, coverage_threshold: 0.8}"
  
  # 2. 查询任务状态
  ros2 service call /mission/get_task_status bot_navigation_msgs/srv/GetTaskStatus \
    "{task_id: 'exploration_xxx'}"
  
  # 3. 暂停探索
  ros2 service call /mission/task_control bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'exploration_xxx', action: 'pause'}"
  
  # 4. 恢复探索
  ros2 service call /mission/task_control bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'exploration_xxx', action: 'resume'}"
  
  # 5. 停止探索（保存地图）
  ros2 service call /mission/task_control bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'exploration_xxx', action: 'cancel'}"

注意事项 / Notes:
  - SLAM 模式下地图会实时更新
  - 探索完成后会自动保存地图到 ~/lododo_bot/maps/<map_name>/
  - 地图包含：rtabmap.db (RTABMap数据库), map.pgm/map.yaml (栅格地图)
  - 不需要预先加载地图（从空白环境开始）

Author: LeKiwi Bot Development Team
Date: 2025-12-31
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    
    # 基础参数
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='navigation_5x5_rgbd.world',
        description='Gazebo world file name'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # MissionPlanner 参数
    declare_update_rate = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Task update rate (Hz)'
    )
    
    declare_task_timeout = DeclareLaunchArgument(
        'task_timeout',
        default_value='600.0',
        description='Default task timeout (seconds)'
    )
    
    declare_enable_auto_recovery = DeclareLaunchArgument(
        'enable_auto_recovery',
        default_value='true',
        description='Enable automatic task recovery'
    )
    
    declare_persistence_dir = DeclareLaunchArgument(
        'persistence_dir',
        default_value='~/workDisk/lododo_bot/mission_data',
        description='Directory for task persistence'
    )
    
    # 测试参数
    declare_run_tests = DeclareLaunchArgument(
        'run_tests',
        default_value='false',
        description='Run automated integration tests'
    )
    
    declare_test_delay = DeclareLaunchArgument(
        'test_delay',
        default_value='10.0',
        description='Delay before starting tests (seconds)'
    )
    
    # 日志级别
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # ===== 获取参数值 =====
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    update_rate = LaunchConfiguration('update_rate')
    task_timeout = LaunchConfiguration('task_timeout')
    enable_auto_recovery = LaunchConfiguration('enable_auto_recovery')
    persistence_dir = LaunchConfiguration('persistence_dir')
    run_tests = LaunchConfiguration('run_tests')
    test_delay = LaunchConfiguration('test_delay')
    log_level = LaunchConfiguration('log_level')
    
    # ===== 启动基础导航系统（SLAM 建图模式）=====
    # 使用 SLAM 模式启动文件，支持实时建图
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bot_bringup_dir,
                'launch',
                'simple_simulation_nav2_rtabmap.launch.py'  # ← SLAM 建图模式
            ])
        ]),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
            'use_simple_model': 'true',
        }.items()
    )
    
    # ===== MissionPlanner 节点 =====
    mission_planner_node = Node(
        package='bot_navigation',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'update_rate': update_rate,
            'task_timeout': task_timeout,
            'enable_auto_recovery': enable_auto_recovery,
            'persistence_dir': persistence_dir,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=False,
        respawn_delay=2.0,
    )
    
    # 延迟启动 MissionPlanner（等待 Nav2 和 RTABMap 完全启动）
    delayed_mission_planner = TimerAction(
        period=15.0,  # 等待15秒让 SLAM 初始化
        actions=[
            LogInfo(msg='[Mission Planner] Starting MissionPlanner node...'),
            mission_planner_node,
        ]
    )
    
    # ===== 可选：自动化测试脚本 =====
    test_script = ExecuteProcess(
        condition=IfCondition(run_tests),
        cmd=[
            'python3',
            PathJoinSubstitution([
                bot_navigation_dir,
                'scripts',
                'test_mission_integration_v2.py'
            ]),
            '--test', '6',  # TEST 6: Exploration mode (SLAM)
            '--verbose'
        ],
        output='screen',
        shell=False,
    )
    
    # 延迟启动测试（等待 MissionPlanner 启动）
    delayed_test = TimerAction(
        condition=IfCondition(run_tests),
        period=PythonExpression([test_delay, ' + 15.0']),  # test_delay + mission_planner delay
        actions=[
            LogInfo(msg='[Test] Starting automated exploration tests...'),
            test_script,
        ]
    )
    
    # ===== 返回启动描述 =====
    return LaunchDescription([
        # 声明参数
        declare_world,
        declare_use_sim_time,
        declare_update_rate,
        declare_task_timeout,
        declare_enable_auto_recovery,
        declare_persistence_dir,
        declare_run_tests,
        declare_test_delay,
        declare_log_level,
        
        # 启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Mission Planner Exploration Test Launch'),
        LogInfo(msg='任务规划器探索建图测试启动'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Mode: SLAM Mapping (RTABMap)'),
        LogInfo(msg='模式: SLAM 建图（RTABMap）'),
        LogInfo(msg='Use Case: Exploration & Mapping'),
        LogInfo(msg='用途: 探索建图'),
        LogInfo(msg='========================================'),
        
        # 启动节点和服务
        navigation_launch,        # Nav2 + RTABMap SLAM
        delayed_mission_planner,  # MissionPlanner (延迟15s)
        delayed_test,            # 测试脚本（可选）
    ])
