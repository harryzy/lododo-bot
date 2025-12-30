#!/usr/bin/env python3
"""
test_mission_planner.launch.py - 任务规划器测试环境

功能 / Features:
  - 启动完整的仿真环境 + Nav2导航系统
  - 加载已有地图进行定位
  - 启动 MissionPlanner 节点
  - 可选启动自动化测试脚本
  - 支持探索、巡航、导航等多种任务测试
  
使用方法 / Usage:
  # 基础启动（需要手动调用服务测试）
  ros2 launch bot_bringup test_mission_planner.launch.py
  
  # 使用特定地图
  ros2 launch bot_bringup test_mission_planner.launch.py \
    map_name:=new2_map
  
  # 启动自动化测试脚本
  ros2 launch bot_bringup test_mission_planner.launch.py \
    run_tests:=true
  
  # 指定路点文件用于巡航任务测试
  ros2 launch bot_bringup test_mission_planner.launch.py \
    waypoint_file:=~/lododo_bot/waypoints/new2_map.yaml
  
手动测试命令 / Manual Test Commands:
  # 1. 列出所有任务
  ros2 service call /mission/list_tasks bot_navigation_msgs/srv/ListTasks "{filter: 'all'}"
  
  # 2. 创建导航任务
  ros2 service call /mission/navigate_to_pose bot_navigation_msgs/srv/NavigateToPose \
    "{x: 2.0, y: 3.0, yaw: 0.0, frame_id: 'map'}"
  
  # 3. 创建探索任务
  ros2 service call /mission/start_exploration bot_navigation_msgs/srv/StartExploration \
    "{map_name: 'test_explore', save_map: true, max_duration: 300.0, coverage_threshold: 0.9}"
  
  # 4. 创建巡航任务（需要提前录制路点）
  ros2 service call /mission/start_patrol bot_navigation_msgs/srv/StartPatrol \
    "{route_name: 'test_route', patrol_mode: 'loop', max_loops: -1}"
  
  # 5. 查询任务状态
  ros2 service call /mission/get_task_status bot_navigation_msgs/srv/GetTaskStatus \
    "{task_id: 'task_xxx'}"
  
  # 6. 取消任务
  ros2 service call /mission/cancel_task bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'task_xxx'}"
  
  # 7. 紧急停止所有任务
  ros2 service call /mission/emergency_stop bot_navigation_msgs/srv/EmergencyStop "{}"
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
    
    # 默认地图路径
    default_map_path = os.path.expanduser('~/lododo_bot/maps/new2_map/rtabmap.db')
    
    # 默认路点文件
    default_waypoint_file = os.path.expanduser('~/lododo_bot/waypoints/new2_map.yaml')
    
    # ===== 启动参数 =====
    
    # 地图参数
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='new2_map',
        description='Map name in map library / 地图库中的地图名称'
    )
    
    rtabmap_db_path_arg = DeclareLaunchArgument(
        'rtabmap_db_path',
        default_value=default_map_path,
        description='Path to RTABMap database file / RTABMap数据库文件路径'
    )
    
    # 路点文件参数（用于巡航任务测试）
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Waypoint file for patrol testing / 用于巡航测试的路点文件'
    )
    
    # 仿真环境参数
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='navigation_5x5_rgbd.world',
        description='Gazebo world name / Gazebo世界名称'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz / 启动RViz'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    # MissionPlanner参数
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='MissionPlanner update rate (Hz)'
    )
    
    task_timeout_arg = DeclareLaunchArgument(
        'task_timeout',
        default_value='300.0',
        description='Task timeout in seconds'
    )
    
    enable_auto_recovery_arg = DeclareLaunchArgument(
        'enable_auto_recovery',
        default_value='true',
        description='Enable automatic task recovery'
    )
    
    persistence_dir_arg = DeclareLaunchArgument(
        'persistence_dir',
        default_value='',
        description='Mission persistence directory (empty = workspace/mission) / 任务持久化目录（空=工作空间/mission）'
    )
    
    # 测试参数
    run_tests_arg = DeclareLaunchArgument(
        'run_tests',
        default_value='false',
        description='Run automated tests / 运行自动化测试'
    )
    
    test_delay_arg = DeclareLaunchArgument(
        'test_delay',
        default_value='10.0',
        description='Delay before running tests (seconds) / 测试启动延迟(秒)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug/info/warn/error)'
    )
    
    # ===== 获取配置 =====
    map_name = LaunchConfiguration('map_name')
    rtabmap_db_path = LaunchConfiguration('rtabmap_db_path')
    waypoint_file = LaunchConfiguration('waypoint_file')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    update_rate = LaunchConfiguration('update_rate')
    task_timeout = LaunchConfiguration('task_timeout')
    enable_auto_recovery = LaunchConfiguration('enable_auto_recovery')
    persistence_dir = LaunchConfiguration('persistence_dir')
    run_tests = LaunchConfiguration('run_tests')
    test_delay = LaunchConfiguration('test_delay')
    log_level = LaunchConfiguration('log_level')
    
    # ===== 启动基础导航系统（定位模式）=====
    # 使用简化版启动文件，只需要基础的Nav2+RTABMap定位
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bot_bringup_dir,
                'launch',
                'simple_simulation_nav2_rtabmap_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'rtabmap_db_path': rtabmap_db_path,
            'world': world,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
            'log_level': log_level,
        }.items()
    )
    
    # ===== 等待导航系统就绪 =====
    # 使用话题检测替代固定延迟
    wait_for_nav2 = ExecuteProcess(
        cmd=['bash', '-c', 
             'until ros2 topic info /map > /dev/null 2>&1 && '
             'ros2 topic info /global_costmap/costmap > /dev/null 2>&1; do '
             'sleep 0.5; done && '
             'echo "[Event] Navigation system ready"'],
        name='wait_for_nav2',
        output='screen',
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
        emulate_tty=True,
    )
    
    # ===== 自动化测试脚本（可选）=====
    test_script = Node(
        condition=IfCondition(run_tests),
        package='bot_navigation',
        executable='test_mission_planner.py',
        name='mission_planner_tester',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        prefix='xterm -e',  # 在独立终端中运行测试
    )
    
    # ===== 延迟启动MissionPlanner =====
    # 等待导航系统就绪后再启动
    delayed_mission_planner = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav2,
            on_exit=[
                LogInfo(msg='[Event] Starting MissionPlanner...'),
                mission_planner_node,
            ]
        )
    )
    
    # ===== 延迟启动测试脚本 =====
    # 给系统额外的时间完全初始化
    delayed_test_script = TimerAction(
        period=test_delay,
        actions=[
            LogInfo(msg='[Event] Starting automated tests...'),
            test_script,
        ]
    )
    
    # ===== 信息提示 =====
    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            '🧪 MissionPlanner 测试环境 / MissionPlanner Test Environment\n',
            '=' * 80, '\n',
            '\n',
            '📋 系统组件 / System Components:\n',
            '  1. Gazebo 仿真环境 + Nav2 导航系统\n',
            '  2. RTABMap 定位模式（使用已有地图）\n',
            '  3. MissionPlanner 任务规划器\n',
            '\n',
            '🗺️  地图配置 / Map Configuration:\n',
            '  - 地图名称: ', map_name, '\n',
            '  - 数据库路径: ', rtabmap_db_path, '\n',
            '  - 路点文件: ', waypoint_file, '\n',
            '\n',
            '🎮 测试配置 / Test Configuration:\n',
            '  - 自动测试: ', run_tests, '\n',
            '  - 测试延迟: ', test_delay, '秒\n',
            '  - 日志级别: ', log_level, '\n',
            '\n',
            '📡 服务接口 / Service Interfaces:\n',
            '  - /mission/create_task          # 创建任务\n',
            '  - /mission/list_tasks            # 列出任务\n',
            '  - /mission/get_task_status       # 查询任务状态\n',
            '  - /mission/start_task            # 启动任务\n',
            '  - /mission/pause_task            # 暂停任务\n',
            '  - /mission/resume_task           # 恢复任务\n',
            '  - /mission/cancel_task           # 取消任务\n',
            '  - /mission/navigate_to_pose      # 导航任务\n',
            '  - /mission/start_exploration     # 探索任务\n',
            '  - /mission/start_patrol          # 巡航任务\n',
            '  - /mission/emergency_stop        # 紧急停止\n',
            '\n',
            '🧪 测试步骤 / Test Steps:\n',
            '  1. 等待系统初始化完成（约10秒）\n',
            '  2. 在RViz中确认机器人定位正确\n',
            '  3. 使用ros2 service call命令测试各项功能\n',
            '  4. 或运行自动化测试脚本\n',
            '\n',
            '📝 快速测试命令 / Quick Test Commands:\n',
            '  # 列出所有任务\n',
            '  ros2 service call /mission/list_tasks bot_navigation_msgs/srv/ListTasks "{}"\n',
            '\n',
            '  # 导航到指定位置\n',
            '  ros2 service call /mission/navigate_to_pose bot_navigation_msgs/srv/NavigateToPose \\\n',
            '    "{x: 2.0, y: 2.0, yaw: 0.0, frame_id: \'map\'}"\n',
            '\n',
            '⚠️  注意事项 / Notes:\n',
            '  - 确保地图文件存在且路径正确\n',
            '  - 探索任务需要未探索区域\n',
            '  - 巡航任务需要提前录制路点文件（单独启动waypoint_recorder）\n',
            '  - WaypointRecorder需要单独在终端中运行（使用ros2 run启动）\n',
            '  - 使用Ctrl+C优雅退出所有节点\n',
            '\n',
            '=' * 80, '\n',
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        map_name_arg,
        rtabmap_db_path_arg,
        waypoint_file_arg,
        world_arg,
        use_rviz_arg,
        use_sim_time_arg,
        update_rate_arg,
        task_timeout_arg,
        enable_auto_recovery_arg,
        persistence_dir_arg,
        run_tests_arg,
        test_delay_arg,
        log_level_arg,
        
        # 信息提示
        info_msg,
        
        # 启动序列
        navigation_launch,           # 1. 启动导航系统
        wait_for_nav2,              # 2. 等待导航系统就绪
        delayed_mission_planner,    # 3. 启动任务规划器（事件触发）
        delayed_test_script,        # 4. 启动测试脚本（可选，定时器触发）
    ])
