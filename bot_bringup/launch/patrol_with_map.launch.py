#!/usr/bin/env python3
"""
加载地图 + 巡航导航
Load Map + Patrol Navigation

功能 / Features:
  - 启动仿真环境 + Nav2导航系统
  - 从地图库或文件加载地图
  - 启动AMCL定位
  - 复用patrol.launch.py启动巡航节点
  - 支持多种巡航模式
  
使用方法 / Usage:
  # 使用地图库中的地图
  ros2 launch bot_bringup patrol_with_map.launch.py use_map_library:=true map_name:=office_floor1
  
  # 使用指定的地图文件
  ros2 launch bot_bringup patrol_with_map.launch.py use_map_library:=false map_file:=/path/to/map.yaml
  
  # 指定巡航模式和配置
  ros2 launch bot_bringup patrol_with_map.launch.py patrol_mode:=loop waypoint_file:=patrol_waypoints.yaml
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
    
    # 默认配置文件路径
    default_waypoint_file = os.path.join(
        get_package_share_directory('bot_navigation'),
        'config',
        'patrol_waypoints.yaml'
    )
    
    # ===== 启动参数 =====
    
    # 环境参数
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
    
    # 地图加载参数
    use_map_library_arg = DeclareLaunchArgument(
        'use_map_library',
        default_value='false',
        description='Load map from map library / 从地图库加载地图'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='auto_explored_map',
        description='Map name in library (if use_map_library=true) / 地图库中的地图名称'
    )
    
    map_version_arg = DeclareLaunchArgument(
        'map_version',
        default_value='0',  # 0表示最新版本
        description='Map version (0=latest) / 地图版本(0=最新版本)'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Direct map file path (if use_map_library=false) / 直接指定地图文件路径'
    )
    
    rtabmap_db_path_arg = DeclareLaunchArgument(
        'rtabmap_db_path',
        default_value='',
        description='RTABMap database path (auto-detected if using map library) / RTABMap数据库路径（使用地图库时自动检测）'
    )
    
    # 巡航参数
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Waypoint configuration file path / 路点配置文件路径'
    )
    
    patrol_mode_arg = DeclareLaunchArgument(
        'patrol_mode',
        default_value='loop',
        description='Patrol mode: loop, ping_pong, once, random / 巡航模式'
    )
    
    max_loops_arg = DeclareLaunchArgument(
        'max_loops',
        default_value='-1',
        description='Maximum patrol loops (-1=infinite) / 最大循环次数(-1=无限)'
    )
    
    default_dwell_time_arg = DeclareLaunchArgument(
        'default_dwell_time',
        default_value='2.0',
        description='Default dwell time at waypoints (seconds) / 默认停留时间(秒)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start patrol / 自动开始巡航'
    )
    
    # ===== 获取配置 =====
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_map_library = LaunchConfiguration('use_map_library')
    map_name = LaunchConfiguration('map_name')
    map_version = LaunchConfiguration('map_version')
    map_file = LaunchConfiguration('map_file')
    rtabmap_db_path = LaunchConfiguration('rtabmap_db_path')  # 新增
    waypoint_file = LaunchConfiguration('waypoint_file')
    patrol_mode = LaunchConfiguration('patrol_mode')
    max_loops = LaunchConfiguration('max_loops')
    default_dwell_time = LaunchConfiguration('default_dwell_time')
    auto_start = LaunchConfiguration('auto_start')
    auto_start = LaunchConfiguration('auto_start')
    
    # ===== 启动仿真+Nav2系统(带定位) =====
    # 使用RTABMap Localization模式（纯定位，不建图）
    # Uses RTABMap Localization mode (localization only, no mapping)
    # 设置日志级别为warn减少输出
    simulation_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bot_bringup_dir,
                'launch',
                'simple_simulation_nav2_rtabmap_localization.launch.py'  # ✅ 使用Localization版本
            ])
        ]),
        launch_arguments={
            'world': world,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
            'log_level': 'warn',  # 减少日志输出
            'rtabmap_db_path': rtabmap_db_path,  # ✅ 传递数据库路径
        }.items()
    )
    
    # ===== 地图加载节点 (如果使用地图库) =====
    map_loader_node = Node(
        package='bot_navigation',
        executable='map_loader_node',
        name='map_loader',
        output='screen',
        parameters=[{
            'map_name': map_name,
            'map_version': map_version,
            'library_path': '~/lododo_bot/maps',
            'auto_load': True,
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],  # 改为INFO以查看输出
        condition=IfCondition(use_map_library),
    )
    
    # ===== Map Server (发布地图到/map话题) =====
    # RTABMap在localization模式下会从数据库生成occupancy grid并发布到/map
    # 但Nav2的static_layer需要从/map订阅,所以我们需要确保RTABMap正确配置
    # 注意: RTABMap的grid_map重映射到/map已在localization launch中配置
    
    # ===== 巡航节点 (复用patrol.launch.py) =====
    patrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_navigation'),
                'launch',
                'patrol.launch.py'
            ])
        ]),
        launch_arguments={
            'waypoint_file': waypoint_file,
            'patrol_mode': patrol_mode,
            'max_loops': max_loops,
            'default_dwell_time': default_dwell_time,
            'auto_start': auto_start,
        }.items()
    )
    
    # ===== 使用事件处理器控制启动顺序（事件驱动，无硬编码延迟） =====
    # 等待Nav2 navigate_to_pose action server准备好，说明导航系统已就绪
    wait_for_nav2 = ExecuteProcess(
        cmd=['bash', '-c', 'until ros2 action list | grep -q navigate_to_pose; do sleep 0.5; done && echo "[Event] Nav2 action server is ready"'],
        name='wait_for_nav2_action',
        output='screen',
    )
    
    # 当Nav2准备好后（wait_for_nav2进程退出），启动地图加载和巡航
    delayed_nodes_start = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_nav2,
            on_exit=[
                LogInfo(msg='[Event] Nav2 ready, loading map and starting patrol...'),
                map_loader_node,
                patrol_launch,
            ]
        )
    )
    
    # ===== 信息提示 =====
    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            '🚶 加载地图 + 自主巡航\n',
            '=' * 80, '\n',
            '\n',
            '📋 启动流程:\n',
            '  1. 启动Gazebo仿真环境\n',
            '  2. 加载地图 (从地图库或文件)\n',
            '  3. 启动AMCL定位\n',
            '  4. 启动Nav2导航系统\n',
            '  5. 启动巡航节点\n',
            '\n',
            '🗺️ 地图配置:\n',
            '  - 使用地图库: ', use_map_library, '\n',
            '  - 地图名称: ', map_name, '\n',
            '  - 地图版本: ', map_version, ' (0=最新)\n',
            '  - 地图文件: ', map_file, '\n',
            '\n',
            '🚶 巡航配置:\n',
            '  - 路点文件: ', waypoint_file, '\n',
            '  - 巡航模式: ', patrol_mode, '\n',
            '  - 最大循环次数: ', max_loops, '\n',
            '  - 默认停留时间: ', default_dwell_time, 's\n',
            '  - 自动开始: ', auto_start, '\n',
            '\n',
            '🎮 巡航控制:\n',
            '  - 开始巡航: ros2 service call /patrol/start std_srvs/srv/Trigger\n',
            '  - 停止巡航: ros2 service call /patrol/stop std_srvs/srv/Trigger\n',
            '  - 暂停巡航: ros2 service call /patrol/pause std_srvs/srv/Trigger\n',
            '  - 恢复巡航: ros2 service call /patrol/resume std_srvs/srv/Trigger\n',
            '\n',
            '🔍 监控话题:\n',
            '  - /patrol/status: 巡航状态\n',
            '  - /patrol/complete: 巡航完成信号\n',
            '  - /amcl_pose: 定位位姿\n',
            '  - /goal_pose: 当前导航目标\n',
            '\n',
            '💡 提示:\n',
            '  - 确保机器人初始位置与地图匹配\n',
            '  - 可使用RViz的"2D Pose Estimate"设置初始位姿\n',
            '  - 路点坐标应该在地图坐标系中\n',
            '\n',
            '=' * 80, '\n',
        ]
    )
    
    return LaunchDescription([
        # 参数
        world_arg,
        use_rviz_arg,
        use_sim_time_arg,
        use_map_library_arg,
        map_name_arg,
        map_version_arg,
        map_file_arg,
        rtabmap_db_path_arg,  # ✅ 新增RTABMap数据库路径参数
        waypoint_file_arg,
        patrol_mode_arg,
        max_loops_arg,
        default_dwell_time_arg,
        auto_start_arg,
        
        # 信息
        info_msg,
        
        # 仿真+Nav2（立即启动）
        simulation_nav2_launch,
        
        # 等待Nav2准备好
        wait_for_nav2,
        
        # 事件处理器：当Nav2就绪后启动巡航
        delayed_nodes_start,
    ])
