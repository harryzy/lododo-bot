#!/usr/bin/env python3
"""
自主探索建图 + 自动保存到地图库
Autonomous Exploration with Auto Map Save

功能 / Features:
  - 启动完整的仿真环境 + Nav2导航系统
  - 自动探索建图
  - 探索完成后自动保存地图到地图库
  - 支持配置地图名称和描述
  
使用方法 / Usage:
  ros2 launch bot_bringup exploration_with_map_save.launch.py
  ros2 launch bot_bringup exploration_with_map_save.launch.py map_name:=office_floor1 description:="办公室1楼"
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
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
    
    # 配置文件路径
    exploration_config = os.path.join(
        get_package_share_directory('bot_navigation'),
        'config',
        'exploration',
        'exploration_manager.yaml'
    )
    
    # ===== 启动参数 =====
    
    # 探索参数
    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius',
        default_value='8.0',
        description='Exploration radius in meters / 探索半径(米)'
    )
    
    completion_threshold_arg = DeclareLaunchArgument(
        'completion_threshold',
        default_value='0.78',
        description='Map completion threshold (0-1) / 地图完成度阈值'
    )
    
    # 地图保存相关参数
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='auto_explored_map',
        description='Name for the saved map / 保存的地图名称'
    )
    
    description_arg = DeclareLaunchArgument(
        'description',
        default_value='Automatically explored map',
        description='Description for the saved map / 地图描述'
    )
    
    tags_arg = DeclareLaunchArgument(
        'tags',
        default_value='auto,exploration',
        description='Comma-separated tags for the map / 地图标签(逗号分隔)'
    )
    
    auto_save_arg = DeclareLaunchArgument(
        'auto_save',
        default_value='true',
        description='Automatically save map when exploration completes / 探索完成后自动保存地图'
    )
    
    # ===== 获取配置 =====
    exploration_radius = LaunchConfiguration('exploration_radius')
    completion_threshold = LaunchConfiguration('completion_threshold')
    map_name = LaunchConfiguration('map_name')
    description = LaunchConfiguration('description')
    tags = LaunchConfiguration('tags')
    auto_save = LaunchConfiguration('auto_save')
    
    # ===== 启动仿真+Nav2系统 =====
    # 启动基础系统，日志级别设为warn
    simulation_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bot_bringup_dir,
                'launch',
                'simple_simulation_nav2_rtabmap.launch.py'
            ])
        ]),
        launch_arguments={'log_level': 'warn'}.items(),
    )
    
    # ===== 探索建图节点 =====
    exploration_node = Node(
        package='bot_navigation',
        executable='exploration_mapper',
        name='exploration_mapper',
        output='screen',
        parameters=[
            exploration_config,
            {
                'exploration_radius': exploration_radius,
                'map_completion_threshold': completion_threshold,
            }
        ],
        # 设置日志级别为WARN，减少日志输出
        arguments=['--ros-args', '--log-level', 'WARN'],
    )
    
    # ===== 地图保存节点 =====
    # 监听探索完成信号并自动保存地图
    map_saver_node = Node(
        package='bot_navigation',
        executable='map_saver_node',
        name='map_saver',
        output='screen',
        parameters=[{
            'map_name': map_name,
            'description': description,
            'tags': tags,
            'auto_save': auto_save,
            'library_path': '~/lododo_bot/maps',
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )
    
    # ===== 使用事件处理器控制启动顺序（事件驱动，无硬编码延迟） =====
    # 等待/map话题可用，说明SLAM已经准备好
    wait_for_map = ExecuteProcess(
        cmd=['bash', '-c', 'until ros2 topic info /map > /dev/null 2>&1; do sleep 0.5; done && echo "[Event] /map topic is ready"'],
        name='wait_for_map_topic',
        output='screen',
    )
    
    # 当/map话题准备好后（wait_for_map进程退出），启动exploration节点
    delayed_nodes_start = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map,
            on_exit=[
                LogInfo(msg='[Event] SLAM ready, starting exploration and map saver nodes...'),
                exploration_node,
                map_saver_node,
            ]
        )
    )
    
    # ===== 信息提示 =====
    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            '🗺️  自主探索建图 + 自动保存\n',
            '=' * 80, '\n',
            '\n',
            '📋 启动流程:\n',
            '  1. 启动Gazebo仿真环境 (使用默认配置)\n',
            '  2. 启动RTABMap SLAM建图\n',
            '  3. 启动Nav2导航系统\n',
            '  4. 启动自主探索节点 (日志级别: WARN)\n',
            '  5. 监听探索完成信号\n',
            '  6. 自动保存地图到地图库\n',
            '\n',
            '🔍 监控话题:\n',
            '  - /map: 实时地图\n',
            '  - /exploration/status: 探索状态\n',
            '  - /exploration/complete: 探索完成信号 (Bool)\n',
            '  - /goal_pose: 当前导航目标\n',
            '\n',
            '💾 地图保存:\n',
            '  - 地图名称: ', map_name, '\n',
            '  - 描述: ', description, '\n',
            '  - 标签: ', tags, '\n',
            '  - 保存路径: ~/lododo_bot/maps/\n',
            '  - 自动保存: ', auto_save, '\n',
            '\n',
            '✅ 完成条件:\n',
            '  - 地图完成度达到阈值(默认78%)\n',
            '  - 或无更多可探索的边界区域\n',
            '  - 地图将自动保存到地图库\n',
            '\n',
            '🎮 手动控制:\n',
            '  - 停止探索: ros2 service call /exploration/stop std_srvs/srv/Trigger\n',
            '  - 手动保存: ros2 service call /map_library/save_current_map std_srvs/srv/Trigger\n',
            '\n',
            '💡 提示:\n',
            '  - 探索节点日志级别已设置为WARN，减少输出干扰\n',
            '  - 主要关注map_saver_node的日志信息\n',
            '\n',
            '=' * 80, '\n',
        ]
    )
    
    return LaunchDescription([
        # 参数
        exploration_radius_arg,
        completion_threshold_arg,
        map_name_arg,
        description_arg,
        tags_arg,
        auto_save_arg,
        
        # 信息
        info_msg,
        
        # 仿真+Nav2（立即启动）
        simulation_nav2_launch,
        
        # 等待关键话题准备好
        wait_for_map,
        
        # 事件处理器：当/map话题就绪后启动探索节点
        delayed_nodes_start,
    ])
