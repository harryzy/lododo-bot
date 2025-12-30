#!/usr/bin/env python3
"""
simulation_waypoint_recorder.launch.py - 仿真航点录制器（Service模式）

功能 / Features:
  - 启动仿真环境 + RTABMap定位 + Nav2导航系统
  - 启动航点录制服务（Service模式，非CLI）
  - 通过ROS2服务接口控制航点录制
  
架构 / Architecture:
  - Service模式：通过 /waypoint_recorder/* 服务控制
  - 适用于：程序化控制、MissionPlanner集成、自动化测试
  - 不适用于：手动交互录制（请使用CLI脚本）
  
使用方法 / Usage:
  # 启动仿真+录制服务
  ros2 launch bot_bringup simulation_waypoint_recorder.launch.py \
    rtabmap_db_path:=~/lododo_bot/maps/test_map/rtabmap.db
  
  # 通过服务录制航点
  ros2 service call /waypoint_recorder/record_current \\
    std_srvs/srv/Trigger
  
  # 保存航点
  ros2 service call /waypoint_recorder/save_waypoints \\
    bot_navigation_msgs/srv/WaypointControl \\
    "{filename: 'my_route.yaml'}"
  
  # 查看所有可用服务
  ros2 service list | grep waypoint_recorder

对比 / Comparison:
  - CLI模式（start_waypoint_recorder_cli.sh）：
    * 交互式命令行界面
    * 适合手动录制航点
    * 实时反馈和列表查看
    * 需要单独启动
  
  - Service模式（本launch文件）：
    * ROS2服务接口
    * 适合程序化控制
    * 可与MissionPlanner集成
    * 自动随系统启动
"""

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
import os


def generate_launch_description():
    """生成启动描述"""
    
    # ===== 包路径 =====
    bot_bringup_dir = FindPackageShare('bot_bringup')
    bot_navigation_dir = FindPackageShare('bot_navigation')
    
    # ===== 启动参数 =====
    
    # RTABMap数据库路径
    rtabmap_db_path_arg = DeclareLaunchArgument(
        'rtabmap_db_path',
        default_value=os.path.expanduser('~/lododo_bot/maps/test_map/rtabmap.db'),
        description='RTABMap database path for localization'
    )
    
    # 航点保存目录
    persistence_dir_arg = DeclareLaunchArgument(
        'persistence_dir',
        default_value=os.path.expanduser('~/lododo_bot/waypoints'),
        description='Directory to save recorded waypoints'
    )
    
    # 定位话题
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/rtabmap/localization_pose',
        description='Topic for robot pose (RTABMap localization or EKF odometry)'
    )
    
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
    persistence_dir = LaunchConfiguration('persistence_dir')
    pose_topic = LaunchConfiguration('pose_topic')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # ===== 启动定位导航系统 =====
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
    
    # ===== 航点录制服务节点 =====
    waypoint_recorder_node = Node(
        package='bot_navigation',
        executable='waypoint_service',  # Service模式的可执行文件
        name='waypoint_recorder',
        output='screen',
        parameters=[{
            'persistence_dir': persistence_dir,
            'pose_topic': pose_topic,
            'use_odom_backup': True,
        }],
        remappings=[
            # 确保使用正确的话题
            ('/pose', pose_topic),
        ],
    )
    
    return LaunchDescription([
        # 参数声明
        rtabmap_db_path_arg,
        persistence_dir_arg,
        pose_topic_arg,
        gui_arg,
        use_rviz_arg,
        
        # 启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Waypoint Recorder (Service Mode)'),
        LogInfo(msg='航点录制器（服务模式）'),
        LogInfo(msg='========================================'),
        LogInfo(msg=''),
        LogInfo(msg='可用服务 / Available Services:'),
        LogInfo(msg='  • /waypoint_recorder/record_current'),
        LogInfo(msg='      录制当前位置 (std_srvs/srv/Trigger)'),
        LogInfo(msg='  • /waypoint_recorder/save_waypoints'),
        LogInfo(msg='      保存航点到文件 (WaypointControl)'),
        LogInfo(msg='  • /waypoint_recorder/load_waypoints'),
        LogInfo(msg='      从文件加载航点 (WaypointControl)'),
        LogInfo(msg='  • /waypoint_recorder/list_waypoints'),
        LogInfo(msg='      列出所有航点 (std_srvs/srv/Trigger)'),
        LogInfo(msg='  • /waypoint_recorder/clear_waypoints'),
        LogInfo(msg='      清除所有航点 (std_srvs/srv/Trigger)'),
        LogInfo(msg=''),
        LogInfo(msg='使用示例 / Usage Examples:'),
        LogInfo(msg='  1. 录制当前位置:'),
        LogInfo(msg='     ros2 service call /waypoint_recorder/record_current \\'),
        LogInfo(msg='       std_srvs/srv/Trigger'),
        LogInfo(msg=''),
        LogInfo(msg='  2. 保存航点:'),
        LogInfo(msg='     ros2 service call /waypoint_recorder/save_waypoints \\'),
        LogInfo(msg='       bot_navigation_msgs/srv/WaypointControl \\'),
        LogInfo(msg='       "{filename: \'my_route.yaml\'}"'),
        LogInfo(msg=''),
        LogInfo(msg='注意 / Note:'),
        LogInfo(msg='  - 如需交互式CLI界面，请使用:'),
        LogInfo(msg='    ./scripts/start_waypoint_recorder_cli.sh'),
        LogInfo(msg=''),
        LogInfo(msg='========================================'),
        
        # 启动定位导航系统
        simulation_nav2_launch,
        
        # 启动航点录制服务
        waypoint_recorder_node,
    ])
