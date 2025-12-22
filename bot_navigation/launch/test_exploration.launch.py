#!/usr/bin/env python3
"""
自主探索建图测试启动文件
Autonomous Exploration Mapping Test Launch File

功能 / Features:
  - 启动完整的RTABMap + Nav2导航系统
  - 自动探索建图节点
  - 保证相机始终朝向运动方向
  
使用方法 / Usage:
  ros2 launch bot_navigation test_exploration.launch.py
  
完成后会自动停止
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
    
    # ===== 参数 =====
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='cafe',
        description='Gazebo world name / Gazebo世界名称'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz / 启动RViz'
    )
    
    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius',
        default_value='8.0',
        description='Exploration radius in meters / 探索半径(米)'
    )
    
    completion_threshold_arg = DeclareLaunchArgument(
        'completion_threshold',
        default_value='0.88',
        description='Map completion threshold (0-1) / 地图完成度阈值'
    )
    
    # ===== 获取配置 =====
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    exploration_radius = LaunchConfiguration('exploration_radius')
    completion_threshold = LaunchConfiguration('completion_threshold')
    
    # # ===== 启动RTABMap + Nav2系统 =====
    # rtabmap_nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             bot_bringup_dir,
    #             'launch',
    #             'simulation_nav2_rtabmap.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'world': world,
    #         'use_rviz': use_rviz,
    #         'use_rtabmapviz': 'false',  # 不需要RTABMap可视化
    #     }.items()
    # )
    
    # ===== 探索建图节点 =====
    exploration_node = Node(
        package='bot_navigation',
        executable='exploration_mapper',
        name='exploration_mapper',
        output='screen',
        parameters=[
            exploration_config,  # 加载配置文件
            {
                # launch参数可以覆盖配置文件
                'exploration_radius': exploration_radius,
                'map_completion_threshold': completion_threshold,
            }
        ],
        # 延迟启动，等待Nav2准备好
        # prefix=['bash -c "sleep 15 && $0 $@"'],
    )
    
    # ===== 信息提示 =====
    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            '🗺️  自主探索建图测试\n',
            '=' * 70, '\n',
            '策略说明:\n',
            '  - 相机(camera_optical_frame)的Z轴指向base_link的+X方向\n',
            '  - 机器人会自动旋转使+X方向对准目标，然后前进\n',
            '  - 这样保证相机60°FOV始终覆盖运动方向\n',
            '  - 基于Frontier算法自动探索未知区域\n',
            '\n',
            '监控话题:\n',
            '  - /map: 实时地图\n',
            '  - /exploration/complete: 探索完成信号\n',
            '  - /goal_pose: 当前导航目标\n',
            '\n',
            '完成条件:\n',
            '  - 地图完成度达到阈值(默认88%)\n',
            '  - 或无更多可探索的边界区域\n',
            '\n',
            '=' * 70, '\n',
        ]
    )
    
    return LaunchDescription([
        # 参数
        world_arg,
        use_rviz_arg,
        exploration_radius_arg,
        completion_threshold_arg,
        
        # 信息
        info_msg,
        
        # RTABMap + Nav2
        # rtabmap_nav2_launch,
        
        # 探索节点
        exploration_node,
    ])
