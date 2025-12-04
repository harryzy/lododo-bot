#!/usr/bin/env python3
"""
Nav2 导航栈启动文件 - 重构简化版本
Nav2 Navigation Stack Launch File - Refactored Simplified Version

简化架构，专注核心导航功能
Simplified architecture, focused on core navigation functionality

功能 / Features:
  - SLAM Toolbox (异步模式)
  - Nav2 导航核心节点
  - 生命周期管理
  - 可选的地图服务器和定位
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """生成启动描述 / Generate launch description"""
    
    # 获取包路径 / Get package paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    
    # 默认参数文件 / Default parameter file
    default_nav2_params = os.path.join(
        bot_navigation_dir, 'config', 'nav2', 'nav2_params.yaml'
    )
    
    # ========================================================================
    # 声明启动参数 / Declare launch arguments
    # ========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Run SLAM instead of localization / 运行SLAM而不是定位'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file / 地图文件路径'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Path to Nav2 parameters file / Nav2参数文件路径'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes / 自动启动Nav2生命周期节点'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz / 启动RViz (由上层控制)'
    )
    
    # ========================================================================
    # 获取参数值 / Get parameter values
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # ========================================================================
    # 参数配置 / Parameter configuration
    # ========================================================================
    
    # 参数替换 / Parameter substitution
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # ========================================================================
    # SLAM 模式节点组 / SLAM mode node group
    # ========================================================================
    slam_group = GroupAction(
        condition=IfCondition(slam),
        actions=[
            # SLAM Toolbox (异步模式)
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
            ),
            
            # Map Saver
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver',
                output='screen',
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
            ),
            
        ]
    )
    
    # ========================================================================
    # 定位模式节点组 / Localization mode node group
    # ========================================================================
    localization_group = GroupAction(
        condition=UnlessCondition(slam),
        actions=[
            # 包含 nav2_bringup 的定位启动文件
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'map': map_file,
                }.items()
            )
        ]
    )
    
    # ========================================================================
    # Nav2 导航核心节点
    # Nav2 navigation core nodes
    # ========================================================================
    
    # 使用 nav2_bringup 的导航启动文件
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )
    
    # ========================================================================
    # RViz (可选，通常由上层控制)
    # RViz (optional, usually controlled by upper layer)
    # ========================================================================
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config', default=os.path.join(
            bot_navigation_dir, 'config', 'nav2.rviz'
        ))],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # ========================================================================
    # 返回启动描述 / Return launch description
    # ========================================================================
    return LaunchDescription([
        # 参数声明 / Argument declarations
        use_sim_time_arg,
        slam_arg,
        map_arg,
        params_file_arg,
        autostart_arg,
        use_rviz_arg,
        
        # 日志信息 / Log information
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting Navigation Stack (Refactored)...'),
        LogInfo(msg=[f'  Mode: ', PythonExpression(['"SLAM"' if slam else '"Localization"'])]),
        LogInfo(msg='========================================'),
        
        # 节点组 / Node groups
        slam_group,
        localization_group,
        
        # 导航核心 / Navigation core
        navigation_launch,
        
        # RViz (可选) / RViz (optional)
        rviz_node,
    ])


# 辅助函数 / Helper functions
def LogInfo(msg):
    """创建日志信息动作 / Create log info action"""
    from launch.actions import LogInfo as LogInfoAction
    return LogInfoAction(msg=msg)
