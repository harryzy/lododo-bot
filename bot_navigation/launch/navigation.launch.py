#!/usr/bin/env python3
"""
Nav2 导航栈启动文件 / Nav2 Navigation Stack Launch File
只负责启动 Nav2 相关节点 / Only responsible for launching Nav2 related nodes

注意 / Note:
  此文件不启动 Gazebo 或仿真环境 / This file does NOT launch Gazebo or simulation
  完整导航仿真请使用 / For full navigation simulation, use:
    ros2 launch bot_bringup simulation_nav2.launch.py
    
修改说明 / Modification Notes:
  - 使用 async slam_toolbox 而不是 sync 模式，对时间同步更宽容
    Use async slam_toolbox instead of sync mode, more tolerant to time synchronization
  - Nav2 导航节点延迟启动，等待 slam_toolbox 建立 TF
    Nav2 navigation nodes delayed to wait for slam_toolbox to establish TF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ========================================================================
    # 获取包路径 / Get package paths
    # ========================================================================
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    
    # 默认参数文件路径 / Default parameter file path
    default_nav2_params = os.path.join(
        bot_navigation_dir, 'config', 'nav2', 'nav2_params.yaml'
    )
    
    # 默认RViz配置 / Default RViz config
    default_rviz_config = os.path.join(bot_navigation_dir, 'config', 'nav2.rviz')
    
    # ========================================================================
    # 声明启动参数 / Declare launch arguments
    # ========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 stack / 自动启动Nav2'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Path to Nav2 parameters file / Nav2参数文件路径'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file (empty for SLAM mode) / 地图文件路径（空则使用SLAM模式）'
    )
    
    # 默认使用SLAM模式（不需要预先地图）/ Default to SLAM mode (no map required)
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Run SLAM instead of localization / 运行SLAM而不是定位'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',  # 默认不启动，由上层控制 / Default false, controlled by upper layer
        description='Launch navigation RViz / 启动导航RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz config file path / RViz配置文件路径'
    )
    
    # ========================================================================
    # 获取参数值 / Get parameter values
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # ========================================================================
    # 参数替换配置 / Parameter substitution configuration
    # ========================================================================
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file
    }
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True
    )
    
    # ========================================================================
    # SLAM Toolbox (异步模式 - 对时间同步更宽容)
    # SLAM Toolbox (async mode - more tolerant to time synchronization)
    # ========================================================================
    slam_toolbox_node = Node(
        condition=IfCondition(slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # 使用 async 而不是 sync / Use async instead of sync
        name='slam_toolbox',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )
    
    # SLAM 生命周期管理器 / SLAM lifecycle manager
    slam_lifecycle_manager = Node(
        condition=IfCondition(slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_saver']
        }],
    )
    
    # Map Saver 节点 / Map Saver node
    map_saver_node = Node(
        condition=IfCondition(slam),
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )
    
    # ========================================================================
    # 定位模式（非SLAM）- 使用 nav2_bringup 的 localization_launch
    # Localization mode (non-SLAM) - use nav2_bringup's localization_launch
    # ========================================================================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_file,
        }.items()
    )
    
    # ========================================================================
    # Nav2 导航核心节点
    # Nav2 navigation core nodes
    # 注意：在仿真模式下，建议使用 simulation_nav2.launch.py 来控制启动顺序
    # Note: In simulation mode, use simulation_nav2.launch.py to control launch order
    # ========================================================================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )
    
    # ========================================================================
    # RViz 节点（导航专用配置，可选）/ RViz node (navigation config, optional)
    # ========================================================================
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ========================================================================
    # 返回启动描述 / Return launch description
    # ========================================================================
    return LaunchDescription([
        # 参数声明 / Argument declarations
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        map_arg,
        slam_arg,
        use_rviz_arg,
        rviz_config_arg,
        
        # SLAM 相关 / SLAM related
        slam_toolbox_node,
        map_saver_node,
        slam_lifecycle_manager,
        
        # 定位模式（非SLAM时）/ Localization mode (when not SLAM)
        localization_launch,
        
        # 导航核心 / Navigation core
        navigation_launch,
        
        # RViz（可选）/ RViz (optional)
        rviz_node,
    ])
