#!/usr/bin/env python3
"""
导航仿真完整启动文件 / Navigation Simulation Complete Launch File
组合Gazebo仿真环境和Nav2导航栈 / Combines Gazebo simulation environment and Nav2 navigation stack

用户统一入口 / User unified entry point:
  ros2 launch bot_bringup simulation_nav2.launch.py

启动架构 / Launch Architecture (使用墙上时钟延迟 / using wall-clock delays):
  simulation_nav2.launch.py (本文件 / this file)
    ├── [0s]  simulation_gazebo.launch.py
    │         ├── gazebo.launch.py (bot_gazebo)
    │         ├── perception.launch.py (bot_perception)
    │         └── controllers (bot_control)
    ├── [15s] SLAM Toolbox (async mode)
    │         └── slam_toolbox + map_saver + lifecycle_manager_slam
    ├── [25s] Nav2 Navigation Core
    │         └── navigation_launch.py (planner, controller, costmaps, etc.)
    └── [35s] RViz (nav2.rviz)

关键：使用 bash sleep 确保墙上时钟延迟，而不是仿真时间
Key: Using bash sleep ensures wall-clock delays, not simulation time

使用示例 / Usage Examples:
  # SLAM模式（默认）/ SLAM mode (default)
  ros2 launch bot_bringup simulation_nav2.launch.py

  # 定位模式（需要地图）/ Localization mode (requires map)
  ros2 launch bot_bringup simulation_nav2.launch.py slam:=False map:=/path/to/map.yaml

  # 自定义世界 / Custom world
  ros2 launch bot_bringup simulation_nav2.launch.py world:=empty_test.world
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    ExecuteProcess,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # ========================================================================
    # 获取包路径 / Get package paths
    # ========================================================================
    bot_bringup_dir = get_package_share_directory('bot_bringup')
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 默认配置 / Default configurations
    default_world = 'simple_obstacles.world'
    default_rviz_config = os.path.join(bot_navigation_dir, 'config', 'nav2.rviz')
    default_nav2_params = os.path.join(bot_navigation_dir, 'config', 'nav2', 'nav2_params.yaml')
    
    # ========================================================================
    # 声明启动参数 / Declare launch arguments
    # ========================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo world file / Gazebo世界文件'
    )
    
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='True',
        description='Run SLAM instead of localization / 运行SLAM而不是定位'
    )
    
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map yaml file / 地图文件路径'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params', default_value=default_nav2_params,
        description='Path to Nav2 parameters file / Nav2参数文件路径'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz / 启动RViz'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='RViz config file path / RViz配置文件路径'
    )
    
    # ========================================================================
    # 获取参数值 / Get parameter values
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # ========================================================================
    # 1. Gazebo仿真启动（立即启动）
    # 1. Gazebo simulation launch (immediate)
    # ========================================================================
    simulation_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_bringup_dir, 'launch', 'simulation_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
            'world': world,
        }.items()
    )
    
    # ========================================================================
    # 2. SLAM Toolbox 启动（延迟15秒，使用bash sleep确保墙上时间）
    # 2. SLAM Toolbox launch (delayed 15s, using bash sleep for wall-clock)
    # 先启动 SLAM，让它建立 map->odom TF
    # Start SLAM first to establish map->odom TF
    # ========================================================================
    delayed_slam_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'echo "[SLAM] Waiting 15 seconds for Gazebo to initialize..." && '
            f'sleep 15 && '
            f'echo "[SLAM] Starting SLAM Toolbox (async mode)..." && '
            f'source /home/hurry/lododo_bot/install/setup.bash && '
            f'ros2 launch slam_toolbox online_async_launch.py '
            f'use_sim_time:=true '
            f'slam_params_file:={default_nav2_params}'
        ],
        output='screen'
    )
    
    # ========================================================================
    # 3. Nav2 导航核心启动（延迟后等待TF可用）
    # 3. Nav2 Navigation Core launch (delayed + wait for TF)
    # 等待 SLAM 建立 TF 后再启动导航节点
    # Wait for SLAM to establish TF before starting navigation nodes
    # ========================================================================
    delayed_nav2_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "[Nav2] Waiting 35 seconds for SLAM to establish TF..." && '
            'sleep 35 && '
            'echo "[Nav2] Starting navigation core nodes..." && '
            f'source /home/hurry/lododo_bot/install/setup.bash && '
            f'ros2 launch nav2_bringup navigation_launch.py '
            f'use_sim_time:=true '
            f'autostart:=true '
            f'params_file:={default_nav2_params}'
        ],
        output='screen'
    )
    
    # ========================================================================
    # 4. RViz启动（延迟45秒）
    # 4. RViz launch (delayed 45s)
    # ========================================================================
    delayed_rviz_cmd = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=[
            'bash', '-c',
            'echo "[RViz] Waiting 45 seconds for Nav2 to initialize..." && '
            'sleep 45 && '
            'echo "[RViz] Starting RViz..." && '
            f'source /home/hurry/lododo_bot/install/setup.bash && '
            f'ros2 run rviz2 rviz2 -d {default_rviz_config} --ros-args -p use_sim_time:=true'
        ],
        output='screen'
    )
    
    # ========================================================================
    # 返回启动描述 / Return launch description
    # ========================================================================
    return LaunchDescription([
        # 参数声明 / Argument declarations
        declare_use_sim_time,
        declare_world,
        declare_slam,
        declare_map,
        declare_nav2_params,
        declare_use_rviz,
        declare_rviz_config,
        
        # 日志信息 / Log info
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting Navigation Simulation...'),
        LogInfo(msg='  1. Gazebo:    Starting now (0s)'),
        LogInfo(msg='  2. SLAM:      Will start in 15 seconds'),
        LogInfo(msg='  3. Nav2:      Will start in 35 seconds'),
        LogInfo(msg='  4. RViz:      Will start in 45 seconds'),
        LogInfo(msg='========================================'),
        
        # 启动项 / Launch items
        simulation_gazebo_launch,    # 0s  - 立即启动 / Start immediately
        delayed_slam_cmd,            # 15s - SLAM Toolbox
        delayed_nav2_cmd,            # 25s - Nav2 导航核心
        delayed_rviz_cmd,            # 35s - RViz
    ])
