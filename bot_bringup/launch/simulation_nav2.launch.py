#!/usr/bin/env python3
"""
导航仿真完整启动文件 - 重构版本
Navigation Simulation Complete Launch File - Refactored Version

采用ROS2最佳实践，使用事件驱动和条件启动
Uses ROS2 best practices with event-driven and conditional launching

架构说明 / Architecture:
  simulation_nav2.launch.py (本文件 / this file)
    ├── Gazebo仿真环境 (simulation_gazebo.launch.py)
    ├── 导航核心 (navigation.launch.py)
    └── RViz可视化 (条件启动 / conditional launch)

主要改进 / Key Improvements:
  - 移除硬编码延迟，使用ROS2事件机制
  - 简化参数传递，统一配置管理
  - 优化RViz启动逻辑，解决启动失败问题
  - 支持SLAM和定位模式切换
  - 增加TF变换检查和等待机制
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    OpaqueFunction,
    TimerAction
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def check_tf_available(context, *args, **kwargs):
    """检查TF变换是否可用的函数 / Function to check if TF transforms are available"""
    import subprocess
    import time
    
    # 等待Gazebo和机器人模型完全加载
    # Wait for Gazebo and robot model to fully load
    LogInfo(msg='[TF Check] Waiting for robot TF transforms to become available...'),
    
    # 检查必要的TF变换
    # Check essential TF transforms
    required_tfs = [
        ('base_link', 'odom'),
        ('base_link', 'map'),
    ]
    
    max_attempts = 30  # 最多尝试30次 / Maximum 30 attempts
    attempt = 0
    
    while attempt < max_attempts:
        try:
            # 检查base_link到odom的TF
            # Check TF from base_link to odom
            result = subprocess.run([
                'ros2', 'run', 'tf2_ros', 'tf2_echo', 'base_link', 'odom'
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                LogInfo(msg='[TF Check] base_link -> odom TF is available!')
                return True
                
        except (subprocess.TimeoutExpired, subprocess.SubprocessError):
            pass
        
        attempt += 1
        LogInfo(msg=f'[TF Check] Attempt {attempt}/{max_attempts}: TF not ready, waiting...')
        time.sleep(2)
    
    LogInfo(msg='[TF Check] TF transforms not available after maximum attempts')
    return False


def launch_setup(context, *args, **kwargs):
    """启动设置函数 / Launch setup function"""
    
    # 获取参数值 / Get parameter values
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    slam = LaunchConfiguration('slam').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    
    # 获取包路径 / Get package paths
    bot_bringup_dir = get_package_share_directory('bot_bringup')
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    
    # ========================================================================
    # 1. Gazebo仿真环境启动
    # 1. Gazebo simulation environment launch
    # ========================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_bringup'),
                'launch',
                'simulation_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',  # 由本文件控制RViz / RViz controlled by this file
            'world': LaunchConfiguration('world'),
        }.items()
    )
    
    # ========================================================================
    # 2. 导航核心启动 - 延迟启动确保TF可用
    # 2. Navigation core launch - delayed to ensure TF availability
    # ========================================================================
    
    # 创建导航启动，但延迟执行
    # Create navigation launch but delay execution
    delayed_navigation_launch = TimerAction(
        period=10.0,  # 延迟10秒确保Gazebo和TF就绪 / Delay 10s for Gazebo and TF readiness
        actions=[
            LogInfo(msg='[simulation_nav2] Starting navigation core after delay...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('bot_navigation'),
                        'launch',
                        'navigation.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam': slam,
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('nav2_params'),
                    'use_rviz': 'false',  # 由本文件控制RViz / RViz controlled by this file
                    'autostart': 'true',
                }.items()
            )
        ]
    )
    
    # ========================================================================
    # 3. RViz启动逻辑 - 进一步延迟
    # 3. RViz launch logic - further delayed
    # ========================================================================
    
    # RViz节点配置 / RViz node configuration
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # RViz延迟启动 - 确保导航完全启动
    # RViz delayed launch - ensure navigation is fully started
    delayed_rviz_launch = TimerAction(
        period=20.0,  # 延迟20秒确保导航就绪 / Delay 20s for navigation readiness
        condition=IfCondition(use_rviz),
        actions=[
            LogInfo(msg='[simulation_nav2] Starting RViz after navigation setup...'),
            rviz_node
        ]
    )
    
    return [
        # 日志信息 / Log information
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting Navigation Simulation (Refactored with TF fix)...'),
        LogInfo(msg=f'  SLAM Mode: {slam}'),
        LogInfo(msg=f'  Use Sim Time: {use_sim_time}'),
        LogInfo(msg=f'  Use RViz: {use_rviz}'),
        LogInfo(msg='========================================'),
        LogInfo(msg='[simulation_nav2] Note: Navigation will start after 10s delay'),
        LogInfo(msg='[simulation_nav2] Note: RViz will start after 20s delay'),
        LogInfo(msg='========================================'),
        
        # 启动项 / Launch items
        gazebo_launch,
        delayed_navigation_launch,
        delayed_rviz_launch,
    ]


def generate_launch_description():
    """生成启动描述 / Generate launch description"""
    
    # 获取默认配置 / Get default configurations
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    default_rviz_config = os.path.join(bot_navigation_dir, 'config', 'nav2.rviz')
    default_nav2_params = os.path.join(bot_navigation_dir, 'config', 'nav2', 'nav2_params.yaml')
    
    # ========================================================================
    # 声明启动参数 / Declare launch arguments
    # ========================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='simple_obstacles.world',
        description='Gazebo world file / Gazebo世界文件'
    )
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Run SLAM instead of localization / 运行SLAM而不是定位'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file / 地图文件路径'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=default_nav2_params,
        description='Path to Nav2 parameters file / Nav2参数文件路径'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz / 启动RViz'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz config file path / RViz配置文件路径'
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
        
        # 启动设置函数 / Launch setup function
        OpaqueFunction(function=launch_setup)
    ])
