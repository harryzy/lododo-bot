#!/usr/bin/env python3
"""
pc_computation.launch.py - PC端计算节点启动文件（分布式部署）
PC-side Computation Nodes for Distributed Deployment

功能 / Features:
  - 在PC端运行所有计算密集型节点
  - 树莓派只需运行hardware_bringup.launch.py
  - 通过CycloneDDS网络通信（ROS_DOMAIN_ID=42）

架构 / Architecture:
  树莓派端（192.168.2.120）:
    - hardware_bringup.launch.py
      ↓ 发布: /wheel/odom, /imu/data, /camera/*
  
  PC端（本文件）:
    - robot_state_publisher (URDF → TF)
    - robot_localization (EKF融合)
    - RTABMap SLAM (建图/定位)
    - Nav2导航栈
    - RViz可视化

使用方法 / Usage:
  # 1. 先在树莓派启动硬件
  ssh lododo@192.168.2.120
  ros2 launch bot_hardware hardware_bringup.launch.py
  
  # 2. 在PC启动计算节点
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ros2 launch bot_bringup pc_computation.launch.py slam:=true
  
  # 定位模式（需要已有地图）
  ros2 launch bot_bringup pc_computation.launch.py \
    slam:=false map_name:=office_floor1

注意事项 / Notes:
  - 确保PC和树莓派ROS_DOMAIN_ID=42
  - 确保两端时钟同步（NTP）
  - use_sim_time固定为false
  
Author: LeKiwi Bot Team
Date: 2026-02-03
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成PC端计算节点Launch描述"""
    
    # ============ 包路径 / Package Paths ============
    bot_description_dir = get_package_share_directory('bot_description')
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    bot_bringup_dir = get_package_share_directory('bot_bringup')
    
    # URDF文件路径（简化模型）
    urdf_file = os.path.join(bot_description_dir, 'urdf', 'lekiwi_bot_simple.urdf.xacro')
    
    # ============ Launch参数 / Launch Arguments ============
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run in SLAM mode (true) or localization mode (false)'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='exploration_test',
        description='Map name for localization mode (ignored if slam:=true)'
    )
    
    enable_nav_arg = DeclareLaunchArgument(
        'enable_nav',
        default_value='true',
        description='Whether to start Nav2 navigation stack'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz visualization'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Log level for all nodes'
    )
    
    # ============ 配置文件路径 / Config Paths ============
    
    # EKF配置（真机专用）
    ekf_config = os.path.join(
        bot_navigation_dir,
        'config',
        'localization',
        'robot_localization.yaml'
    )
    
    # RTABMap配置
    rtabmap_config = os.path.join(
        bot_navigation_dir,
        'config',
        'rtabmap',
        'rtabmap_config.yaml'
    )
    
    # Nav2配置
    nav2_config = os.path.join(
        bot_navigation_dir,
        'config',
        'nav2',
        'nav2_params_imu.yaml'
    )
    
    # 地图路径（定位模式）
    map_path = PathJoinSubstitution([
        bot_bringup_dir,
        '..',
        '..',
        'maps',
        LaunchConfiguration('map_name'),
        'map.yaml'
    ])
    
    # ============ 节点定义 / Node Definitions ============
    
    # 1. robot_state_publisher - 发布URDF转TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': urdf_file}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # 2. static_map_to_odom - 在定位模式发布map→odom
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(LaunchConfiguration('slam'))
    )
    
    # 3. robot_localization - EKF传感器融合
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False}
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # 4. RTABMap SLAM/Localization
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_slam'),
                'launch',
                'rtabmap_rgbd_sync.launch.py'
            ])
        ]),
        launch_arguments={
            'slam_mode': LaunchConfiguration('slam'),
            'use_sim_time': 'false',
            'map_name': LaunchConfiguration('map_name'),
            'localization_map_path': map_path,
            'database_path': PathJoinSubstitution([
                bot_bringup_dir, '..', '..', 'maps',
                LaunchConfiguration('map_name'), 'rtabmap.db'
            ]),
            'rtabmap_viz': 'false',  # PC端可以开启,但默认关闭减少负载
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )
    
    # 5. Nav2导航栈
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_config,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_nav'))
    )
    
    # 6. RViz可视化
    rviz_config = os.path.join(
        bot_bringup_dir,
        'rviz',
        'real_robot.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # ============ 信息输出 / Info Log ============
    info_log = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            '      PC Computation Nodes for Distributed Deployment\n',
            '=' * 70, '\n',
            'Mode: ', LaunchConfiguration('slam'), ' (SLAM/Localization)\n',
            'Map: ', LaunchConfiguration('map_name'), '\n',
            'Nav2: ', LaunchConfiguration('enable_nav'), '\n',
            'RViz: ', LaunchConfiguration('use_rviz'), '\n',
            '\n',
            'Expected Topics from Raspberry Pi:\n',
            '  /wheel/odom        ← Wheel odometry\n',
            '  /imu/data          ← IMU raw data\n',
            '  /camera/color/*    ← RGB camera\n',
            '  /camera/depth/*    ← Depth camera\n',
            '\n',
            'Publishing Topics:\n',
            '  /odometry/filtered → Fused odometry (EKF)\n',
            '  /map               → Occupancy grid map\n',
            '  /cmd_vel           → Velocity commands to robot\n',
            '\n',
            '⚠️  Make sure Raspberry Pi is running:\n',
            '   ros2 launch bot_hardware hardware_bringup.launch.py\n',
            '=' * 70, '\n'
        ]
    )
    
    return LaunchDescription([
        # Launch参数
        slam_arg,
        map_name_arg,
        enable_nav_arg,
        use_rviz_arg,
        log_level_arg,
        
        # 信息输出
        info_log,
        
        # 节点
        robot_state_publisher,
        static_map_to_odom,
        ekf_node,
        rtabmap_launch,
        nav2_launch,
        rviz_node,
    ])
