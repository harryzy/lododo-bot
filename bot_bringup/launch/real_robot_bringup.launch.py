#!/usr/bin/env python3
"""
real_robot_bringup.launch.py - 真机完整系统启动文件

功能 / Features:
  - 启动完整的真机系统（硬件层 + 传感器融合 + SLAM/定位 + 导航）
  - 支持SLAM建图模式和定位模式
  - 事件驱动启动序列，确保节点依赖顺序正确
  
组件 / Components:
  1. 硬件层（hardware_bringup）- 舵机 + 里程计 + IMU + 相机
  2. EKF传感器融合（robot_localization）- 融合轮式里程计和IMU
  3. RTABMap视觉SLAM - SLAM建图或定位模式
  4. Nav2导航栈 - 路径规划和避障
  5. RViz可视化（可选）
  
架构 / Architecture:
  Hardware → EKF Fusion → RTABMap → Nav2
      ↓          ↓            ↓        ↓
   /wheel/odom  /odometry    /map   /cmd_vel
   /imu/data    /filtered
   /camera/*

使用方法 / Usage:
  # SLAM建图模式（从空白环境开始）
  ros2 launch bot_bringup real_robot_bringup.launch.py slam:=true
  
  # 定位导航模式（需要已有地图）
  ros2 launch bot_bringup real_robot_bringup.launch.py \
    slam:=false \
    map_name:=office_floor1
  
  # 仅硬件+传感器融合测试（不启动导航）
  ros2 launch bot_bringup real_robot_bringup.launch.py \
    enable_nav:=false
  
  # 启动RViz可视化
  ros2 launch bot_bringup real_robot_bringup.launch.py \
    use_rviz:=true

注意事项 / Notes:
  - 使用事件驱动启动，禁止TimerAction延迟启动
  - 地图路径使用工作空间相对路径（maps/<map_name>）
  - 默认日志级别为warn，减少IO开销
  - 真机固定 use_sim_time:=false

Author: LeKiwi Bot Development Team
Date: 2026-01-21
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_setup(context: LaunchContext, *args, **kwargs):
    """
    Dynamic launch setup function / 动态启动设置函数
    Uses event-driven startup sequence for reliable node ordering
    使用事件驱动的启动顺序确保节点可靠启动
    """
    
    # ==========================================================================
    # Get Launch Configurations / 获取启动配置
    # ==========================================================================
    slam = LaunchConfiguration('slam')
    slam_str = LaunchConfiguration('slam').perform(context)
    map_name = LaunchConfiguration('map_name')
    map_name_str = LaunchConfiguration('map_name').perform(context)
    enable_nav = LaunchConfiguration('enable_nav')
    enable_nav_str = LaunchConfiguration('enable_nav').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    log_level = LaunchConfiguration('log_level')
    log_level_str = LaunchConfiguration('log_level').perform(context)
    config_file = LaunchConfiguration('config_file')
    config_file_str = LaunchConfiguration('config_file').perform(context)
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_bot_hardware = get_package_share_directory('bot_hardware')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    pkg_bot_slam = get_package_share_directory('bot_slam')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # 获取工作空间根目录（用于地图路径）
    # Get workspace root (for map paths)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_bot_bringup))))
    maps_dir = os.path.join(workspace_root, 'maps')
    
    # ==========================================================================
    # Configuration Files / 配置文件
    # ==========================================================================
    # EKF配置（真机版本，使用RTABMap兼容配置）
    ekf_config = os.path.join(pkg_bot_navigation, 'config', 'localization', 'robot_localization_rtabmap.yaml')
    
    # RTABMap配置
    rtabmap_config = os.path.join(pkg_bot_slam, 'config', 'slam', 'rtabmap.yaml')
    
    # 地图文件路径（定位模式使用）
    map_file = os.path.join(maps_dir, map_name_str, 'map.yaml') if map_name_str else ''
    
    # ==========================================================================
    # Parameter Substitutions / 参数替换
    # ==========================================================================
    param_substitutions = {
        'use_sim_time': 'false',  # 真机固定false
    }
    
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    configured_ekf_config = RewrittenYaml(
        source_file=ekf_config,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    configured_rtabmap_config = RewrittenYaml(
        source_file=rtabmap_config,
        root_key='',  # Empty: config file key 'rtabmap_slam' matches node name
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # ==========================================================================
    # PHASE 1: Hardware Layer / 第一阶段：硬件层
    # ==========================================================================
    
    # 硬件启动（包含舵机、IMU、相机）
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bot_hardware, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'config_file': config_file_str,
            'enable_servo': 'true',
            'enable_sensors': 'true',
            'enable_imu': 'true',
            'enable_camera': 'true',
            'publish_static_tf': 'true',
        }.items()
    )
    
    # ==========================================================================
    # PHASE 2: EKF Sensor Fusion / 第二阶段：EKF传感器融合
    # ==========================================================================
    
    # EKF定位节点
    # 融合轮式里程计（x, y, vx, vy）+ IMU（yaw, vyaw）
    # 输出：/odometry/filtered
    # TF：odom → base_link
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[configured_ekf_config],
    )
    
    # ==========================================================================
    # PHASE 3: RTABMap Visual SLAM / 第三阶段：RTABMap视觉SLAM
    # ==========================================================================
    
    # RTABMap SLAM节点（SLAM建图模式）
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[
            configured_rtabmap_config,
            {'delete_db_on_start': True}  # 删除旧数据库，开始新地图
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/odometry/filtered'),
        ],
        condition=IfCondition(slam)
    )
    
    # RTABMap定位节点（定位模式）
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[
            configured_rtabmap_config,
            {'Mem/IncrementalMemory': 'false'},  # 定位模式，不建图
            {'Mem/InitWMWithAllNodes': 'true'},  # 从已有地图初始化
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/odometry/filtered'),
        ],
        condition=UnlessCondition(slam)
    )
    
    # RTABMap可视化节点
    # rtabmap_viz = Node(
    #     package='rtabmap_viz',
    #     executable='rtabmap_viz',
    #     name='rtabmap_viz',
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', 'warn'],
    #     parameters=[configured_rtabmap_config],
    #     remappings=[
    #         ('rgb/image', '/camera/color/image_raw'),
    #         ('rgb/camera_info', '/camera/color/camera_info'),
    #         ('depth/image', '/camera/depth/image_raw'),
    #         ('odom', '/odometry/filtered'),
    #     ],
    #     condition=IfCondition(use_rviz)
    # )
    
    # ==========================================================================
    # PHASE 3: Depth Image Processing / 第三阶段：深度图像处理
    # ==========================================================================
    
    # 深度图转点云（用于Nav2障碍物检测）
    point_cloud_xyz = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'use_sim_time': False,
            'decimation': 4,
            'voxel_size': 0.05,
            'approx_sync': True,
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('cloud', '/camera/cloud'),
        ]
    )
    
    # 点云障碍物分割（地面/障碍物分离）
    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'wait_for_transform': 15.0,
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MinObstacleHeight': '0.1',
        }],
        remappings=[
            ('cloud', '/camera/cloud'),
            ('obstacles', '/camera/obstacles'),
            ('ground', '/camera/ground'),
        ]
    )
    
    # ==========================================================================
    # PHASE 4: Nav2 Navigation Stack / 第四阶段：Nav2导航栈
    # ==========================================================================
    
    # Nav2导航启动
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': configured_nav2_params,
            'autostart': 'true',
            'map': map_file,  # 定位模式需要地图
        }.items(),
        condition=IfCondition(enable_nav)
    )
    
    # ==========================================================================
    # PHASE 5: RViz Visualization / 第五阶段：RViz可视化
    # ==========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz)
    )
    
    # ==========================================================================
    # Event-Driven Launch Sequence / 事件驱动启动序列
    # 注意：OnProcessStart只能监听Node或ExecuteProcess，不能监听IncludeLaunchDescription
    # 因此我们监听具体的Node节点作为触发点
    # ==========================================================================
    
    # Event 1: EKF启动完成 → 启动RTABMap + 深度处理
    event_ekf_started = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_filter,
            on_start=[
                LogInfo(msg='[Event] EKF filter started!'),
                LogInfo(msg='[Phase 3] Starting RTABMap SLAM/Localization and depth processing...'),
                rtabmap_slam,          # SLAM模式（条件启动）
                rtabmap_localization,  # 定位模式（条件启动）
                # rtabmap_viz,           # 可视化
                point_cloud_xyz,       # 深度转点云
                obstacles_detection,   # 障碍物检测
            ]
        )
    )
    
    # Event 2: RTABMap SLAM启动完成 → 启动Nav2（SLAM模式）
    event_rtabmap_slam_started = RegisterEventHandler(
        OnProcessStart(
            target_action=rtabmap_slam,
            on_start=[
                LogInfo(msg='[Event] RTABMap SLAM started!'),
                LogInfo(msg='[Phase 4] Starting Nav2 navigation stack...'),
                nav2_bringup,
            ]
        )
    )
    
    # Event 3: RTABMap定位启动完成 → 启动Nav2（定位模式）
    event_rtabmap_localization_started = RegisterEventHandler(
        OnProcessStart(
            target_action=rtabmap_localization,
            on_start=[
                LogInfo(msg='[Event] RTABMap Localization started!'),
                LogInfo(msg='[Phase 4] Starting Nav2 navigation stack...'),
                nav2_bringup,
            ]
        )
    )
    
    # Event 4: Nav2启动完成 → 启动RViz（如果未在Phase3启动）
    # RViz已经在Phase3按需启动，这里只是预留扩展点
    
    # ==========================================================================
    # Return Launch Actions / 返回启动动作
    # ==========================================================================
    # Return Launch Actions / 返回启动动作
    # ==========================================================================
    return [
        # Startup info / 启动信息
        LogInfo(msg='='*70),
        LogInfo(msg='Real Robot Complete System Bringup'),
        LogInfo(msg='真机完整系统启动'),
        LogInfo(msg='='*70),
        LogInfo(msg=f'Mode / 模式: {"SLAM (Building Map)" if slam_str == "true" else "Localization (Navigation)"}'),
        LogInfo(msg=f'Map / 地图: {map_name_str if map_name_str else "N/A (SLAM mode)"}'),
        LogInfo(msg=f'Navigation / 导航: {"Enabled" if enable_nav_str == "true" else "Disabled"}'),
        LogInfo(msg=f'RViz: {"Enabled" if use_rviz_str == "true" else "Disabled"}'),
        LogInfo(msg=f'Log Level / 日志级别: {log_level_str}'),
        LogInfo(msg='='*70),
        LogInfo(msg='Architecture / 架构:'),
        LogInfo(msg='  [1] Hardware Layer (Servo + IMU + Camera)'),
        LogInfo(msg='  [2] EKF Fusion (Wheel Odom + IMU)'),
        LogInfo(msg='  [3] RTABMap (SLAM or Localization)'),
        LogInfo(msg='  [4] Nav2 (Path Planning + Obstacle Avoidance)'),
        LogInfo(msg='='*70),
        LogInfo(msg='Startup Sequence / 启动序列:'),
        LogInfo(msg='  → Hardware + EKF start immediately'),
        LogInfo(msg='  → RTABMap starts (on EKF ready)'),
        LogInfo(msg='  → Nav2 starts (on RTABMap ready)'),
        LogInfo(msg='='*70),
        
        # Phase 1+2: Start Hardware Layer + EKF immediately
        # 第一、二阶段：立即启动硬件层和EKF
        LogInfo(msg='[Phase 1] Starting hardware layer...'),
        hardware_bringup,
        LogInfo(msg='[Phase 2] Starting EKF sensor fusion...'),
        ekf_filter,
        
        # Event handlers (define the subsequent startup sequence)
        # 事件处理器（定义后续启动顺序）
        event_ekf_started,
        event_rtabmap_slam_started,
        event_rtabmap_localization_started,
        rviz_node,  # RViz独立启动，不依赖事件
    ]


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    # ==========================================================================
    # Package Directories (for default values) / 包目录（用于默认值）
    # ==========================================================================
    pkg_bot_hardware = get_package_share_directory('bot_hardware')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    # ==========================================================================
    # Declare Launch Arguments / 声明启动参数
    # ==========================================================================
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Use SLAM mode (true) or localization mode (false) / 使用SLAM建图模式（true）或定位模式（false）'
    )
    
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='Map name for localization mode (relative to workspace/maps/) / 定位模式使用的地图名称（相对于workspace/maps/）'
    )
    
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav',
        default_value='true',
        description='Enable Nav2 navigation stack / 启用Nav2导航栈'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz2 visualization / 启动RViz2可视化'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_bot_navigation, 'config', 'rviz', 'nav2_rtabmap.rviz'),
        description='RViz config file / RViz配置文件'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(pkg_bot_navigation, 'config', 'nav2', 'nav2_params_imu.yaml'),
        description='Nav2 parameters file / Nav2参数文件'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Log level (debug, info, warn, error, fatal) / 日志级别'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_bot_hardware, 'config', 'hardware_config.yaml'),
        description='Hardware configuration file / 硬件配置文件'
    )
    
    # ==========================================================================
    # Launch Description / 启动描述
    # ==========================================================================
    
    return LaunchDescription([
        declare_slam,
        declare_map_name,
        declare_enable_nav,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        declare_log_level,
        declare_config_file,
        OpaqueFunction(function=launch_setup)
    ])
