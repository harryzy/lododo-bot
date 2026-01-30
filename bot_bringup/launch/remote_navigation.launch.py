#!/usr/bin/env python3
"""
remote_navigation.launch.py - PC端远程定位导航启动文件

功能 / Features:
  - PC端计算节点启动（用于分布式部署）
  - 硬件层在树莓派运行，PC运行计算密集型任务
  - 定位导航模式（需要已有地图）
  
组件（PC端）/ Components (PC Side):
  1. EKF传感器融合 - 融合轮式里程计和IMU
  2. RTABMap定位 - 使用已有地图进行重定位
  3. Nav2导航栈 - 路径规划和避障
  4. RViz可视化（可选）
  
架构 / Architecture:
  ┌─────────────────────┐  WiFi/Ethernet  ┌──────────────────────┐
  │   树莓派（Hardware） │ ←────────────→  │    PC（Computation）  │
  │ - omni_hardware     │                 │ - EKF Fusion         │
  │ - ybimu_driver      │                 │ - RTABMap Localize   │
  │ - imu_filter        │                 │ - Nav2               │
  │ - astra_camera      │                 │ - RViz               │
  └─────────────────────┘                 └──────────────────────┘

前置条件 / Prerequisites:
  1. 树莓派硬件层已启动（需要先运行）
  2. 已有地图文件存在于 workspace/maps/<map_name>/
  3. ROS_DOMAIN_ID 在PC和树莓派端必须一致
  4. 网络配置正确（Cyclone DDS配置、socket缓冲区）

使用方法 / Usage:
  # 标准导航模式
  ros2 launch bot_bringup remote_navigation.launch.py map_name:=office_floor1
  
  # 启动RViz可视化
  ros2 launch bot_bringup remote_navigation.launch.py \
    map_name:=office_floor1 \
    use_rviz:=true

导航测试 / Navigation Test:
  # 通过RViz设置目标点：2D Goal Pose
  # 或使用命令行：
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"

注意事项 / Notes:
  - 必须先启动树莓派端的硬件驱动
  - use_sim_time固定为false（真机模式）
  - 首次定位可能需要在RViz中设置初始位姿（2D Pose Estimate）
  - 确保PC和树莓派时钟同步（使用chrony）

Author: LeKiwi Bot Development Team
Date: 2026-01-29
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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def launch_setup(context: LaunchContext, *args, **kwargs):
    """
    Dynamic launch setup function / 动态启动设置函数
    """
    
    # ==========================================================================
    # Get Launch Configurations / 获取启动配置
    # ==========================================================================
    map_name_str = LaunchConfiguration('map_name').perform(context)
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    log_level_str = LaunchConfiguration('log_level').perform(context)
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_description = get_package_share_directory('bot_description')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    pkg_bot_slam = get_package_share_directory('bot_slam')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # 获取工作空间根目录（用于地图路径）
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_bot_navigation))))
    maps_dir = os.path.join(workspace_root, 'maps')
    
    # ==========================================================================
    # Configuration Files / 配置文件
    # ==========================================================================
    ekf_config = os.path.join(pkg_bot_navigation, 'config', 'localization', 'robot_localization_rtabmap.yaml')
    rtabmap_config = os.path.join(pkg_bot_slam, 'config', 'slam', 'rtabmap_real_minimal.yaml')
    map_file = os.path.join(maps_dir, map_name_str, 'map.yaml')
    
    # 验证地图文件存在
    if not os.path.exists(map_file):
        raise FileNotFoundError(f"Map file not found: {map_file}")
    
    # ==========================================================================
    # Parameter Substitutions / 参数替换
    # ==========================================================================
    param_substitutions = {'use_sim_time': 'false'}
    
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
    # Robot Description / 机器人模型
    # ==========================================================================
    # Robot State Publisher（发布简化URDF的TF）
    urdf_path = PathJoinSubstitution([
        FindPackageShare('bot_description'),
        'urdf',
        'lekiwi_bot_simple.urdf.xacro'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[
            {
                'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str),
                'use_sim_time': False,
            }
        ],
    )
    
    # Joint State Publisher（发布wheel joints的TF状态）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[{'use_sim_time': False}],
    )
    
    # ==========================================================================
    # PHASE 1: EKF Sensor Fusion / 第一阶段：EKF传感器融合
    # ==========================================================================
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[configured_ekf_config],
    )
    
    # ==========================================================================
    # PHASE 2: RTABMap Localization / 第二阶段：RTABMap定位
    # ==========================================================================
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
            {
                'qos': 1,  # RELIABLE匹配EKF
                'qos_odom': 1,  # RELIABLE匹配EKF
                'qos_image': 1,
                'qos_camera_info': 1,
            }
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/odometry/filtered'),
            ('grid_map', '/map'),  # ← CRITICAL: Output occupancy grid for Nav2
        ]
    )
    
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
    # PHASE 4: Nav2 Navigation / 第四阶段：Nav2导航
    # ==========================================================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': configured_nav2_params,
            'map': map_file,
        }.items()
    )
    
    # 使用事件驱动启动：RTABMap启动后再启动Nav2
    nav2_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ekf_filter,
            on_start=[nav2_bringup]
        )
    )
    
    # ==========================================================================
    # PHASE 5: RViz Visualization / 第五阶段：RViz可视化
    # ==========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # ==========================================================================
    # Event-Driven Launch Sequence / 事件驱动启动序列
    # ==========================================================================
    # Event 1: EKF启动完成 → 启动RTABMap定位 + 深度处理
    event_ekf_started = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_filter,
            on_start=[
                LogInfo(msg='[Event] EKF filter started!'),
                LogInfo(msg='[Phase 2] Starting RTABMap Localization and depth processing...'),
                rtabmap_localization,  # 定位节点
                point_cloud_xyz,       # 深度转点云
                obstacles_detection,   # 障碍物检测
            ]
        )
    )
    
    # Event 2: RTABMap定位启动完成 → 启动Nav2
    event_rtabmap_localization_started = RegisterEventHandler(
        OnProcessStart(
            target_action=rtabmap_localization,
            on_start=[
                LogInfo(msg='[Event] RTABMap Localization started!'),
                LogInfo(msg='[Phase 3] Starting Nav2 navigation stack...'),
                nav2_bringup,
            ]
        )
    )
    
    # ==========================================================================
    # Launch Actions / 启动动作
    # ==========================================================================
    return [
        # Phase 0: Robot Model
        LogInfo(msg='[Phase 0] Loading robot model...'),
        robot_state_publisher,
        joint_state_publisher,
        
        # Phase 1: Start EKF immediately
        LogInfo(msg='[Phase 1] Starting EKF sensor fusion...'),
        ekf_filter,
        
        # Event handlers (define subsequent startup sequence)
        event_ekf_started,
        event_rtabmap_localization_started,
        
        # RViz独立启动
        rviz_node,
    ]


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    # ==========================================================================
    # Declare Launch Arguments / 声明启动参数
    # ==========================================================================
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='Map name for localization (required) / 定位使用的地图名称（必填）'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for navigation visualization / 启动RViz2用于导航可视化'
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
    
    # ==========================================================================
    # Launch Description / 启动描述
    # ==========================================================================
    return LaunchDescription([
        # 启动信息
        LogInfo(msg='='*70),
        LogInfo(msg='Remote Navigation Mode (PC Side)'),
        LogInfo(msg='远程导航模式（PC端）'),
        LogInfo(msg='='*70),
        LogInfo(msg='Purpose / 用途: Computation nodes for distributed deployment'),
        LogInfo(msg='            分布式部署的计算节点'),
        LogInfo(msg='='*70),
        LogInfo(msg='Prerequisites / 前置条件:'),
        LogInfo(msg='  1. Hardware layer must be running on Raspberry Pi'),
        LogInfo(msg='     硬件层必须在树莓派上运行'),
        LogInfo(msg='  2. Map file must exist: workspace/maps/<map_name>/'),
        LogInfo(msg='     地图文件必须存在：workspace/maps/<map_name>/'),
        LogInfo(msg='  3. ROS_DOMAIN_ID must match between PC and Pi'),
        LogInfo(msg='     ROS_DOMAIN_ID在PC和树莓派上必须匹配'),
        LogInfo(msg='='*70),
        
        # 参数声明
        declare_map_name,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        declare_log_level,
        
        # 动态启动配置
        OpaqueFunction(function=launch_setup),
    ])
