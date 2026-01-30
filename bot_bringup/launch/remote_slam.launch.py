#!/usr/bin/env python3
"""
remote_slam.launch.py - PC端远程SLAM建图启动文件

功能 / Features:
  - PC端计算节点启动（用于分布式部署）
  - 硬件层在树莓派运行，PC运行SLAM建图任务
  - SLAM建图模式（创建新地图）
  
组件（PC端）/ Components (PC Side):
  1. EKF传感器融合 - 融合轮式里程计和IMU
  2. RTABMap SLAM - RGB-D视觉SLAM建图
  3. Nav2导航栈（可选）- 建图期间的路径规划
  4. RViz可视化（推荐）
  
架构 / Architecture:
  ┌─────────────────────┐  WiFi/Ethernet  ┌──────────────────────┐
  │   树莓派（Hardware） │ ←────────────→  │    PC（Computation）  │
  │ - omni_hardware     │                 │ - EKF Fusion         │
  │ - ybimu_driver      │                 │ - RTABMap SLAM       │
  │ - imu_filter        │                 │ - Nav2 (Optional)    │
  │ - astra_camera      │                 │ - RViz               │
  └─────────────────────┘                 └──────────────────────┘

前置条件 / Prerequisites:
  1. 树莓派硬件层已启动（需要先运行）
  2. ROS_DOMAIN_ID 在PC和树莓派端必须一致
  3. 网络配置正确（Cyclone DDS配置、socket缓冲区）
  4. 建议使用有线以太网（WiFi可能延迟较大）

使用方法 / Usage:
  # 标准SLAM建图
  ros2 launch bot_bringup remote_slam.launch.py
  
  # SLAM建图 + 启用Nav2（建图期间可导航）
  ros2 launch bot_bringup remote_slam.launch.py enable_nav:=true
  
  # 启动RViz可视化（推荐）
  ros2 launch bot_bringup remote_slam.launch.py use_rviz:=true

建图技巧 / Mapping Tips:
  1. 缓慢驾驶机器人（使用手柄或键盘遥控）
  2. 避免快速旋转，防止运动模糊
  3. 覆盖所有需要建图的区域
  4. 重访起点以闭合回环
  5. 确保光照充足，避免逆光

保存地图 / Save Map:
  # 方法1: RTABMap自动保存（关闭节点时）
  Ctrl+C 关闭launch文件
  
  # 方法2: 手动触发保存
  ros2 service call /rtabmap/save_map std_srvs/srv/Empty

注意事项 / Notes:
  - 必须先启动树莓派端的硬件驱动
  - use_sim_time固定为false（真机模式）
  - SLAM模式会清空已有数据库（--delete_db_on_start）
  - 建议关闭Nav2以获得更好的SLAM性能
  - 确保PC和树莓派时钟同步（使用chrony）

Author: LeKiwi Bot Development Team
Date: 2026-01-29
"""

import os

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
    enable_nav_str = LaunchConfiguration('enable_nav').perform(context)
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
    
    # ==========================================================================
    # Configuration Files / 配置文件
    # ==========================================================================
    ekf_config = os.path.join(pkg_bot_navigation, 'config', 'localization', 'robot_localization_rtabmap.yaml')
    rtabmap_config = os.path.join(pkg_bot_slam, 'config', 'slam', 'rtabmap_real_minimal.yaml')
    
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
    
    # Static TF: map → odom (SLAM初始化回退方案)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_fallback',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            '--ros-args', '--log-level', log_level_str
        ],
        parameters=[{'use_sim_time': False}],
    )
    
    # ==========================================================================
    # PHASE 2: RTABMap SLAM / 第二阶段：RTABMap SLAM
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
            {
                'delete_db_on_start': True,  # 删除旧数据库，开始新地图
                # ⚠️ NOTE: subscribe_odom no longer needed in parameters
                # It's now controlled by YAML config (without odom_frame_id set)
                # 注意：subscribe_odom不再需要在parameters中设置
                # 现在由YAML配置控制（不设置odom_frame_id）
                
                # ⚠️ WiFi分布式环境：QoS配置
                # FastDDS在混合QoS模式下有Bug，必须全用RELIABLE匹配相机发布者
                'qos': 1,  # RELIABLE匹配EKF
                'qos_odom': 1,  # RELIABLE匹配EKF  
                'qos_image': 1,  # RELIABLE匹配Astra相机 (BEST_EFFORT在FastDDS下无法接收)
                'qos_camera_info': 1,  # RELIABLE匹配Astra相机
                
                # ⚠️ 时间同步容忍度（WiFi环境+代理优化后已改善）
                'approx_sync_max_interval': 0.2,  # 允许200ms时间戳差异
            }
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/odometry/filtered'),
            ('grid_map', '/map'),  # ← CRITICAL: Output occupancy grid for Nav2 / 输出占据栅格给Nav2
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
    # PHASE 4: Nav2 Navigation (Optional) / 第四阶段：Nav2导航（可选）
    # ==========================================================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': configured_nav2_params,
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_nav'))
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
    # Event 1: EKF启动完成 → 启动RTABMap + 深度处理
    event_ekf_started = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_filter,
            on_start=[
                LogInfo(msg='[Event] EKF filter started!'),
                LogInfo(msg='[Phase 2] Starting static map→odom TF, RTABMap SLAM and depth processing...'),
                static_map_to_odom,    # Static TF fallback
                rtabmap_slam,          # SLAM节点
                point_cloud_xyz,       # 深度转点云
                obstacles_detection,   # 障碍物检测
            ]
        )
    )
    
    # Event 2: RTABMap SLAM启动完成 → 启动Nav2（如果启用）
    event_rtabmap_slam_started = RegisterEventHandler(
        OnProcessStart(
            target_action=rtabmap_slam,
            on_start=[
                LogInfo(msg='[Event] RTABMap SLAM started!'),
                LogInfo(msg='[Phase 3] Starting Nav2 navigation stack (if enabled)...'),
                nav2_bringup,
            ]
        ),
        condition=IfCondition(LaunchConfiguration('enable_nav'))
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
        event_rtabmap_slam_started,
        
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
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav',
        default_value='false',
        description='Enable Nav2 during SLAM (may affect mapping quality) / SLAM期间启用Nav2（可能影响建图质量）'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for SLAM visualization / 启动RViz2用于SLAM可视化'
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
        LogInfo(msg='Remote SLAM Mapping Mode (PC Side)'),
        LogInfo(msg='远程SLAM建图模式（PC端）'),
        LogInfo(msg='='*70),
        LogInfo(msg='Purpose / 用途: SLAM mapping nodes for distributed deployment'),
        LogInfo(msg='            分布式部署的SLAM建图节点'),
        LogInfo(msg='='*70),
        LogInfo(msg='Prerequisites / 前置条件:'),
        LogInfo(msg='  1. Hardware layer must be running on Raspberry Pi'),
        LogInfo(msg='     硬件层必须在树莓派上运行'),
        LogInfo(msg='  2. ROS_DOMAIN_ID must match between PC and Pi'),
        LogInfo(msg='     ROS_DOMAIN_ID在PC和树莓派上必须匹配'),
        LogInfo(msg='  3. Camera topics must be available from Raspberry Pi'),
        LogInfo(msg='     相机话题必须从树莓派可用'),
        LogInfo(msg='='*70),
        LogInfo(msg='Mapping Tips / 建图技巧:'),
        LogInfo(msg='  1. Drive slowly for better feature tracking'),
        LogInfo(msg='     缓慢驾驶以获得更好的特征跟踪'),
        LogInfo(msg='  2. Avoid fast rotation to prevent motion blur'),
        LogInfo(msg='     避免快速旋转以防止运动模糊'),
        LogInfo(msg='  3. Revisit starting point for loop closure'),
        LogInfo(msg='     重访起点以闭合回环'),
        LogInfo(msg='='*70),
        
        # 参数声明
        declare_enable_nav,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        declare_log_level,
        
        # 动态启动配置
        OpaqueFunction(function=launch_setup),
    ])
