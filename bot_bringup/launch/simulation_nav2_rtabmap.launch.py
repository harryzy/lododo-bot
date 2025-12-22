#!/usr/bin/env python3
"""
RTABMap Visual SLAM Navigation Simulation Launch File
RTABMap 视觉SLAM 导航仿真启动文件

Complete simulation with RGB-D Visual SLAM for high-quality mapping
完整的 RGB-D 视觉SLAM 仿真，实现高质量建图

Key Difference from depth-to-laser approach / 相比深度转激光方案的主要区别:
    - Uses RTABMap RGB-D Visual SLAM (500+ visual features)
    - 使用 RTABMap RGB-D 视觉SLAM（500+视觉特征）
    - No depthimage_to_laserscan (eliminates 20-point limitation)
    - 无深度转激光（消除20点限制）
    - Superior loop closure detection (visual bag-of-words)
    - 优越的闭环检测（视觉词袋模型）
    - Stable mapping in featureless environments (texture vs geometry)
    - 在少特征环境中稳定建图（纹理 vs 几何）

Architecture / 架构:
    ┌───────────────────────────────────────────────────────────────┐
    │        simulation_nav2_rtabmap.launch.py                      │
    │                                                               │
    │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐       │
    │  │  ros2_ctrl  │───▶│ Wheel+Vis   │───▶│    Nav2     │       │
    │  │(wheel phys) │    │ +IMU (EKF)  │    │ (navigation)│       │
    │  │             │    │             │    │             │       │
    │  └─────────────┘    └─────────────┘    └─────────────┘       │
    │         │                  │                  │               │
    │         ▼                  ▼                  ▼               │
    │   /camera/*           /odom (fused)      /cmd_vel            │
    │   /joint_states       odom->base_link    /map                │
    │   /imu/data           map->odom          /map_cloud          │
    │                                                               │
    │  ┌─────────────────────────────────────────┐                 │
    │  │ RTABMap RGB-D Visual SLAM               │                 │
    │  │ - Visual odometry (ORB features)        │                 │
    │  │ - Loop closure (bag-of-words)           │                 │
    │  │ - 2D occupancy grid generation          │                 │
    │  └─────────────────────────────────────────┘                 │
    └───────────────────────────────────────────────────────────────┘

Startup Sequence (Event-Driven) / 启动顺序（事件驱动）:
    Phase 1: Gazebo Simulation
        - Start Gazebo server and client
        - Spawn robot model
        - Event: OnProcessStart(spawn_robot) -> Phase 2
    
    Phase 2: Robot Control (ros2_control)
        - Load joint_state_broadcaster
        - Load omni_wheel_controller
        - Start omni_controller_node (kinematics)
        - Event: OnProcessStart(omni_controller_node) -> Phase 3
    
    Phase 3: Visual SLAM & Fusion
        - Start RTABMap (visual odom + SLAM)
        - Start EKF fusion (wheel + visual + IMU)
        - Event: OnProcessStart(rtabmap) -> Phase 4
    
    Phase 4: Navigation
        - Start Nav2 navigation stack
        - Event: OnProcessStart(nav2) -> Phase 5
    
    Phase 5: Visualization (optional)
        - Start RViz with RTABMap config

QoS Compatibility / QoS 兼容性:
    - Wheel Odometry: Default QoS (Reliable, Volatile)
    - Visual Odometry: Default QoS (Reliable, Volatile)
    - IMU Data: Sensor Data QoS (Best Effort, Volatile)
    - Camera topics: Sensor Data QoS (Best Effort, Volatile)
    - TF: Transient Local for static, Volatile for dynamic
    - Map: Transient Local for latched delivery
"""

import os
from typing import List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    GroupAction,
    RegisterEventHandler,
    EmitEvent,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    world = LaunchConfiguration('world')
    world_str = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui')
    gui_str = LaunchConfiguration('gui').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    pkg_bot_slam = get_package_share_directory('bot_slam')
    pkg_bot_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bot_description = get_package_share_directory('bot_description')
    pkg_bot_control = get_package_share_directory('bot_control')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # ==========================================================================
    # Configuration Files / 配置文件
    # ==========================================================================
    # RTABMap configuration / RTABMap 配置
    rtabmap_config = os.path.join(pkg_bot_navigation, 'config', 'slam', 'rtabmap.yaml')
    
    # EKF configuration (with visual odometry) / EKF 配置（包含视觉里程计）
    ekf_config = os.path.join(pkg_bot_navigation, 'config', 'localization', 'robot_localization_rtabmap.yaml')
    
    # ==========================================================================
    # World File Resolution / 世界文件解析
    # ==========================================================================
    if not world_str or world_str == '':
        world_file = os.path.join(pkg_bot_gazebo, 'worlds', 'navigation_5x5_rgbd.world')
    elif not os.path.isabs(world_str):
        world_file = os.path.join(pkg_bot_gazebo, 'worlds', world_str)
    else:
        world_file = world_str
    
    # ==========================================================================
    # URDF Processing / URDF 处理
    # ==========================================================================
    urdf_file = os.path.join(pkg_bot_description, 'urdf', 'lekiwi_bot_imu_sim.xacro')
    
    # Controller configuration file / 控制器配置文件
    controller_config = os.path.join(pkg_bot_control, 'config', 'omni_wheel_controller.yaml')
    
    # Process xacro with publish_odom_tf:=false (EKF will publish odom->base_link)
    # 处理 xacro，publish_odom_tf:=false（EKF 将发布 odom->base_link）
    # Pass controller config file via xacro argument
    # 通过 xacro 参数传递控制器配置文件路径
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' publish_odom_tf:=false',  # Disable Gazebo odom TF
            ' controller_config_file:=', controller_config  # Pass controller config to xacro
        ]),
        value_type=str
    )
    
    # ==========================================================================
    # Parameter Substitutions / 参数替换
    # ==========================================================================
    param_substitutions = {
        'use_sim_time': use_sim_time_str,
    }
    
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # ==========================================================================
    # Set Gazebo model path / 设置 Gazebo 模型路径
    # ==========================================================================
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    share_dir = os.path.dirname(pkg_bot_description)
    if gazebo_model_path:
        full_model_path = share_dir + ':' + gazebo_model_path
    else:
        full_model_path = share_dir
    
    # ==========================================================================
    # PHASE 1: Gazebo Environment Nodes / 第一阶段：Gazebo 环境节点
    # ==========================================================================
    
    # Gazebo server / Gazebo 服务器
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': full_model_path}
    )
    
    # Gazebo client / Gazebo 客户端 (conditional on gui parameter)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': full_model_path},
        condition=IfCondition(gui)
    )
    
    # Robot state publisher / 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'robot_description': robot_description_content
        }]
    )
    
    # Spawn robot / 生成机器人
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'lekiwi_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )
    
    # ==========================================================================
    # PHASE 2: Robot Control with ros2_control / 第二阶段：基于ros2_control的机器人控制
    # ==========================================================================
    
    # Load ros2_control controllers spawner (joint_state_broadcaster)
    # 加载 ros2_control 控制器生成器（关节状态广播器）
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # Load omni wheel velocity controller
    # 加载全向轮速度控制器
    load_omni_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'omni_wheel_controller'],
        output='screen'
    )
    
    # Omnidirectional wheel controller node (cmd_vel → wheel speeds, wheel speeds → odom)
    # 全向轮控制节点（cmd_vel → 轮速，轮速 → 里程计）
    # Publishes /wheel/odom (raw wheel odometry without TF)
    # EKF will fuse /wheel/odom + /imu/data and publish odom→base_link TF
    # 发布 /wheel/odom（原始轮子里程计，不发布TF）
    # EKF 将融合 /wheel/odom + /imu/data 并发布 odom→base_link TF
    omni_controller_node = Node(
        package='bot_control',
        executable='omni_controller_node',
        name='omni_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'publish_odom_tf': False,  # Disable TF - let EKF publish it
        }]
    )
    
    # ==========================================================================
    # PHASE 3: Sensor Data Processing / 第三阶段：传感器数据处理
    # ==========================================================================
    
    # IMU data relay with QoS configuration / IMU 数据中继（配置QoS）
    # Relay from sensor QoS (BEST_EFFORT) to reliable QoS for EKF
    # 从传感器QoS（BEST_EFFORT）中继到可靠QoS供EKF使用
    imu_relay = Node(
        package='topic_tools',
        executable='relay',
        name='imu_relay',
        output='screen',
        arguments=[
            '/imu_plugin/out',
            '/imu/data',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'input_qos': 'SENSOR_DATA',  # Input: BEST_EFFORT from Gazebo
            'output_qos': 'RELIABLE',     # Output: RELIABLE for EKF
        }]
    )
    
    # ==========================================================================
    # PHASE 3: RTABMap Visual SLAM / 第三阶段：RTABMap 视觉SLAM
    # ==========================================================================
    
    # RTABMap SLAM node / RTABMap SLAM 节点
    # Performs visual odometry, mapping, and loop closure
    # 执行视觉里程计、建图和闭环检测
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_config],
        remappings=[
            # RGB-D camera topics / RGB-D 相机话题
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            
            # Odometry input: Use raw wheel odom (unfused, before EKF)
            # RTABMap performs visual SLAM using wheel odom as initial guess
            # 里程计输入：使用原始轮子里程计（未融合，EKF之前）
            # RTABMap 使用轮子里程计作为初始猜测执行视觉SLAM
            ('odom', '/wheel/odom'),
            
            # Output occupancy grid for Nav2 / 输出占据栅格给 Nav2
            ('grid_map', '/map'),
        ],
        arguments=[
            '--delete_db_on_start',  # Start fresh each time / 每次重新开始
            '--ros-args', '--log-level', 'rtabmap:=warn'  # Set log level to WARN / 设置日志级别为警告
        ]
    )
    
    # RTABMap visualization node (optional) / RTABMap 可视化节点（可选）
    # Provides rich 3D visualization of point clouds and loop closures
    # 提供丰富的3D点云和闭环可视化
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen',
        parameters=[rtabmap_config],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/rtabmap/odom'),  # Use visual odom / 使用视觉里程计
        ],
        condition=IfCondition(
            PythonExpression(["'", use_rviz_str, "' == 'false'"])  # Only if RViz not used
        )
    )
    
    # ==========================================================================
    # PHASE 3: Obstacle Detection for Nav2 / 第三阶段：Nav2障碍物检测
    # ==========================================================================
    
    # Convert depth image to point cloud / 深度图转点云
    # This provides 3D obstacle information for Nav2 local costmap
    # 为Nav2局部代价地图提供3D障碍物信息
    point_cloud_xyz = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'decimation': 2,        # Decimate depth image by factor 2 (faster) / 深度图降采样2倍（更快）
            'max_depth': 3.0,       # Max depth for obstacles (3m) / 障碍物最大深度
            'voxel_size': 0.02,     # Voxel filter size (2cm) / 体素滤波大小
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/camera_info'),
            ('cloud', '/camera/cloud'),
        ]
    )
    
    # Segment ground from obstacles / 分割地面和障碍物
    # This is critical for Nav2 to distinguish drivable surface from obstacles
    # 这对于Nav2区分可行驶表面和障碍物至关重要
    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'Grid/MaxObstacleHeight': '2.0',    # Same as RTABMap / 与RTABMap相同
            'Grid/MinObstacleHeight': '0.1',    # Filter low obstacles / 过滤低障碍物
        }],
        remappings=[
            ('cloud', '/camera/cloud'),
            ('obstacles', '/camera/obstacles'),  # Obstacle point cloud for Nav2 / Nav2障碍物点云
            ('ground', '/camera/ground'),        # Ground point cloud for clearing / 地面点云用于清除
        ]
    )
    
    # ==========================================================================
    # PHASE 4: EKF Fusion (Wheel + IMU) / 第四阶段：EKF融合
    # ==========================================================================
    
    # EKF localization node / EKF 定位节点
    # Fuses wheel odometry + IMU for robust state estimation
    # 融合轮式里程计 + IMU 实现鲁棒状态估计
    #
    # Input: /wheel/odom (x, y, vx, vy) + /imu/data (yaw, vyaw)
    # Output: /odometry/filtered (fused odometry)
    # Publishes: odom→base_link TF (prevents wheel slip drift)
    # 输入：/wheel/odom (x, y, vx, vy) + /imu/data (yaw, vyaw)
    # 输出：/odometry/filtered（融合里程计）
    # 发布：odom→base_link TF（防止轮子打滑漂移）
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )
    
    # ==========================================================================
    # PHASE 4: Nav2 Navigation Stack / 第四阶段：Nav2 导航栈
    # ==========================================================================
    
    # Include Nav2 bringup / 包含 Nav2 启动
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'params_file': configured_nav2_params,
            'autostart': 'true',
        }.items()
    )
    
    # ==========================================================================
    # PHASE 5: RViz Visualization / 第五阶段：RViz 可视化
    # ==========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time_str == 'true'}],
        condition=IfCondition(use_rviz)
    )
    
    # ==========================================================================
    # Event-Driven Launch Sequence / 事件驱动启动序列
    # ==========================================================================
    #
    # Startup chain (using OnProcessStart/OnProcessExit events):
    # 启动链（使用 OnProcessStart/OnProcessExit 事件）:
    #
    #   gazebo_server starts
    #         │
    #         ▼ OnProcessStart
    #   robot_state_publisher + gazebo_client start
    #         │
    #         ▼ OnProcessStart(robot_state_publisher)
    #   spawn_robot starts
    #         │
    #         ▼ OnProcessExit(spawn_robot) - robot spawned successfully
    #   Phase 2: control nodes start
    #         │
    #         ▼ OnProcessStart(omni_controller)
    #   Phase 3: RTABMap SLAM + EKF start
    #         │
    #         ▼ OnProcessStart(rtabmap_slam)
    #   Phase 4: Nav2 navigation start
    #         │
    #         ▼ OnProcessStart(nav2) - cannot detect, use GroupAction workaround
    #   Phase 5: RViz starts
    #
    # ==========================================================================
    
    # Event: When gazebo_server starts -> start robot_state_publisher and gazebo_client
    # 事件：当 gazebo_server 启动后 -> 启动 robot_state_publisher 和 gazebo_client
    event_gazebo_started = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo_server,
            on_start=[
                LogInfo(msg='[Event] Gazebo server started, launching robot_state_publisher and client...'),
                robot_state_publisher,
                gazebo_client,
            ]
        )
    )
    
    # Event: When robot_state_publisher starts -> spawn robot
    # 事件：当 robot_state_publisher 启动后 -> 生成机器人
    event_rsp_started = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                LogInfo(msg='[Event] Robot state publisher started, spawning robot...'),
                spawn_robot,
            ]
        )
    )
    
    # Event: When spawn_robot completes (exits) -> start Phase 2 (control)
    # 事件：当 spawn_robot 完成（退出）后 -> 启动第二阶段（控制）
    event_robot_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                LogInfo(msg='[Event] Robot spawned successfully!'),
                LogInfo(msg='[Phase 2] Starting ros2_control controllers...'),
                load_joint_state_broadcaster,
                load_omni_wheel_controller,
                omni_controller_node,
            ]
        )
    )
    
    # Event: When omni_controller_node starts -> start Phase 3 (Sensor Processing & SLAM)
    # 事件：当 omni_controller_node 启动后 -> 启动第三阶段（传感器处理和SLAM）
    event_control_started = RegisterEventHandler(
        OnProcessStart(
            target_action=omni_controller_node,
            on_start=[
                LogInfo(msg='[Event] Control nodes started!'),
                LogInfo(msg='[Phase 3] Starting sensor processing, EKF fusion, and RTABMap SLAM...'),
                imu_relay,           # IMU QoS relay for EKF
                ekf_filter,          # EKF fusion (wheel odom + IMU)
                rtabmap_slam,        # RTABMap visual SLAM
                rtabmap_viz,         # RTABMap visualization
                point_cloud_xyz,     # Depth to point cloud
                obstacles_detection, # Point cloud segmentation
            ]
        )
    )
    
    # Event: When rtabmap_slam starts -> start Phase 4 (Navigation)
    # 事件：当 rtabmap_slam 启动后 -> 启动第四阶段（导航）
    event_slam_started = RegisterEventHandler(
        OnProcessStart(
            target_action=rtabmap_slam,
            on_start=[
                LogInfo(msg='[Event] RTABMap SLAM started!'),
                LogInfo(msg='[Phase 4] Starting Nav2 navigation stack...'),
                nav2_bringup,
            ]
        )
    )
    
        # Event: When EKF filter starts -> start Phase 5 (RViz)
    # 事件：当 EKF 滤波器启动后 -> 启动第五阶段（RViz）
    # Note: We use EKF as trigger instead of nav2_bringup because IncludeLaunchDescription
    # doesn't emit OnProcessStart events
    # 注意：使用 EKF 作为触发器而非 nav2_bringup，因为 IncludeLaunchDescription 不会发出 OnProcessStart 事件
    event_ekf_started = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_filter,
            on_start=[
                LogInfo(msg='[Event] EKF filter started!'),
                LogInfo(msg='[Phase 5] Starting RViz visualization...'),
                rviz_node,
            ]
        )
    )
    
    # ==========================================================================
    # Return Launch Actions / 返回启动动作
    # ==========================================================================
    return [
        # Startup info / 启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='RTABMap Visual SLAM Navigation Simulation'),
        LogInfo(msg='  (RGB-D Visual SLAM with ORB Features)'),
        LogInfo(msg='  (Wheel + Visual + IMU Fusion)'),
        LogInfo(msg='  (Event-Driven Startup Sequence)'),
        LogInfo(msg=f'  World: {world_str or "navigation_5x5_rgbd.world"}'),
        LogInfo(msg=f'  RViz: {use_rviz_str}'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Key Advantages over depth-to-laser:'),
        LogInfo(msg='  - 500+ visual features vs 20 scan points'),
        LogInfo(msg='  - Visual bag-of-words loop closure'),
        LogInfo(msg='  - Stable mapping in featureless environments'),
        LogInfo(msg='  - No geometry-only scan matching drift'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Startup sequence:'),
        LogInfo(msg='  [1] Gazebo server'),
        LogInfo(msg='  [2] Robot state publisher + Gazebo client'),
        LogInfo(msg='  [3] Spawn robot'),
        LogInfo(msg='  [4] Control nodes (on robot spawned)'),
        LogInfo(msg='  [5] RTABMap SLAM + EKF (on control ready)'),
        LogInfo(msg='  [6] Nav2 navigation (on SLAM ready)'),
        LogInfo(msg='  [7] RViz (on EKF ready)'),
        LogInfo(msg='========================================'),
        
        # Phase 1: Start Gazebo server (triggers the event chain)
        # 第一阶段：启动 Gazebo 服务器（触发事件链）
        LogInfo(msg='[Phase 1] Starting Gazebo simulation environment...'),
        gazebo_server,
        
        # Event handlers (define the startup sequence)
        # 事件处理器（定义启动顺序）
        event_gazebo_started,
        event_rsp_started,
        event_robot_spawned,
        event_control_started,
        event_slam_started,
        event_ekf_started,  # Renamed from event_ekf_started
    ]


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    # ==========================================================================
    # Package Directories (for default values) / 包目录（用于默认值）
    # ==========================================================================
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    # ==========================================================================
    # Declare Launch Arguments / 声明启动参数
    # ==========================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock / 使用仿真（Gazebo）时钟'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='World file name or path (default: navigation_5x5_rgbd.world) / 世界文件名或路径'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI / 启动 Gazebo GUI'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 / 启动 RViz2'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_bot_navigation, 'rviz', 'nav2_rtabmap.rviz'),
        description='RViz config file / RViz 配置文件'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(pkg_bot_navigation, 'config', 'nav2', 'nav2_params_imu.yaml'),
        description='Nav2 parameters file / Nav2 参数文件'
    )
    
    # ==========================================================================
    # Launch Description / 启动描述
    # ==========================================================================
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_gui,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        OpaqueFunction(function=launch_setup)
    ])
