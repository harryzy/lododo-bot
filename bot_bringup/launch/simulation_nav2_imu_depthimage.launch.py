#!/usr/bin/env python3
"""
IMU Navigation Simulation Launch File (with Official depthimage_to_laserscan)
IMU 导航仿真启动文件（使用官方 depthimage_to_laserscan 包）

Complete simulation with Wheel Odometry + IMU fusion for navigation
完整的轮式里程计 + IMU 融合导航仿真

This version uses ROS2 official depthimage_to_laserscan package instead of custom implementation
本版本使用 ROS2 官方的 depthimage_to_laserscan 包而非自定义实现

Key Difference / 主要区别:
    - Uses official depthimage_to_laserscan_node with correct perspective projection
    - 使用官方 depthimage_to_laserscan_node，具有正确的透视投影转换
    - Eliminates arc distortion when viewing straight walls
    - 消除直墙显示为弧线的畸变问题

Architecture / 架构:
    ┌─────────────────────────────────────────────────────────────┐
    │            simulation_nav2_imu_depthimage.launch.py         │
    │                                                             │
    │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
    │  │   Gazebo    │───▶│ Wheel+IMU   │───▶│    Nav2     │     │
    │  │(wheel odom) │    │   (EKF)     │    │ (navigation)│     │
    │  │ (no TF)     │    │             │    │             │     │
    │  └─────────────┘    └─────────────┘    └─────────────┘     │
    │         │                  │                  │             │
    │         ▼                  ▼                  ▼             │
    │   /camera/*           /odom (fused)      /cmd_vel          │
    │   /odom (wheel)       odom->base_link    /map              │
    │   /imu/data                              /scan              │
    │                                                             │
    │  ┌─────────────────────────────────────────┐               │
    │  │ Official depthimage_to_laserscan        │               │
    │  │ (Correct pinhole camera projection)     │               │
    │  └─────────────────────────────────────────┘               │
    └─────────────────────────────────────────────────────────────┘

Startup Sequence (Event-Driven) / 启动顺序（事件驱动）:
    Phase 1: Gazebo Simulation
        - Start Gazebo server and client
        - Spawn robot model
        - Event: OnProcessStart(spawn_robot) -> Phase 2
    
    Phase 2: Robot Control & Perception
        - Start omni_controller
        - Start official depthimage_to_laserscan
        - Event: OnProcessStart(depth_to_laserscan) -> Phase 3
    
    Phase 3: IMU Fusion
        - Start EKF fusion (wheel odometry + IMU)
        - Event: OnProcessStart(ekf_filter) -> Phase 4
    
    Phase 4: Navigation
        - Start SLAM Toolbox
        - Start Nav2 navigation stack
        - Event: OnProcessStart(slam_toolbox) -> Phase 5
    
    Phase 5: Visualization (optional)
        - Start RViz with nav2 config

QoS Compatibility / QoS 兼容性:
    - Wheel Odometry: Default QoS (Reliable, Volatile)
    - IMU Data: Sensor Data QoS (Best Effort, Volatile)
    - Camera topics: Sensor Data QoS (Best Effort, Volatile)
    - LaserScan: Reliable QoS (matching Nav2 requirements)
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
    TimerAction,
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
    slam = LaunchConfiguration('slam')
    slam_str = LaunchConfiguration('slam').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    
    # ==========================================================================
    # Package Directories / 包目录
    # ==========================================================================
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    pkg_bot_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bot_description = get_package_share_directory('bot_description')
    pkg_bot_control = get_package_share_directory('bot_control')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # ==========================================================================
    # Configuration Files / 配置文件
    # ==========================================================================
    ekf_config = os.path.join(pkg_bot_navigation, 'config', 'robot_localization_imu.yaml')
    # Use slam_toolbox config that subscribes to /scan (not /scan_reliable)
    # 使用订阅 /scan 话题的 slam_toolbox 配置（而非 /scan_reliable）
    slam_config = os.path.join(pkg_bot_navigation, 'config', 'slam_toolbox_imu_official.yaml')
    
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
    
    # Process xacro with publish_odom_tf:=false for IMU mode
    # 处理 xacro，IMU 模式下 publish_odom_tf:=false（禁用Gazebo odom TF，由EKF发布）
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' publish_odom_tf:=false'  # Disable Gazebo odom TF, EKF will publish it
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
    
    # Gazebo client / Gazebo 客户端
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': full_model_path}
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
    # PHASE 2: Robot Control & Perception Nodes / 第二阶段：机器人控制和感知节点
    # ==========================================================================
    
    # Omni-directional wheel controller / 全向轮控制器
    omni_controller = Node(
        package='bot_control',
        executable='omni_controller',
        name='omni_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'wheel_radius': 0.05,
            'rear_wheel_dist': 0.105,
            'front_wheel_dist': 0.085,
            'max_wheel_speed': 4.712,
            'publish_rate': 50.0
        }]
    )
    
    # Wheel joint publisher / 轮子关节发布器
    wheel_joint_publisher = Node(
        package='bot_control',
        executable='wheel_joint_publisher',
        name='wheel_joint_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true'
        }]
    )
    
    # ==========================================================================
    # Official ROS2 depthimage_to_laserscan Node / ROS2 官方深度图转激光雷达节点
    # ==========================================================================
    # This node correctly handles pinhole camera perspective projection
    # 此节点正确处理针孔相机透视投影
    # Eliminates the arc distortion issue seen with custom implementation
    # 消除自定义实现中看到的弧线畸变问题
    #
    # Key improvements / 关键改进:
    #   - Uses camera_info intrinsics for correct angle-to-pixel mapping
    #   - 使用 camera_info 内参进行正确的角度到像素映射
    #   - Applies proper tan() based projection instead of linear sampling
    #   - 应用正确的基于 tan() 的投影而非线性采样
    #   - Straight walls in Gazebo will appear as straight lines in RViz
    #   - Gazebo 中的直墙在 RViz 中显示为直线
    # ==========================================================================
    
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            
            # Scan configuration / 扫描配置
            'scan_height': 10,              # Number of pixel rows to use / 使用的像素行数
            'scan_time': 0.033,             # Time between scans (seconds) / 扫描间隔（秒）
            
            # Range limits / 范围限制
            'range_min': 0.05,              # Min range 5cm for close detection / 最小范围5cm用于近距离检测
            'range_max': 8.0,               # Max range 8m / 最大范围8米
            
            # Output frame / 输出坐标系
            # CRITICAL: Parameter name is 'output_frame', NOT 'output_frame_id'
            # 关键：参数名是 'output_frame'，不是 'output_frame_id'
            'output_frame': 'base_link',    # Output in base_link for Nav2 compatibility
        }],
        remappings=[
            # Input topics / 输入话题
            ('depth', '/camera/depth/image_raw'),           # Depth image from Gazebo
            ('depth_camera_info', '/camera/depth/camera_info'),  # Camera intrinsics
            
            # Output topic / 输出话题
            ('scan', '/scan'),              # LaserScan output
        ]
    )
    
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
    # PHASE 3: IMU Fusion Nodes / 第三阶段：IMU 融合节点
    # ==========================================================================
    
    # Robot localization EKF for wheel + IMU fusion / 机器人定位 EKF（轮式+IMU融合）
    # Output: /odometry/filtered (default, no remapping needed)
    # 输出：/odometry/filtered（默认输出，无需remapping）
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time_str == 'true'}
        ],
    )
    
    # ==========================================================================
    # PHASE 4: Navigation Stack Nodes / 第四阶段：导航栈节点
    # ==========================================================================
    
    # SLAM Toolbox / SLAM 工具箱（仅用于建图，不使用视觉里程计）
    slam_toolbox = Node(
        condition=IfCondition(slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time_str == 'true'}
        ],
    )
    
    # Map saver / 地图保存器
    map_saver = Node(
        condition=IfCondition(slam),
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[
            configured_nav2_params,
            {'use_sim_time': use_sim_time_str == 'true'}
        ],
    )
    
    # Nav2 navigation stack / Nav2 导航栈
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'autostart': 'true',
            'params_file': nav2_params,
        }.items()
    )
    
    # Localization (when not using SLAM) / 定位（不使用 SLAM 时）
    localization_launch = IncludeLaunchDescription(
        condition=UnlessCondition(slam),
        launch_description_source=PythonLaunchDescriptionSource([
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'autostart': 'true',
            'params_file': nav2_params,
            'map': LaunchConfiguration('map'),
        }.items()
    )
    
    # ==========================================================================
    # PHASE 5: Visualization Node / 第五阶段：可视化节点
    # ==========================================================================
    
    # RViz node / RViz 节点
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time_str == 'true'}],
    )
    
    # ==========================================================================
    # Event-Driven Startup Sequence / 事件驱动的启动顺序
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
    #   Phase 2: control + perception start
    #         │
    #         ▼ OnProcessStart(depthimage_to_laserscan)
    #   Phase 3: IMU fusion start
    #         │
    #         ▼ OnProcessStart(ekf_filter)
    #   Phase 4: Navigation nodes start
    #         │
    #         ▼ OnProcessStart(slam_toolbox)
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
    
    # Event: When spawn_robot completes (exits) -> start Phase 2 (control & perception)
    # 事件：当 spawn_robot 完成（退出）后 -> 启动第二阶段（控制和感知）
    event_robot_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                LogInfo(msg='[Event] Robot spawned successfully!'),
                LogInfo(msg='[Phase 2] Starting robot control and official depthimage_to_laserscan...'),
                omni_controller,
                wheel_joint_publisher,
                depthimage_to_laserscan,  # Official ROS2 node / 官方 ROS2 节点
                imu_relay,
            ]
        )
    )
    
    # Event: When depthimage_to_laserscan starts -> start Phase 3 (IMU Fusion)
    # 事件：当 depthimage_to_laserscan 启动后 -> 启动第三阶段（IMU融合）
    event_perception_started = RegisterEventHandler(
        OnProcessStart(
            target_action=depthimage_to_laserscan,
            on_start=[
                LogInfo(msg='[Event] Official depthimage_to_laserscan node started!'),
                LogInfo(msg='[Phase 3] Starting IMU Fusion (Wheel + IMU EKF)...'),
                ekf_filter,
            ]
        )
    )
    
    # Event: When EKF filter starts -> start Phase 4 (Navigation)
    # 事件：当 EKF 滤波器启动后 -> 启动第四阶段（导航）
    event_ekf_started = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_filter,
            on_start=[
                LogInfo(msg='[Event] EKF filter started!'),
                LogInfo(msg='[Phase 4] Starting Navigation stack...'),
                slam_toolbox,
                map_saver,
                nav2_navigation,
                localization_launch,
            ]
        )
    )
    
    # Event: When SLAM toolbox starts -> start Phase 5 (RViz)
    # 事件：当 SLAM 工具箱启动后 -> 启动第五阶段（RViz）
    event_slam_started = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox,
            on_start=[
                LogInfo(msg='[Event] SLAM Toolbox started!'),
                LogInfo(msg='[Phase 5] Starting RViz visualization...'),
                rviz_node,
            ]
        )
    )
    
    # ==========================================================================
    # Return all launch actions / 返回所有启动动作
    # ==========================================================================
    return [
        # Startup info / 启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting IMU Navigation Simulation'),
        LogInfo(msg='  (Using Official depthimage_to_laserscan)'),
        LogInfo(msg='  (Wheel Odometry + IMU Fusion)'),
        LogInfo(msg='  (Event-Driven Startup Sequence)'),
        LogInfo(msg=f'  World: {world_str or "navigation_5x5_rgbd.world"}'),
        LogInfo(msg=f'  SLAM Mode: {slam_str}'),
        LogInfo(msg=f'  RViz: {use_rviz_str}'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Key Feature: Using ROS2 official depthimage_to_laserscan'),
        LogInfo(msg='  - Correct pinhole camera perspective projection'),
        LogInfo(msg='  - No arc distortion for straight walls'),
        LogInfo(msg='  - Better angle-to-pixel mapping with camera_info'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Startup sequence:'),
        LogInfo(msg='  [1] Gazebo server'),
        LogInfo(msg='  [2] Robot state publisher + Gazebo client'),
        LogInfo(msg='  [3] Spawn robot'),
        LogInfo(msg='  [4] Control + Official Perception (on robot spawned)'),
        LogInfo(msg='  [5] IMU Fusion (on perception ready)'),
        LogInfo(msg='  [6] Navigation (on EKF ready)'),
        LogInfo(msg='  [7] RViz (on SLAM ready)'),
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
        event_perception_started,
        event_ekf_started,
        event_slam_started,
    ]


def generate_launch_description():
    """Generate launch description / 生成启动描述"""
    
    # ==========================================================================
    # Package Paths / 包路径
    # ==========================================================================
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    # Default configuration files / 默认配置文件
    default_rviz_config = os.path.join(pkg_bot_navigation, 'config', 'nav2.rviz')
    default_nav2_params = os.path.join(pkg_bot_navigation, 'config', 'nav2', 'nav2_params_imu.yaml')
    
    # ==========================================================================
    # Launch Arguments / 启动参数
    # ==========================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='navigation_5x5_rgbd.world',
        description='Gazebo world file (default: navigation_5x5_rgbd.world) / Gazebo 世界文件'
    )
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Run SLAM instead of localization / 运行 SLAM 而不是定位'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file (for localization mode) / 地图文件路径（定位模式）'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=default_nav2_params,
        description='Path to Nav2 parameters file / Nav2 参数文件路径'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization / 启动 RViz 可视化'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz config file path / RViz 配置文件路径'
    )
    
    # ==========================================================================
    # Return Launch Description / 返回启动描述
    # ==========================================================================
    return LaunchDescription([
        # Argument declarations / 参数声明
        declare_use_sim_time,
        declare_world,
        declare_slam,
        declare_map,
        declare_nav2_params,
        declare_use_rviz,
        declare_rviz_config,
        
        # Dynamic launch setup / 动态启动设置
        OpaqueFunction(function=launch_setup),
    ])
