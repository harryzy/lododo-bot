#!/usr/bin/env python3
"""
VIO Navigation Simulation Launch File / VIO 导航仿真启动文件
Complete simulation with Visual-Inertial Odometry integration
完整的视觉惯性里程计集成仿真

Architecture / 架构:
    ┌─────────────────────────────────────────────────────────────┐
    │                      simulation_nav2_vio.launch.py          │
    │                                                             │
    │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
    │  │   Gazebo    │───▶│     VIO     │───▶│    Nav2     │     │
    │  │ (no odom TF)│    │ (EKF fusion)│    │ (navigation)│     │
    │  └─────────────┘    └─────────────┘    └─────────────┘     │
    │         │                  │                  │             │
    │         ▼                  ▼                  ▼             │
    │   /camera/*           /odom (fused)      /cmd_vel          │
    │   /odom (wheel)       odom->base_link    /map              │
    └─────────────────────────────────────────────────────────────┘

Startup Sequence (Event-Driven) / 启动顺序（事件驱动）:
    Phase 1: Gazebo Simulation
        - Start Gazebo server and client
        - Spawn robot model
        - Event: OnProcessStart(spawn_robot) -> Phase 2
    
    Phase 2: Robot Control & Perception
        - Start omni_controller
        - Start perception nodes (depth_to_laserscan)
        - Event: OnProcessStart(perception) -> Phase 3
    
    Phase 3: Visual Odometry
        - Start rgbd_odometry
        - Start EKF fusion
        - Event: OnProcessStart(ekf_filter) -> Phase 4
    
    Phase 4: Navigation
        - Start SLAM Toolbox
        - Start Nav2 navigation stack
        - Event: OnProcessStart(slam_toolbox) -> Phase 5
    
    Phase 5: Visualization (optional)
        - Start RViz with nav2 config

QoS Compatibility / QoS 兼容性:
    - Camera topics: Sensor Data QoS (Best Effort, Volatile)
    - Odometry topics: Default QoS (Reliable, Volatile)
    - LaserScan: Sensor Data QoS (Best Effort, Volatile)
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
    pkg_bot_perception = get_package_share_directory('bot_perception')
    pkg_bot_control = get_package_share_directory('bot_control')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # ==========================================================================
    # Configuration Files / 配置文件
    # ==========================================================================
    rtabmap_config = os.path.join(pkg_bot_navigation, 'config', 'rtabmap_odom.yaml')
    ekf_config = os.path.join(pkg_bot_navigation, 'config', 'robot_localization.yaml')
    slam_config = os.path.join(pkg_bot_navigation, 'config', 'slam_toolbox_vio.yaml')
    
    # ==========================================================================
    # World File Resolution / 世界文件解析
    # ==========================================================================
    if not world_str or world_str == '':
        world_file = os.path.join(pkg_bot_gazebo, 'worlds', 'textured_test.world')
    elif not os.path.isabs(world_str):
        world_file = os.path.join(pkg_bot_gazebo, 'worlds', world_str)
    else:
        world_file = world_str
    
    # ==========================================================================
    # URDF Processing / URDF 处理
    # ==========================================================================
    urdf_file = os.path.join(pkg_bot_description, 'urdf', 'lekiwi_bot_sim.xacro')
    
    # Process xacro with publish_odom_tf:=false for VIO mode
    # 处理 xacro，VIO 模式下 publish_odom_tf:=false
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' publish_odom_tf:=false'  # Disable Gazebo odom TF for VIO
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
    
    # Perception launch (includes depth_to_laserscan with venv wrapper)
    # 感知启动（包含使用虚拟环境的 depth_to_laserscan）
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_bot_perception, 'launch', 'perception.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
        }.items()
    )
    
    # ==========================================================================
    # PHASE 3: Visual Odometry (VIO) Nodes / 第三阶段：视觉里程计 (VIO) 节点
    # ==========================================================================
    
    # Wheel odometry relay / 轮式里程计中继
    wheel_odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='wheel_odom_relay',
        output='log',
        parameters=[{'use_sim_time': use_sim_time_str == 'true'}],
        arguments=['/odom', '/wheel_odom'],
    )
    
    # RTAB-Map rgbd_odometry / RTAB-Map 视觉里程计
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            rtabmap_config,
            {'use_sim_time': use_sim_time_str == 'true'}
        ],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/visual_odom'),
        ],
    )
    
    # Robot localization EKF / Robot Localization EKF
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time_str == 'true'}
        ],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )
    
    # ==========================================================================
    # PHASE 4: Navigation Stack Nodes / 第四阶段：导航栈节点
    # ==========================================================================
    
    # SLAM Toolbox / SLAM 工具箱
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
    #         ▼ OnProcessStart(depth_to_laserscan)
    #   Phase 3: VIO nodes start
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
                LogInfo(msg='[Phase 2] Starting robot control and perception...'),
                omni_controller,
                wheel_joint_publisher,
                perception_launch,
            ]
        )
    )
    
    # Event: When perception launch completes -> start Phase 3 (VIO)
    # 事件：当感知启动完成后 -> 启动第三阶段（VIO）
    # Note: IncludeLaunchDescription triggers OnExecutionComplete when all its nodes start
    # 注意：IncludeLaunchDescription 在其所有节点启动时触发 OnExecutionComplete
    event_perception_started = RegisterEventHandler(
        OnExecutionComplete(
            target_action=perception_launch,
            on_completion=[
                LogInfo(msg='[Event] Perception nodes started!'),
                LogInfo(msg='[Phase 3] Starting Visual Odometry (VIO) system...'),
                wheel_odom_relay,
                rgbd_odometry,
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
        LogInfo(msg='Starting VIO Navigation Simulation'),
        LogInfo(msg='  (Event-Driven Startup Sequence)'),
        LogInfo(msg=f'  World: {world_str or "textured_test.world"}'),
        LogInfo(msg=f'  SLAM Mode: {slam_str}'),
        LogInfo(msg=f'  RViz: {use_rviz_str}'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Startup sequence:'),
        LogInfo(msg='  [1] Gazebo server'),
        LogInfo(msg='  [2] Robot state publisher + Gazebo client'),
        LogInfo(msg='  [3] Spawn robot'),
        LogInfo(msg='  [4] Control + Perception (on robot spawned)'),
        LogInfo(msg='  [5] VIO system (on perception ready)'),
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
    default_nav2_params = os.path.join(pkg_bot_navigation, 'config', 'nav2', 'nav2_params.yaml')
    
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
        default_value='textured_test.world',
        description='Gazebo world file (default: textured_test.world for VIO) / Gazebo 世界文件'
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
