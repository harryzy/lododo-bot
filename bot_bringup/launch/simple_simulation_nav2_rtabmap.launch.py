#!/usr/bin/env python3
"""
RTABMap Visual SLAM Navigation Simulation Launch File (Simplified Model)
RTABMap 视觉SLAM 导航仿真启动文件（简化模型）

Complete simulation with RGB-D Visual SLAM for high-quality mapping
完整的 RGB-D 视觉SLAM 仿真，实现高质量建图

使用简化机器人模型进行导航和SLAM测试
Uses simplified robot model for navigation and SLAM testing

Key Features / 关键特性:
    - Simplified URDF model (single cylinder base, 3 omni wheels)
    - 简化URDF模型（单一圆柱底盘，3个全向轮）
    - RTABMap RGB-D Visual SLAM (500+ visual features)
    - RTABMap RGB-D 视觉SLAM（500+视觉特征）
    - EKF sensor fusion (wheel odom + IMU)
    - EKF传感器融合（轮式里程计 + IMU）
    - Nav2 navigation stack
    - Nav2导航栈

Architecture / 架构:
    Gazebo → Simplified URDF → ros2_control → Kinematics Node
                                    ↓
                              Wheel Odometry
                                    ↓
                            EKF Fusion (+ IMU)
                                    ↓
                          RTABMap Visual SLAM
                                    ↓
                            Nav2 Navigation
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
    pkg_bot_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bot_description = get_package_share_directory('bot_description')
    pkg_bot_control = get_package_share_directory('bot_control')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # ==========================================================================
    # Configuration Files / 配置文件
    # ==========================================================================
    # RTABMap configuration / RTABMap 配置
    rtabmap_config = os.path.join(pkg_bot_navigation, 'config', 'rtabmap.yaml')
    
    # EKF configuration (with visual odometry) / EKF 配置（包含视觉里程计）
    ekf_config = os.path.join(pkg_bot_navigation, 'config', 'robot_localization_rtabmap.yaml')
    
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
    # URDF Processing (Simplified Model) / URDF 处理（简化模型）
    # ==========================================================================
    urdf_file = os.path.join(pkg_bot_description, 'urdf', 'lekiwi_bot_simple.gazebo.xacro')
    
    # Controller configuration file (simplified controller) / 控制器配置文件（简化控制器）
    controller_config = os.path.join(pkg_bot_control, 'config', 'simple_omni_controller.yaml')
    
    # Process xacro - simplified model doesn't need controller_config_file parameter
    # 处理 xacro - 简化模型不需要 controller_config_file 参数
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
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
    
    # Spawn robot (simplified model) / 生成机器人（简化模型）
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'lekiwi_simple',
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
    
    # Load simplified omni wheel velocity controller
    # 加载简化版全向轮速度控制器
    load_omni_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'simple_omni_wheel_controller'],
        output='screen'
    )
    
    # Omnidirectional wheel controller node (simplified model mode)
    # 全向轮控制节点（简化模型模式）
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_simple_model': True,  # Use simplified model
            'publish_odom_tf': False,  # Disable TF - let EKF publish it
            # Wheel correction factors (for real robot calibration)
            'wheel_correction.wheel_1': 1.0,
            'wheel_correction.wheel_2': 1.0,
            'wheel_correction.wheel_3': 1.0,
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
            '/imu/data',
            '/imu/data',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'input_qos': 'SENSOR_DATA',  # Input: BEST_EFFORT from Gazebo
            'output_qos': 'RELIABLE',     # Output: RELIABLE for EKF
        }]
    )
    
    # Static TF: map → odom (initialization fallback)
    # RTABMap will override this once SLAM is initialized
    # 静态TF: map → odom (初始化回退方案)
    # RTABMap初始化后会接管这个变换
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_fallback',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
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
            # RGB-D camera topics (matching simplified model output)
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            
            # Odometry input: Use raw wheel odom (unfused, before EKF)
            # RTABMap performs visual SLAM using wheel odom as initial guess
            # 里程计输入：使用原始轮子里程计（未融合，EKF之前）
            # RTABMap 使用轮子里程计作为初始猜测执行视觉SLAM
            #('odom', '/wheel/odom'),
            ('odom', '/odometry/filtered'),  # Use fused odom for better results / 使用融合后的里程计以获得更好效果
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
            ('depth/camera_info', '/camera/depth/camera_info'),
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
            'wait_for_transform': 15.0,         # Wait up to 15s for TF / 等待15秒TF建立
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
                static_map_to_odom,  # Static map→odom TF (fallback)
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
        LogInfo(msg='  (Simplified Robot Model)'),
        LogInfo(msg='  (RGB-D Visual SLAM with ORB Features)'),
        LogInfo(msg='  (Wheel + Visual + IMU Fusion)'),
        LogInfo(msg='  (Event-Driven Startup Sequence)'),
        LogInfo(msg=f'  World: {world_str or "navigation_5x5_rgbd.world"}'),
        LogInfo(msg=f'  RViz: {use_rviz_str}'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Simplified Model Features:'),
        LogInfo(msg='  - Single cylinder base (reduced complexity)'),
        LogInfo(msg='  - 3 omni wheels with accurate kinematics'),
        LogInfo(msg='  - RGB-D camera + IMU sensors'),
        LogInfo(msg='  - Suitable for Nav2/SLAM development'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Startup sequence:'),
        LogInfo(msg='  [1] Gazebo server'),
        LogInfo(msg='  [2] Robot state publisher + Gazebo client'),
        LogInfo(msg='  [3] Spawn robot (simplified model)'),
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
        event_ekf_started,
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
