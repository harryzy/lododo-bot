#!/usr/bin/env python3
"""
RTABMap Localization Navigation Simulation Launch File (Simplified Model)
RTABMap 定位导航仿真启动文件（简化模型）

Complete simulation with RGB-D localization for patrol navigation
完整的 RGB-D 定位仿真，用于巡航导航

使用简化机器人模型进行定位和导航测试
Uses simplified robot model for localization and navigation testing

Key Features / 关键特性:
    - Simplified URDF model (single cylinder base, 3 omni wheels)
    - 简化URDF模型（单一圆柱底盘，3个全向轮）
    - RTABMap Localization Mode (load pre-built map)
    - RTABMap 定位模式（加载预构建地图）
    - EKF sensor fusion (wheel odom + IMU)
    - EKF传感器融合（轮式里程计 + IMU）
    - Nav2 navigation stack
    - Nav2导航栈

Differences from SLAM mode / 与SLAM模式的区别:
    - No mapping, only localization
    - Lower computational load (~60-70% of SLAM)
    - Requires pre-built RTABMap database

Architecture / 架构:
    Gazebo → Simplified URDF → ros2_control → Kinematics Node
                                    ↓
                              Wheel Odometry
                                    ↓
                            EKF Fusion (+ IMU)
                                    ↓
                  RTABMap Localization (load .db)
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
    log_level = LaunchConfiguration('log_level')
    log_level_str = LaunchConfiguration('log_level').perform(context)
    rtabmap_db_path = LaunchConfiguration('rtabmap_db_path')
    rtabmap_db_path_str = LaunchConfiguration('rtabmap_db_path').perform(context)
    # 展开路径中的 ~ 符号
    rtabmap_db_path_str = os.path.expanduser(rtabmap_db_path_str)
    
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
    # RTABMap localization configuration / RTABMap 定位配置
    rtabmap_config = os.path.join(pkg_bot_slam, 'config', 'slam', 'rtabmap_localization.yaml')
    
    # EKF configuration / EKF 配置
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
    # URDF Processing (Simplified Model) / URDF 处理（简化模型）
    # ==========================================================================
    urdf_file = os.path.join(pkg_bot_description, 'urdf', 'lekiwi_bot_simple.gazebo.xacro')
    
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
    
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': full_model_path}
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': full_model_path},
        condition=IfCondition(gui)
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'robot_description': robot_description_content
        }],
        arguments=['--ros-args', '--log-level', log_level_str]
    )
    
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
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_omni_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'simple_omni_wheel_controller'],
        output='screen'
    )
    
    omni_controller_node = Node(
        package='bot_control',
        executable='omni_controller_node',
        name='omni_controller_node',
        output='screen',
        arguments=[
            '--ros-args', '--log-level', log_level_str
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_simple_model': True,
            'publish_wheel_odom': False,
            'publish_odom_tf': False,
            'wheel_correction.wheel_1': 1.0,
            'wheel_correction.wheel_2': 1.0,
            'wheel_correction.wheel_3': 1.0,
        }]
    )
    
    # ==========================================================================
    # PHASE 3: Sensor Data Processing / 第三阶段：传感器数据处理
    # ==========================================================================
    
    imu_relay = Node(
        package='topic_tools',
        executable='relay',
        name='imu_relay',
        output='screen',
        arguments=[
            '/imu/data',
            '/imu/data',
            '--ros-args', '--log-level', log_level_str
        ],
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'RELIABLE',
        }]
    )
    
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_fallback',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom', '--ros-args', '--log-level', log_level_str],
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
        }]
    )
    
    # ==========================================================================
    # PHASE 3: RTABMap Localization / 第三阶段：RTABMap 定位
    # ==========================================================================
    
    # RTABMap Localization node / RTABMap 定位节点
    # ⚠️ Key Difference: Load existing database, no mapping
    # 关键区别：加载已有数据库，不建图
    # 
    # 🔑 关键: 数据库路径必须通过parameters['database_path']传递,而不是命令行参数!
    #    RTABMap ROS wrapper会读取database_path参数来加载数据库
    #
    # ⚠️ 重要: 数据库中存储的参数会覆盖YAML配置,需要在这里强制覆盖关键参数
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],  # 设置日志级别为WARN
        parameters=[
            rtabmap_config,  # YAML配置文件路径
            {
                'database_path': rtabmap_db_path_str,  # 数据库路径
                # ========================================================================
                # 定位模式关键参数 - 确保数据库只读
                # ========================================================================
                # ⚠️ 注意: RTABMap参数必须是字符串类型!
                'Mem/IncrementalMemory': 'false',  # ❗不建图,只定位(防止写入数据库)
                'Mem/InitWMWithAllNodes': 'true',  # 用所有节点初始化Working Memory
                'Mem/BinDataKept': 'true',  # ⚠️ 保持二进制数据(包括词汇) - 必须true才能加载词典
                'Kp/IncrementalFlann': 'true',  # ⚠️ 改为true！定位模式需要FLANN索引来匹配特征
                
                # 额外的保护措施 - 禁止任何可能修改数据库的操作
                'Mem/STMSize': '30',  # 短期记忆保留30个节点（定位需要足够的STM）
                # 'Mem/ImageKept': 'false',  # ⚠️ 注释掉！这会阻止加载词典数据
                'Mem/NotLinkedNodesKept': 'false',  # 不保存未链接的节点
                'DbSqlite3/InMemory': 'false',  # 不使用内存数据库(确保从磁盘只读)
                'DbSqlite3/JournalMode': '0',  # ⚠️ DELETE模式（只读安全）
                
                # 地图参数 - 防止地图被重新生成
                'GridGlobal/MinSize': '0.0',  # ⚠️ 使用0让RTABMap根据实际poses计算大小
                'GridGlobal/UpdateError': '100.0',  # ⚠️ 关键！防止地图频繁更新
                'GridGlobal/FullUpdate': 'false',  # 不使用完整更新（定位模式不需要）
            }
        ],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/odometry/filtered'),
            ('grid_map', '/map'),
        ],
    )
    
    # RTABMap visualization node (optional)
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
            ('odom', '/rtabmap/odom'),
        ],
        condition=IfCondition(
            PythonExpression(["'", use_rviz_str, "' == 'false'"])
        )
    )
    
    # ==========================================================================
    # PHASE 3: Obstacle Detection for Nav2 / 第三阶段：Nav2障碍物检测
    # ==========================================================================
    
    point_cloud_xyz = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
            'decimation': 2,
            'max_depth': 3.0,
            'voxel_size': 0.02,
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('cloud', '/camera/cloud'),
        ]
    )
    
    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[{
            'use_sim_time': use_sim_time_str == 'true',
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
    # PHASE 4: EKF Fusion (Wheel + IMU) / 第四阶段：EKF融合
    # ==========================================================================
    
    ekf_filter = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_str],
        parameters=[ekf_config],
    )
    
    # ==========================================================================
    # PHASE 4: Nav2 Navigation Stack / 第四阶段：Nav2 导航栈
    # ==========================================================================
    
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
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': use_sim_time_str == 'true'}],
        condition=IfCondition(use_rviz)
    )
    
    # ==========================================================================
    # Event-Driven Launch Sequence / 事件驱动启动序列
    # ==========================================================================
    
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
    
    event_rsp_started = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                LogInfo(msg='[Event] Robot state publisher started, spawning robot...'),
                spawn_robot,
            ]
        )
    )
    
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
    
    event_control_started = RegisterEventHandler(
        OnProcessStart(
            target_action=omni_controller_node,
            on_start=[
                LogInfo(msg='[Event] Control nodes started!'),
                LogInfo(msg='[Phase 3] Starting sensor processing, EKF fusion, and RTABMap Localization...'),
                # static_map_to_odom,  # RTABMap定位模式会自己发布map->odom，不需要静态变换
                imu_relay,
                ekf_filter,
                rtabmap_localization,  # Localization mode
                rtabmap_viz,
                point_cloud_xyz,
                obstacles_detection,
            ]
        )
    )
    
    event_localization_started = RegisterEventHandler(
        OnProcessStart(
            target_action=rtabmap_localization,
            on_start=[
                LogInfo(msg='[Event] RTABMap Localization started!'),
                LogInfo(msg='[Phase 4] Starting Nav2 navigation stack...'),
                nav2_bringup,
            ]
        )
    )
    
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
        LogInfo(msg='========================================'),
        LogInfo(msg='RTABMap Localization Navigation Simulation'),
        LogInfo(msg='  (Simplified Robot Model)'),
        LogInfo(msg='  (RGB-D Localization - No Mapping)'),
        LogInfo(msg='  (Wheel + IMU Fusion)'),
        LogInfo(msg='  (Event-Driven Startup Sequence)'),
        LogInfo(msg=f'  World: {world_str or "navigation_5x5_rgbd.world"}'),
        LogInfo(msg=f'  RViz: {use_rviz_str}'),
        LogInfo(msg=f'  RTABMap DB: {rtabmap_db_path_str}'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Localization Mode Features:'),
        LogInfo(msg='  - Load pre-built RTABMap database'),
        LogInfo(msg='  - Localization only (no mapping)'),
        LogInfo(msg='  - Lower computational load'),
        LogInfo(msg='  - Suitable for patrol navigation'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Startup sequence:'),
        LogInfo(msg='  [1] Gazebo server'),
        LogInfo(msg='  [2] Robot state publisher + Gazebo client'),
        LogInfo(msg='  [3] Spawn robot'),
        LogInfo(msg='  [4] Control nodes'),
        LogInfo(msg='  [5] RTABMap Localization + EKF'),
        LogInfo(msg='  [6] Nav2 navigation'),
        LogInfo(msg='  [7] RViz'),
        LogInfo(msg='========================================'),
        
        LogInfo(msg='[Phase 1] Starting Gazebo simulation environment...'),
        gazebo_server,
        
        event_gazebo_started,
        event_rsp_started,
        event_robot_spawned,
        event_control_started,
        event_localization_started,
        event_ekf_started,
    ]


def generate_launch_description():
    """
    Generate the launch description / 生成启动描述
    """
    
    pkg_bot_navigation = get_package_share_directory('bot_navigation')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock / 使用仿真（Gazebo）时钟'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='World file name or path / 世界文件名或路径'
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
        default_value=os.path.join(pkg_bot_navigation,'config' ,'rviz', 'nav2_rtabmap.rviz'),
        description='RViz config file / RViz 配置文件'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(pkg_bot_navigation, 'config', 'nav2', 'nav2_params_imu.yaml'),
        description='Nav2 parameters file / Nav2 参数文件'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal) / 日志级别'
    )
    
    # ⚠️ 新增参数：RTABMap数据库路径 / New parameter: RTABMap database path
    declare_rtabmap_db_path = DeclareLaunchArgument(
        'rtabmap_db_path',
        default_value='',
        description='Path to RTABMap database file (.db) / RTABMap数据库文件路径'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_gui,
        declare_use_rviz,
        declare_rviz_config,
        declare_nav2_params,
        declare_log_level,
        declare_rtabmap_db_path,  # 新增参数
        OpaqueFunction(function=launch_setup)
    ])
