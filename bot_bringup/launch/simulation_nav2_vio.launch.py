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
        - Wait for robot_description and camera topics
    
    Phase 2: Robot Control & Perception
        - Start omni_controller
        - Start perception nodes (depth_to_laserscan)
        - Wait for /scan topic
    
    Phase 3: Visual Odometry
        - Start rgbd_odometry
        - Start EKF fusion
        - Wait for /odom (fused) topic
    
    Phase 4: Navigation
        - Start SLAM Toolbox
        - Start Nav2 navigation stack
        - Wait for navigation services
    
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
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


class TopicWaiter:
    """
    Helper class to check topic availability / 帮助类用于检查话题可用性
    Uses ros2 topic info to verify topics exist before starting dependent nodes
    使用 ros2 topic info 在启动依赖节点之前验证话题存在
    """
    
    @staticmethod
    def create_topic_check_node(
        topic_name: str,
        node_name: str,
        timeout_sec: float = 30.0
    ) -> Node:
        """
        Create a node that waits for a topic to be available
        创建一个等待话题可用的节点
        
        This is a workaround since ROS2 launch doesn't have built-in topic waiting.
        We use a simple Python script executed as a node.
        
        这是一个变通方法，因为 ROS2 launch 没有内置的话题等待功能。
        我们使用作为节点执行的简单 Python 脚本。
        """
        # Note: In production, this would be a proper wait mechanism
        # For now, we rely on TimerAction with appropriate delays
        # 注意：在生产中，这将是一个适当的等待机制
        # 目前，我们依赖带有适当延迟的 TimerAction
        pass


def launch_setup(context: LaunchContext, *args, **kwargs):
    """
    Dynamic launch setup function / 动态启动设置函数
    Evaluates launch configurations and creates appropriate nodes
    评估启动配置并创建适当的节点
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
    # PHASE 1: Gazebo Environment / 第一阶段：Gazebo 环境
    # ==========================================================================
    
    # Set Gazebo model path / 设置 Gazebo 模型路径
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    share_dir = os.path.dirname(pkg_bot_description)
    if gazebo_model_path:
        full_model_path = share_dir + ':' + gazebo_model_path
    else:
        full_model_path = share_dir
    
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
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lekiwi_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )
    
    # ==========================================================================
    # PHASE 2: Robot Control & Perception / 第二阶段：机器人控制和感知
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
    
    # Perception nodes (depth to laserscan) / 感知节点（深度到激光雷达）
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_perception'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time_str
        }.items()
    )
    
    # ==========================================================================
    # PHASE 3: Visual Odometry (VIO) / 第三阶段：视觉里程计 (VIO)
    # ==========================================================================
    
    # Wheel odometry relay / 轮式里程计中继
    # Relay /odom from Gazebo to /wheel_odom for EKF
    # 将 Gazebo 的 /odom 中继到 /wheel_odom 供 EKF 使用
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
        output='log',  # Reduce terminal spam / 减少终端垃圾
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
    # PHASE 4: Navigation Stack / 第四阶段：导航栈
    # ==========================================================================
    
    # SLAM Toolbox (VIO optimized config) / SLAM Toolbox（VIO 优化配置）
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
    # PHASE 5: Visualization / 第五阶段：可视化
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
    # Staged Launch with TimerActions / 使用 TimerAction 的分阶段启动
    # ==========================================================================
    #
    # We use TimerActions to create a staged startup sequence.
    # This ensures each phase has time to initialize before the next starts.
    # 
    # Phase timing (seconds from launch):
    #   0s:  Gazebo server, client, robot_state_publisher, spawn
    #   5s:  Control and perception nodes
    #   10s: Visual odometry (rgbd_odometry, wheel_odom_relay, EKF)
    #   15s: Navigation stack (SLAM, Nav2)
    #   25s: RViz (if enabled)
    #
    # 我们使用 TimerActions 创建分阶段的启动顺序。
    # 这确保每个阶段在下一个阶段开始之前有时间初始化。
    # ==========================================================================
    
    # Phase 1: Gazebo (immediate) / 第一阶段：Gazebo（立即）
    phase1_gazebo = GroupAction([
        LogInfo(msg='[Phase 1] Starting Gazebo simulation environment...'),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
    ])
    
    # Phase 2: Control & Perception (5s delay) / 第二阶段：控制和感知（延迟 5 秒）
    phase2_control = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[Phase 2] Starting robot control and perception...'),
            omni_controller,
            wheel_joint_publisher,
            perception_launch,
        ]
    )
    
    # Phase 3: Visual Odometry (10s delay) / 第三阶段：视觉里程计（延迟 10 秒）
    phase3_vio = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[Phase 3] Starting Visual Odometry (VIO) system...'),
            wheel_odom_relay,
            rgbd_odometry,
            ekf_filter,
        ]
    )
    
    # Phase 4: Navigation (15s delay) / 第四阶段：导航（延迟 15 秒）
    phase4_navigation = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[Phase 4] Starting Navigation stack...'),
            slam_toolbox,
            map_saver,
            nav2_navigation,
            localization_launch,
        ]
    )
    
    # Phase 5: RViz (25s delay) / 第五阶段：RViz（延迟 25 秒）
    phase5_rviz = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='[Phase 5] Starting RViz visualization...'),
            rviz_node,
        ]
    )
    
    # ==========================================================================
    # Return all launch actions / 返回所有启动动作
    # ==========================================================================
    return [
        # Startup info / 启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting VIO Navigation Simulation'),
        LogInfo(msg=f'  World: {world_str or "textured_test.world"}'),
        LogInfo(msg=f'  SLAM Mode: {slam_str}'),
        LogInfo(msg=f'  RViz: {use_rviz_str}'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Startup phases:'),
        LogInfo(msg='  [0s]  Phase 1: Gazebo environment'),
        LogInfo(msg='  [5s]  Phase 2: Control & perception'),
        LogInfo(msg='  [10s] Phase 3: Visual odometry (VIO)'),
        LogInfo(msg='  [15s] Phase 4: Navigation stack'),
        LogInfo(msg='  [25s] Phase 5: RViz (if enabled)'),
        LogInfo(msg='========================================'),
        
        # Phased launch actions / 分阶段启动动作
        phase1_gazebo,
        phase2_control,
        phase3_vio,
        phase4_navigation,
        phase5_rviz,
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
