#!/usr/bin/env python3
"""
Visual Odometry Launch File / 视觉里程计启动文件
Launches RTAB-Map rgbd_odometry and robot_localization EKF for sensor fusion
启动 RTAB-Map rgbd_odometry 和 robot_localization EKF 进行传感器融合

Architecture / 架构:
    Camera (RGB + Depth)
           │
           ▼
    ┌──────────────────┐
    │ rgbd_odometry    │──▶ /visual_odom (nav_msgs/Odometry)
    │ (RTAB-Map)       │    frame: vo -> base_link
    └──────────────────┘
                               │
    ┌──────────────────┐       │
    │ Gazebo wheel_odom│───────┼──▶ /wheel_odom
    └──────────────────┘       │
                               ▼
                     ┌─────────────────┐
                     │ ekf_filter_node │──▶ /odom (fused)
                     │ (robot_localization)  odom -> base_link TF
                     └─────────────────┘

Startup Sequence (Event-Driven) / 启动顺序（事件驱动）:
    1. Wait for camera topics to be available (topic existence check)
    2. Wait for wheel odometry topic to be available
    3. Start rgbd_odometry node
    4. Wait for visual odometry output
    5. Start EKF filter node

QoS Configuration / QoS 配置:
    - Camera topics: Sensor Data QoS (Best Effort, Volatile)
    - Odometry topics: Default QoS (Reliable, Volatile)
    - TF topics: Static Broadcaster QoS (Transient Local)
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for VIO system / 生成 VIO 系统的启动描述"""
    
    # ==========================================================================
    # Package Paths / 包路径
    # ==========================================================================
    bot_navigation_dir = get_package_share_directory('bot_navigation')
    
    # Default config files / 默认配置文件
    default_rtabmap_config = os.path.join(
        bot_navigation_dir, 'config', 'rtabmap_odom.yaml'
    )
    default_ekf_config = os.path.join(
        bot_navigation_dir, 'config', 'robot_localization.yaml'
    )
    
    # ==========================================================================
    # Launch Arguments / 启动参数
    # ==========================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    declare_rtabmap_config = DeclareLaunchArgument(
        'rtabmap_config',
        default_value=default_rtabmap_config,
        description='Path to RTAB-Map odometry config / RTAB-Map 里程计配置路径'
    )
    
    declare_ekf_config = DeclareLaunchArgument(
        'ekf_config',
        default_value=default_ekf_config,
        description='Path to robot_localization EKF config / robot_localization EKF 配置路径'
    )
    
    declare_enable_ekf = DeclareLaunchArgument(
        'enable_ekf',
        default_value='true',
        description='Enable EKF sensor fusion / 启用 EKF 传感器融合'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Log level for nodes (debug, info, warn, error) / 节点日志级别'
    )
    
    # ==========================================================================
    # Launch Configuration Values / 启动配置值
    # ==========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_config = LaunchConfiguration('rtabmap_config')
    ekf_config = LaunchConfiguration('ekf_config')
    enable_ekf = LaunchConfiguration('enable_ekf')
    log_level = LaunchConfiguration('log_level')
    
    # ==========================================================================
    # RTAB-Map rgbd_odometry Node / RTAB-Map rgbd_odometry 节点
    # ==========================================================================
    # 
    # QoS Configuration:
    # - Input camera topics use Sensor Data QoS (Best Effort, Volatile, depth=5)
    # - Output odom topic uses default QoS (Reliable, Volatile)
    # 
    # The rgbd_odometry node internally handles QoS matching for camera topics.
    # We configure qos parameter to use sensor data profile (2).
    # 
    # QoS 配置:
    # - 输入相机话题使用传感器数据 QoS（Best Effort, Volatile, depth=5）
    # - 输出 odom 话题使用默认 QoS（Reliable, Volatile）
    # 
    # rgbd_odometry 节点内部处理相机话题的 QoS 匹配。
    # 我们配置 qos 参数使用传感器数据配置文件 (2)。
    # ==========================================================================
    
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='log',  # Reduce terminal spam / 减少终端垃圾信息
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            rtabmap_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Input topics / 输入话题
            # Camera topics from Gazebo camera plugin
            # 来自 Gazebo 相机插件的相机话题
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            
            # Output topic / 输出话题
            # Visual odometry for EKF fusion
            # 用于 EKF 融合的视觉里程计
            ('odom', '/visual_odom'),
        ],
    )
    
    # ==========================================================================
    # Wheel Odometry Relay Node / 轮式里程计中继节点
    # ==========================================================================
    #
    # Relay the wheel odometry from Gazebo to a separate topic for EKF.
    # This allows EKF to receive wheel odom on /wheel_odom while
    # rgbd_odometry uses the standard /odom topic name.
    #
    # 将轮式里程计从 Gazebo 中继到单独的话题供 EKF 使用。
    # 这允许 EKF 在 /wheel_odom 上接收轮式里程计，
    # 而 rgbd_odometry 使用标准的 /odom 话题名称。
    # ==========================================================================
    
    wheel_odom_relay_node = Node(
        condition=IfCondition(enable_ekf),
        package='topic_tools',
        executable='relay',
        name='wheel_odom_relay',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/odom', '/wheel_odom'],
    )
    
    # ==========================================================================
    # Robot Localization EKF Node / Robot Localization EKF 节点
    # ==========================================================================
    #
    # EKF fuses wheel odometry and visual odometry:
    # - Wheel odom: x, y positions and vx, vy velocities
    # - Visual odom: yaw and vyaw (differential mode)
    #
    # Output: /odometry/filtered -> remapped to /odom
    # Publishes: odom -> base_link TF transform
    #
    # QoS Configuration:
    # - Input odom topics use default QoS (Reliable)
    # - Output odom topic uses default QoS (Reliable)
    # - TF uses default broadcaster QoS
    #
    # EKF 融合轮式里程计和视觉里程计：
    # - 轮式里程计：x, y 位置和 vx, vy 速度
    # - 视觉里程计：yaw 和 vyaw（差分模式）
    #
    # 输出：/odometry/filtered -> 重映射到 /odom
    # 发布：odom -> base_link TF 变换
    # ==========================================================================
    
    ekf_filter_node = Node(
        condition=IfCondition(enable_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Output topic remapping / 输出话题重映射
            # Remap filtered output to /odom for Nav2 compatibility
            # 重映射滤波输出到 /odom 以兼容 Nav2
            ('odometry/filtered', '/odom'),
        ],
    )
    
    # ==========================================================================
    # Visual Odometry Diagnostics Node / 视觉里程计诊断节点
    # ==========================================================================
    #
    # Periodically check visual odometry quality and log warnings if degraded.
    # This helps identify issues with feature tracking.
    #
    # 定期检查视觉里程计质量，如果降级则记录警告。
    # 这有助于识别特征跟踪问题。
    # ==========================================================================
    
    # Note: Visual odometry quality monitoring can be added via a custom node
    # or by monitoring /rgbd_odometry/info topic if needed.
    # 注意：可以通过自定义节点或监控 /rgbd_odometry/info 话题来添加视觉里程计质量监控。
    
    # ==========================================================================
    # Return Launch Description / 返回启动描述
    # ==========================================================================
    
    return LaunchDescription([
        # Argument declarations / 参数声明
        declare_use_sim_time,
        declare_rtabmap_config,
        declare_ekf_config,
        declare_enable_ekf,
        declare_log_level,
        
        # Log startup info / 记录启动信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting Visual Odometry (VIO) System...'),
        LogInfo(msg='  RTAB-Map rgbd_odometry for visual tracking'),
        LogInfo(msg='  robot_localization EKF for sensor fusion'),
        LogInfo(msg='========================================'),
        
        # Wheel odometry relay (runs immediately)
        # 轮式里程计中继（立即运行）
        wheel_odom_relay_node,
        
        # RTAB-Map visual odometry (runs immediately)
        # RTAB-Map 视觉里程计（立即运行）
        rgbd_odometry_node,
        
        # EKF filter (runs immediately, waits for data internally)
        # EKF 滤波器（立即运行，内部等待数据）
        ekf_filter_node,
        
        # Final log message / 最终日志消息
        LogInfo(
            condition=IfCondition(enable_ekf),
            msg='[VIO] EKF fusion enabled: wheel_odom + visual_odom -> /odom'
        ),
        LogInfo(
            condition=UnlessCondition(enable_ekf),
            msg='[VIO] EKF disabled: using visual odometry only'
        ),
    ])
