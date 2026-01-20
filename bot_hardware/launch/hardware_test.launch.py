#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LeKiwi Robot Hardware Test Launch File
全向轮机器人硬件测试启动文件

功能 / Features:
- 启动OmniHardwareInterface
- 加载ros2_control controller_manager
- 启动控制器 (joint_state_broadcaster + omni_wheel_controller)
- 可选: 启动keyboard teleop用于手动测试

测试目的 / Test Purpose:
- 验证ros2_control硬件接口集成
- 验证state_interfaces和command_interfaces工作正常
- 验证/cmd_vel → 轮速控制流程
- 验证/wheel/odom发布

使用方法 / Usage:
    # 基础测试 (仅硬件接口+控制器)
    ros2 launch bot_hardware hardware_test.launch.py
    
    # 带键盘控制
    ros2 launch bot_hardware hardware_test.launch.py enable_teleop:=true
    
    # 使用自定义配置
    ros2 launch bot_hardware hardware_test.launch.py \\
        config_file:=/path/to/custom_config.yaml

设计参考 / Design Reference:
- HARDWARE_DEPLOYMENT_DESIGN.md §3.4.2 (行3350-3550)
- P2阶段实现路线: P2.3 URDF集成

创建日期 / Created: 2026-01-19
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成Launch描述 / Generate launch description"""
    
    # ============================================================================
    # Launch参数声明 / Launch Arguments Declaration
    # ============================================================================
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('bot_hardware'),
            'config',
            'hardware_config.yaml'
        ]),
        description='硬件配置文件路径 / Path to hardware config file'
    )
    
    declare_controller_config = DeclareLaunchArgument(
        'controller_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('bot_hardware'),
            'config',
            'controllers.yaml'
        ]),
        description='控制器配置文件路径 / Path to controller config file'
    )
    
    declare_enable_teleop = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='是否启动键盘控制 / Enable keyboard teleop'
    )
    
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='是否为仿真模式 / Simulation mode'
    )
    
    # ============================================================================
    # 配置变量 / Configuration Variables
    # ============================================================================
    
    config_file = LaunchConfiguration('config_file')
    controller_config = LaunchConfiguration('controller_config')
    enable_teleop = LaunchConfiguration('enable_teleop')
    use_sim = LaunchConfiguration('use_sim')
    
    # URDF生成 / URDF Generation
    # 使用xacro处理bot_hardware.ros2_control.xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('bot_hardware'),
            'urdf',
            'bot_hardware.ros2_control.xacro'
        ]),
        ' name:=bot_hardware_interface',
        ' config_package:=bot_hardware',
        ' config_file:=', config_file,
        ' use_sim:=', use_sim
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # ============================================================================
    # 节点定义 / Node Definitions
    # ============================================================================
    
    # 1. Robot State Publisher (发布TF变换)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 2. Controller Manager (ros2_control核心)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            controller_config
        ]
    )
    
    # 3. Joint State Broadcaster (发布/joint_states)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # 4. Omni Wheel Controller (接收/cmd_vel)
    omni_wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_wheel_controller'],
        output='screen'
    )
    
    # 5. Keyboard Teleop (可选)
    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # 在新终端窗口打开
        condition=IfCondition(enable_teleop)
    )
    
    # ============================================================================
    # 事件处理器 / Event Handlers (启动顺序控制)
    # ============================================================================
    
    # 延迟启动控制器 - 等待controller_manager初始化
    # Delay controller spawning until controller_manager is ready
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                LogInfo(msg='Controller manager started, waiting 2s before loading controllers...'),
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )
    
    # 串行加载控制器 - joint_state_broadcaster → omni_wheel_controller
    delay_omni_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                LogInfo(msg='Joint state broadcaster loaded, waiting 1s before loading omni controller...'),
                TimerAction(
                    period=1.0,
                    actions=[omni_wheel_controller_spawner]
                )
            ]
        )
    )
    
    # ============================================================================
    # Launch描述组装 / Launch Description Assembly
    # ============================================================================
    
    return LaunchDescription([
        # 参数声明 / Argument declarations
        declare_config_file,
        declare_controller_config,
        declare_enable_teleop,
        declare_use_sim,
        
        # 节点启动 / Node launches
        robot_state_publisher,
        controller_manager,
        
        # 事件处理器 / Event handlers
        delay_joint_state_broadcaster,
        delay_omni_wheel_controller,
        
        # 可选节点 / Optional nodes
        teleop_twist_keyboard,
        
        # 启动信息 / Launch info
        LogInfo(msg='========================================'),
        LogInfo(msg='LeKiwi Robot Hardware Test Launch'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Config file: '), LogInfo(msg=config_file),
        LogInfo(msg='Controller config: '), LogInfo(msg=controller_config),
        LogInfo(msg='Teleop enabled: '), LogInfo(msg=enable_teleop),
        LogInfo(msg='========================================'),
    ])


if __name__ == '__main__':
    generate_launch_description()
