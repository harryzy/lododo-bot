#!/usr/bin/env python3
"""
简化机器人仿真启动文件
启动 Gazebo + 简化 URDF + 控制器
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 包路径
    pkg_bot_description = FindPackageShare('bot_description')
    pkg_bot_gazebo = FindPackageShare('bot_gazebo')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    
    # Launch 参数
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Gazebo world 文件名'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    # Gazebo 服务器
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={
            'world': PathJoinSubstitution([
                pkg_bot_gazebo,
                'worlds',
                LaunchConfiguration('world')
            ]),
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )
    
    # Gazebo 客户端
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        )
    )
    
    # Robot State Publisher（发布简化 URDF 的 TF）
    urdf_path = PathJoinSubstitution([
        pkg_bot_description,
        'urdf',
        'lekiwi_bot_simple.gazebo.xacro'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
        }]
    )
    
    # 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lekiwi_simple',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'  # 稍微抬高避免穿透地面
        ],
        output='screen'
    )
    
    # 加载并启动控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('bot_control'),
                'config',
                'simple_omni_controller.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # 启动 joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # 启动 simple_omni_wheel_controller
    omni_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_omni_wheel_controller'],
        output='screen'
    )
    
    # 启动全向轮控制节点（使用简化模型模式）
    omni_controller_node = Node(
        package='bot_control',
        executable='omni_controller_node',
        name='omni_controller',
        output='screen',
        parameters=[{
            'use_simple_model': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner,
        omni_controller_spawner,
        omni_controller_node,
    ])
