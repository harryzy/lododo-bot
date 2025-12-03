#!/usr/bin/env python3
"""
LeKiwi机器人仿真完整启动文件
包括Gazebo、控制器、感知节点、RViz等
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    # 包含Gazebo启动文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_gazebo'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # 包含感知节点启动文件（在虚拟环境中运行）
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_perception'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # 全向轮控制器
    omni_controller = Node(
        package='bot_control',
        executable='omni_controller',
        name='omni_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_radius': 0.05,
            'rear_wheel_dist': 0.105,
            'front_wheel_dist': 0.085,
            'max_wheel_speed': 4.712,
            'publish_rate': 50.0
        }]
    )
    
    # 轮子关节状态发布器
    wheel_joint_publisher = Node(
        package='bot_control',
        executable='wheel_joint_publisher',
        name='wheel_joint_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # RViz2可视化
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bot_gazebo'),
            'config',
            'gazebo_bot.rviz'
        ])],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gazebo_launch,
        perception_launch,  # 感知节点（虚拟环境）
        omni_controller,
        wheel_joint_publisher,
        rviz,
    ])
