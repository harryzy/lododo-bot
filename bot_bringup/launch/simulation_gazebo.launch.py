#!/usr/bin/env python3
"""
LeKiwi Robot Simulation Complete Launch File / LeKiwi机器人仿真完整启动文件
Including Gazebo, controllers, perception nodes, RViz, etc. / 包括Gazebo、控制器、感知节点、RViz等
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # ========================================================================
    # 声明参数 / Declare arguments
    # ========================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz / 是否启动RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('bot_gazebo'),
            'config',
            'gazebo_bot.rviz'
        ]),
        description='RViz config file path / RViz配置文件路径'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',  # 空字符串表示使用 gazebo.launch.py 的默认值 / Empty means use default in gazebo.launch.py
        description='Gazebo world file path / Gazebo世界文件路径'
    )
    
    # ========================================================================
    # 包含子启动文件 / Include sub-launch files
    # ========================================================================
    
    # 包含Gazebo启动文件 / Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_gazebo'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
        }.items()
    )
    
    # 包含感知节点启动文件（在虚拟环境中运行）/ Include perception node launch file (running in virtual environment)
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
    
    # ========================================================================
    # 控制器节点 / Controller nodes
    # ========================================================================
    
    # 全向轮控制器 / Omni-directional wheel controller
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
    
    # 轮子关节状态发布器 / Wheel joint state publisher
    wheel_joint_publisher = Node(
        package='bot_control',
        executable='wheel_joint_publisher',
        name='wheel_joint_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # ========================================================================
    # RViz（可选）/ RViz (optional)
    # ========================================================================
    
    # RViz2 可视化 / RViz2 visualization
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        # 参数 / Arguments
        use_sim_time_arg,
        use_rviz_arg,
        rviz_config_arg,
        world_arg,
        # 启动项 / Launch items
        gazebo_launch,
        perception_launch,
        omni_controller,
        wheel_joint_publisher,
        rviz,
    ])
