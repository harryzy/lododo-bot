#!/usr/bin/env python3
"""
Gazebo仿真启动文件
启动Gazebo并加载LeKiwi机器人模型
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # 获取包路径
    pkg_bot_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bot_description = get_package_share_directory('bot_description')
    
    # 世界文件路径
    world_file = os.path.join(pkg_bot_gazebo, 'worlds', 'empty.world')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_bot_description, 'urdf', 'lekiwi_bot_sim.xacro')
    
    # 声明参数
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Gazebo世界文件路径'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    # 启动Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', LaunchConfiguration('world'), '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open(urdf_file).read()
        }]
    )
    
    # 在Gazebo中生成机器人
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lekiwi_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
