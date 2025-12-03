#!/usr/bin/env python3
"""
Gazebo Simulation Launch File / Gazebo仿真启动文件
Launch Gazebo and load LeKiwi robot model / 启动Gazebo并加载LeKiwi机器人模型
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """
    根据参数动态设置启动配置 / Dynamically setup launch configuration based on parameters
    """
    # 获取包路径 / Get package paths
    pkg_bot_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bot_description = get_package_share_directory('bot_description')
    
    # 获取world参数的实际值 / Get actual value of world parameter
    world_param = LaunchConfiguration('world').perform(context)
    
    # 如果world为空，使用默认值 / If world is empty, use default
    if not world_param or world_param == '':
        world_file = os.path.join(pkg_bot_gazebo, 'worlds', 'cafe.world')
    else:
        # 检查是否是相对路径（只有文件名）/ Check if it's a relative path (filename only)
        if not os.path.isabs(world_param):
            # 尝试在worlds目录中查找 / Try to find in worlds directory
            world_file = os.path.join(pkg_bot_gazebo, 'worlds', world_param)
            if not os.path.exists(world_file):
                world_file = world_param  # 使用原始值 / Use original value
        else:
            world_file = world_param
    
    # URDF文件路径 / URDF file path
    urdf_file = os.path.join(pkg_bot_description, 'urdf', 'lekiwi_bot_sim.xacro')
    
    # 使用xacro处理URDF文件 / Process URDF with xacro
    robot_description_content = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    # 启动Gazebo服务器 / Start Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', 
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )
    
    # 启动Gazebo客户端 / Start Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
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
            'robot_description': robot_description_content
        }]
    )
    
    # 在Gazebo中生成机器人 / Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lekiwi_bot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
        ],
        output='screen'
    )
    
    return [
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
    ]


def generate_launch_description():
    
    # 获取包路径 / Get package paths
    pkg_bot_description = get_package_share_directory('bot_description')
    
    # 设置Gazebo模型路径 / Set Gazebo model path
    share_dir = os.path.dirname(pkg_bot_description)
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        share_dir = share_dir + ':' + gazebo_model_path
    
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=share_dir
    )
    
    # ========================================================================
    # 声明参数 / Declare arguments
    # ========================================================================
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',  # 空字符串，由OpaqueFunction处理默认值 / Empty, default handled by OpaqueFunction
        description='Gazebo world file (filename or full path) / Gazebo世界文件（文件名或完整路径）'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time / 使用仿真时间'
    )
    
    # 机器人初始位置参数 / Robot initial position arguments
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='Robot initial X position / 机器人初始X位置'
    )
    
    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='0.0',
        description='Robot initial Y position / 机器人初始Y位置'
    )
    
    robot_z_arg = DeclareLaunchArgument(
        'robot_z',
        default_value='0.1',
        description='Robot initial Z position / 机器人初始Z位置'
    )
    
    return LaunchDescription([
        # 环境变量 / Environment variables
        gazebo_resource_path,
        # 参数 / Arguments
        world_arg,
        use_sim_time_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        # 动态启动配置 / Dynamic launch setup
        OpaqueFunction(function=launch_setup),
    ])
