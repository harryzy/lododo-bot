"""
CommandAdapter启动文件 / CommandAdapter launch file

启动统一命令接口的核心节点
Launch the core node of unified command interface
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成launch描述 / Generate launch description"""
    
    # 获取包路径 / Get package path
    pkg_dir = get_package_share_directory('bot_cmd_interface')
    
    # 默认配置文件路径 / Default config file path
    default_config_file = os.path.join(pkg_dir, 'config', 'command_config.yaml')
    
    # 声明launch参数 / Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to command adapter configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug/info/warn/error)'
    )
    
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Use mock mode (true) or real services (false)'
    )
    
    service_timeout_arg = DeclareLaunchArgument(
        'service_timeout',
        default_value='10.0',
        description='Service call timeout in seconds'
    )
    
    # CommandAdapter节点 / CommandAdapter node
    command_adapter_node = Node(
        package='bot_cmd_interface',
        executable='command_adapter',
        name='command_adapter',
        output='screen',
        parameters=[
            {
                'config_file': LaunchConfiguration('config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_mock': LaunchConfiguration('use_mock'),
                'service_timeout': LaunchConfiguration('service_timeout'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # 可选：重映射Topic名称 / Optional: Remap topic names
            # ('/cmd/request', '/robot/cmd/request'),
            # ('/cmd/response', '/robot/cmd/response'),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        use_mock_arg,
        service_timeout_arg,
        command_adapter_node,
    ])
