#!/usr/bin/env python3
"""
Web Server Launch File
Launch rosbridge_server and Web Terminal Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    
    # Declare parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # rosbridge_server node
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 9090,  # ROSBridge WebSocket port
        }],
        output='screen'
    )
    
    # web_video_server node (optional)
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 8080,  # Video stream port
        }],
        output='screen'
    )
    
    # Launch FastAPI server (using ExecuteProcess)
    teleop_dir = Path(__file__).parent.parent
    fastapi_server = ExecuteProcess(
        cmd=[
            'bash',
            str(teleop_dir / 'scripts' / 'start_web_server.sh')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        rosbridge_server,
        # web_video_server,  # 取消注释以启用视频流
        # fastapi_server,  # 注意：FastAPI 建议手动启动，避免 launch 文件管理复杂
    ])
