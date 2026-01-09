#!/usr/bin/env python3
"""
Web服务器 Launch文件
启动 rosbridge_server 和 Web Terminal Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # rosbridge_server 节点
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 9090,  # ROSBridge WebSocket 端口
        }],
        output='screen'
    )
    
    # web_video_server 节点（可选）
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 8080,  # 视频流端口
        }],
        output='screen'
    )
    
    # 启动 FastAPI 服务器（使用 ExecuteProcess）
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
