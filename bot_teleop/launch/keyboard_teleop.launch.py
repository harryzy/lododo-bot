#!/usr/bin/env python3
"""
键盘遥控启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    keyboard_teleop = Node(
        package='bot_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',  # 在新终端中运行
    )
    
    return LaunchDescription([
        keyboard_teleop,
    ])
