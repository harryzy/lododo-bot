#!/usr/bin/env python3
"""
Keyboard Teleop Launch File / Keyboard Teleop Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    keyboard_teleop = Node(
        package='bot_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',  # Run in new terminal / Run in new terminal
    )
    
    return LaunchDescription([
        keyboard_teleop,
    ])
