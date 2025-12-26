#!/usr/bin/env python3
"""
waypoint_recorder.launch.py - 启动路点录制器

启动 WaypointRecorderNode 用于交互式路点录制
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""
    
    # 声明参数
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/rtabmap/localization_pose',
        description='Pose topic to subscribe to'
    )
    
    backup_pose_topic_arg = DeclareLaunchArgument(
        'backup_pose_topic',
        default_value='/odom',
        description='Backup odometry topic'
    )
    
    use_odom_backup_arg = DeclareLaunchArgument(
        'use_odom_backup',
        default_value='true',
        description='Use odometry as backup pose source'
    )
    
    recording_interval_arg = DeclareLaunchArgument(
        'recording_interval',
        default_value='1.0',
        description='Auto recording interval in seconds'
    )
    
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.5',
        description='Minimum distance between waypoints in meters'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for waypoints'
    )
    
    # WaypointRecorder 节点
    waypoint_recorder_node = Node(
        package='bot_navigation',
        executable='waypoint_recorder',
        name='waypoint_recorder',
        output='screen',
        parameters=[{
            'pose_topic': LaunchConfiguration('pose_topic'),
            'backup_pose_topic': LaunchConfiguration('backup_pose_topic'),
            'use_odom_backup': LaunchConfiguration('use_odom_backup'),
            'recording_interval': LaunchConfiguration('recording_interval'),
            'min_distance': LaunchConfiguration('min_distance'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        emulate_tty=True,
        prefix='xterm -e',  # 在独立终端窗口中运行
    )
    
    return LaunchDescription([
        pose_topic_arg,
        backup_pose_topic_arg,
        use_odom_backup_arg,
        recording_interval_arg,
        min_distance_arg,
        frame_id_arg,
        LogInfo(msg='Starting Waypoint Recorder...'),
        waypoint_recorder_node,
    ])
