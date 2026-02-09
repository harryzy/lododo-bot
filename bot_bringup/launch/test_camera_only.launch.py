"""
测试相机独立运行 - USB资源隔离测试
Test Camera Only - USB Resource Isolation Test

用途 / Purpose:
  验证相机在无其他USB设备干扰时是否稳定工作
  
运行 / Run:
  ros2 launch bot_bringup test_camera_only.launch.py
  
验证 / Verify:
  ros2 topic hz /camera/color/image_raw  # 应该稳定15Hz
  ros2 topic hz /camera/depth/image_raw  # 应该稳定30Hz
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_hardware = get_package_share_directory('bot_hardware')
    
    # 仅启动相机
    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hardware, 'launch', 'sensor_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'enable_imu': 'false',      # 关闭IMU（减少USB设备）
            'enable_camera': 'true',
        }.items()
    )
    
    return LaunchDescription([
        sensor_bringup,
    ])
