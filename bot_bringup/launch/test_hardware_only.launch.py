"""
测试硬件层独立运行 - USB资源隔离测试
Test Hardware Only - USB Resource Isolation Test

用途 / Purpose:
  验证舵机+IMU在无相机干扰时是否稳定工作
  
运行 / Run:
  ros2 launch bot_bringup test_hardware_only.launch.py
  
验证 / Verify:
  ros2 topic hz /wheel/odom        # 应该稳定50Hz
  ros2 topic hz /imu/data          # 应该稳定100Hz
  ros2 node info /omni_hardware_node  # 检查节点状态
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_hardware = get_package_share_directory('bot_hardware')
    pkg_description = get_package_share_directory('bot_description')
    
    # Robot model
    urdf_file = os.path.join(pkg_description, 'urdf', 'lekiwi_bot_simple.urdf.xacro')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': False
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # 仅启动舵机+IMU
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hardware, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )
    
    # IMU传感器
    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hardware, 'launch', 'sensor_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'enable_imu': 'true',
            'enable_camera': 'false',   # 关闭相机（减少USB带宽）
        }.items()
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        hardware_bringup,
        sensor_bringup,
    ])
