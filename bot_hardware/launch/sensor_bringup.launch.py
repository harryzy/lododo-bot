#!/usr/bin/env python3
"""
传感器统一启动文件 - Sensor Bringup Launch
===========================================

功能：
- 启动IMU驱动和滤波节点
- 启动Astra Pro RGB-D相机
- 发布静态TF（传感器坐标系）

作者：hurry
日期：2026-01-20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成传感器启动描述"""
    
    # 声明launch参数
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='是否启用IMU (true/false)'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='是否启用Astra Pro相机 (true/false)'
    )
    
    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_tf',
        default_value='true',
        description='是否发布静态TF (true/false)'
    )
    
    # 获取launch配置
    enable_imu = LaunchConfiguration('enable_imu')
    enable_camera = LaunchConfiguration('enable_camera')
    publish_static_tf = LaunchConfiguration('publish_static_tf')
    
    # ========================================
    # IMU传感器节点
    # ========================================
    
    # YBIMU驱动节点
    ybimu_driver_node = Node(
        package='bot_hardware',
        executable='ybimu_driver',
        name='ybimu_driver',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        condition=IfCondition(enable_imu)
    )
    
    # IMU滤波节点（NED→ENU + 平滑滤波）
    imu_filter_node = Node(
        package='bot_hardware',
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'filter_window_size': 5,
        }],
        condition=IfCondition(enable_imu)
    )
    
    # ========================================
    # 相机传感器
    # ========================================
    
    # Astra Pro相机launch（UVC模式，禁用IR）
    astra_camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('astra_camera'),
                'launch',
                'astra_pro.launch.xml'
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'enable_ir': 'false',  # 禁用红外避免与RGB冲突
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_point_cloud': 'false',  # ⚠️ 禁用驱动层点云，减少USB带宽（RTABMap会生成点云）
            'use_uvc_camera': 'true',  # UVC模式获取RGB
            'uvc_camera_format': 'mjpeg',  # MJPEG格式
            'color_width': '640',
            'color_height': '480',  # ⚠️ 升级到640x480@30Hz获得更多特征点 / Upgrade to 640x480@30Hz for more features
            'color_fps': '30',  # 30fps（硬件支持，比15fps更稳定）
            'depth_width': '640',
            'depth_height': '480',  # 480p分辨率与RGB保持同步 / Synchronized resolution with RGB
            'depth_fps': '30',  # 30fps与RGB保持同步，避免RGB-D错配导致点云抖动 / 30fps synchronized with RGB to prevent point cloud jitter
            'color_depth_synchronization': 'true',  # ⚠️ 关键！启用RGB-D硬件同步
            'depth_registration': 'false',  # 不启用硬件深度对齐（RTABMap软件对齐）
            'enable_d2c_viewer': 'false',  # 禁用动态TF发布
            'publish_tf': 'false',  # ⚠️ 关键！禁用相机驱动TF，使用static TF
        }.items(),
        condition=IfCondition(enable_camera)
    )
    
    # ========================================
    # 静态TF发布（传感器坐标系）
    # ========================================
    
    # base_link → imu_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=[
            '0', '0', '0.05',  # x, y, z (IMU位于底盘上方5cm)
            '0', '0', '0',      # roll, pitch, yaw
            'base_link',
            'imu_link'
        ],
        condition=IfCondition(publish_static_tf)
    )
    
    # base_link → camera_link  
    # ⚠️ 已注释：使用robot_state_publisher从URDF发布camera_link的TF
    # static_tf_camera = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_camera',
    #     arguments=[
    #         '0.15', '0', '0.20',  # x, y, z (相机位于前方15cm，高20cm)
    #         '0', '0', '0',         # roll, pitch, yaw
    #         'base_link',
    #         'camera_link'
    #     ],
    #     condition=IfCondition(publish_static_tf)
    # )
    
    # camera_link → camera_color_frame (RGB相机坐标系)
    static_tf_camera_color = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_color',
        arguments=[
            '0', '0', '0',  # 与camera_link重合
            '0', '0', '0',
            'camera_link',
            'camera_color_frame'
        ],
        condition=IfCondition(publish_static_tf)
    )
    
    # camera_color_frame → camera_color_optical_frame (光学坐标系转换)
    # 光学坐标系: X右, Y下, Z前 (ROS相机标准)
    static_tf_camera_color_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_color_optical',
        arguments=[
            '0', '0', '0',
            '-1.5707963267948966', '0', '-1.5707963267948966',  # -90°绕X, -90°绕Z
            'camera_color_frame',
            'camera_color_optical_frame'
        ],
        condition=IfCondition(publish_static_tf)
    )
    
    # camera_link → camera_depth_frame (深度相机坐标系)
    static_tf_camera_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_depth',
        arguments=[
            '0', '0', '0',  # 与camera_link重合
            '0', '0', '0',
            'camera_link',
            'camera_depth_frame'
        ],
        condition=IfCondition(publish_static_tf)
    )
    
    # camera_depth_frame → camera_depth_optical_frame (深度光学坐标系)
    static_tf_camera_depth_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_depth_optical',
        arguments=[
            '0', '0', '0',
            '-1.5707963267948966', '0', '-1.5707963267948966',  # -90°绕X, -90°绕Z
            'camera_depth_frame',
            'camera_depth_optical_frame'
        ],
        condition=IfCondition(publish_static_tf)
    )
    
    return LaunchDescription([
        # 参数声明
        enable_imu_arg,
        enable_camera_arg,
        publish_static_tf_arg,
        
        # IMU节点
        ybimu_driver_node,
        imu_filter_node,
        
        # 相机启动
        astra_camera_launch,
        
        # 静态TF  
        static_tf_imu,
        # static_tf_camera,  # ⚠️ 已删除：使用robot_state_publisher从URDF发布
        static_tf_camera_color,
        static_tf_camera_color_optical,
        static_tf_camera_depth,
        static_tf_camera_depth_optical,
    ])
