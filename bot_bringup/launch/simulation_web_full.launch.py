from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    """
    完整Web控制环境启动文件
    
    包含：
    1. Gazebo + 机器人
    2. Nav2导航栈
    3. RTABMap SLAM/定位
    4. rosbridge_server
    5. Web服务器
    """
    
    # 声明参数
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='启用SLAM建图模式（true）或定位模式（false）'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='exploration_test',
        description='定位模式下使用的地图名称'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='是否启动Gazebo GUI'
    )
    
    # 获取参数
    slam = LaunchConfiguration('slam')
    map_name = LaunchConfiguration('map_name')
    gui = LaunchConfiguration('gui')
    
    # 1. 启动基础仿真环境（Gazebo + Nav2 + RTABMap）
    base_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bot_bringup'),
                'launch',
                'simple_simulation_nav2_rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'slam': slam,
            'map_name': map_name,
            'gui': gui,
            'use_sim_time': 'true'
        }.items()
    )
    
    # 2. 启动rosbridge_server（用于前端 ROS 连接）
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # 注意：Web服务器（FastAPI）需要手动启动
    # 在新终端运行：cd ~/lododo_bot/src/bot_teleop && bash scripts/start_web_server.sh
    
    return LaunchDescription([
        slam_arg,
        map_name_arg,
        gui_arg,
        base_simulation,
        rosbridge_server,
    ])
