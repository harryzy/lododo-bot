# LeKiwi机器人技术设计方案

**项目名称**: LeKiwi全向移动机器人系统  
**ROS版本**: ROS2 Humble  
**开发语言**: Python 3.10  
**目标平台**: 树莓派4B + Ubuntu 22.04  
**更新日期**: 2025年12月2日

---

## 目录
1. [系统概述](#1-系统概述)
2. [硬件配置](#2-硬件配置)
3. [运动学分析](#3-运动学分析)
4. [功能包架构](#4-功能包架构)
5. [依赖插件与工具](#5-依赖插件与工具)
6. [详细设计](#6-详细设计)
7. [开发路线图](#7-开发路线图)
8. [待确认事项](#8-待确认事项)

---

## 1. 系统概述

### 1.1 设计目标
LeKiwi是一个基于ROS2 Humble的全向移动机器人平台，具备以下核心能力：
- ✅ 全向移动（三轮全向轮配置）
- ✅ 自主导航与避障
- ✅ 2D/3D建图与定位
- ✅ 物体识别与跟踪
- ✅ 语音控制
- ✅ 远程操控（键盘/网页）
- ✅ Gazebo仿真支持

### 1.2 系统特点
- **全向移动**: 三个全向轮120°均布，可实现原地旋转和全方向平移
- **无IMU设计**: 采用软件里程计融合方案（视觉里程计 + 轮式里程计）
- **RGB-D感知**: 使用奥比中光Astra Pro深度相机
- **模块化设计**: 功能包解耦，支持仿真和实体平滑切换

---

## 2. 硬件配置

### 2.1 核心硬件清单

| 组件类型 | 型号/规格 | 数量 | 说明 |
|---------|----------|------|------|
| **主控** | 树莓派4B (4GB+) | 1 | Ubuntu 22.04 Server |
| **驱动电机** | ST3215舵机 | 3 | TTL串口协议，1000000波特率 |
| **舵机控制器** | USB转TTL串口 | 1 | /dev/ttyACM0 |
| **全向轮** | 100mm全向轮 | 3 | 轮子半径: 0.05m |
| **深度相机** | 奥比中光Astra Pro | 1 | RGB-D，支持ROS2 |
| **电源** | 12V 5.2Ah锂电池 | 1 | DC5521接口 |
| **网络** | WiFi/以太网 | 1 | 远程控制 |

### 2.2 传感器配置

#### RGB-D相机 (Astra Pro)
- **分辨率**: RGB 1920×1080, 深度 640×480
- **视场角**: 60° H × 49.5° V
- **深度范围**: 0.6m - 8m
- **帧率**: 30fps
- **接口**: USB 3.0
- **ROS2驱动**: `astra_camera` (官方ROS2包)

#### 软件里程计方案（无硬件IMU）
由于树莓派4B没有内置IMU，我们采用以下融合方案：

**方案A: 轮式里程计 + 视觉里程计融合（推荐）**
- **轮式里程计**: 基于电机编码器反馈（ST3215支持位置读取）
- **视觉里程计**: 使用`rtabmap_ros`的视觉里程计功能
- **融合方式**: 通过`robot_localization`进行EKF融合
- **精度**: 适中，成本低，无需额外硬件

**方案B: 添加硬件IMU（可选升级）**
- **推荐型号**: MPU6050/MPU9250（I2C接口，￥10-30元）
- **优势**: 提供实时姿态和角速度，提高里程计精度
- **安装**: 连接树莓派GPIO I2C引脚
- **驱动**: `mpu6050_driver` ROS2包

**选择建议**: 先采用方案A进行开发测试，如果发现定位漂移严重，再升级到方案B。

---

## 3. 运动学分析

### 3.1 三轮全向轮布局

根据URDF文件和实测数据，三个全向轮的安装位置：

```
轮子1 (后轮 - 正Y轴负方向):
  - 电机支架位置: (-0.02, -0.1, 0) 相对于base_plate
  - 转换到base_link: (0.1, 0.02, 0)  [经过-90°旋转]
  - 到中心距离: L1 = 0.105m ✅ (实测确认)
  - 方向角: θ1 = 90° (指向Y轴正方向)

轮子2 (右前轮 - 120°偏移):
  - 电机支架位置: (0.07928203, 0.02267949, 0)
  - 转换到base_link: (-0.02267949, 0.07928203, 0)
  - 到中心距离: L2 = 0.085m ✅ (实测确认)
  - 方向角: θ2 = 210° (120°偏移)

轮子3 (左前轮 - 240°偏移):
  - 电机支架位置: (-0.05928203, 0.05732051, 0)
  - 转换到base_link: (-0.05732051, -0.05928203, 0)
  - 到中心距离: L3 = 0.085m ✅ (实测确认)
  - 方向角: θ3 = 330° (240°偏移)
```

**注意**: 由于轮子1的距离(0.105m)与轮子2、3(0.085m)不同，这是**非对称布局**，需要在运动学计算中特别处理。

### 3.2 全向轮运动学模型

#### 正向运动学（轮速 → 机器人速度）
```
Vx = f1(ω1, ω2, ω3)
Vy = f2(ω1, ω2, ω3)
ωz = f3(ω1, ω2, ω3)
```

#### 逆向运动学（机器人速度 → 轮速）
```python
# 参数定义（已实测确认）
R = 0.05    # 轮子半径 (m) ✅
L1 = 0.105  # 后轮到中心距离 (m) ✅
L2 = 0.085  # 右前轮到中心距离 (m) ✅
L3 = 0.085  # 左前轮到中心距离 (m) ✅

# 轮子方向角（全向轮滚子方向）
θ1 = 90°    # 后轮：Y轴方向
θ2 = 210°   # 右前轮：120°偏移
θ3 = 330°   # 左前轮：240°偏移

# 运动学矩阵（三轮全向轮）
# ω1 = (1/R) * (-sin(θ1)*Vx + cos(θ1)*Vy + L1*ωz)
# ω2 = (1/R) * (-sin(θ2)*Vx + cos(θ2)*Vy + L2*ωz)
# ω3 = (1/R) * (-sin(θ3)*Vx + cos(θ3)*Vy + L3*ωz)
```

### 3.3 里程计发布

**话题**: `/odom` (nav_msgs/Odometry)  
**坐标系**: `odom` → `base_link`  
**更新频率**: 50Hz  
**数据来源**: 
- ST3215舵机位置反馈（主要）
- 视觉里程计辅助（通过robot_localization融合）

---

## 4. 功能包架构

### 4.1 功能包划分

```
src/
├── bot_description/           # 已存在 - 机器人模型描述
│   ├── urdf/                 # URDF/Xacro模型
│   ├── meshes/               # 3D网格文件
│   ├── config/               # RViz配置
│   └── launch/               # 模型可视化launch
│
├── bot_bringup/              # 【核心】系统启动与集成
│   ├── launch/
│   │   ├── robot.launch.py           # 真实硬件总启动
│   │   ├── simulation.launch.py     # Gazebo仿真总启动
│   │   ├── navigation.launch.py     # 导航系统启动
│   │   └── perception.launch.py     # 感知系统启动
│   ├── config/
│   │   ├── nav2_params.yaml          # Nav2参数
│   │   ├── slam_params.yaml          # SLAM参数
│   │   └── robot_localization.yaml   # EKF融合参数
│   └── params/
│
├── bot_hardware/             # 硬件驱动层
│   ├── bot_hardware/
│   │   ├── st3215_driver.py          # ST3215舵机驱动
│   │   ├── servo_controller.py       # 舵机控制器接口
│   │   └── hardware_interface.py     # ROS2 Control硬件接口
│   ├── config/
│   │   └── hardware_config.yaml
│   └── launch/
│       └── hardware.launch.py
│
├── bot_control/              # 运动控制
│   ├── bot_control/
│   │   ├── omni_controller.py        # 全向轮运动学控制器
│   │   ├── velocity_smoother.py      # 速度平滑器
│   │   └── odometry_publisher.py     # 里程计发布
│   ├── config/
│   │   ├── control_config.yaml
│   │   └── ros2_control.yaml         # ROS2 Control配置
│   └── launch/
│       └── control.launch.py
│
├── bot_navigation/           # 导航系统
│   ├── config/
│   │   ├── nav2/                     # Nav2完整配置
│   │   ├── costmap_common.yaml
│   │   ├── local_costmap.yaml
│   │   ├── global_costmap.yaml
│   │   └── planner_server.yaml
│   ├── maps/                         # 保存的地图
│   └── launch/
│       ├── navigation.launch.py
│       └── localization.launch.py
│
├── bot_slam/                 # 建图与定位
│   ├── config/
│   │   ├── slam_toolbox.yaml         # 2D SLAM配置
│   │   ├── rtabmap.yaml              # 3D SLAM配置
│   │   └── cartographer.yaml         # 备用方案
│   ├── launch/
│   │   ├── slam_2d.launch.py         # 2D建图
│   │   ├── slam_3d.launch.py         # 3D建图
│   │   └── map_saver.launch.py       # 地图保存工具
│   └── maps/
│
├── bot_perception/           # 感知系统
│   ├── bot_perception/
│   │   ├── object_detector.py        # 物体检测
│   │   ├── object_tracker.py         # 物体跟踪
│   │   └── obstacle_detector.py      # 障碍物检测
│   ├── config/
│   │   ├── camera_config.yaml
│   │   ├── detection_config.yaml
│   │   └── yolo_config.yaml
│   ├── launch/
│   │   ├── camera.launch.py          # Astra Pro驱动
│   │   └── perception.launch.py
│   └── models/                       # AI模型文件
│
├── bot_voice/                # 语音控制
│   ├── bot_voice/
│   │   ├── voice_recognizer.py       # Vosk语音识别
│   │   ├── command_parser.py         # 命令解析
│   │   └── tts_node.py               # 语音合成（可选）
│   ├── config/
│   │   └── voice_commands.yaml       # 命令映射
│   ├── launch/
│   │   └── voice.launch.py
│   └── models/
│       └── vosk-model-small-cn/      # 中文语音模型
│
├── bot_teleop/               # 远程操控
│   ├── bot_teleop/
│   │   ├── keyboard_teleop.py        # 键盘控制
│   │   ├── web_server.py             # Web服务器
│   │   └── gamepad_teleop.py         # 手柄控制（可选）
│   ├── web/                          # Web界面资源
│   │   ├── index.html
│   │   ├── style.css
│   │   └── app.js
│   ├── config/
│   │   └── teleop_config.yaml
│   └── launch/
│       ├── keyboard_teleop.launch.py
│       └── web_teleop.launch.py
│
└── bot_gazebo/               # Gazebo仿真
    ├── worlds/
    │   ├── empty.world
    │   ├── house.world
    │   └── warehouse.world
    ├── models/                       # 自定义模型
    ├── config/
    │   └── gazebo_config.yaml
    └── launch/
        ├── gazebo.launch.py
        └── spawn_robot.launch.py
```

### 4.2 话题与服务设计

#### 核心话题
```yaml
# 控制命令
/cmd_vel                    # geometry_msgs/Twist - 速度命令

# 传感器数据
/camera/color/image_raw     # sensor_msgs/Image - RGB图像
/camera/depth/image_raw     # sensor_msgs/Image - 深度图像
/camera/depth/points        # sensor_msgs/PointCloud2 - 点云
/scan                       # sensor_msgs/LaserScan - 虚拟激光雷达

# 里程计与定位
/odom                       # nav_msgs/Odometry - 轮式里程计
/visual_odom                # nav_msgs/Odometry - 视觉里程计
/odom/filtered              # nav_msgs/Odometry - 融合里程计

# 导航
/map                        # nav_msgs/OccupancyGrid - 地图
/global_costmap/costmap     # nav_msgs/OccupancyGrid - 全局代价地图
/local_costmap/costmap      # nav_msgs/OccupancyGrid - 局部代价地图
/plan                       # nav_msgs/Path - 规划路径

# 感知
/detected_objects           # vision_msgs/Detection2DArray - 检测结果
/tracked_objects            # vision_msgs/Detection3DArray - 跟踪结果

# 语音
/voice_command              # std_msgs/String - 语音命令
/voice_text                 # std_msgs/String - 识别文本
```

#### 关键服务
```yaml
/slam_toolbox/save_map           # 保存2D地图
/rtabmap/save_map               # 保存3D地图
/navigate_to_pose               # 导航到目标点
/emergency_stop                 # 紧急停止
```

---

## 5. 依赖插件与工具

### 5.1 ROS2核心包

```bash
# 基础工具
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher

# Gazebo仿真
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

# TF与可视化
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-rqt* \
    ros-humble-rviz2
```

### 5.2 导航系统 (Nav2)

```bash
# Nav2完整套件
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-gazebo  # 参考示例

# 行为树
sudo apt install -y \
    ros-humble-behaviortree-cpp-v3

# AMCL定位
sudo apt install -y \
    ros-humble-nav2-amcl
```

**主要组件**:
- `planner_server`: 全局路径规划（支持NavFn, Smac, Theta*等算法）
- `controller_server`: 局部路径跟踪（DWB, TEB, RPP等）
- `bt_navigator`: 行为树导航逻辑
- `recoveries_server`: 恢复行为（旋转、后退等）
- `waypoint_follower`: 路点跟随
- `lifecycle_manager`: 生命周期管理

### 5.3 SLAM建图

#### 2D SLAM - Slam Toolbox (推荐)
```bash
sudo apt install -y ros-humble-slam-toolbox
```
**优势**:
- 专为ROS2优化
- 支持在线建图和离线优化
- 低资源占用，适合树莓派
- 支持地图保存/加载

#### 3D SLAM - RTABMap
```bash
sudo apt install -y \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-util
```
**功能**:
- RGB-D SLAM
- 视觉里程计
- 闭环检测
- 3D点云地图

#### 备用方案 - Cartographer
```bash
sudo apt install -y ros-humble-cartographer-ros
```

### 5.4 相机驱动 (Astra Pro)

```bash
# 奥比中光Astra驱动
sudo apt install -y \
    ros-humble-astra-camera \
    ros-humble-astra-camera-msgs

# 或手动编译（如果apt版本不可用）
cd ~/lododo_bot/src
git clone https://github.com/orbbec/ros2_astra_camera.git
cd ..
colcon build --packages-select astra_camera
```

**发布话题**:
- `/camera/color/image_raw` - RGB图像
- `/camera/depth/image_raw` - 深度图像
- `/camera/depth/points` - 点云
- `/camera/color/camera_info` - 相机参数

### 5.5 深度图转激光雷达

```bash
# depthimage_to_laserscan - 将深度图转换为2D激光雷达
sudo apt install -y ros-humble-depthimage-to-laserscan
```

**用途**: 为Nav2提供2D激光雷达数据（从深度相机生成）

### 5.6 传感器融合

```bash
# robot_localization - EKF/UKF传感器融合
sudo apt install -y ros-humble-robot-localization
```

**融合源**:
- 轮式里程计 (`/odom`)
- 视觉里程计 (`/visual_odom`)
- IMU数据（如果添加）

### 5.7 物体检测与跟踪

#### YOLO系列
```bash
# YOLOv8 ROS2封装
cd ~/lododo_bot/src
git clone https://github.com/mgonzs13/yolov8_ros.git
cd ..
pip3 install ultralytics
colcon build --packages-select yolov8_ros
```

#### OpenCV与深度学习
```bash
pip3 install \
    opencv-python \
    opencv-contrib-python \
    numpy \
    scipy
```

### 5.8 语音控制 (Vosk)

```bash
# Vosk离线语音识别
pip3 install vosk sounddevice

# 下载中文模型
cd ~/lododo_bot/src/bot_voice/models
wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip
unzip vosk-model-small-cn-0.22.zip
mv vosk-model-small-cn-0.22 vosk-model-small-cn
```

**特点**:
- 完全离线运行
- 支持中文识别
- 低延迟（<200ms）
- 低资源占用

### 5.9 Web界面

```bash
# rosbridge - WebSocket桥接
sudo apt install -y \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server

# Python Web框架
pip3 install flask flask-cors
```

### 5.10 串口通信

```bash
# PySerial - ST3215舵机通信
pip3 install pyserial

# 设置串口权限
sudo usermod -aG dialout $USER
# 需要重新登录生效
```

### 5.11 其他工具

```bash
# 数学与数据处理
pip3 install \
    numpy \
    scipy \
    transforms3d

# 可视化
pip3 install matplotlib

# 配置文件处理
pip3 install pyyaml
```

---

## 6. 详细设计

### 6.1 硬件接口层 (bot_hardware)

#### ST3215舵机通信协议
```python
# 串口配置
PORT = '/dev/ttyACM0'
BAUDRATE = 1000000
TIMEOUT = 0.01

# 舵机ID分配
SERVO_IDS = {
    'wheel_rear': 1,      # 后轮
    'wheel_right': 2,     # 右前轮
    'wheel_left': 3       # 左前轮
}

# 舵机性能参数（ST3215规格）✅
ENCODER_RESOLUTION = 4096        # 编码器精度：360°/4096
ENCODER_RANGE = (0, 4095)        # 取值范围
MAX_RPM = 45                     # 最大转速：45 RPM (空载)
MAX_SPEED_DEG_S = 270            # 最大角速度：270°/s (60°/0.222s)
MAX_SPEED_RAD_S = 4.712          # 最大角速度：4.712 rad/s

# 根据轮子半径计算机器人最大线速度
WHEEL_RADIUS = 0.05              # m
MAX_WHEEL_LINEAR_SPEED = MAX_SPEED_RAD_S * WHEEL_RADIUS  # ≈ 0.236 m/s

# 控制指令
CMD_WRITE_POSITION = 0x03  # 写入目标位置
CMD_READ_POSITION = 0x04   # 读取当前位置
CMD_WRITE_SPEED = 0x05     # 写入速度
CMD_READ_SPEED = 0x06      # 读取当前速度
```

**关键功能**:
1. **位置控制模式** - 精确位置控制（4096步/圈）
2. **速度控制模式** - 连续旋转（最大45 RPM）
3. **位置反馈读取** - 用于里程计计算（精度：0.088°/步）
4. **错误检测与恢复** - 过载保护、通信超时处理

**里程计计算公式**:
```python
# 编码器值转角度
angle_rad = (encoder_value / 4096.0) * 2 * π

# 轮子转动距离
wheel_distance = angle_rad * WHEEL_RADIUS  # (rad * 0.05m)

# 理论精度
position_resolution = (2 * π / 4096) * 0.05  # ≈ 0.077mm/步
```

#### ROS2 Control集成
```yaml
# ros2_control.yaml
hardware:
  - name: omni_wheel_system
    type: system
    ros__parameters:
      joints:
        - wheel_rear_joint
        - wheel_right_joint
        - wheel_left_joint
      command_interfaces:
        - velocity
      state_interfaces:
        - position
        - velocity
```

### 6.2 运动控制 (bot_control)

#### 全向轮控制器
```python
class OmniController:
    """三轮全向轮运动学控制器"""
    
    def __init__(self):
        # 机器人物理参数（实测确认）✅
        self.wheel_radius = 0.05  # m
        self.wheel_distances = {
            'rear': 0.105,    # 后轮到中心距离 (m)
            'front': 0.085    # 前轮到中心距离 (m)
        }
        self.robot_mass = 1.5     # kg (总重量)
        
        # 速度限制（基于ST3215性能）
        self.max_wheel_speed = 4.712      # rad/s (270°/s)
        self.max_linear_speed = 0.20      # m/s (安全裕量，实际可达0.236)
        self.max_angular_speed = 1.5      # rad/s (保守估计)
        
    def inverse_kinematics(self, vx, vy, omega_z):
        """计算各轮速度
        
        Args:
            vx: X方向线速度 (m/s)
            vy: Y方向线速度 (m/s)
            omega_z: 角速度 (rad/s)
            
        Returns:
            (w_rear, w_right, w_left): 三个轮子的角速度 (rad/s)
        """
        # 根据实际几何关系计算
        # 需要考虑非对称布局
        pass
```

#### 速度平滑器
```python
class VelocitySmoother:
    """速度指令平滑，防止急加速"""
    
    def __init__(self):
        self.max_linear_accel = 0.5   # m/s²
        self.max_angular_accel = 1.0  # rad/s²
```

### 6.3 导航系统 (bot_navigation)

#### Nav2配置要点
```yaml
# nav2_params.yaml核心参数

# 控制器配置（针对全向轮）
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # 全向轮可以侧向移动
      holonomic_robot: true
      
      # 速度限制（基于ST3215性能）✅
      max_vel_x: 0.20        # m/s (安全值，实际可达0.236)
      min_vel_x: -0.20
      max_vel_y: 0.20
      min_vel_y: -0.20
      max_vel_theta: 1.5     # rad/s (约86°/s)
      
      # 加速度限制
      max_vel_x_accel: 0.5   # m/s²
      max_vel_y_accel: 0.5
      max_vel_theta_accel: 1.0  # rad/s²

# 代价地图配置
local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    robot_radius: 0.15  # 根据机器人实际尺寸
    plugins: ["obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: scan
      scan:
        topic: /scan
        sensor_frame: camera_depth_frame
```

### 6.4 SLAM建图 (bot_slam)

#### 2D建图流程
```bash
# 1. 启动相机和深度转激光
ros2 launch bot_perception camera.launch.py

# 2. 启动SLAM
ros2 launch bot_slam slam_2d.launch.py

# 3. 手动遥控建图
ros2 run bot_teleop keyboard_teleop

# 4. 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

#### 3D建图配置
```yaml
# rtabmap.yaml
rtabmap:
  frame_id: base_link
  subscribe_depth: true
  subscribe_rgb: true
  subscribe_scan_cloud: true
  
  # RGB-D相机配置
  approx_sync: true
  queue_size: 30
  
  # 建图参数
  Mem/IncrementalMemory: "true"
  Mem/InitWMWithAllNodes: "false"
```

### 6.5 物体检测 (bot_perception)

#### YOLOv8集成
```python
class ObjectDetector:
    """物体检测节点"""
    
    def __init__(self):
        self.model = YOLO('yolov8n.pt')  # nano模型适合树莓派
        
    def image_callback(self, msg):
        """处理图像并检测物体"""
        # 转换ROS图像到OpenCV
        # 运行YOLOv8检测
        # 发布检测结果
        pass
```

#### 物体跟踪
```python
class ObjectTracker:
    """结合深度信息的3D物体跟踪"""
    
    def __init__(self):
        # 使用SORT/DeepSORT算法
        # 结合深度信息获取3D位置
        pass
```

### 6.6 语音控制 (bot_voice)

#### 语音识别节点
```python
import vosk
import sounddevice as sd

class VoiceRecognizer:
    def __init__(self):
        model_path = "models/vosk-model-small-cn"
        self.model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(self.model, 16000)
        
    def recognize_stream(self):
        """实时语音识别"""
        with sd.RawInputStream(samplerate=16000, channels=1):
            while True:
                data = stream.read(4000)
                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    text = result['text']
                    self.publish_command(text)
```

#### 命令映射
```yaml
# voice_commands.yaml
commands:
  "前进": {action: "move", params: {direction: "forward"}}
  "后退": {action: "move", params: {direction: "backward"}}
  "停止": {action: "stop"}
  "导航到厨房": {action: "navigate", params: {goal: "kitchen"}}
  "跟随我": {action: "follow_person"}
```

### 6.7 Web控制界面 (bot_teleop)

#### 架构
```
浏览器 <--WebSocket--> rosbridge_server <--ROS2--> 机器人节点
```

#### 功能模块
1. **虚拟摇杆**: 发送`/cmd_vel`
2. **视频流**: 通过`web_video_server`
3. **地图显示**: 订阅`/map`话题
4. **状态监控**: 电池、速度、传感器状态
5. **导航点设置**: 点击地图设置目标点

### 6.8 Gazebo仿真 (bot_gazebo)

#### 仿真插件配置
```xml
<!-- lekiwi_bot.gazebo.xacro -->
<gazebo>
  <!-- 差速驱动插件（需修改为全向轮） -->
  <plugin name="omni_drive" filename="libgazebo_ros_planar_move.so">
    <update_rate>50</update_rate>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
  </plugin>
  
  <!-- 深度相机插件 -->
  <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
    <camera_name>camera</camera_name>
    <frame_name>camera_depth_frame</frame_name>
    <image_topic>color/image_raw</image_topic>
    <depth_topic>depth/image_raw</depth_topic>
    <point_cloud_topic>depth/points</point_cloud_topic>
  </plugin>
</gazebo>
```

---

## 7. 开发路线图

### Phase 1: 基础框架搭建（第1-2周）
- [ ] 创建所有功能包框架
- [ ] 配置URDF/Xacro仿真模型
- [ ] 实现ST3215舵机基础驱动
- [ ] Gazebo仿真环境搭建
- [ ] 基础遥控功能实现

### Phase 2: 运动控制与里程计（第3-4周）
- [ ] 全向轮运动学控制器开发
- [ ] 轮式里程计实现
- [ ] ROS2 Control集成
- [ ] 速度平滑与安全保护
- [ ] 仿真环境测试

### Phase 3: 感知系统（第5-6周）
- [ ] Astra Pro相机驱动集成
- [ ] 深度图转激光雷达
- [ ] 视觉里程计配置
- [ ] robot_localization传感器融合
- [ ] 基础障碍物检测

### Phase 4: SLAM建图（第7-8周）
- [ ] Slam Toolbox 2D建图
- [ ] RTABMap 3D建图
- [ ] 地图保存与加载
- [ ] 多场景地图测试

### Phase 5: 导航系统（第9-10周）
- [ ] Nav2完整配置
- [ ] AMCL定位
- [ ] 路径规划与跟踪
- [ ] 动态避障
- [ ] 恢复行为

### Phase 6: 高级功能（第11-12周）
- [ ] 物体检测与跟踪
- [ ] 语音控制集成
- [ ] Web控制界面
- [ ] 系统集成测试

### Phase 7: 硬件部署（第13-14周）
- [ ] 真实硬件调试
- [ ] 参数校准与优化
- [ ] 实际环境测试
- [ ] 文档完善

---

## 8. 待确认事项

### 8.1 机器人参数确认

| 参数 | 当前值 | 状态 | 备注 |
|------|--------|------|------|
| 轮子半径 | 0.05m | ✅ 已确认 | |
| 后轮到中心距离 | 0.105m | ✅ 已确认 | 实测值 |
| 前轮到中心距离 | 0.085m | ✅ 已确认 | 实测值 |
| 机器人总重量 | 1.5kg | ✅ 已确认 | 含电池 |
| 舵机编码器精度 | 4096步/圈 | ✅ 已确认 | 0.088°/步 |
| 舵机最大转速 | 45 RPM | ✅ 已确认 | 空载270°/s |
| 机器人最大线速度 | ~0.236 m/s | ✅ 已计算 | 建议限制0.20 m/s |
| 机器人最大角速度 | ~1.5 rad/s | ✅ 已估算 | 约86°/s |
| 电池电压范围 | 12V | ✅ 已确认 | 5.2Ah |

### 8.2 硬件配置确认

- [x] **IMU选择**: ✅ **已确认采用无IMU方案**
  - **方案**: 先用轮式里程计 + 视觉里程计融合测试
  - **升级路径**: 如定位不稳定，后续可添加MPU6050（￥15-30）
  - **优势**: 降低硬件成本和复杂度

- [x] **舵机编码器精度**: ✅ **已确认**
  - **精度**: 4096步/圈 (360°/4096 = 0.088°/步)
  - **取值范围**: 0 ~ 4095
  - **理论位置分辨率**: 约0.077mm/步
  - **结论**: 精度足够用于高质量里程计

- [ ] **网络配置**: WiFi还是以太网？
  - **WiFi**: 更灵活，适合移动机器人
  - **以太网**: 更稳定，开发调试时推荐
  - **建议**: 开发时用以太网，部署时用WiFi

### 8.3 运动学参数校准

由于三轮布局非对称（后轮距离0.105m，前轮0.085m），需要：

1. **实测轮子实际位置** ✅ **已完成**
   - 后轮到中心: 0.105m
   - 前轮到中心: 0.085m
   - 轮子夹角: 理论120°（需验证实际偏差）

2. **运动学矩阵推导** 🔄 **待实现**
   - 根据实测数据推导正/逆运动学公式
   - 考虑非对称布局的影响
   - 编写单元测试验证
   - 实现Python控制器类

3. **实机校准** 🔄 **待测试**
   ```python
   # 校准测试项目
   tests = [
       "前进1米测试 - 验证线速度准确性",
       "后退1米测试 - 验证对称性",
       "原地旋转360°测试 - 验证角速度",
       "侧向移动测试 - 验证全向轮性能",
       "圆形轨迹测试 - 综合验证",
       "里程计漂移测试 - 长时间运行精度"
   ]
   ```

4. **预期性能指标**
   - 位置误差: < 5% (1米误差<5cm)
   - 角度误差: < 3° (360°旋转)
   - 里程计漂移: < 2% (10米行驶)

### 8.4 其他待定事项

- [ ] **地图存储位置**: 默认保存在哪个目录？
- [ ] **日志级别**: 生产环境用INFO还是WARN？
- [ ] **远程访问**: 是否需要SSH隧道或VPN？
- [ ] **安全机制**: 紧急停止按钮的实现方式？
- [ ] **电池监控**: 如何读取电池电量？

---

## 9. 快速启动指南

### 9.1 仿真环境

```bash
# 终端1: 启动Gazebo仿真
ros2 launch bot_bringup simulation.launch.py

# 终端2: 启动导航
ros2 launch bot_bringup navigation.launch.py use_sim_time:=true

# 终端3: 键盘控制
ros2 run bot_teleop keyboard_teleop
```

### 9.2 真实硬件

```bash
# 终端1: 启动机器人硬件
ros2 launch bot_bringup robot.launch.py

# 终端2: 启动导航
ros2 launch bot_bringup navigation.launch.py

# 终端3: Web界面
ros2 launch bot_teleop web_teleop.launch.py
# 浏览器访问: http://<树莓派IP>:8080
```

### 9.3 建图模式

```bash
# 2D建图
ros2 launch bot_bringup robot.launch.py
ros2 launch bot_slam slam_2d.launch.py

# 3D建图
ros2 launch bot_bringup robot.launch.py
ros2 launch bot_slam slam_3d.launch.py
```

---

## 10. 性能优化建议

### 10.1 树莓派4B优化

```bash
# 1. 使用Ubuntu Server（无GUI）节省资源
# 2. 关闭不必要的服务
sudo systemctl disable bluetooth
sudo systemctl disable ModemManager

# 3. 超频（可选，注意散热）
# 编辑 /boot/firmware/config.txt
over_voltage=6
arm_freq=2000

# 4. 增加swap（如果内存不足）
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile  # CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

### 10.2 ROS2优化

```python
# 使用QoS配置优化话题传输
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# 传感器话题使用BEST_EFFORT减少延迟
self.create_subscription(Image, '/camera/image_raw', callback, sensor_qos)
```

### 10.3 图像处理优化

```python
# 降低分辨率
image_resized = cv2.resize(image, (320, 240))

# 使用灰度图（如果不需要颜色）
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 限制处理帧率
if self.frame_count % 3 == 0:  # 每3帧处理一次
    self.process_image(image)
```

---

## 11. 故障排查

### 常见问题

#### 1. 舵机无响应
```bash
# 检查串口权限
ls -l /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0

# 测试串口通信
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0', 1000000); print('OK')"
```

#### 2. 相机无法识别
```bash
# 检查USB设备
lsusb | grep Orbbec

# 重新安装驱动
sudo apt reinstall ros-humble-astra-camera
```

#### 3. 导航不工作
```bash
# 检查TF树
ros2 run tf2_tools view_frames

# 检查话题
ros2 topic list
ros2 topic echo /scan --no-arr
```

#### 4. 里程计漂移严重
- 检查轮子是否打滑
- 校准运动学参数
- 考虑添加IMU
- 增加视觉里程计权重

---

## 12. 参考资源

### 官方文档
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [Nav2文档](https://navigation.ros.org/)
- [Gazebo文档](https://gazebosim.org/)

### 示例项目
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [ROSbot 2.0](https://github.com/husarion/rosbot_ros)

### 教程
- [Nav2入门教程](https://navigation.ros.org/getting_started/index.html)
- [SLAM Toolbox教程](https://github.com/SteveMacenski/slam_toolbox)

---

## 附录A: 完整依赖列表

```bash
#!/bin/bash
# install_dependencies.sh

# ROS2基础
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-rqt*

# 导航
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-behaviortree-cpp-v3

# SLAM
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-rtabmap-ros \
    ros-humble-cartographer-ros

# 相机
sudo apt install -y \
    ros-humble-astra-camera \
    ros-humble-depthimage-to-laserscan

# 传感器融合
sudo apt install -y \
    ros-humble-robot-localization

# Web
sudo apt install -y \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server

# Python依赖
pip3 install \
    pyserial \
    vosk \
    sounddevice \
    ultralytics \
    opencv-python \
    numpy \
    scipy \
    transforms3d \
    flask \
    flask-cors

echo "安装完成！"
```

---

**文档版本**: v1.0  
**最后更新**: 2025年12月2日  
**维护者**: LeKiwi项目组

**下一步行动**:
1. 确认待定事项（第8节）
2. 开始Phase 1开发
3. 定期更新本文档
