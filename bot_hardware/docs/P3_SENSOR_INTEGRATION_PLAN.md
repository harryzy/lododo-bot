# P3阶段: 传感器集成 - 详细工作计划

**创建日期**: 2026-01-20  
**预计工作量**: 3天  
**依赖**: P2硬件控制节点完成 ✅  
**目标**: 集成IMU和Astra Pro相机，为导航和SLAM提供传感器数据

---

## 阶段目标

1. **IMU集成**: 适配ybimu_driver到ROS2，实现坐标转换和数据滤波
2. **Camera集成**: 配置Astra Pro相机驱动，发布RGB-D数据
3. **数据验证**: 确保传感器数据格式正确、坐标系对齐、时间戳同步

**成功标准**:
- [ ] `/imu/data`话题发布频率~100Hz，数据格式正确
- [ ] 静止时重力向量指向z轴正方向 ([0, 0, 9.81] ±0.5 m/s²)
- [ ] `/camera/color/image_raw`和`/camera/depth/image_raw`正常发布 (~30fps)
- [ ] RGB和Depth时间戳同步（误差<10ms）

---

## 工作分解 (WBS)

### P3.1 适配ybimu_driver IMU驱动 (预计0.5天)

**参考设计文档**: §3.5.2 IMU传感器（HARDWARE_DEPLOYMENT_DESIGN.md 行4048-4139）

#### 任务清单

- [ ] **Task 3.1.1**: 确认ybimu_driver代码位置
  - 文件: `bot_hardware/imu_ros2_device/ybimu_driver.py`
  - 检查是否已存在ROS2版本
  - 如果不存在，从原始仓库适配

- [ ] **Task 3.1.2**: 配置串口设备
  - 创建udev规则: `/etc/udev/rules.d/99-lekiwi-imu.rules`
  ```bash
  # LeKiwi Robot IMU (ybimu)
  SUBSYSTEM=="tty", ATTRS{idVendor}=="<vendor_id>", ATTRS{idProduct}=="<product_id>", SYMLINK+="lekiwi_imu", MODE="0666"
  ```
  - 重载udev: `sudo udevadm control --reload-rules && sudo udevadm trigger`
  - 验证: `ls -l /dev/lekiwi_imu`

- [ ] **Task 3.1.3**: 确认hardware_config.yaml中IMU配置正确
  
  **⚠️ 注意**: 根据设计原则，不创建单独的imu_config.yaml，所有参数从hardware_config.yaml读取
  
  **检查项**:
  ```bash
  # 查看hardware_config.yaml中的IMU配置
  nano config/hardware_config.yaml
  
  # 确认以下参数已配置:
  # - serial.imu_port: '/dev/ybimu'
  # - serial.imu_baudrate: 115200
  # - imu.publish_rate: 50
  # - imu.frame_id: 'imu_link'
  # - imu.mounting_rotation: {roll, pitch, yaw}
  ```

- [ ] **Task 3.1.4**: 测试IMU原始数据发布
  ```bash
  # 启动IMU驱动（从hardware_config.yaml读取配置）
  ros2 run bot_hardware ybimu_driver
  
  # 验证话题
  ros2 topic hz /imu/data_raw  # 应为~50Hz (根据hardware_config.yaml)
  ros2 topic echo /imu/data_raw --once
  ```

#### 验收标准
- [x] `/imu/data_raw`话题存在
- [x] 发布频率在45-55Hz范围内（hardware_config.yaml中配置为50Hz）
- [x] 数据单位正确（加速度: m/s², 角速度: rad/s）
- [x] frame_id为"imu_link"

---

### P3.2 实现imu_filter_node滤波节点 (预计1天)

**参考设计文档**: §3.5.2.2 imu_filter_node设计（行4248-4517）

#### 任务清单

- [ ] **Task 3.2.1**: 创建imu_filter_node.py
  - 文件: `bot_hardware/imu_ros2_device/imu_filter_node.py`
  - 订阅: `/imu/data_raw` (sensor_msgs/Imu)
  - 发布: `/imu/data` (sensor_msgs/Imu)

- [ ] **Task 3.2.2**: 实现REP-103坐标系转换 (NED → ENU)
  
  **背景**: ybimu默认输出NED坐标系，ROS2使用ENU坐标系
  
  **转换矩阵**:
  ```python
  # NED → ENU转换
  # X_enu = Y_ned
  # Y_enu = X_ned
  # Z_enu = -Z_ned
  
  # 加上mounting_rotation（IMU物理安装角度）
  import numpy as np
  from scipy.spatial.transform import Rotation as R
  
  class IMUFilter:
      def __init__(self, mounting_rotation):
          # mounting_rotation格式: [roll, pitch, yaw] (度)
          self.R_mount = R.from_euler('xyz', mounting_rotation, degrees=True).as_matrix()
          
          # NED → ENU转换矩阵
          self.R_ned_to_enu = np.array([
              [0,  1,  0],
              [1,  0,  0],
              [0,  0, -1]
          ])
      
      def transform_vector(self, vec_ned):
          # 1. NED → ENU坐标系转换
          vec_enu = self.R_ned_to_enu @ vec_ned
          
          # 2. 应用mounting rotation
          vec_body = self.R_mount @ vec_enu
          
          return vec_body
  ```

- [ ] **Task 3.2.3**: 实现滑动平均滤波
  ```python
  from collections import deque
  
  class MovingAverageFilter:
      def __init__(self, window_size=5):
          self.window = deque(maxlen=window_size)
      
      def update(self, value):
          self.window.append(value)
          return np.mean(self.window, axis=0)
  ```

- [ ] **Task 3.2.4**: 保留原始时间戳
  
  **⚠️ 关键设计决策**: 使用方案B（保留ybimu_driver时间戳）
  
  **理由**:
  - imu_filter_node处理延迟<5ms（远小于robot_localization要求的50ms）
  - 避免重复记录时间戳带来的延迟
  - 简化代码，降低维护成本
  
  ```python
  def imu_raw_callback(self, msg):
      # 保留原始时间戳
      filtered_msg = Imu()
      filtered_msg.header.stamp = msg.header.stamp  # ⭐ 关键：不修改时间戳
      filtered_msg.header.frame_id = 'base_link'
      
      # 坐标转换
      accel_transformed = self.transform_vector(
          np.array([msg.linear_acceleration.x, 
                    msg.linear_acceleration.y, 
                    msg.linear_acceleration.z])
      )
      
      # 滤波
      accel_filtered = self.accel_filter.update(accel_transformed)
      
      # 赋值
      filtered_msg.linear_acceleration.x = accel_filtered[0]
      filtered_msg.linear_acceleration.y = accel_filtered[1]
      filtered_msg.linear_acceleration.z = accel_filtered[2]
      
      self.imu_pub.publish(filtered_msg)
  ```

- [ ] **Task 3.2.5**: 配置mounting_rotation参数
  
  **⚠️ 注意**: mounting_rotation参数已在hardware_config.yaml中定义（line 83-86），初始值为0.0
  
  **测量和调整方法**:
  1. 将机器人放置在水平地面
  2. 运行`ros2 topic echo /imu/data_raw`观察重力向量
  3. 理想情况：重力应指向[0, 0, 9.81] (base_link坐标系下)
  4. 如果不是，调整hardware_config.yaml中的mounting_rotation参数
  
  **当前配置检查** (hardware_config.yaml line 83-86):
  ```yaml
  imu:
    mounting_rotation:
      roll: 0.0    # 绕X轴旋转（rad） / Rotation around X axis (rad)
      pitch: 0.0   # 绕Y轴旋转（rad） / Rotation around Y axis (rad)
      yaw: 0.0     # 绕Z轴旋转（rad） / Rotation around Z axis (rad)
  ```
  
  **滤波参数** (hardware_config.yaml line 113-118):
  ```yaml
  filter:
    enable_dynamic_bias: true           # 启用动态零偏补偿
    low_pass_alpha: 0.2                 # 低通滤波系数 (截止频率约10Hz @ 50Hz采样)
    median_window_size: 5               # 中值滤波窗口大小
    complementary_alpha: 0.98           # 互补滤波系数（姿态融合）
  ```

- [ ] **Task 3.2.6**: 在setup.py注册节点
  ```python
  entry_points={
      'console_scripts': [
          'omni_hardwar（频率~50Hz，与hardware_config.yaml一致）
- [x] 静止时重力向量指向z轴正方向 ([0, 0, 9.81] ±0.5 m/s²)
- [x] 时间戳延迟<5ms（使用后续P3.3工具验证）
- [x] frame_id为"base_link"
- [x] 滤波参数从hardware_config.yaml正确加载
  }
  ```

#### 验收标准
- [x] `/imu/data`话题正常发布
- [x] 静止时重力向量指向z轴正方向 ([0, 0, 9.81] ±0.5 m/s²)
- [x] 时间戳延迟<5ms（使用后续P3.3工具验证）
- [x] frame_id为"base_link"

---

### P3.3 实现test_imu_coordinate验证工具 (预计0.5天)

**参考设计文档**: §3.5.2.3 IMU坐标系验证工具（行4518-4943）

#### 任务清单

- [ ] **Task 3.3.1**: 创建test_imu_coordinate.py
  - 文件: `bot_hardware/tools/test_imu_coordinate.py`
  
  ```python
  #!/usr/bin/env python3
  import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import Imu
  import numpy as np
  
  class IMUCoordinateTest(Node):
      def __init__(self):
          super().__init__('test_imu_coordinate')
          self.subscription = self.create_subscription(
              Imu, '/imu/data', self.imu_callback, 10)
          
          self.samples = []
          self.max_samples = 100
          
          self.get_logger().info('收集IMU数据中，请保持机器人静止...')
      
      def imu_callback(self, msg):
          if len(self.samples) < self.max_samples:
              accel = np.array([
                  msg.linear_acceleration.x,
                  msg.linear_acceleration.y,
                  msg.linear_acceleration.z
              ])
              self.samples.append(accel)
              
              if len(self.samples) == self.max_samples:
                  self.analyze_results()
                  rclpy.shutdown()
      
      def analyze_results(self):
          samples = np.array(self.samples)
          mean = np.mean(samples, axis=0)
          std = np.std(samples, axis=0)
          
          # 预期重力向量 (base_link坐标系, ENU)
          expected_gravity = np.array([0.0, 0.0, 9.81])
          error = np.linalg.norm(mean - expected_gravity)
          
          print("\n========== IMU坐标系验证结果 ==========")
          print(f"平均重力向量: [{mean[0]:.3f}, {mean[1]:.3f}, {mean[2]:.3f}] m/s²")
          print(f"标准差:       [{std[0]:.3f}, {std[1]:.3f}, {std[2]:.3f}] m/s²")
          print(f"误差:         {error:.3f} m/s²")
          print(f"预期值:       [0.000, 0.000, 9.810] m/s²")
          
          # 判定
          if error < 0.2:
              status = "✅ GOOD"
              advice = "坐标系配置正确"
          elif error < 0.5:
              status = "⚠️ WARN"
              advice = "建议微调mounting_rotation参数"
          else:
              status = "❌ FAIL"
              advice = "请重新配置mounting_rotation参数"
          
          print(f"\n判定: {status}")
          print(f"建议: {advice}")
          print("======================================\n")
  
  def main():
      rclpy.init()
      node = IMUCoordinateTest()
      rclpy.spin(node)
  ```

- [ ] **Task 3.3.2**: 在setup.py注册工具
  ```python
  entry_points={
      'console_scripts': [
          'test_imu_coordinate = bot_hardware.tools.test_imu_coordinate:main',
      ],
  }
  ```

- [ ] **Task 3.3.3**: 创建使用说明
  - 文件: `docs/TEST_IMU_COORDINATE_USAGE.md`

#### 验收标准
- [x] `ros2 run bot_hardware test_imu_coordinate`能运行
- [x] 输出包含平均值、标准差、误差、判定结果
- [x] 判定阈值符合设计（GOOD: <0.2, WARN: <0.5, FAIL: ≥0.5）

---

### P3.4 集成Astra Pro相机驱动 (预计0.5天)

**参考设计文档**: §3.5.1 Astra Pro相机（行3924-3936）

#### 任务清单

- [ ] **Task 3.4.1**: 安装astra_camera驱动
  ```bash
  # 克隆ROS2 Humble版本
  cd ~/lododo_bot/src
  git clone -b ros2 https://github.com/orbbec/ros_astra_camera.git
  
  # 安装依赖
  sudo apt install ros-humble-image-transport \
                   ros-humble-camera-info-manager \
                   ros-humble-compressed-image-transport
  
  # 编译
  cd ~/lododo_bot
  colcon build --packages-select astra_camera
  source install/setup.bash
  ```

- [ ] **Task 3.4.2**: 配置USB权限
  
  **创建udev规则**:
  ```bash
  sudo nano /etc/udev/rules.d/99-lekiwi-camera.rules
  ```
  
  **内容**:
  ```bash
  # LeKiwi Robot - Astra Pro Camera
  SUBSYSTEM=="usb", AT确认hardware_config.yaml中相机配置正确
  
  **⚠️ 注意**: 根据设计原则，不创建单独的camera_config.yaml，所有参数从hardware_config.yaml读取
  
  **检查项** (hardware_config.yaml line 120-145):
  ```yaml
  camera:
    device_uri: ''  # 空字符串使用默认设备
    rgb:
      width: 640
      height: 480
      fps: 30
    depth:
      width: 640
      height: 480
      fps: 30
      min_range: 0.6  # 米
      max_range: 8.0  # 米
    enable_alignment: true  # 启用硬件对齐
    topics:
      rgb_image: '/camera/color/image_raw'
      depth_image: '/camera/depth/image_raw'
      rgb_camera_info: '/camera/color/camera_info'
      depth_camera_info: '/camera/depth/camera_info'
  ```
  
  **注意**: astra_camera_node需要适配以从hardware_config.yaml读取这些参数 
      # 对齐
      enable_d2c_viewer: true  # Depth to Color alignment
      
      # 发布话题
      enable_color: true
      enab（通过launch文件加载hardware_config.yaml）
  ros2 launch bot_hardware camera_bringup.launch.py
  
  # 或直接运行（需要在launch中配置参数映射）
  # ros2 run astra_camera astra_camera_node
  ```

- [ ] **Task 3.4.4**: 测试相机数据发布
  ```bash
  # 启动相机驱动
  ros2 run astra_camera astra_camera_node --ros-args \
    --params-file config/camera_config.yaml
  
  # 验证话题
  ros2 topic list | grep camera
  # 预期输出:
  # /camera/color/image_raw
  # /camera/depth/image_raw
  # /camera/color/camera_info
  # /camera/depth/camera_info
  
  # 检查频率
  ros2 topic hz /camera/color/image_raw  # 应为~30Hz
  ros2 topic hz /camera/depth/image_raw  # 应为~30Hz
  
  # 可视化
  ros2 run rqt_image_view rqt_image_view
  ```

#### 验收标准
- [x] 相机话题正常发布（~30fps）
- [x] RGB和Depth分辨率正确（640x480）
- [x] camera_info包含内参矩阵（即使未标定，也应有默认值）
- [x] rqt_image_view能显示彩色和深度图像

---

### P3.5 执行相机标定 (预计0.5天)

**参考设计文档**: §3.5.3.1 相机内参标定流程（行3937-4047）

**⚠️ 注意**: 此任务可以延后到P4或P5阶段，不阻塞P3完成。标定主要用于提高精度，未标定的相机也能基本工作。

#### 任务清单

- [ ] **Task 3.5.1**: 准备棋盘格标定板
  - 规格: 8x6（内角点数量）
  - 方格尺寸: 25mm x 25mm
  - 打印或购买标定板

- [ ] **Task 3.5.2**: 运行camera_calibration工具
  ```bash
  # 安装工具
  sudo apt install ros-humble-camera-calibration
  
  # 启动标定
  ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    image:=/camera/color/image_raw \
    camera:=/camera/colo并保存标定文件
  
  **⚠️ 关键**: 标定数据单独存放在`calibration/`目录，不混入config/
  
  **camera_calibration输出**: `/tmp/calibrationdata.tar.gz`
  
  **提取并保存到标定目录**:
  ```bash
  cd /tmp
  tar -xzf calibrationdata.tar.gz
  
  # 转换为camera_info.yaml
  ros2 run camera_calibration_parsers convert \
    ost.yaml camera_info.yaml
  
  # 复制到标定目录（不是config/目录）
  mkdir -p ~/lododo_bot/calibration
  cp camera_info.yaml ~/lododo_bot/calibration/camera_info.yaml
  ```
  
  **标定文件位置检查** (hardware_config.yaml line 322):
  ```yaml
  paths:
    camera_calibration_file: 'calibration/camera_info.yaml'
  ```

- [ ] **Task 3.5.4**: 配置相机驱动加载标定文件
  
  **在launch文件中配置**:
  ```python
  # camera_bringup.launch.py
  camera_info_path = os.path.join(
      os.path.expanduser('~'), 'lododo_bot', 
      'calibration', 'camera_info.yaml'
  )
  
  Node(
      package='astra_camera',
      executable='astra_camera_node',
      parameters=[{
          'camera_info_url': f'file://{camera_info_path}'
      }]
  )
  ```

- [ ] **Task 3.5.4**: 配置相机驱动加载标定文件
  
  **修改camera_config.yaml**:
  ```yaml
  astra_camera:
    ros__parameters:
      camera_info_url: "file://$(find bot_hardware)/config/camera_info.yaml"
  ```

- [ ] **Task 3.5.5**: 验证标定效果
  ```bash
  # 重启相机驱动
  ros2 run astra_camera astra_camera_node --ros-args \
    --params-file config/camera_config.yaml
  
  # 检查camera_info
  ros2 topic echo /camera/color/camera_info --once
  # 确认K矩阵（内参）和D向量（畸变）已更新
  ```

#### 验收标准
- [x] 重投影误差<0.5像素
- [x] `/camera/color/camera_info`包含标定后的K矩阵和D向量
- [x] 标定文件正确加载（camera_info_url有效）

---

## 集成测试计划

### Test P3.1: IMU数据有效性测试

**前置条件**: IMU驱动和滤波节点已启动

**测试步骤**:
1. 启动IMU nodes:
   ```bash
   ros2 launch bot_hardware imu_bringup.launch.py
   ```

2. 检查话题频率:
   ```bash
   ros2 topic hz /imu/data_raw
   ros2 topic hz /imu/data
   ```

3. 运行坐标系验证:
   ```bash
   ros2 run bot_hardware test_imu_coordinate
   ```

**预期结果**:
- `/imu/data_raw`频率: 95-105 Hz
- `/imu/data`频率: 95-105 Hz
- test_imu_coordinate输出: ✅ GOOD 或 ⚠️ WARN

---

### Test P3.2: 相机数据有效性测试

**前置条件**: 相机驱动已启动

**测试步骤**:
1. 启动相机:
   ```bash
   ros2 run astra_camera astra_camera_node --ros-args \
     --params-file config/camera_config.yaml
   ```

2. 检查话题频率:
   ```bash
   ros2 topic hz /camera/color/image_raw
   ros2 topic hz /camera/depth/image_raw
   ```

3. 可视化检查:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   # 选择/camera/color/image_raw，检查图像清晰
   # 选择/camera/depth/image_raw，检查深度图有效
   ```

**预期结果**:
- RGB频率: 28-32 Hz
- Depth频率: 28-32 Hz
- 图像清晰，无明显噪声
- 深度图在有效距离内（0.4-8m）显示数据

---

### Test P3.3: 传感器时间戳同步测试

**前置条件**: IMU和相机都已启动

**测试步骤**:
1. 使用ROS2 bag录制数据:
   ```bash
   ros2 bag record -o sensor_sync_test \
     /imu/data /camera/color/image_raw /camera/depth/image_raw
   # 录制10秒
   ```

2. 回放并分析时间戳:
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    IMU启动文件
    所有参数从统一的hardware_config.yaml读取
    """
    # 获取统一配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('bot_hardware'),
        'config'
    )
    hardware_config = os.path.join(config_dir, 'hardware_config.yaml')
    
    return LaunchDescription([
        # ybimu原始驱动（从hardware_config.yaml读取serial.imu_*参数）
        Node(
            package='bot_hardware',
            executable='ybimu_driver',
            name='ybimu_driver',
            output='screen',
            parameters=[hardware_config]  # 统一配置文件
        ),
        
        # IMU滤波和坐标转换节点（从hardware_config.yaml读取imu.*参数）
        Node(
            package='bot_hardware',
            executable='imu_filter_node',
            name='imu_filter_node',
            output='screen',
            parameters=[hardware_config]  # 统一配置文件
    imu_config = os.path.join(config_dir, 'imu_config.yaml')
    hardware_config = os.path.join(config_dir, 'hardware_config.yaml')
    
    return LaunchDescription([
        # ybimu原始驱动
        Node(
            package='bot_hardware',
            executable='ybimu_driver',
            name='ybimu_driver',
            output='screen',
            parameters=[imu_config]
        ),
    """
    相机启动文件
    从hardware_config.yaml读取相机参数，标定文件从calibration/目录读取
    """
    # 获取统一配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('bot_hardware'),
        'config'
    )
    hardware_config = os.path.join(config_dir, 'hardware_config.yaml')
    
    # 标定文件路径（单独存放在calibration/目录）
    calibration_dir = os.path.join(
        os.path.expanduser('~'), 'lododo_bot', 'calibration'
    )
    camera_info_file = os.path.join(calibration_dir, 'camera_info.yaml')
    
    # 从hardware_config.yaml读取相机参数
    # 注意：astra_camera可能需要参数名称映射
    return LaunchDescription([
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            output='screen',
            parameters=[
                hardware_config,  # 读取camera.*参数
                {
                    'camera_info_url': f'file://{camera_info_file}'  # 标定文件
                }
            
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('bot_hardware'),
        'config'
    )
    
    camera_config = os.path.join(config_dir, 'camera_config.yaml')
    
    return LaunchDescription([
        Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='astra_camera',
            output='screen',
            parameters=[camera_config]
        ),
    ])
```

### sensors_bringup.launch.py（集成版）

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bot_hardware_dir = get_package_share_directory('bot_hardware')
    
    return LaunchDescription([
        # 包含IMU启动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bot_hardware_dir, 'launch', 'imu_bringup.launch.py')
            )
        ),
        
        # 包含相机启动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bot_hardware_dir, 'launch', 'camera_bringup.launch.py')
            )
        ),
    ])
```

---

## 常见问题与排查

### Q1: IMU数据为全0

**可能原因**:
1. 串口未连接或设备名错误
2. udev规则未生效
3. 驱动未正确初始化
4. hardware_config.yaml中串口参数错误

**排查步骤**:
```bash
# 检查串口设备（hardware_config.yaml中配置为/dev/ybimu）
ls -l /dev/ybimu

# 检查所有USB串口
ls -l /dev/ttyUSB*

# 检查串口是否被占用
sudo lsof /dev/ttyUSB*

# 手动测试串口通信
sudo minicom -D /dev/ybimu -b 115200

# 检查hardware_config.yaml配置
grep -A 5 "imu_port" config/hardware_config.yaml
```

---

### Q2: 重力向量不是[0, 0, 9.81]

**可检查hardware_config.yaml中的mounting_rotation参数（line 83-86）
3. 运行`ros2 topic echo /imu/data_raw`观察原始数据
4. 逐步调整mounting_rotation参数
5. 使用test_imu_coordinate验证

**参数调整示例** (hardware_config.yaml):
```yaml
imu:
  mounting_rotation:
    roll: 0.0    # 如果x轴方向不对，调整roll
    pitch: 0.0   # 如果y轴方向不对，调整pitch
    yaw: 3.14159 # 如果IMU旋转180°，设置yaw=π
```
3. IMU物理安装方向错误

**排查步骤**:
1. 确认IMU物理安装方向（标签朝向）
2. 运行`ros2 topic echo /imu/data_raw`观察原始数据
3. 逐步调整mounting_rotation参数
4. 使用test_imu_coordinate验证

---

### Q3: 相机话题不发布
 (适配从hardware_config.yaml读取参数)
- [ ] `bot_hardware/imu_ros2_device/imu_filter_node.py` (从hardware_config.yaml读取滤波参数)
- [ ] `bot_hardware/tools/test_imu_coordinate.py`
- [ ] `launch/imu_bringup.launch.py` (使用统一配置文件)
- [ ] `launch/camera_bringup.launch.py` (使用统一配置文件 + 标定文件)
- [ ] `launch/sensors_bringup.launch.py` (集成launch)

### 配置文件检查
- [ ] **hardware_config.yaml** (确认已包含所有运行参数，无需新增配置文件)
  - `serial.imu_port`, `serial.imu_baudrate` ✅ (line 15-17)
  - `imu.publish_rate`, `imu.frame_id` ✅ (line 73-76)
  - `imu.mounting_rotation` ✅ (line 83-86)
  - `imu.filter.*` ✅ (line 113-118)
  - `camera.rgb`, `camera.depth` ✅ (line 125-136)
  - `camera.topics` ✅ (line 141-145)
  - `paths.imu_calibration_file` ✅ (line 321)
  - `paths.camera_calibration_file` ✅ (line 322)

### 标定文件（单独存放在calibration/目录）
- [ ] `~/lododo_bot/calibration/imu_bias.yaml` (IMU零偏标定数据)
- [ ] `~/lododo_bot/calibration/camera_info.yaml` (相机内参标定数据)

### 系统配置
# 检查相机权限
ls -l /dev/bus/usb/*/*  # 查找Orbbec设备

# 测试相机（不通过ROS2）
sudo apt install astra-tools
astra-viewer  # Orbbec官方工具
```
alignment: true`（hardware_config.yaml line 139）
2. 降低分辨率或帧率（在hardware_config.yaml中调整）
3. 使用专用USB3.0端口（避免USB hub）

**参数调整** (hardware_config.yaml):
```yaml
camera:
  rgb:
    fps: 15  # 降低到15fps
  depth:
    fps: 15
  enable_alignment: true  # 确保启用对齐
```
### Q4: RGB和Depth时间戳差异过大

**可能原因**:
1. 相机配置中未启用硬件同步
2. USB带宽限制导致帧延迟

**解决方案**:
1. 确保`enable_d2c_viewer: true`（Depth to Color对齐）
2. 降低分辨率或帧率
3. 使用专用USB3.0端口（避免USB hub）

---

## 交付清单

### 代码文件
- [ ] `bot_hardware/imu_ros2_device/ybimu_driver.py`
- [ ] `bot_hardware/imu_ros2_device/imu_filter_node.py`
- [ ] `bot_hardware/tools/test_imu_coordinate.py`
- [ ] `config/imu_config.yaml`
- [ ] `config/camera_config.yaml`
- [ ] `launch/imu_bringup.launch.py`
- [ ] `launch/camera_bringup.launch.py`
- [ ] `launch/sensors_bringup.launch.py`

### 配置文件更新
- [ ] `hardware_config.yaml` (添加imu.mounting_rotation)
- [ ] `setup.py` (注册新的entry_points)
- [ ] `/etc/udev/rules.d/99-lekiwi-imu.rules`
- [ ] `/etc/udev/rules.d/99-lekiwi-camera.rules`

### 文档
- [ ] `docs/TEST_IMU_COORDINATE_USAGE.md`
- [ ] `docs/CAMERA_CALIBRATION_GUIDE.md`（可选）
- [ ] 本文档更新（标记完成状态）

**⚠️ 配置管理原则总结**:
1. **运行参数** → `hardware_config.yaml` (统一配置源)
   - IMU串口、波特率、发布频率
   - 相机分辨率、帧率、话题名称
   - mounting_rotation（安装角度）
   - 滤波参数

2. **标定数据** → `calibration/` 目录 (单独存放)
   - `calibration/imu_bias.yaml` - IMU零偏
   - `calibration/camera_info.yaml` - 相机内参
   - 这些文件由标定工具生成，不混入config/

3. **禁止创建的文件**:
   - ❌ `config/imu_config.yaml` (应使用hardware_config.yaml)
   - ❌ `config/camera_config.yaml` (应使用hardware_config.yaml)
   - ❌ 任何单独的传感器配置文件

---

## 下一步: P4阶段预览

**P4目标**: 创建完整的硬件启动launch文件，集成P2硬件控制 + P3传感器

**关键Launch文件**:
```
hardware_bringup.launch.py（完整版）
  ├─ omni_hardware_node（舵机控制+里程计）
  ├─ servo_health_monitor（健康监控）
  ├─ imu_bringup.launch.py（IMU）
  └─ camera_bringup.launch.py（相机）
```

**验证目标**:
- 所有硬件节点稳定运行
- 所有话题正常发布（/wheel/odom, /imu/data, /camera/*)
- 为Nav2和RTABMap准备好数据输入

---

**文档结束**
