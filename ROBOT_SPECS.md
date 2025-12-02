# LeKiwi机器人规格速查表

**更新日期**: 2025年12月2日

## 核心参数 ✅ (已全部确认)

### 机械参数
```yaml
机器人总重量: 1.5 kg
轮子半径: 0.05 m (50 mm)
后轮到中心距离: 0.105 m
前轮到中心距离: 0.085 m
轮子布局: 三轮全向轮，120°均布（非对称）
机器人半径: ~0.15 m (安全半径)
```

### 电机参数 (ST3215舵机)
```yaml
型号: ST3215
数量: 3个
通信协议: TTL串口
波特率: 1000000 bps
编码器精度: 4096步/圈 (0.088°/步)
编码器范围: 0 ~ 4095
最大转速: 45 RPM (空载)
最大角速度: 270°/s (4.712 rad/s)
位置分辨率: 0.077 mm/步
```

### 性能指标
```yaml
# 理论最大速度
最大轮速: 4.712 rad/s
最大线速度: 0.236 m/s (理论值)
建议线速度: 0.20 m/s (安全值，85%功率)
建议角速度: 1.5 rad/s (~86°/s)

# 加速度限制
最大线加速度: 0.5 m/s²
最大角加速度: 1.0 rad/s²
```

### 传感器配置
```yaml
RGB-D相机:
  型号: 奥比中光 Astra Pro
  RGB分辨率: 1920×1080
  深度分辨率: 640×480
  视场角: 60° H × 49.5° V
  深度范围: 0.6m ~ 8m
  帧率: 30 fps
  接口: USB 3.0

IMU: 无 (采用软件里程计融合方案)

里程计方案:
  主要: 轮式里程计 (ST3215编码器反馈)
  辅助: 视觉里程计 (RTABMap)
  融合: robot_localization (EKF)
```

### 电源系统
```yaml
电池类型: 锂电池
电压: 12V
容量: 5.2 Ah
接口: DC5521
```

### 控制接口
```yaml
舵机控制器:
  设备: /dev/ttyACM0
  波特率: 1000000
  协议: TTL串口

主控:
  平台: 树莓派4B
  系统: Ubuntu 22.04 Server
  Python版本: 3.10
```

## 运动学公式

### 三轮全向轮逆向运动学
```python
# 参数定义
R = 0.05    # 轮子半径 (m)
L1 = 0.105  # 后轮到中心距离 (m)
L2 = 0.085  # 右前轮到中心距离 (m)
L3 = 0.085  # 左前轮到中心距离 (m)

# 轮子方向角（相对于X轴，逆时针为正）
θ1 = π/2        # 90°  - 后轮
θ2 = 7π/6       # 210° - 右前轮
θ3 = 11π/6      # 330° - 左前轮

# 逆向运动学矩阵
# 输入: (Vx, Vy, ωz) - 机器人速度（base_link坐标系）
# 输出: (ω1, ω2, ω3) - 三个轮子的角速度

import numpy as np

def inverse_kinematics(vx, vy, omega_z):
    """
    计算轮子角速度
    
    Args:
        vx: X方向速度 (m/s)
        vy: Y方向速度 (m/s)
        omega_z: 旋转角速度 (rad/s)
    
    Returns:
        (w1, w2, w3): 三个轮子的角速度 (rad/s)
    """
    # 运动学矩阵（需要根据实际几何关系推导）
    # J = 1/R * [[-sin(θ1), cos(θ1), L1],
    #            [-sin(θ2), cos(θ2), L2],
    #            [-sin(θ3), cos(θ3), L3]]
    
    J = np.array([
        [-np.sin(np.pi/2),    np.cos(np.pi/2),    L1],  # 后轮
        [-np.sin(7*np.pi/6),  np.cos(7*np.pi/6),  L2],  # 右前轮
        [-np.sin(11*np.pi/6), np.cos(11*np.pi/6), L3],  # 左前轮
    ]) / R
    
    velocity = np.array([vx, vy, omega_z])
    wheel_speeds = J @ velocity
    
    return wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]

# 简化版（展开后）
def inverse_kinematics_simple(vx, vy, omega_z):
    """简化版本的逆运动学"""
    w1 = (1/R) * (vy + L1 * omega_z)                    # 后轮
    w2 = (1/R) * (-0.866*vx - 0.5*vy + L2 * omega_z)   # 右前轮
    w3 = (1/R) * (0.866*vx - 0.5*vy + L3 * omega_z)    # 左前轮
    return w1, w2, w3
```

### 正向运动学（轮速 → 机器人速度）
```python
def forward_kinematics(w1, w2, w3):
    """
    从轮子角速度计算机器人速度
    
    Args:
        w1, w2, w3: 三个轮子的角速度 (rad/s)
    
    Returns:
        (vx, vy, omega_z): 机器人速度
    """
    # 伪逆矩阵方法（最小二乘解）
    J = np.array([
        [-np.sin(np.pi/2),    np.cos(np.pi/2),    L1],
        [-np.sin(7*np.pi/6),  np.cos(7*np.pi/6),  L2],
        [-np.sin(11*np.pi/6), np.cos(11*np.pi/6), L3],
    ]) / R
    
    wheel_speeds = np.array([w1, w2, w3])
    J_pinv = np.linalg.pinv(J)
    velocity = J_pinv @ wheel_speeds
    
    return velocity[0], velocity[1], velocity[2]
```

### 里程计计算
```python
def encoder_to_distance(encoder_delta):
    """
    编码器增量转换为轮子移动距离
    
    Args:
        encoder_delta: 编码器增量（步数）
    
    Returns:
        distance: 轮子移动距离 (m)
    """
    angle_rad = (encoder_delta / 4096.0) * 2 * np.pi
    distance = angle_rad * R  # R = 0.05m
    return distance

# 理论分辨率
position_resolution = (2 * np.pi / 4096) * 0.05  # ≈ 0.077 mm
```

## ROS2话题映射

### 控制输入
```yaml
/cmd_vel: geometry_msgs/Twist
  - linear.x: 前进速度 (m/s) [-0.20, 0.20]
  - linear.y: 侧移速度 (m/s) [-0.20, 0.20]
  - angular.z: 旋转速度 (rad/s) [-1.5, 1.5]
```

### 传感器输出
```yaml
/odom: nav_msgs/Odometry (50 Hz)
  - 轮式里程计，基于ST3215编码器

/camera/color/image_raw: sensor_msgs/Image (30 fps)
/camera/depth/image_raw: sensor_msgs/Image (30 fps)
/camera/depth/points: sensor_msgs/PointCloud2 (30 fps)

/scan: sensor_msgs/LaserScan (10 Hz)
  - 从深度图转换的虚拟激光雷达
```

### 关节状态
```yaml
/joint_states: sensor_msgs/JointState (50 Hz)
  - wheel_rear_joint
  - wheel_right_joint
  - wheel_left_joint
```

## 校准检查清单

### 几何参数验证
- [ ] 精确测量轮子到中心距离（误差<1mm）
- [ ] 验证三个轮子的夹角（理论120°，允许±2°）
- [ ] 确认轮子半径（实际充气/磨损后）
- [ ] 测量机器人实际质心位置

### 电机性能测试
- [ ] 单个舵机空载测试（验证45 RPM）
- [ ] 负载测试（1.5kg整机）
- [ ] 编码器精度验证（旋转10圈误差<1°）
- [ ] 串口通信稳定性测试（连续运行1小时）

### 运动学校准
- [ ] 直线前进1米测试（误差<5cm）
- [ ] 直线后退1米测试（对称性验证）
- [ ] 原地旋转360°测试（误差<3°）
- [ ] 侧向移动测试（全向轮性能）
- [ ] 圆形轨迹测试（半径0.5m，3圈）
- [ ] 长距离里程计漂移测试（10米，误差<20cm）

### 传感器校准
- [ ] 相机内参标定（使用棋盘格）
- [ ] 深度相机精度测试（0.6-3m范围）
- [ ] TF树验证（所有坐标系关系正确）
- [ ] 时间同步检查（传感器时间戳对齐）

## 故障排查速查

### 舵机不响应
```bash
# 检查串口
ls -l /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0

# 测试串口通信
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0', 1000000); print(s.is_open)"

# 检查舵机供电（需要12V）
# 检查舵机ID设置（1, 2, 3）
```

### 运动异常
```bash
# 检查运动学参数配置
ros2 param get /omni_controller wheel_radius

# 检查速度限制
ros2 param get /controller_server FollowPath.max_vel_x

# 实时监控轮速
ros2 topic echo /joint_states
```

### 里程计漂移
```bash
# 检查编码器读取
ros2 topic hz /joint_states  # 应该是50Hz

# 检查视觉里程计
ros2 topic echo /visual_odom --no-arr

# 检查融合输出
ros2 topic echo /odom/filtered --no-arr
```

---

## 快速命令参考

### 启动机器人
```bash
# 仿真
ros2 launch bot_bringup simulation.launch.py

# 真实硬件
ros2 launch bot_bringup robot.launch.py

# 导航模式
ros2 launch bot_bringup navigation.launch.py
```

### 手动控制
```bash
# 键盘控制
ros2 run bot_teleop keyboard_teleop

# 发送单次速度命令
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 调试工具
```bash
# 查看TF树
ros2 run tf2_tools view_frames
evince frames.pdf

# 监控话题频率
ros2 topic hz /odom
ros2 topic hz /scan

# RViz2可视化
rviz2 -d $(ros2 pkg prefix bot_description)/share/bot_description/config/lekiwi_bot.rviz
```

---

**注意事项**:
1. 首次运行前必须校准运动学参数
2. 建议在仿真中充分测试后再部署到硬件
3. 定期检查舵机编码器精度（每月一次）
4. 保持电池充足（低于11V可能影响性能）
5. 避免急加速急减速，保护机械结构
