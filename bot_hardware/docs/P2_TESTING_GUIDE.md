# P2阶段测试快速指南

**版本**: v1.0  
**日期**: 2026-01-19  
**适用**: OmniHardwareNode独立节点测试

---

## 快速开始

### 1. 构建确认 ✅

```bash
cd ~/lododo_bot
colcon build --packages-select bot_hardware --symlink-install
source install/setup.bash
```

**验证**:
```bash
ros2 pkg executables bot_hardware
# 应该输出: bot_hardware omni_hardware_node
```

---

## 测试方式

### 方式1: Mock测试（无需硬件）⏳

**待补充**: 创建Mock驱动进行单元测试

```bash
# 待实现
python3 -m pytest test/test_omni_hardware_node_mock.py
```

### 方式2: 真机测试（需要硬件）

#### 硬件准备清单:
- [ ] ST3215舵机×3（ID配置为7, 8, 9）
- [ ] USB转串口模块（连接到舵机）
- [ ] 12V电源适配器
- [ ] 串口设备路径确认（/dev/ttyUSB0或/dev/st3215_servo）

#### 步骤1: 配置串口设备

编辑配置文件:
```bash
nano ~/lododo_bot/src/bot_hardware/config/hardware_config.yaml
```

修改串口路径:
```yaml
serial:
  servo_port: '/dev/ttyUSB0'  # 改为实际串口设备
  servo_baudrate: 1000000
```

#### 步骤2: 启动节点

```bash
source ~/lododo_bot/install/setup.bash
ros2 launch bot_hardware hardware_bringup.launch.py
```

**预期输出**:
```
[INFO] [omni_hardware_node]: === OmniHardwareNode Starting ===
[INFO] [omni_hardware_node]: Configuration loaded from: ...
[INFO] [omni_hardware_node]: Servo IDs configured: [7, 8, 9]
[INFO] [omni_hardware_node]: ST3215 driver initialized
[INFO] [omni_hardware_node]: All servos online
[INFO] [omni_hardware_node]: EncoderHandler baseline: [...]
[INFO] [omni_hardware_node]: VelocityRamp initialized with current velocity
[INFO] [omni_hardware_node]: Control loop started at 50Hz
[INFO] [omni_hardware_node]: === OmniHardwareNode Initialized Successfully ===
```

#### 步骤3: 监控话题

**打开新终端**:
```bash
source ~/lododo_bot/install/setup.bash

# 检查话题列表
ros2 topic list
# 应该看到: /cmd_vel, /wheel/odom, /tf

# 查看里程计发布频率（应该接近50Hz）
ros2 topic hz /wheel/odom

# 查看里程计数据
ros2 topic echo /wheel/odom
```

#### 步骤4: 发送测试指令

**前进测试**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

**观察**: 轮子应该开始转动，/wheel/odom应该显示位置变化

**左平移测试**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

**旋转测试**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
  --rate 10
```

**停止**:
```bash
# Ctrl+C停止pub命令
# 等待0.5s，看门狗应该触发，轮子停止
```

---

## 常见问题排查

### 问题1: 节点启动失败

**症状**: `Failed to initialize hardware`

**可能原因**:
1. 串口设备路径错误
2. 串口权限不足
3. 舵机未上电
4. 舵机ID不匹配

**解决方法**:
```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 添加串口权限
sudo usermod -a -G dialout $USER
# 注销重新登录

# 手动测试舵机连通性
python3 -c "
from bot_hardware.drivers.st3215_driver import ST3215Driver
driver = ST3215Driver('/dev/ttyUSB0', 1000000, 0.01)
print('Servo 7 ping:', driver.ping(7))
print('Servo 8 ping:', driver.ping(8))
print('Servo 9 ping:', driver.ping(9))
"
```

### 问题2: 话题发布频率低于50Hz

**症状**: `ros2 topic hz /wheel/odom` 显示 <50Hz

**可能原因**:
1. CPU负载过高
2. 串口读取超时
3. Python GIL限制

**解决方法**:
```bash
# 检查CPU负载
top

# 降低日志输出级别
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# 检查串口超时配置
# 编辑hardware_config.yaml，增加timeout值
```

### 问题3: 轮子不转

**症状**: 发送cmd_vel但轮子不动

**可能原因**:
1. 速度过小（被VelocityRamp限制）
2. 舵机扭矩不足
3. 运动学参数错误

**解决方法**:
```bash
# 增加测试速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10

# 检查VelocityRamp日志
# 应该看到速度限制信息

# 直接测试舵机（绕过节点）
python3 -c "
from bot_hardware.drivers.st3215_driver import ST3215Driver
import time
driver = ST3215Driver('/dev/ttyUSB0', 1000000, 0.01)
driver.write_speed(7, 10.0)  # 10 RPM
time.sleep(2)
driver.write_speed(7, 0.0)
"
```

### 问题4: 看门狗未触发

**症状**: 停止发送cmd_vel，但轮子继续转

**可能原因**:
1. 看门狗超时配置过大
2. last_command_time未更新

**解决方法**:
```bash
# 检查配置
grep watchdog_timeout ~/lododo_bot/src/bot_hardware/config/hardware_config.yaml

# 应该显示: watchdog_timeout: 0.5

# 如果不存在，添加到control区域:
control:
  update_rate: 50
  watchdog_timeout: 0.5
```

---

## 性能验证清单

| 测试项 | 目标值 | 测试命令 | 状态 |
|--------|--------|---------|------|
| 话题发布频率 | 50Hz | `ros2 topic hz /wheel/odom` | ⏳ |
| 前进运动 | 正常 | `pub linear.x=0.1` | ⏳ |
| 左平移运动 | 正常 | `pub linear.y=0.1` | ⏳ |
| 旋转运动 | 正常 | `pub angular.z=0.5` | ⏳ |
| 看门狗触发 | 0.5s | 停止pub，观察停止 | ⏳ |
| 里程计精度 | TBD | 移动1m，测量误差 | ⏳ |

---

## 调试技巧

### 1. 增加日志详细度

编辑 `hardware_config.yaml`:
```yaml
debug:
  enable_verbose_logging: true
  log_encoder_deltas: true
  log_velocity_commands: true
  log_odometry: true
```

### 2. 使用rqt_graph查看话题连接

```bash
rqt_graph
```

### 3. 使用PlotJuggler可视化数据

```bash
ros2 run plotjuggler plotjuggler
```

添加话题: `/wheel/odom/twist/twist/linear/x`

### 4. 录制bag文件分析

```bash
ros2 bag record /cmd_vel /wheel/odom /tf -o test_run
ros2 bag play test_run
```

---

## 下一步

测试通过后，可以进入P3阶段：
1. IMU集成
2. 相机集成
3. EKF融合
4. Nav2导航测试

---

**快速参考命令**:
```bash
# 启动
ros2 launch bot_hardware hardware_bringup.launch.py

# 监控
ros2 topic hz /wheel/odom

# 测试
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --rate 10

# 停止
Ctrl+C
```
