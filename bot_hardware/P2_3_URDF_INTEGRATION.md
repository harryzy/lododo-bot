# P2.3阶段 - URDF集成与ros2_control配置

## 📋 完成内容

### 1. URDF Hardware Plugin配置

**文件**: `urdf/bot_hardware.ros2_control.xacro`

定义了ros2_control硬件接口：
- Hardware Plugin: `bot_hardware/OmniHardwareInterface`
- State Interfaces (6个): position_x, position_y, position_theta, velocity_x, velocity_y, velocity_theta
- Command Interfaces (3个): linear_x, linear_y, angular_z
- 支持仿真/真机切换: `use_sim` 参数

### 2. Controller配置

**文件**: `config/controllers.yaml`

配置了两个控制器：
- `joint_state_broadcaster`: 发布/joint_states
- `omni_wheel_controller`: 接收/cmd_vel，转换为command_interfaces

关键参数：
- 更新频率: 50Hz
- 看门狗超时: 0.5秒
- 速度限制: 与hardware_config.yaml一致

### 3. 测试Launch文件

**文件**: `launch/hardware_test.launch.py`

功能：
- 启动controller_manager
- 加载robot_state_publisher
- 自动加载控制器（事件驱动）
- 可选keyboard teleop

## 🚀 使用方法

### 构建

```bash
cd ~/lododo_bot
colcon build --packages-select bot_hardware --symlink-install
source install/setup.bash
```

### 测试（仅验证配置，不启动真实硬件）

```bash
# 基础测试
ros2 launch bot_hardware hardware_test.launch.py

# 带键盘控制
ros2 launch bot_hardware hardware_test.launch.py enable_teleop:=true
```

### 验证命令

```bash
# 1. 检查控制器状态
ros2 control list_controllers

# 预期输出:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# omni_wheel_controller[velocity_controllers/OmniWheelController] active

# 2. 检查硬件接口
ros2 control list_hardware_interfaces

# 预期输出:
# Command interfaces:
#   base_link/linear_x [available] [claimed]
#   base_link/linear_y [available] [claimed]
#   base_link/angular_z [available] [claimed]
# State interfaces:
#   base_link/position_x [available]
#   base_link/position_y [available]
#   base_link/position_theta [available]
#   base_link/velocity_x [available]
#   base_link/velocity_y [available]
#   base_link/velocity_theta [available]

# 3. 测试速度指令
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 4. 监控里程计
ros2 topic echo /wheel/odom
```

## ⚠️ 注意事项

### P2.3阶段限制

1. **Hardware Interface未注册为插件**
   - 当前OmniHardwareInterface Python实现
   - ros2_control需要C++插件或特殊配置
   - **解决方案**: 下一阶段P2.4创建C++包装器或使用ros2_control_py

2. **Controller类型不匹配**
   - `velocity_controllers/OmniWheelController` 不是标准控制器
   - 需要自定义控制器或使用`diff_drive_controller`

3. **测试环境**
   - 当前配置为真机模式 (`use_sim:=false`)
   - 实际硬件测试需要连接ST3215舵机
   - 可使用仿真模式测试: `use_sim:=true`

### 下一步工作 (P2.4)

1. **创建C++ Hardware Interface插件**
   - 使用pluginlib导出OmniHardwareInterface
   - 或者探索ros2_control_py方案

2. **实现自定义Controller**
   - 基于`controller_interface`创建OmniWheelController
   - 或配置diff_drive_controller适配全向轮

3. **硬件测试**
   - 在Raspberry Pi 4B上测试
   - 验证ST3215舵机通信
   - 验证完整控制流程

## 📚 参考文档

- URDF配置: `urdf/bot_hardware.ros2_control.xacro`
- Controller配置: `config/controllers.yaml`
- Launch文件: `launch/hardware_test.launch.py`
- 设计文档: `docs/HARDWARE_DEPLOYMENT_DESIGN.md` §3.4
- ros2_control文档: https://control.ros.org/humble/

## 🎯 P2.3完成检查清单

- [x] 创建bot_hardware.ros2_control.xacro
- [x] 定义state_interfaces (6个)
- [x] 定义command_interfaces (3个)
- [x] 创建controllers.yaml配置
- [x] 创建hardware_test.launch.py
- [x] 更新package.xml依赖
- [x] 更新setup.py安装配置
- [ ] **待P2.4**: 注册Hardware Interface插件
- [ ] **待P2.4**: 实现或配置Controller
- [ ] **待P2.4**: 硬件测试验证

**状态**: P2.3基础配置完成，P2.4需要进行插件注册和测试
