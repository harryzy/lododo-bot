# LeKiwi机器人仿真环境快速开始指南

**更新日期**: 2025年12月2日  
**状态**: ✅ 基础仿真框架已完成

---

## 🎉 已完成的工作

### 1. 依赖安装 ✅
- ROS2 Humble基础包
- Gazebo仿真环境
- Nav2导航系统
- SLAM工具 (slam_toolbox, rtabmap)
- 传感器融合 (robot_localization)
- Python依赖 (numpy, opencv等)

### 2. 功能包创建 ✅
已创建9个功能包：
```
src/
├── bot_description/    # 机器人模型（已存在）
├── bot_bringup/        # 系统启动集成 ✅
├── bot_gazebo/         # Gazebo仿真 ✅
├── bot_control/        # 运动控制 ✅
├── bot_navigation/     # 导航系统（框架）
├── bot_slam/           # SLAM建图（框架）
├── bot_perception/     # 感知系统（框架）
├── bot_teleop/         # 远程操控 ✅
├── bot_hardware/       # 硬件驱动（预留）
└── bot_voice/          # 语音控制（预留）
```

### 3. 核心功能实现 ✅

#### 运动学控制器 (`bot_control/omni_controller.py`)
- ✅ 三轮全向轮逆运动学
- ✅ 速度限制和保护
- ✅ 实时轮速计算
- ✅ 基于实测参数（0.105m, 0.085m）

#### 键盘遥控 (`bot_teleop/keyboard_teleop.py`)
- ✅ WASD前后左右移动
- ✅ QE左右旋转
- ✅ 对角线移动（I/O/K/L）
- ✅ 速度动态调整
- ✅ 急停功能

#### Gazebo仿真配置
- ✅ 空世界环境
- ✅ RGB-D相机插件（模拟Astra Pro）
- ✅ 全向移动插件
- ✅ 里程计发布
- ✅ 机器人模型集成

---

## 🚀 快速启动

### 步骤1: 环境设置
```bash
cd ~/lododo_bot
source install/setup.bash
```

### 步骤2: 启动仿真（方式A - 完整启动）
```bash
# 一键启动：Gazebo + 控制器 + RViz
ros2 launch bot_bringup simulation.launch.py
```

**包含内容**:
- Gazebo仿真环境
- LeKiwi机器人模型
- 全向轮控制器
- Joint State Publisher
- RViz2可视化

### 步骤2: 启动仿真（方式B - 分步启动）
```bash
# 终端1: 启动Gazebo
ros2 launch bot_gazebo gazebo.launch.py

# 终端2: 启动控制器
ros2 run bot_control omni_controller

# 终端3 (可选): RViz可视化
rviz2
```

### 步骤3: 键盘控制
```bash
# 新终端
cd ~/lododo_bot
source install/setup.bash
ros2 run bot_teleop keyboard_teleop
```

**控制按键**:
```
移动:
  w/s - 前进/后退
  a/d - 左移/右移
  q/e - 左转/右转
  i/o/k/l - 对角线移动

速度:
  z/x - 增加/减少线速度
  c/v - 增加/减少角速度

其他:
  space - 急停
  h - 帮助
  Ctrl+C - 退出
```

---

## 📊 功能测试

### 测试1: 基础移动测试
```bash
# 前进1秒
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}}"

# 侧移1秒
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}}"

# 旋转1秒
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"

# 停止
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"
```

### 测试2: 传感器数据检查
```bash
# 查看所有话题
ros2 topic list

# 监控里程计
ros2 topic echo /odom --no-arr

# 监控相机图像
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth/points
```

### 测试3: TF树验证
```bash
# 生成TF树PDF
ros2 run tf2_tools view_frames

# 查看frames.pdf
evince frames.pdf
```

预期TF树：
```
odom
└── base_link
    ├── base_plate_layer1-v5
    │   ├── 轮组1 (后轮)
    │   ├── 轮组2 (右前轮)
    │   ├── 轮组3 (左前轮)
    │   └── Camera-Mount-v8
    │       └── Camera-Model-v3
    └── base_plate_layer2-v3
```

---

## 🐛 故障排查

### 问题1: Gazebo黑屏或无法启动
```bash
# 重置Gazebo
killall gzserver gzclient
rm -rf ~/.gazebo

# 检查环境变量
echo $GAZEBO_MODEL_PATH
```

### 问题2: 机器人不动
```bash
# 检查控制器是否运行
ros2 node list | grep omni_controller

# 检查cmd_vel话题
ros2 topic info /cmd_vel
ros2 topic hz /cmd_vel

# 手动发送测试命令
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --rate 10
```

### 问题3: 相机数据无输出
```bash
# 检查相机话题
ros2 topic list | grep camera

# 在Gazebo中检查插件是否加载
# Gazebo界面 -> World -> Plugins
```

### 问题4: RViz无法显示机器人
```bash
# 检查robot_description
ros2 topic echo /robot_description --once

# 检查TF
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 📁 文件结构

### 核心文件
```
src/
├── bot_description/urdf/
│   ├── lekiwi_bot.xacro           # 主URDF模型
│   ├── lekiwi_bot.gazebo.xacro    # Gazebo插件配置 ✅ 新增
│   └── lekiwi_bot_sim.xacro       # 仿真集成文件 ✅ 新增
│
├── bot_gazebo/
│   ├── worlds/empty.world         # 空世界 ✅
│   └── launch/gazebo.launch.py    # Gazebo启动 ✅
│
├── bot_control/
│   └── bot_control/
│       └── omni_controller.py     # 运动学控制器 ✅
│
├── bot_teleop/
│   ├── bot_teleop/
│   │   └── keyboard_teleop.py     # 键盘控制 ✅
│   └── launch/
│       └── keyboard_teleop.launch.py
│
└── bot_bringup/
    └── launch/
        └── simulation.launch.py   # 仿真总启动 ✅
```

---

## 🎯 下一步开发计划

### Phase 1: 仿真完善 (当前阶段)
- [x] Gazebo基础仿真环境
- [x] 全向轮运动控制
- [x] 键盘遥控
- [ ] 添加障碍物世界
- [ ] 相机数据可视化验证
- [ ] 运动学参数校准测试

### Phase 2: 导航系统集成
- [ ] Nav2配置文件
- [ ] 深度图转激光雷达
- [ ] Costmap配置
- [ ] 路径规划测试
- [ ] 动态避障测试

### Phase 3: SLAM建图
- [ ] Slam Toolbox配置
- [ ] 2D地图构建
- [ ] RTABMap 3D建图
- [ ] 地图保存与加载

### Phase 4: 高级功能
- [ ] Web控制界面
- [ ] 视觉里程计集成
- [ ] 传感器融合 (EKF)
- [ ] 物体检测（后期）
- [ ] 语音控制（后期）

### Phase 5: 硬件适配（最后）
- [ ] ST3215舵机驱动
- [ ] 真实硬件接口
- [ ] 参数迁移
- [ ] 真机测试

---

## 📝 重要参数

### 机器人物理参数
```yaml
轮子半径: 0.05 m
后轮到中心: 0.105 m
前轮到中心: 0.085 m
机器人质量: 1.5 kg
```

### 速度限制
```yaml
最大线速度: 0.20 m/s (建议值)
最大角速度: 1.5 rad/s
最大轮速: 4.712 rad/s (270°/s)
```

### 相机参数 (模拟Astra Pro)
```yaml
RGB分辨率: 640x480
深度范围: 0.6 - 8.0 m
视场角: 60°
更新频率: 30 Hz
```

---

## 🔧 开发工具

### 常用ROS2命令
```bash
# 编译
colcon build --symlink-install

# 仅编译特定包
colcon build --packages-select bot_control

# 清理重编译
rm -rf build install log && colcon build --symlink-install

# 查看节点
ros2 node list
ros2 node info /omni_controller

# 查看话题
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic hz /odom

# 查看参数
ros2 param list
ros2 param get /omni_controller wheel_radius
```

### 性能监控
```bash
# CPU使用率
top
htop

# Gazebo性能
gz stats

# ROS2性能
ros2 run ros2_control_node ros2_control_node
```

---

## 📚 参考资源

### 项目文档
- `TECHNICAL_DESIGN.md` - 完整技术方案
- `ROBOT_SPECS.md` - 参数速查表

### 外部文档
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo](http://gazebosim.org/)
- [Nav2](https://navigation.ros.org/)

---

## ✅ 验收标准

### 当前阶段完成标准
- [x] 仿真环境能正常启动
- [x] 机器人模型正确显示
- [x] 键盘控制正常响应
- [x] 全向移动功能正常（前后左右旋转）
- [x] 里程计正常发布
- [x] 相机数据正常发布
- [ ] 运动学精度测试（下一步）

---

**祝开发顺利！** 🎉

有问题随时查看故障排查部分或参考技术文档。
