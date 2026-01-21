# 真机Launch文件使用指南

## 文件列表

P4.2阶段创建了4个分层设计的真机启动文件（共1206行代码）：

| 文件名 | 行数 | 用途 | 默认参数 |
|--------|------|------|----------|
| `real_robot_bringup.launch.py` | 509 | 基础版（通用） | slam=false, enable_nav=true, use_rviz=false |
| `real_robot_slam.launch.py` | 171 | SLAM建图专用 | 固定slam=true, use_rviz=true |
| `real_robot_navigation.launch.py` | 185 | 定位导航专用 | 固定slam=false+enable_nav=true |
| `real_robot_web_full.launch.py` | 341 | Web控制完整版 | 包含rosbridge+MissionPlanner |

---

## 快速启动

### 场景1: SLAM建图（首次建图）

```bash
ros2 launch bot_bringup real_robot_slam.launch.py
```

**特点**:
- 固定SLAM模式（slam=true）
- 默认启动RViz可视化
- 默认不启动Nav2（提高建图质量）
- 建图完成后地图自动保存

**保存地图**:
```bash
# 方法1: RTABMap自动保存（关闭节点时）
# 方法2: 手动触发
ros2 service call /rtabmap/save_map std_srvs/srv/Empty
```

---

### 场景2: 定位导航（自主导航）

```bash
ros2 launch bot_bringup real_robot_navigation.launch.py map_name:=office_floor1
```

**特点**:
- 固定定位模式（slam=false）
- 强制启用Nav2导航
- 需要指定已有地图名称
- 默认启动RViz

**导航命令**:
```bash
# 在RViz中: 2D Goal Pose工具
# 或通过话题:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}}}"
```

---

### 场景3: Web控制（SLAM建图）

```bash
# 1. 启动完整系统（SLAM模式）
ros2 launch bot_bringup real_robot_web_full.launch.py slam:=true

# 2. 另一个终端启动Web服务器
cd ~/lododo_bot/src/bot_teleop
bash scripts/start_web_server.sh

# 3. 浏览器访问
http://<robot_ip>:8000
```

---

### 场景4: Web控制（定位导航）

```bash
# 1. 启动完整系统（定位模式）
ros2 launch bot_bringup real_robot_web_full.launch.py \
  slam:=false \
  map_name:=office_floor1

# 2. 启动Web服务器（同上）
# 3. 浏览器访问（同上）
```

---

### 场景5: 仅硬件+传感器融合测试

```bash
ros2 launch bot_bringup real_robot_bringup.launch.py \
  enable_nav:=false \
  use_rviz:=false
```

**用途**: P4.3硬件连通性测试，不启动导航模块

---

## 参数说明

### 通用参数（所有launch文件）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `log_level` | string | warn | 日志级别（debug/info/warn/error） |
| `config_file` | string | hardware_config.yaml | 硬件配置文件路径 |
| `use_rviz` | bool | false（bringup）<br>true（slam/nav） | 是否启动RViz |
| `rviz_config` | string | nav2_rtabmap.rviz | RViz配置文件 |
| `nav2_params` | string | nav2_params_imu.yaml | Nav2参数文件 |

### real_robot_bringup.launch.py 专用

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `slam` | bool | false | SLAM模式（true）或定位模式（false） |
| `map_name` | string | '' | 定位模式必填，地图名称（相对路径） |
| `enable_nav` | bool | true | 是否启动Nav2导航栈 |

### real_robot_web_full.launch.py 专用

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `rosbridge_port` | int | 9090 | rosbridge WebSocket端口 |

---

## 架构说明

### 5阶段启动序列（事件驱动）

```
[Phase 1] Hardware Layer
    ↓ (OnProcessStart)
[Phase 2] EKF Sensor Fusion
    ↓ (OnProcessStart)
[Phase 3] RTABMap SLAM/Localization + Depth Processing
    ↓ (OnProcessStart)
[Phase 4] Nav2 Navigation Stack
    ↓ (OnProcessStart, web_full only)
[Phase 5] MissionPlanner + CommandAdapter + rosbridge
```

### 模块依赖关系

```
hardware_bringup (bot_hardware)
  ├─ servo_bringup (舵机+里程计)
  └─ sensor_bringup (IMU+相机+TF)
      ↓
ekf_filter_node (robot_localization)
  └─ 输出: /odometry/filtered
      ↓
rtabmap_slam / rtabmap_localization (rtabmap_slam)
  ├─ 输出: /map
  └─ 输出: /rtabmap/localization_pose
      ↓
nav2_bringup (nav2)
  └─ 输出: /cmd_vel
      ↓
mission_planner (bot_navigation, web_full only)
  └─ 服务: /mission/*
      ↓
command_adapter (bot_cmd_interface, web_full only)
  └─ 话题: /cmd/request, /cmd/response
      ↓
rosbridge_server (rosbridge_server, web_full only)
  └─ WebSocket: ws://0.0.0.0:9090
```

---

## 配置文件选择

| 模块 | 配置文件 | 路径 | 说明 |
|------|---------|------|------|
| EKF | robot_localization_rtabmap.yaml | bot_navigation/config/localization/ | 真机版本（禁用wheel yaw） |
| RTABMap | rtabmap.yaml | bot_slam/config/slam/ | RGB-D SLAM配置 |
| Nav2 | nav2_params_imu.yaml | bot_navigation/config/nav2/ | IMU融合版本 |
| Hardware | hardware_config.yaml | bot_hardware/config/ | 硬件端口配置 |

---

## 地图管理

### 地图存储位置

```
<workspace>/maps/<map_name>/
  ├─ rtabmap.db          # RTABMap数据库
  ├─ map.pgm             # 2D栅格地图
  └─ map.yaml            # 地图元数据
```

### 地图路径规则

- 使用**相对路径**（相对于工作空间）
- launch文件自动获取工作空间路径
- 示例: `map_name:=office_floor1` → `<workspace>/maps/office_floor1/map.yaml`

---

## 调试技巧

### 提高日志详细度

```bash
ros2 launch bot_bringup real_robot_bringup.launch.py log_level:=info
```

### 查看节点状态

```bash
ros2 node list
ros2 topic list
ros2 topic hz /wheel/odom
ros2 topic hz /odometry/filtered
```

### 检查TF树

```bash
ros2 run tf2_tools view_frames
# 生成: frames.pdf
```

### 监控导航状态

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

---

## 注意事项

### ⚠️ 关键要求

1. **地图名称**: 定位模式下`map_name`参数必填
2. **首次定位**: 需要在RViz中手动设置初始位姿（2D Pose Estimate）
3. **网络端口**: 确保防火墙允许9090端口（rosbridge）和8000端口（Web服务器）
4. **传感器校准**: IMU和相机TF必须正确配置
5. **事件驱动**: 不使用TimerAction延迟启动，完全基于事件触发

### 🐛 常见问题

**Q1: EKF融合输出异常（机器人旋转漂移）**
- 检查: 确认使用`robot_localization_rtabmap.yaml`
- 验证: wheel odom的yaw/vyaw已禁用
- 验证: IMU数据发布正常（/imu/data @ 10Hz）

**Q2: RTABMap重定位失败**
- 检查: 地图文件存在且路径正确
- 尝试: 在RViz中手动设置初始位姿
- 尝试: 驾驶机器人到已建图区域

**Q3: Nav2路径规划失败**
- 检查: /map话题发布正常
- 检查: /odometry/filtered话题发布正常
- 验证: 目标点在自由空间内

**Q4: Web前端连接失败**
- 检查: rosbridge_server节点运行正常
- 验证: WebSocket端口9090未被占用
- 测试: `telnet <robot_ip> 9090`

---

## 下一步（P4.3/P4.4）

### P4.3 硬件连通性测试

```bash
# 测试舵机通信
ros2 run bot_hardware test_servo_control

# 测试IMU坐标系
ros2 run bot_hardware test_imu_coordinate

# 测试传感器集成
bash src/bot_hardware/scripts/test_sensor_integration.sh
```

### P4.4 运动学校准测试

需要在2m x 2m测试场地执行：
- 直线1m运动测试（误差<5cm）
- 360°旋转测试（误差<5°）
- 对角线(1,1)运动测试（误差<10cm）
- 8字形轨迹返回测试（误差<15cm）

---

## 相关文档

- [硬件部署设计文档](../bot_hardware/docs/HARDWARE_DEPLOYMENT_DESIGN.md)
- [实现路线图](../bot_hardware/docs/IMPLEMENTATION_ROADMAP.md) (P4.2 完成)
- [MissionPlanner使用指南](./MISSION_PLANNER_USAGE.md)
- [CommandAdapter文档](../bot_cmd_interface/README.md)

---

**创建日期**: 2026-01-21  
**作者**: lododo Bot Development Team  
**版本**: v1.0 - P4.2完成版
