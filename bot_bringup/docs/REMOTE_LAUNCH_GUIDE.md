# 分布式部署Launch文件说明

本目录包含用于分布式部署的launch文件，将系统分为树莓派（硬件层）和PC（计算层）两部分运行。

## 文件列表

### 树莓派端（硬件层）
- **位置**: `bot_hardware/launch/hardware_bringup.launch.py`
- **功能**: 启动硬件驱动（舵机、IMU、相机）
- **用法**: 
  ```bash
  ros2 launch bot_hardware hardware_bringup.launch.py \
    enable_servo:=true \
    enable_imu:=true \
    enable_camera:=true \
    publish_static_tf:=true
  ```

### PC端（计算层）

#### 1. remote_navigation.launch.py
**定位导航模式** - 使用已有地图进行自主导航

**组件**:
- EKF传感器融合
- RTABMap定位
- Nav2导航
- RViz可视化（可选）

**用法**:
```bash
# 基本用法
ros2 launch bot_bringup remote_navigation.launch.py map_name:=office_floor1

# 启用RViz
ros2 launch bot_bringup remote_navigation.launch.py \
  map_name:=office_floor1 \
  use_rviz:=true
```

**参数**:
- `map_name`: 地图名称（必填，如 `office_floor1`）
- `use_rviz`: 是否启动RViz（默认: `true`）
- `rviz_config`: RViz配置文件路径
- `nav2_params`: Nav2参数文件路径
- `log_level`: 日志级别（默认: `warn`）

---

#### 2. remote_slam.launch.py
**SLAM建图模式** - 创建新地图或更新现有地图

**组件**:
- EKF传感器融合
- RTABMap SLAM建图
- Nav2导航（可选）
- RViz可视化（推荐）

**用法**:
```bash
# 基本SLAM建图
ros2 launch bot_bringup remote_slam.launch.py

# SLAM + 启用Nav2（建图期间可导航）
ros2 launch bot_bringup remote_slam.launch.py \
  enable_nav:=true \
  use_rviz:=true
```

**参数**:
- `enable_nav`: 是否启用Nav2（默认: `false`）
- `use_rviz`: 是否启动RViz（默认: `true`）
- `rviz_config`: RViz配置文件路径
- `nav2_params`: Nav2参数文件路径
- `log_level`: 日志级别（默认: `warn`）

**建图技巧**:
1. 缓慢驾驶机器人（使用手柄或键盘遥控）
2. 避免快速旋转，防止运动模糊
3. 覆盖所有需要建图的区域
4. 重访起点以闭合回环

**保存地图**:
```bash
# 方法1: 关闭launch文件（Ctrl+C），RTABMap自动保存
# 方法2: 手动触发保存
ros2 service call /rtabmap/save_map std_srvs/srv/Empty
```

---

#### 3. remote_web_full.launch.py
**完整Web控制环境** - 包含任务调度和Web接口

**组件**:
- 上述所有功能（EKF + RTABMap + Nav2）
- MissionPlanner任务调度
- CommandAdapter统一命令接口
- rosbridge_server Web连接桥
- RViz可视化（可选）

**用法**:
```bash
# SLAM模式 + Web控制
ros2 launch bot_bringup remote_web_full.launch.py slam:=true

# 定位模式 + Web控制
ros2 launch bot_bringup remote_web_full.launch.py \
  slam:=false \
  map_name:=office_floor1

# 启用RViz可视化
ros2 launch bot_bringup remote_web_full.launch.py \
  map_name:=office_floor1 \
  use_rviz:=true
```

**参数**:
- `slam`: 使用SLAM模式（默认: `false`）
- `map_name`: 地图名称（定位模式必填）
- `use_rviz`: 是否启动RViz（默认: `false`）
- `rviz_config`: RViz配置文件路径
- `nav2_params`: Nav2参数文件路径
- `log_level`: 日志级别（默认: `warn`）
- `rosbridge_port`: WebSocket端口（默认: `9090`）

**Web前端访问**:
```bash
# 1. 启动Web服务器（另一个终端）
cd ~/lododo_bot/src/bot_teleop
bash scripts/start_web_server.sh

# 2. 浏览器访问
http://<PC的IP>:8000

# 3. WebSocket连接
ws://<PC的IP>:9090
```

**命令接口测试**:
```bash
# 发送导航请求
ros2 topic pub --once /cmd/request std_msgs/msg/String \
  'data: "{\"header\":{\"request_id\":\"nav-001\",\"timestamp\":\"2026-01-29T12:00:00\",\"priority\":3},\"body\":{\"action\":\"navigate_to_pose\",\"params\":{\"x\":2.0,\"y\":3.0,\"yaw\":0.0},\"timeout\":300.0}}"'

# 监听响应
ros2 topic echo /cmd/response
```

---

## Launch文件对比

| 特性 | remote_navigation | remote_slam | remote_web_full |
|------|------------------|-------------|-----------------|
| EKF融合 | ✅ | ✅ | ✅ |
| RTABMap定位 | ✅ | - | ✅（定位模式） |
| RTABMap SLAM | - | ✅ | ✅（SLAM模式） |
| Nav2导航 | ✅ | 可选 | ✅ |
| MissionPlanner | - | - | ✅ |
| CommandAdapter | - | - | ✅ |
| rosbridge_server | - | - | ✅ |
| RViz | 可选 | 可选 | 可选 |
| 适用场景 | 自主导航 | 地图构建 | Web控制 |

---

## 标准启动流程

### 1. 树莓派端启动（第一步）

```bash
# SSH连接到树莓派
ssh ubuntu@<树莓派IP>

# 进入工作空间
cd ~/lododo_bot
source install/setup.bash

# 启动硬件层
ros2 launch bot_hardware hardware_bringup.launch.py \
  enable_servo:=true \
  enable_imu:=true \
  enable_camera:=true \
  publish_static_tf:=true
```

**检查硬件层是否正常**:
```bash
# 在PC上检查话题（另一个终端）
ros2 topic list | grep -E "wheel|imu|camera"
# 应该看到：/wheel/odom, /imu/data, /camera/color/image_raw 等
```

### 2. PC端启动（第二步，等待5秒）

```bash
# 进入工作空间
cd ~/lododo_bot
source install/setup.bash

# 选择以下之一：

# 选项A: 定位导航
ros2 launch bot_bringup remote_navigation.launch.py \
  map_name:=office_floor1 \
  use_rviz:=true

# 选项B: SLAM建图
ros2 launch bot_bringup remote_slam.launch.py \
  use_rviz:=true

# 选项C: 完整Web控制
ros2 launch bot_bringup remote_web_full.launch.py \
  slam:=false \
  map_name:=office_floor1
```

---

## 前置条件

### 网络配置

**ROS_DOMAIN_ID** (PC和树莓派必须一致):
```bash
# 在 ~/.bashrc 添加
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Cyclone DDS配置** (PC和树莓派):
```bash
# 创建 ~/cyclonedds.xml
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

**Socket缓冲区** (PC和树莓派):
```bash
# 临时设置
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.wmem_max=10485760

# 永久设置
echo "net.core.rmem_max = 10485760" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max = 10485760" | sudo tee -a /etc/sysctl.conf
```

**时钟同步** (使用chrony):
- PC作为时钟服务器
- 树莓派从PC同步时钟

详细配置参见: [DISTRIBUTED_DEPLOYMENT.md](../docs/DISTRIBUTED_DEPLOYMENT.md)

### 地图文件（定位模式）

定位模式需要已有地图文件：
```
workspace/maps/<map_name>/
├── rtabmap.db          # RTABMap数据库
├── map.pgm             # 占据栅格地图
├── map.yaml            # 地图元数据
└── metadata.yaml       # 自定义元数据（可选）
```

---

## 验证通信

**检查节点列表** (PC上执行):
```bash
ros2 node list
# 应该看到树莓派的硬件节点和PC的计算节点
```

**检查话题列表**:
```bash
ros2 topic list | grep -E "odom|imu|camera|map|cmd_vel"
```

**检查话题频率**:
```bash
ros2 topic hz /wheel/odom       # 应该 ~50 Hz
ros2 topic hz /imu/data         # 应该 ~100 Hz
ros2 topic hz /camera/color/image_raw  # 应该 ~15-30 Hz
```

**检查TF树**:
```bash
ros2 run tf2_tools view_frames
# 应该看到完整的TF树：map → odom → base_link → camera_link
```

---

## 故障排查

### 问题1: 节点无法发现

**原因**: ROS_DOMAIN_ID不一致或网络配置问题

**解决**:
```bash
# 检查ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # PC和树莓派应该相同

# 检查Cyclone DDS
echo $RMW_IMPLEMENTATION  # 应该是 rmw_cyclonedds_cpp
echo $CYCLONEDDS_URI      # 应该指向配置文件

# 检查网络连通性
ping <对方IP>
```

### 问题2: Socket缓冲区错误

**症状**:
```
failed to increase socket receive buffer size to at least 10485760 bytes
```

**解决**:
```bash
# PC和树莓派都需要执行
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.wmem_max=10485760
```

### 问题3: 图像传输卡顿

**原因**: WiFi带宽不足

**解决**:
```bash
# 降低相机分辨率/帧率（树莓派端）
ros2 param set /astra_camera color_width 320
ros2 param set /astra_camera color_height 240
ros2 param set /astra_camera color_fps 15

# 或者使用有线以太网（强烈推荐）
```

### 问题4: TF不同步

**原因**: 时钟不同步

**解决**:
```bash
# 在树莓派上检查时钟同步状态
chronyc tracking
# "System time" 偏差应该小于1ms

# 强制同步
sudo chronyc -a makestep
```

---

## 性能优化

### 树莓派优化
```bash
# 禁用不必要服务
sudo systemctl disable bluetooth
sudo systemctl disable cups

# 固定CPU频率（避免降频）
echo "arm_freq=1800" | sudo tee -a /boot/firmware/config.txt
```

### 网络优化
- **强烈推荐使用有线以太网** (WiFi延迟10-50ms，以太网<1ms)
- 降低相机分辨率/帧率（如果必须用WiFi）
- 配置QoS: 大数据用BEST_EFFORT，控制指令用RELIABLE

### 内存占用
- **树莓派**: 硬件层 ~600MB (剩余3.4GB足够)
- **PC**: 计算层 ~3-4GB (推荐8GB+ RAM)

---

## 相关文档

- [分布式部署详细指南](../docs/DISTRIBUTED_DEPLOYMENT.md)
- [任务调度器使用](../docs/MISSION_PLANNER_USAGE.md)
- [命令接口文档](../../bot_cmd_interface/README.md)
- [快速入门](../../QUICKSTART.md)

---

**作者**: LeKiwi Bot Development Team  
**日期**: 2026-01-29  
**版本**: 1.0.0
