# 分布式部署架构 | Distributed Deployment Architecture

**更新日期**: 2026-01-29  
**硬件**: 树莓派4B 4GB + PC工作站

## 架构概述

```
┌─────────────────────────────────────────────────────────────────┐
│                        PC工作站（主计算节点）                      │
│  - Ubuntu 22.04 Desktop x86_64                                  │
│  - ROS2 Humble                                                  │
├─────────────────────────────────────────────────────────────────┤
│  节点列表：                                                        │
│  ✓ robot_localization (EKF传感器融合)                            │
│  ✓ rtabmap_slam (RGB-D SLAM建图)                                │
│  ✓ nav2 (自主导航)                                               │
│  ✓ mission_planner (任务管理)                                    │
│  ✓ command_adapter (统一命令接口)                                │
│  ✓ exploration_handler (自主探索)                                │
│  ✓ patrol_handler (巡逻控制)                                     │
│  ✓ rviz2 (可视化)                                                │
└─────────────────────────────────────────────────────────────────┘
                            ↕ 以太网/WiFi
┌─────────────────────────────────────────────────────────────────┐
│                   树莓派4B（硬件接口节点）                         │
│  - Ubuntu 22.04 Server arm64（推荐）                             │
│  - ROS2 Humble                                                  │
├─────────────────────────────────────────────────────────────────┤
│  节点列表：                                                        │
│  ✓ omni_hardware_node (全向轮控制 + 轮式里程计)                   │
│  ✓ ybimu_driver (IMU驱动 /dev/ttyUSB0)                          │
│  ✓ imu_filter_node (IMU数据滤波)                                 │
│  ✓ astra_camera (RGB-D相机驱动)                                  │
│  ✓ servo_controller (舵机控制，如果有)                            │
└─────────────────────────────────────────────────────────────────┘
```

## 系统安装步骤

### 1. 树莓派系统安装

**下载镜像**：
```bash
# Ubuntu 22.04 Server arm64（推荐）
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.3-preinstalled-server-arm64+raspi.img.xz
```

**烧录系统**（使用Raspberry Pi Imager或balenaEtcher）：
1. 选择"Ubuntu Server 22.04.3 LTS (64-bit)"
2. 配置WiFi和SSH（在Imager高级选项中）
3. 烧录到SD卡

**首次启动配置**：
```bash
# SSH连接（默认用户名ubuntu，密码ubuntu，首次登录需修改）
ssh ubuntu@<树莓派IP>

# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装必要工具
sudo apt install -y git wget curl vim htop

# 设置时区
sudo timedatectl set-timezone Asia/Shanghai

# 安装chrony时钟同步（关键！）
sudo apt install -y chrony
sudo systemctl enable chrony
```

### 问题7：RTABMap报"Did not receive data since 5 seconds"（QoS不匹配）

**症状**：
```
[rtabmap-6] [WARN] rtabmap_slam: Did not receive data since 5 seconds!
[rtabmap-6] rtabmap_slam subscribed to (approx sync):
[rtabmap-6]    /odometry/filtered \
[rtabmap-6]    /camera/color/image_raw \
[rtabmap-6]    /camera/depth/image_raw
```

**诊断步骤**（按顺序执行）：

**1. 检查chrony时钟同步（不是简单的date命令！）**：
```bash
# PC端执行
sudo chronyc tracking
# 关键指标：
#   - System time: 应该小于 ±1ms
#   - Last offset: 上次时间校正值

sudo chronyc sources -v
# 检查同步源状态
#   - ^* 表示当前活跃源
#   - Reach: 应该是 377 (八进制，表示最近8次测量全部成功)

# 树莓派端执行（SSH连接）
ssh lododo@192.168.2.120
sudo chronyc tracking
# 应该看到 Reference ID: 192.168.2.169（PC的IP）
# System time: 应该小于 ±1ms
```

**2. 检查网络延迟（WiFi质量）**：
```bash
# PC端执行
ping -c 20 192.168.2.120 | tail -4
# 关键指标：
#   - rtt avg: 应该 <10ms（以太网）或 <30ms（WiFi）
#   - packet loss: 应该 0%
#   - 如果看到 "+N duplicates"，说明网络有问题

# 如果延迟>30ms或有丢包，建议：
#   - 方案A（推荐）：改用有线以太网
#   - 方案B：调整WiFi信道，避免干扰
#   - 方案C：降低图像分辨率/帧率
```

**3. 检查QoS匹配（关键！）**：
```bash
# 检查IMU话题的QoS
ros2 topic info /imu/data -v
# 输出示例：
#   Publisher: Reliability: RELIABLE      ← 发布者
#   Subscription: Reliability: BEST_EFFORT ← 订阅者（EKF）
# ⚠️ 如果 RELIABLE → BEST_EFFORT，这是ROS2不兼容的组合！

# 检查相机话题（作为对比）
ros2 topic info /camera/color/image_raw -v
# RELIABLE → RELIABLE 是兼容的

# 验证话题是否能接收
timeout 3 ros2 topic echo /imu/data --once
# 如果超时无输出，确认是QoS不匹配
```

**4. 检查重复节点**：
```bash
# 检查是否有重复的EKF节点
ros2 node list | grep ekf
# 警告信息："nodes share exact name" 说明有重复

ps aux | grep ekf_filter
# 如果看到多个进程，清理它们：
pkill -9 -f ekf_node
```

**根本原因**：
- **QoS策略不匹配**：IMU驱动发布 `RELIABLE`（保证送达），但EKF订阅 `BEST_EFFORT`（尽力而为）
- **robot_localization默认QoS**：BEST_EFFORT（适合高频低延迟传感器）
- **ROS2兼容规则**：RELIABLE发布者 → BEST_EFFORT订阅者 = **不兼容**

**解决方案A：修改IMU驱动QoS（推荐）**：
```bash
# 找到IMU驱动节点的启动文件
find ~/lododo_bot/src -name "*imu*.launch.py" -o -name "*imu*.py" | grep -v build

# 在IMU发布者代码中添加QoS设置：
# Python示例：
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 改为BEST_EFFORT
    depth=10
)
self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile)
```

**解决方案B：robot_localization添加QoS覆盖（backup）**：
```bash
# 编辑 robot_localization_rtabmap.yaml
vim ~/lododo_bot/src/bot_navigation/config/localization/robot_localization_rtabmap.yaml

# 添加（在ekf_filter_node.ros__parameters下）：
qos_overrides:
  /imu/data:
    subscription:
      reliability: reliable  # 强制使用RELIABLE匹配IMU发布者
```

**解决方案C：降低对传感器的依赖（临时）**：
```bash
# 禁用IMU输入，仅用wheel odometry（精度会降低！）
# 编辑 robot_localization_rtabmap.yaml：
imu0: /imu/data
imu0_config: [false, false, false,  # 全部设为false禁用
              false, false, false,
              false, false, false,
              false, false, false,
              false, false, false]
```

**验证修复**：
```bash
# 重启PC端remote_slam
ros2 launch bot_bringup remote_slam.launch.py

# 验证QoS已匹配
ros2 topic info /imu/data -v
# 现在应该看到 BEST_EFFORT → BEST_EFFORT 或 RELIABLE → RELIABLE

# 验证RTABMap不再报错
ros2 topic echo /rtabmap/info --once
# 应该看到实时地图数据

# 验证EKF输出
ros2 topic hz /odometry/filtered
# 应该看到 ~20Hz
```

**时间同步永久解决方案**：
```bash
# PC端作为NTP服务器（见问题7）
sudo chronyc tracking
# System time 应该小于1ms

# 树莓派端每天自动同步
sudo crontab -e
# 添加（每小时强制同步）：
0 * * * * /usr/bin/chronyc makestep >> /var/log/chrony_sync.log 2>&1
```

### 问题8：RViz无法显示地图，相机图像无法传输（DDS消息大小限制）

**症状**：
```
[planner_server] [WARN] global_costmap.global_costmap: Can't update static costmap layer, no map received
[rtabmap-6] [WARN] rtabmap_slam: Did not receive data since 5 seconds!
```

**PC端检查**：
```bash
# 1. 检查相机话题发布者数量
ros2 topic info /camera/color/image_raw -v | grep "Publisher count"
# 输出：Publisher count: 0  ← 虽然话题存在，但无数据传输

# 2. 验证EKF正常（排除其他问题）
ros2 topic hz /odometry/filtered
# 输出：average rate: 19.xxx  ← 正常

# 3. 检查树莓派相机节点运行状态
ssh lododo@192.168.2.120 "ps aux | grep astra_camera_node"
# 输出：进程存在，CPU 68%  ← 节点正常运行
```

**根本原因**：
- **Cyclone DDS默认`MaxMessageSize=65500B`（64KB）太小**
- **相机图像大小**：640x480 RGB = 921KB >> 64KB
- **WiFi单播模式必须配置`Peers`和足够大的`MaxMessageSize`**

**诊断步骤**：
```bash
# 1. 检查当前MaxMessageSize配置
cat ~/cyclonedds.xml | grep MaxMessageSize
# 如果显示 65500B 或 65KB，则确认问题

# 2. 检查树莓派相机节点日志（可能有发送失败错误）
ssh lododo@192.168.2.120 "journalctl -u astra_camera --since '10 minutes ago' | tail -50"
```

**解决方案：增加DDS消息大小到16MB**

**PC端配置**（`~/cyclonedds.xml`）：
```bash
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <!-- ⚠️ CRITICAL: 4MB是WiFi环境的实测最大稳定值 -->
      <MaxMessageSize>4MB</MaxMessageSize>
      <!-- MTU对齐的分片大小，避免IP层分片 -->
      <FragmentSize>1200B</FragmentSize>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="192.168.2.120"/>  <!-- 替换为树莓派实际IP -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      <SPDPInterval>1s</SPDPInterval>
    </Discovery>
    <Internal>
      <!-- 接收和发送缓冲区都要配置 -->
      <SocketReceiveBufferSize min="10MB"/>
      <SocketSendBufferSize min="10MB"/>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
```

**树莓派端配置**（`~/cyclonedds.xml`）：
```bash
ssh lododo@192.168.2.120
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <!-- ⚠️ CRITICAL: 4MB是WiFi环境的实测最大稳定值 -->
      <MaxMessageSize>4MB</MaxMessageSize>
      <!-- MTU对齐的分片大小，避免IP层分片 -->
      <FragmentSize>1200B</FragmentSize>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="192.168.2.169"/>  <!-- 替换为PC实际IP -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      <SPDPInterval>1s</SPDPInterval>
    </Discovery>
    <Internal>
      <!-- 接收和发送缓冲区都要配置 -->
      <SocketReceiveBufferSize min="10MB"/>
      <SocketSendBufferSize min="10MB"/>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
exit  # 退出SSH
```

**重启流程**：
```bash
# 1. 树莓派端：Ctrl+C停止hardware_bringup，然后重新启动
ssh lododo@192.168.2.120
cd ~/lododo_bot
source install/setup.bash
ros2 launch bot_hardware hardware_bringup.launch.py \
  enable_servo:=true \
  enable_imu:=true \
  enable_camera:=true \
  publish_static_tf:=true

# 2. PC端：Ctrl+C停止remote_slam，重新启动
cd ~/workDisk/lododo_bot
source install/setup.bash
ros2 launch bot_bringup remote_slam.launch.py use_rviz:=true

# 3. 验证修复（等待10秒后执行）
ros2 topic info /camera/color/image_raw -v | grep "Publisher count"
# 应该显示：Publisher count: 1

ros2 topic hz /camera/color/image_raw
# 应该显示：average rate: 5.xxx（相机帧率）

ros2 topic echo /rtabmap/info --once
# 应该能接收到RTABMap地图信息
```

**验证地图发布**：
```bash
# 检查RTABMap是否开始发布地图
ros2 topic list | grep map
# 应该看到：/map, /rtabmap/grid_map

ros2 topic hz /map
# 应该显示低频率（1-2Hz），这是正常的

# RViz中检查
# 1. Fixed Frame 应该是 "map"
# 2. 添加 Map display，Topic 选择 /map
# 3. 应该能看到逐渐构建的占用栅格地图
```

**关键配置参数说明**：
- `MaxMessageSize: 4MB` - **WiFi环境实测最大稳定值**（16MB会导致retcode -58错误）
- `FragmentSize: 1200B` - 保守的MTU大小，适配WiFi环境
- `SocketSendBufferSize: 10MB` - **必须配置**，否则大消息发送失败
- `AllowMulticast: false` - WiFi环境禁用多播，提高可靠性
- `Peers` - 单播模式必须显式配置对等节点IP
- `127.0.0.1` - 本地loopback，支持同机器节点通信

**错误码说明**：
- `retcode -58`: UDP发送缓冲区不足或消息太大，需降低MaxMessageSize或增加SocketSendBufferSize
- `retcode -5`: 多播发送失败（WiFi常见），需禁用多播改用单播

**如果问题仍然存在**：
```bash
# 方案A：降低相机分辨率（强烈推荐！）
ssh lododo@192.168.2.120
ros2 param set /camera color_width 320
ros2 param set /camera color_height 240
# 减少数据量：921KB → 230KB（降低75%）

# 方案B：使用image_transport压缩传输
ros2 run image_transport republish compressed \
  --ros-args --remap in:=/camera/color/image_raw \
             --remap out:=/camera/color/compressed

# 方案C：切换到有线以太网（根本解决！）
# WiFi带宽限制和延迟是根本问题
```

**性能监控**：
```bash
# 监控网络流量
iftop -i wlan0  # WiFi接口

# 监控DDS传输性能
ros2 topic bw /camera/color/image_raw
# 应该显示约 4-5 MB/s（5fps x 921KB）

# 检查丢包率
ros2 topic hz /camera/color/image_raw
# 如果帧率不稳定（3-7Hz跳动），说明网络质量差
```

**最佳实践**：
1. **优先使用有线以太网**（WiFi即使配置正确也会有延迟和丢包）
2. **生产环境建议降低相机分辨率**（320x240足够导航使用）
3. **使用image_transport压缩**（可减少50%网络带宽）
4. **监控chrony时钟同步**（大图像传输时时间戳更关键）

### 问题9：树莓派相机发送失败 `retcode -58`（UDP缓冲区不足）

**症状**（树莓派端日志）：
```
[astra_camera_node-4] ddsi_udp_conn_write to udp/192.168.2.169:17923 failed with retcode -58
[astra_camera_node-4] ddsi_udp_conn_write to udp/192.168.2.169:17917 failed with retcode -58
```

**根本原因**：
- **错误码-58**：`ESHUTDOWN`或UDP发送缓冲区不足
- **MaxMessageSize=16MB过大**：WiFi网络无法稳定传输如此大的消息
- **缺少`SocketSendBufferSize`配置**：仅配置接收缓冲区不够

**诊断步骤**：
```bash
# 1. 检查系统发送缓冲区（树莓派）
ssh lododo@192.168.2.120
sysctl net.core.wmem_max net.core.wmem_default
# wmem_max 应该 >= 10485760 (10MB)
# 如果不够，执行：sudo sysctl -w net.core.wmem_max=10485760

# 2. 检查DDS配置是否包含发送缓冲区
cat ~/cyclonedds.xml | grep -i SocketSendBufferSize
# 如果没有此配置，会导致发送失败

# 3. 监控网络丢包率
ping -c 100 192.168.2.169 | tail -3
# packet loss 应该 <1%，否则WiFi质量太差
```

**解决方案**：
1. **降低MaxMessageSize到4MB**（WiFi环境实测最大稳定值）
2. **添加SocketSendBufferSize配置**（与接收缓冲区对称）
3. **减小FragmentSize到1200B**（更保守的MTU设置）

**已更新配置**（见问题8的PC端和树莓派端配置）：
```xml
<MaxMessageSize>4MB</MaxMessageSize>  <!-- 16MB → 4MB -->
<FragmentSize>1200B</FragmentSize>    <!-- 1400B → 1200B -->
<Internal>
  <SocketReceiveBufferSize min="10MB"/>
  <SocketSendBufferSize min="10MB"/>  <!-- ⚠️ 新增 -->
</Internal>
```

**重启验证**：
```bash
# 1. 树莓派：重启hardware_bringup（应用新配置）
# Ctrl+C停止，然后重启

# 2. 观察日志，retcode -58错误应该消失
ssh lododo@192.168.2.120
tail -f ~/lododo_bot/log/latest/hardware.log | grep -E "(retcode|ERROR)"

# 3. PC端验证数据传输
ros2 topic hz /camera/color/image_raw
# 应该看到稳定的5Hz

ros2 topic bw /camera/color/image_raw
# 应该显示约 4-5 MB/s
```

**如果仍有错误**：
```bash
# 终极方案：降低相机分辨率到320x240
ssh lododo@192.168.2.120
ros2 param set /camera color_width 320
ros2 param set /camera color_height 240
ros2 param set /camera depth_width 320
ros2 param set /camera depth_height 240

# 数据量对比：
# 640x480 RGB: 921 KB/frame
# 320x240 RGB: 230 KB/frame (降低75%)
```

**长期解决方案**：
- **使用有线以太网**（千兆以太网可稳定传输16MB消息）
- **修改相机launch参数**（永久降低分辨率）：
```yaml
# bot_hardware/config/camera_params.yaml
color_width: 320
color_height: 240
depth_width: 320
depth_height: 240
```

### 问题10：FastDDS vs Cyclone DDS 选择（WiFi环境推荐FastDDS）

**症状**：Cyclone DDS持续出现retcode -58错误，即使降低MaxMessageSize到1MB仍无法解决。

**对比分析**：

| 特性 | FastDDS (ROS2默认) | Cyclone DDS |
|-----|-------------------|-------------|
| **WiFi稳定性** | ✅ 自适应机制好 | ❌ 配置复杂易出错 |
| **大消息传输** | ✅ 自动分片优化 | ❌ 手动配MaxMessageSize易失败 |
| **节点发现** | ✅ 多播在局域网稳定 | ✅ 单播模式（需配置Peers） |
| **配置复杂度** | ✅ 无需配置文件 | ❌ 需XML配置文件 |
| **调试难度** | 中等 | 较高（retcode错误难排查） |
| **默认状态** | ✅ ROS2 Humble默认 | 需手动安装 |

**结论**：**WiFi环境推荐使用FastDDS**（ROS2默认），除非有特殊需求才使用Cyclone DDS。

**切换到FastDDS**：
```bash
# PC端配置
cat >> ~/.bashrc << 'EOF'

# 使用FastDDS (ROS2默认)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# FastDDS自动适应网络环境，无需配置文件
EOF

# 树莓派端配置（SSH连接后）
cat >> ~/.bashrc << 'EOF'

# 使用FastDDS (ROS2默认)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EOF

# 重新加载配置
source ~/.bashrc

# 验证
echo $RMW_IMPLEMENTATION
# 应该显示: rmw_fastrtps_cpp
```

**如果之前配置了Cyclone DDS**：
```bash
# 编辑 ~/.bashrc，注释掉Cyclone DDS配置
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # ← 注释掉
# export CYCLONEDDS_URI=file:///home/hurry/cyclonedds.xml  # ← 注释掉

# 或者直接删除这些行
sed -i '/CYCLONEDDS/d' ~/.bashrc
sed -i '/rmw_cyclonedds/d' ~/.bashrc

# 重新加载
source ~/.bashrc
```

**FastDDS默认行为**（无需配置）：
- 自动多播发现（局域网内）
- 自动处理大消息分片（无需手动设MaxMessageSize）
- 自适应网络质量（自动调整重传机制）
- Socket缓冲区自动使用系统配置（确保`net.core.rmem_max=10MB`）

**验证FastDDS工作**：
```bash
# 1. 重启两端节点（新终端，让.bashrc生效）

# 2. 检查是否还有retcode -58
# 树莓派日志应该干净，没有DDS发送错误

# 3. 验证数据传输
ros2 topic hz /camera/color/image_raw
# 应该显示稳定的频率

ros2 topic info /camera/color/image_raw -v | grep "Publisher count"
# 应该显示 Publisher count: 1

# 4. FastDDS不会打印MaxMessageSize警告
# Cyclone DDS常见的 "message too large" 错误应该消失
```

**FastDDS优势实例**：
- 640x480 RGB图像（921KB）会自动分片为约15个UDP包（每个64KB）
- WiFi丢包时自动重传，无需手动调整参数
- 多播在局域网内通常比单播更可靠（不需要配置Peers列表）

**何时使用Cyclone DDS**：
- ✅ 有线以太网环境（配置简单，性能好）
- ✅ 需要严格的QoS控制
- ❌ WiFi环境（配置复杂，容易出错）

## 2. ROS2 Humble安装（树莓派）

```bash
# 设置locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 添加ROS2 apt源
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Humble Base（无GUI工具）
sudo apt update
sudo apt install -y ros-humble-ros-base

# 安装开发工具
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# 初始化rosdep
sudo rosdep init
rosdep update

# 自动source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 工作空间同步

**方案A：Git克隆（推荐）**
```bash
# 在树莓派上克隆代码（仅克隆硬件相关包）
cd ~
git clone https://github.com/<your-repo>/lododo_bot.git
cd lododo_bot

# 只编译硬件驱动相关包
colcon build --packages-select \
  bot_hardware \
  bot_control \
  bot_description \
  astra_camera \
  astra_camera_msgs \
  --symlink-install

source install/setup.bash
```

**如果编译astra_camera时遇到依赖问题**：
```bash
# libglog在Ubuntu 22.04 arm64可能不可用，需要从源码编译
cd ~/
git clone https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_SHARED_LIBS=ON
make -j4
sudo make install
sudo ldconfig

# 如果libuvc-dev也找不到，同样源码编译
cd ~/
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j4
sudo make install
sudo ldconfig

# 回到工作空间重新编译
cd ~/lododo_bot
rosdep install --from-paths src --ignore-src -r -y

# 清理后重新编译astra_camera
rm -rf build/astra_camera install/astra_camera
colcon build --packages-select astra_camera astra_camera_msgs --symlink-install
```

**方案B：rsync同步（开发时使用）**
```bash
# 从PC推送代码到树莓派
rsync -avz --exclude='build' --exclude='install' --exclude='log' \
  ~/lododo_bot/ ubuntu@<树莓派IP>:~/lododo_bot/
```

### 4. 硬件驱动依赖安装

**树莓派上安装传感器驱动**：
```bash
# 基础依赖和ROS2包
sudo apt install -y \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-camera-info-manager \
  ros-humble-image-publisher \
  ros-humble-v4l2-camera \
  libusb-1.0-0-dev \
  libudev-dev \
  libuvc-dev \
  libglog-dev \
  python3-serial

# 1. 检查是否有pkg-config文件
pkg-config --list-all | grep glog

# 2. 如果没有，创建pkg-config文件
sudo mkdir -p /usr/lib/aarch64-linux-gnu/pkgconfig
sudo tee /usr/lib/aarch64-linux-gnu/pkgconfig/libglog.pc > /dev/null << 'EOF'
prefix=/usr
exec_prefix=${prefix}
libdir=/lib/aarch64-linux-gnu
includedir=${prefix}/include

Name: libglog
Description: Google Logging Library
Version: 0.6.0
Libs: -L${libdir} -lglog
Cflags: -I${includedir}
EOF

# 3. 更新pkg-config环境变量
export PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
echo 'export PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc

# 4. 验证能找到
pkg-config --modversion libglog

# 5. 重新编译
cd ~/lododo_bot
rm -rf build/astra_camera
colcon build --packages-select astra_camera astra_camera_msgs --symlink-install

# 1. 手动删除glog 0.6.0的文件
sudo rm -rf /usr/include/glog
sudo rm -f /usr/lib/libglog*
sudo rm -f /usr/lib/pkgconfig/libglog.pc
sudo rm -f /usr/lib/cmake/glog
sudo ldconfig

cd ~/
rm -rf glog

# 2. 安装0.4.0版本
git clone https://github.com/google/glog.git
cd glog
git checkout v0.4.0
mkdir build && cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install
sudo ldconfig

# 3. 创建pkg-config文件（如果没有自动生成）
sudo tee /usr/lib/aarch64-linux-gnu/pkgconfig/libglog.pc > /dev/null << 'EOF'
prefix=/usr
exec_prefix=${prefix}
libdir=/usr/lib/aarch64-linux-gnu
includedir=${prefix}/include

Name: libglog
Description: Google Logging Library
Version: 0.4.0
Libs: -L${libdir} -lglog
Cflags: -I${includedir}
EOF

export PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH

# 4. 验证
pkg-config --modversion libglog

# 5. 重新编译
cd ~/lododo_bot
rm -rf build/astra_camera* install/astra_camera*
colcon build --packages-select astra_camera astra_camera_msgs --symlink-install

# 安装nlohmann-json库
sudo apt install -y nlohmann-json3-dev

cd ~/lododo_bot
rm -rf build/astra_camera



# 使用Debug模式编译（快3-5倍）
colcon build --packages-select astra_camera astra_camera_msgs \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  --parallel-workers 1

# 检查当前内存使用
free -h

# 增加SWAP到4GB（如果小于2GB）
sudo swapoff /swapfile
sudo dd if=/dev/zero of=/swapfile bs=1M count=4096
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
free -h  # 验证SWAP已增加

# 编译完成后，单独编译相机（使用单线程）
colcon build --packages-select \
  astra_camera_msgs \
  astra_camera \
  --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release


# 回到工作空间重新编译
cd ~/lododo_bot
rm -rf build/astra_camera install/astra_camera
colcon build --packages-select astra_camera_msgs astra_camera \
  --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 清理之前的失败编译
rm -rf build/astra_camera install/astra_camera log/latest*

# 限制编译并行度和内存
colcon build --packages-select astra_camera \
  --symlink-install \
  --parallel-workers 1 \
  --executor sequential \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF


# Astra相机驱动需要从源码编译（已包含在bot_hardware包中）
# 无需额外安装，bot_hardware包中已经包含了astra_camera驱动

# 使用rosdep自动安装所有依赖（推荐）
cd ~/lododo_bot
rosdep install --from-paths src --ignore-src -r -y

# 串口权限（重要！）
sudo usermod -a -G dialout $USER
# 注销重新登录生效或执行：newgrp dialout

步骤 1：增加系统 socket 缓冲区（关键！）
# 临时修改（立即生效）
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.wmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760
# 永久修改（重启后依然有效）
echo "# Increase socket buffer size for ROS2 DDS" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_max = 10485760" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max = 10485760" | sudo tee -a /etc/sysctl.conf  
echo "net.core.rmem_default = 10485760" | sudo tee -a /etc/sysctl.conf
# 验证
sysctl net.core.rmem_max  # 应该显示 10485760

# 检查是否有 YbImuLib 源码（应该在 bot_hardware 包中）
find ~/lododo_bot/src -name "YbImuLib*"

# 如果找到 setup.py，安装它
cd ~/lododo_bot/src/bot_hardware/bot_hardware/YbImuLib  # 根据实际路径调整
pip3 install .

sudo pip3 install pyserial
sudo pip3 install smbus2

# 安装 tf_transformations（omni_hardware_node 依赖）
pip3 install transforms3d
sudo apt install -y ros-humble-tf-transformations python3-transforms3d

# 配置 Astra 相机 USB 权限
sudo usermod -a -G video $USER
sudo usermod -a -G plugdev $USER

# 创建 Orbbec 相机 udev 规则
sudo tee /etc/udev/rules.d/56-orbbec-usb.rules > /dev/null << 'EOF'
# Orbbec Astra cameras
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
EOF

# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger
# 3. 检查相机是否被识别
lsusb | grep -i orbbec
# 或者如果有 .whl 文件
# pip3 install YbImuLib-*.whl

# 1. 卸载系统的旧版 OpenNI2
sudo apt remove -y libopenni2-0 libopenni2-dev
sudo apt autoremove -y

# 2. 验证系统库已删除
ls -l /lib/aarch64-linux-gnu/libOpenNI2* 2>/dev/null
# 应该没有输出

# 1. 安装缺失的依赖
sudo apt install -y \
  ros-humble-image-geometry \
  ros-humble-camera-info-manager \
  ros-humble-image-transport

# 2. 使用 rosdep 自动安装 astra_camera 的所有依赖
cd ~/lododo_bot
rosdep install --from-paths src/ros2_astra_camera --ignore-src -r -y --rosdistro humble


# 3. 设置环境变量指向项目中的 OpenNI2
export LD_LIBRARY_PATH=~/lododo_bot/src/ros2_astra_camera/astra_camera/openni2_redist/arm64:$LD_LIBRARY_PATH
export OPENNI2_REDIST=~/lododo_bot/src/ros2_astra_camera/astra_camera/openni2_redist/arm64

# 4. 验证 astra_camera_node 现在使用项目中的库
ldd ~/lododo_bot/install/astra_camera/lib/astra_camera/astra_camera_node | grep OpenNI2
# 应该显示项目路径或 "not found"（会在运行时通过 LD_LIBRARY_PATH 找到）

# 5. 永久设置环境变量
echo "export LD_LIBRARY_PATH=~/lododo_bot/src/ros2_astra_camera/astra_camera/openni2_redist/arm64:\$LD_LIBRARY_PATH" >> ~/.bashrc
echo "export OPENNI2_REDIST=~/lododo_bot/src/ros2_astra_camera/astra_camera/openni2_redist/arm64" >> ~/.bashrc

# 6. 使设置生效
source ~/.bashrc

# 7. 重新编译 astra_camera（让它重新链接）
cd ~/lododo_bot
rm -rf build/astra_camera install/astra_camera
colcon build --packages-select astra_camera \
  --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 8. 验证链接
source install/setup.bash
ldd install/astra_camera/lib/astra_camera/astra_camera_node | grep OpenNI2
# 现在应该指向项目中的 libOpenNI2.so

# 9. 重新启动
cd src/bot_bringup/scripts
. ~/lododo_bot/src/bot_bringup/scripts/start_robot_hardware.sh



```


### 5. 网络配置（关键！）

#### 5.1 ROS2 DDS通信配置

**⚠️ WiFi环境必须配置，否则节点无法发现！**

**PC端配置**（`~/.bashrc`添加）：
```bash
# ROS2网络配置
export ROS_DOMAIN_ID=42  # ⚠️ 必须与树莓派一致

# WiFi环境：使用Cyclone DDS（比默认FastDDS更稳定）
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Cyclone DDS配置文件路径（注意：PC用户名是hurry）
export CYCLONEDDS_URI=file:///home/hurry/cyclonedds.xml
```

**树莓派配置**（`~/.bashrc`添加）：
```bash
# ROS2网络配置（与PC相同）
export ROS_DOMAIN_ID=42  # ⚠️ 必须与PC一致

# WiFi环境：使用Cyclone DDS
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Cyclone DDS配置文件路径（注意：树莓派用户名是ubuntu）
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
```

**让配置立即生效**：
```bash
# PC端执行
source ~/.bashrc

# 树莓派执行（SSH连接后）
source ~/.bashrc
```

#### 5.2 Cyclone DDS配置文件

**⚠️ WiFi必须创建此文件，提高稳定性和发现速度！**

**PC端**：创建 `/home/hurry/cyclonedds.xml`
```bash
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true"/>
      </Interfaces>
      <!-- 禁用多播，使用单播 - WiFi环境推荐 -->
      <AllowMulticast>false</AllowMulticast>
      <!-- 增加最大消息大小到16MB，支持相机图像传输 -->
      <MaxMessageSize>16MB</MaxMessageSize>
      <!-- 启用数据分片，支持大消息 -->
      <FragmentSize>1400B</FragmentSize>
    </General>
    <Discovery>
      <!-- CRITICAL: 配置对等点 - 本地节点 + 树莓派IP -->
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="192.168.2.120"/>  <!-- 替换为实际树莓派IP -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      <SPDPInterval>1s</SPDPInterval>
    </Discovery>
    <Internal>
      <!-- 增加socket接收缓冲区 -->
      <SocketReceiveBufferSize min="10MB"/>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
```
# PC端也需要增加socket缓冲区！在PC上执行
# 临时修改（立即生效）
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.wmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760

# 永久修改（重启后依然有效）
echo "# Increase socket buffer size for ROS2 DDS" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_max = 10485760" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max = 10485760" | sudo tee -a /etc/sysctl.conf  
echo "net.core.rmem_default = 10485760" | sudo tee -a /etc/sysctl.conf

# 验证
sysctl net.core.rmem_max  # 应该显示 10485760

**树莓派**：创建 `/home/lododo/cyclonedds.xml`
```bash
# 在树莓派SSH连接后执行
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true"/>
      </Interfaces>
      <!-- 禁用多播，使用单播 - WiFi环境推荐 -->
      <AllowMulticast>false</AllowMulticast>
      <!-- 增加最大消息大小到16MB，支持相机图像传输 -->
      <MaxMessageSize>16MB</MaxMessageSize>
      <!-- 启用数据分片，支持大消息 -->
      <FragmentSize>1400B</FragmentSize>
    </General>
    <Discovery>
      <!-- CRITICAL: 配置对等点 - 本地节点 + PC IP -->
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="192.168.2.169"/>  <!-- 替换为实际PC IP -->
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      <SPDPInterval>1s</SPDPInterval>
    </Discovery>
    <Internal>
      <!-- 增加socket接收缓冲区 -->
      <SocketReceiveBufferSize min="10MB"/>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
```

#### 5.3 安装Cyclone DDS（如果未安装）

**PC和树莓派都需要执行**：
```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

#### 5.4 防火墙配置

**PC和树莓派都需要执行**（如果启用了防火墙）：
```bash
# 检查防火墙状态
sudo ufw status

# 如果启用，开放ROS2端口（7400-7500为DDS端口）
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp

# 或者完全关闭防火墙（仅测试环境）
sudo ufw disable
```

#### 5.5 时钟同步配置（关键！）

**⚠️ WiFi延迟大，时钟同步更重要！**

**PC端配置**（编辑 `/etc/chrony/chrony.conf`）：
```bash
# 在PC上执行
sudo apt install -y chrony

# 2. 验证配置文件存在
ls -l /etc/chrony/chrony.conf

# 3. 添加配置
sudo tee -a /etc/chrony/chrony.conf > /dev/null << 'EOF'

# 允许局域网设备同步时钟
allow 192.168.2.0/16

# 作为本地时钟服务器（备用）
local stratum 10
EOF

# 4. 重启 chrony 服务
sudo systemctl restart chrony
sudo systemctl enable chrony

# 5. 验证状态
sudo systemctl status chrony
chronyc tracking
```

**树莓派配置**（编辑 `/etc/chrony/chrony.conf`）：
```bash
# 在树莓派上执行（替换<PC的IP>为实际IP，如192.168.1.100）
echo "server 192.168.2.169 iburst prefer" | sudo tee -a /etc/chrony/chrony.conf

# 重启chrony服务
sudo systemctl restart chrony

# 强制立即同步
sudo chronyc -a makestep
```

#### 5.6 验证网络配置

**1. 检查ROS2环境变量**：
```bash
# PC和树莓派都执行
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"  # 应该都是42
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"  # 应该都是rmw_cyclonedds_cpp
echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"  # 应该指向各自的配置文件
```

**2. 检查网络连通性**：
```bash
# 在PC上ping树莓派
ping <树莓派IP> -c 4

# 在树莓派上ping PC
ping <PC的IP> -c 4
```

**3. 检查时钟同步**：
```bash
# 在树莓派上执行
chronyc tracking
# 输出应该显示 "Reference ID" 为PC的IP
# "System time" 偏差应该小于1ms
```

**4. 测试ROS2通信**：
```bash
# 在树莓派上启动测试发布者
ros2 topic pub /test std_msgs/msg/String "data: 'hello from raspberry pi'" &

# 在PC上监听（应该能收到消息）
ros2 topic echo /test
```

### 6. 启动脚本和Launch文件

#### 6.1 树莓派端（硬件层）

**启动脚本**（`~/start_robot_hardware.sh`）：
```bash
#!/bin/bash
set -e

cd ~/lododo_bot
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动硬件驱动节点
ros2 launch bot_hardware hardware_bringup.launch.py \
  enable_servo:=true \
  enable_imu:=true \
  enable_camera:=true \
  publish_static_tf:=true
```

#### 6.2 PC端（计算层）

PC端提供3个专用的launch文件用于分布式部署：

**1. `remote_navigation.launch.py` - 定位导航模式**
```bash
# 标准导航模式（需要已有地图）
ros2 launch bot_bringup remote_navigation.launch.py map_name:=office_floor1

# 启用RViz可视化
ros2 launch bot_bringup remote_navigation.launch.py \
  map_name:=office_floor1 \
  use_rviz:=true
```

**功能**：
- EKF传感器融合
- RTABMap定位（使用已有地图）
- Nav2自主导航
- RViz可视化（可选）

**2. `remote_slam.launch.py` - SLAM建图模式**
```bash
# 标准SLAM建图
ros2 launch bot_bringup remote_slam.launch.py

# SLAM + 启用Nav2（建图期间可导航）
ros2 launch bot_bringup remote_slam.launch.py \
  enable_nav:=true \
  use_rviz:=true
```

**功能**：
- EKF传感器融合
- RTABMap SLAM建图
- Nav2导航（可选）
- RViz可视化（推荐）

**3. `remote_web_full.launch.py` - 完整Web控制环境**
```bash
# SLAM模式 + Web控制
ros2 launch bot_bringup remote_web_full.launch.py slam:=true

# 定位模式 + Web控制
ros2 launch bot_bringup remote_web_full.launch.py \
  slam:=false \
  map_name:=office_floor1 \
  use_rviz:=true
```

**功能**：
- 包含上述所有功能
- MissionPlanner任务调度
- CommandAdapter统一命令接口
- rosbridge_server Web连接桥（端口9090）

**Web前端访问**：
```bash
# 1. 启动Web服务器（另一个终端）
cd ~/lododo_bot/src/bot_teleop
bash scripts/start_web_server.sh

# 2. 浏览器访问
http://<PC的IP>:8000

# 3. WebSocket连接
ws://<PC的IP>:9090
```

## 启动流程

### 标准启动顺序

**1. 树莓派端（先启动）**：
```bash
ssh ubuntu@<树莓派IP>
cd ~/lododo_bot
source install/setup.bash

# 启动硬件层
ros2 launch bot_hardware hardware_bringup.launch.py \
  enable_servo:=true \
  enable_imu:=true \
  enable_camera:=true \
  publish_static_tf:=true
```

**2. PC端（等待硬件层完全启动后，约5秒）**：

```bash
cd ~/lododo_bot
source install/setup.bash

# 选择以下之一：

# 方案A: 定位导航模式
ros2 launch bot_bringup remote_navigation.launch.py \
  map_name:=office_floor1 \
  use_rviz:=true

# 方案B: SLAM建图模式
ros2 launch bot_bringup remote_slam.launch.py \
  use_rviz:=true

# 方案C: 完整Web控制环境
ros2 launch bot_bringup remote_web_full.launch.py \
  slam:=false \
  map_name:=office_floor1
```

### 验证通信

**在PC上检查**：
```bash
# 检查节点列表（应该看到树莓派的节点）
ros2 node list

# 检查话题列表
ros2 topic list

# 测试摄像头数据
ros2 topic hz /camera/color/image_raw

# 测试IMU数据
ros2 topic hz /imu/data

# 测试轮式里程计
ros2 topic echo /wheel/odom --once
```

## 性能优化建议

### 树莓派优化

1. **禁用不必要服务**：
```bash
sudo systemctl disable bluetooth
sudo systemctl disable cups
```

2. **增加swap（可选）**：
```bash
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

3. **CPU频率固定（避免降频）**：
```bash
# 添加到/boot/firmware/config.txt
arm_freq=1800
over_voltage=6
```

### 网络优化

1. **使用有线以太网**（强烈推荐）：
   - WiFi延迟：10-50ms
   - 以太网延迟：<1ms

2. **相机数据压缩**（如果必须用WiFi）：
```bash
# 降低分辨率或帧率
ros2 param set /astra_camera color_fps 15
```

3. **QoS配置**：
   - 大数据（图像）：BEST_EFFORT
   - 控制指令：RELIABLE

## 故障排查

### 问题0：树莓派编译astra_camera时卡死

**症状**：编译停在 `[astra_camera:build 42%]` 长时间无响应

**原因**：内存耗尽（树莓派4GB内存不足以并行编译C++代码）

**解决**：
```bash
# 1. 强制停止编译（Ctrl+C 或新SSH会话执行）
pkill -9 colcon

# 2. 增加SWAP到4GB
sudo swapoff -a
sudo dd if=/dev/zero of=/swapfile bs=1M count=4096
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
free -h  # 验证SWAP已增加

# 3. 使用单线程编译
cd ~/lododo_bot
colcon build --packages-select astra_camera_msgs astra_camera \
  --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 问题0.1：ARM64架构找不到OpenNI2库

**症状**：
```
CMake Error: can't find 'openni2_redist/arm64/libOpenNI2.so'
```

**原因**：astra_camera预编译的OpenNI2只有x86_64版本，缺少ARM64版本

**解决（推荐使用预编译包）**：
```bash
# 方案A：下载Orbbec官方预编译包（推荐，1-2分钟完成）
cd ~/
wget https://github.com/orbbec/OpenNI_SDK/releases/download/v2.3.0.86-beat6/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip
sudo apt install -y unzip unrar

# 解压两层压缩包
unzip OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip
unrar x 066797_OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d.rar

# 查找并复制库文件
mkdir -p ~/lododo_bot/src/ros2_astra_camera/astra_camera/openni2_redist/arm64
find . -name "libOpenNI2.so"  # 找到实际位置
cp -r OpenNI-Linux-Arm64-2.3.0/* ~/lododo_bot/src/ros2_astra_camera/astra_camera/openni2_redist/arm64/

# 方案B：从源码编译（见上方"编译完成后，单独编译相机"章节的方案B）
```

**注意**：优先使用方案A预编译包，除非有特殊需求才从源码编译！

### 问题0.2：编译OpenNI2时报"Can't determine host platform"

**症状**：
```
ThirdParty/PSCommon/BuildSystem/CommonDefs.mak:22: *** Can't determine host platform.
```

**原因**：使用了错误的 OpenNI2 仓库（occipital 版本不支持 ARM64）

**解决**：不要从源码编译，直接使用 Orbbec 官方预编译包
```bash
# 删除错误的尝试
rm -rf ~/OpenNI2

# 使用预编译包（见问题0.1的方案A）
wget https://github.com/orbbec/OpenNI_SDK/releases/download/v2.3.0.86-beat6/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip
sudo apt install -y unzip unrar
unzip OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip
unrar x 066797_OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d.rar
```

### 问题1：节点互相发现不了

**检查**：
```bash
# 检查ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # 两边应该相同

# 检查防火墙
sudo ufw status  # 应该关闭或开放7400-7500端口

# 检查网络
ping <对方IP>
```

### 问题2：图像传输卡顿

**解决**：
```bash
# 方案1：降低分辨率
ros2 param set /astra_camera color_width 320
ros2 param set /astra_camera color_height 240

# 方案2：使用image_transport压缩
ros2 run image_transport republish compressed \
  --ros-args --remap in:=/camera/color/image_raw \
             --remap out:=/camera/color/image_compressed
```

### 问题3：时钟不同步（TF报错）

**解决**：
```bash
# 检查时钟差异
ssh ubuntu@<树莓派IP> date && date

# 强制同步
sudo chronyc -a makestep
```

### 问题4：缺少 tf_transformations 模块

**症状**：
```
[omni_hardware_node] ERROR: No module named 'tf_transformations'
```

**原因**：缺少 Python 的 tf_transformations 库

**解决**：
```bash
# 方案A：安装 ROS2 版本（推荐）
sudo apt install -y ros-humble-tf-transformations python3-transforms3d

# 方案B：使用 pip 安装
pip3 install transforms3d

# 验证安装
python3 -c "import transforms3d; print('OK')"
```

### 问题5：Astra 相机无法连接

**症状**：
```
[astra_camera_node] INFO: Waiting for device connection...
```

**原因**：USB 设备权限不足或相机未正确识别

**解决**：
```bash
# 1. 检查 USB 设备
lsusb | grep -i orbbec
# 应该看到类似：Bus 001 Device 004: ID 2bc5:xxxx

# 2. 配置用户组权限
sudo usermod -a -G video $USER
sudo usermod -a -G plugdev $USER

# 3. 创建 udev 规则（永久生效）
sudo tee /etc/udev/rules.d/56-orbbec-usb.rules > /dev/null << 'EOF'
# Orbbec Astra cameras
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
EOF

# 4. 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 5. 注销重新登录或重启
sudo reboot

# 6. 如果还不行，检查 USB 供电
# 树莓派 USB 端口供电有限，建议使用有源 USB Hub
```

**验证**：
```bash
# 检查设备节点权限
ls -l /dev/bus/usb/*/* | grep 2bc5

# 应该看到权限为 crw-rw-rw-（0666）
```

### 问题6：ROS2 命令行工具缺失（ros2 topic/launch 不可用）

**症状**：
```bash
ros2 topic list
# 报错：invalid choice: 'topic' (choose from 'daemon', 'extension_points', 'extensions', 'node', 'param', 'service')
```

**原因**：执行 `sudo apt autoremove` 时误删了 ROS2 CLI 工具包（通常发生在删除系统库后）

**诊断**：
```bash
# 检查安装的包数量
dpkg -l | grep ros-humble | wc -l  # 应该有 300+ 个包

# 检查是否缺少 CLI 工具
dpkg -l | grep -E "ros-humble-ros2(cli|topic|launch|pkg)"
# 如果只显示 topic-tools 而没有 ros2cli、ros2topic、ros2launch，则确认缺失
```

**解决**：
```bash
# 重新安装 ROS2 命令行工具
sudo apt update
sudo apt install -y \
  ros-humble-ros2cli \
  ros-humble-ros2topic \
  ros-humble-ros2launch \
  ros-humble-ros2pkg \
  ros-humble-ros2run \
  ros-humble-ros2node \
  ros-humble-ros2param \
  ros-humble-ros2service

# 验证修复
source /opt/ros/humble/setup.bash
ros2 --help
ros2 topic list
```

**预防措施**：
```bash
# 删除系统包前，先查看会删除什么
sudo apt autoremove --dry-run

# 如果看到会删除 ros-humble-* 包，取消操作并手动删除
sudo apt remove <specific-package>  # 不要加 autoremove
```

## Launch文件总结

### 分布式部署Launch文件

**树莓派端（Hardware Layer）**：
- `hardware_bringup.launch.py` - 硬件驱动启动（舵机+IMU+相机）

**PC端（Computation Layer）**：
1. **`remote_navigation.launch.py`** - 定位导航模式
   - 组件：EKF Fusion + RTABMap Localization + Nav2
   - 用途：使用已有地图进行自主导航
   - 参数：`map_name`（必填）、`use_rviz`

2. **`remote_slam.launch.py`** - SLAM建图模式
   - 组件：EKF Fusion + RTABMap SLAM + Nav2（可选）
   - 用途：创建新地图或更新现有地图
   - 参数：`enable_nav`、`use_rviz`

3. **`remote_web_full.launch.py`** - 完整Web控制环境
   - 组件：上述所有功能 + MissionPlanner + CommandAdapter + rosbridge_server
   - 用途：Web前端控制、语音控制、任务调度
   - 参数：`slam`、`map_name`、`use_rviz`、`rosbridge_port`

### 对比真机Launch文件

| Launch文件 | 部署方式 | 包含硬件层 | 适用场景 |
|-----------|---------|-----------|---------|
| `real_robot_bringup.launch.py` | 单机 | ✅ | 树莓派单机运行所有节点 |
| `real_robot_navigation.launch.py` | 单机 | ✅ | 单机定位导航 |
| `real_robot_slam.launch.py` | 单机 | ✅ | 单机SLAM建图 |
| `real_robot_web_full.launch.py` | 单机 | ✅ | 单机Web控制 |
| `remote_navigation.launch.py` | **分布式** | ❌ | PC端定位导航 |
| `remote_slam.launch.py` | **分布式** | ❌ | PC端SLAM建图 |
| `remote_web_full.launch.py` | **分布式** | ❌ | PC端Web控制 |

### Launch文件引用关系

```
remote_web_full.launch.py
  ├─ [SLAM模式] remote_slam.launch.py
  │    └─ rtabmap_real.launch.py (SLAM)
  │
  ├─ [定位模式] remote_navigation.launch.py
  │    └─ rtabmap_real.launch.py (Localization)
  │
  ├─ mission_planner (Node)
  ├─ cmd_adapter.launch.py
  └─ rosbridge_server (Node)
```

## 内存占用监控

**树莓派上实时监控**：
```bash
# 持续监控
watch -n 1 'free -h && echo && ps aux --sort=-%mem | head -10'
```

**预期内存占用**：
- **树莓派端**：
  - 系统空闲：~300MB
  - 硬件层启动后：~600MB
  - 剩余内存：3.4GB（足够！）
  
- **PC端**：
  - EKF + RTABMap + Nav2：~2-3GB
  - 加上MissionPlanner + Web：~3-4GB
  - 推荐配置：8GB+ RAM

## 下一步行动

1. ✅ 安装Ubuntu 22.04 Server arm64到树莓派
2. ✅ 安装ROS2 Humble（PC和树莓派）
3. ✅ 配置网络和时钟同步
4. ✅ 创建分布式部署launch文件
5. ⏳ 测试分布式通信
6. ⏳ 性能调优和网络优化

## 快速启动指南

### 首次部署检查清单

**网络配置**：
- [ ] ROS_DOMAIN_ID 在PC和树莓派一致（建议42）
- [ ] Cyclone DDS配置文件已创建
- [ ] Socket缓冲区已增加到10MB
- [ ] 防火墙已配置或关闭
- [ ] 时钟同步已配置（PC作为时钟服务器）

**软件安装**：
- [ ] 树莓派：ROS2 Humble Base + 硬件驱动依赖
- [ ] PC：ROS2 Humble Desktop + 导航栈 + Web依赖
- [ ] 代码已同步到两端（Git或rsync）
- [ ] 编译成功（树莓派仅编译硬件包）

**硬件连接**：
- [ ] IMU连接到 /dev/ttyUSB0，权限已配置
- [ ] Astra相机USB连接，udev规则已创建
- [ ] 舵机控制板连接（如果有）

### 标准启动命令

```bash
# === 树莓派端 ===
ssh ubuntu@<树莓派IP>
cd ~/lododo_bot
source install/setup.bash
ros2 launch bot_hardware hardware_bringup.launch.py \
  enable_servo:=true \
  enable_imu:=true \
  enable_camera:=true \
  publish_static_tf:=true

# === PC端（等待5秒后执行）===
cd ~/lododo_bot
source install/setup.bash

# 定位导航模式
ros2 launch bot_bringup remote_navigation.launch.py \
  map_name:=office_floor1 \
  use_rviz:=true

# 或者 SLAM建图模式
ros2 launch bot_bringup remote_slam.launch.py use_rviz:=true

# 或者 完整Web控制
ros2 launch bot_bringup remote_web_full.launch.py \
  slam:=false \
  map_name:=office_floor1
```

---

**关键提示**：
- ⚠️ 强烈推荐**有线以太网**，WiFi延迟会导致TF不同步
- ⚠️ 必须配置**时钟同步**（chrony），否则传感器数据时间戳错乱
- ⚠️ `ROS_DOMAIN_ID`两边必须一致
- ⚠️ PC和树莓派都需要增加socket缓冲区（10MB）
- ⚠️ 树莓派必须先启动，PC等待5秒后再启动

