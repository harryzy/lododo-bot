# 分布式部署架构 | Distributed Deployment Architecture

**更新日期**: 2026-01-26  
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

### 2. ROS2 Humble安装（树莓派）

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
      <!-- 自动选择网络接口（WiFi或以太网） -->
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <!-- WiFi环境：允许IP分片（大图像数据） -->
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500</MaxMessageSize>
    </General>
    <Internal>
      <!-- 增大接收缓冲区（处理WiFi数据包） -->
      <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
    </Internal>
    <Discovery>
      <!-- 加快节点发现速度 -->
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      <!-- WiFi环境：更短的心跳间隔 -->
      <SPDPInterval>1s</SPDPInterval>
    </Discovery>
    <Tracing>
      <!-- 调试时启用日志（生产环境可关闭） -->
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
```

**树莓派**：创建 `/home/ubuntu/cyclonedds.xml`
```bash
# 在树莓派SSH连接后执行（内容与PC相同）
cat > ~/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500</MaxMessageSize>
    </General>
    <Internal>
      <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
    </Internal>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      <SPDPInterval>1s</SPDPInterval>
    </Discovery>
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
sudo tee -a /etc/chrony/chrony.conf > /dev/null << 'EOF'

# 允许局域网设备同步时钟
allow 192.168.0.0/16

# 作为本地时钟服务器（备用）
local stratum 10
EOF

# 重启chrony服务
sudo systemctl restart chrony
```

**树莓派配置**（编辑 `/etc/chrony/chrony.conf`）：
```bash
# 在树莓派上执行（替换<PC的IP>为实际IP，如192.168.1.100）
echo "server <PC的IP> iburst prefer" | sudo tee -a /etc/chrony/chrony.conf

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

### 6. 启动脚本

**树莓派启动脚本**（`~/start_robot_hardware.sh`）：
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

**PC启动脚本**（`~/start_robot_navigation.sh`）：
```bash
#!/bin/bash
set -e

cd ~/lododo_bot
source install/setup.bash

# 启动导航和SLAM节点
ros2 launch bot_bringup remote_navigation.launch.py \
  slam:=true \
  use_rviz:=true
```

## 启动流程

### 标准启动顺序

1. **树莓派（先启动）**：
```bash
ssh ubuntu@<树莓派IP>
~/start_robot_hardware.sh
```

2. **PC（等待2秒后启动）**：
```bash
~/start_robot_navigation.sh
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

## 待创建的Launch文件

需要创建分布式部署专用的launch文件：

1. **`hardware_only.launch.py`**（树莓派专用）
   - 仅启动硬件驱动节点
   - 不启动EKF、SLAM等计算密集型节点

2. **`remote_navigation.launch.py`**（PC专用）
   - 启动所有非硬件节点
   - 假设硬件话题从网络接收

## 内存占用监控

**树莓派上实时监控**：
```bash
# 持续监控
watch -n 1 'free -h && echo && ps aux --sort=-%mem | head -10'
```

**预期内存占用**：
- 系统空闲：~300MB
- ROS2节点启动后：~600MB
- **剩余内存**：3.4GB（足够！）

## 下一步行动

1. ✅ 安装Ubuntu 22.04 Server arm64到树莓派
2. ⏳ 安装ROS2 Humble
3. ⏳ 配置网络和时钟同步
4. ⏳ 创建硬件专用launch文件
5. ⏳ 测试分布式通信
6. ⏳ 性能调优

---

**关键提示**：
- ⚠️ 必须使用**有线以太网**，WiFi延迟会导致TF不同步
- ⚠️ 必须配置**时钟同步**（chrony），否则传感器数据时间戳错乱
- ⚠️ `ROS_DOMAIN_ID`两边必须一致
