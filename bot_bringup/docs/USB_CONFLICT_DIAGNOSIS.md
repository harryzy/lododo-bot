# USB资源冲突诊断报告

**日期**: 2026-02-02  
**问题**: 舵机写入失败 + RTABMap无数据 + 相机不稳定  
**根本原因**: USB总线调度冲突（相机阻塞串口设备）

---

## 1. 问题症状

```
[omni_hardware_node] Servo 7 write_speed failed: result=-6, retry 1/3
[rtabmap_slam] Did not receive data since 5 seconds!
[robot_localization] /odometry/filtered 间歇性消失
[astra_camera] 帧率2-4Hz（预期15Hz）
```

**影响**: SLAM无法同步4路topic → `/map` 不发布 → 导航功能完全失效

---

## 2. USB硬件拓扑分析

### 当前连接结构

```
Bus 01 (xhci_hcd, 480Mbps)
  └─ Port 1: Hub Dev 2 (480Mbps)
      ├─ Port 1: Hub Dev 3 (480Mbps) ← 二级Hub
      │   ├─ Port 1: Dev 5 (相机 Video, 480Mbps) ← 高带宽设备
      │   └─ Port 2: Dev 7 (相机 Audio, 480Mbps)
      ├─ Port 3: Dev 4 (IMU ch341, 12Mbps) ← 低速串口
      └─ Port 4: Dev 6 (舵机 cdc_acm, 12Mbps) ← 低速串口
```

### 问题根源

1. **相机通过二级Hub连接**:
   - 增加USB协议转发开销
   - 共享一级Hub的带宽池

2. **USB调度策略**:
   - Linux USB驱动优先处理高带宽设备（相机）
   - 低速设备（舵机串口）写请求被延迟
   - 延迟超过ST3215舵机超时限制（~50ms）→ result=-6

3. **级联失效**:
   ```
   相机数据突发 → 舵机写入超时 → /wheel/odom丢失 
   → EKF失去输入 → /odometry/filtered中断 
   → RTABMap 4-topic同步失败 → /map停止发布
   ```

---

## 3. USB错误日志证据

```bash
[ 1992.775427] usb 1-1.1.1: device descriptor read/64, error -110
[ 2008.391679] usb 1-1.1.1: device descriptor read/64, error -110
[ 2013.763688] usb 1-1.1.1: device descriptor read/64, error -110
[ 2029.387704] usb 1-1.1.1: device descriptor read/64, error -110
```

**错误代码**: `-110 = -ETIMEDOUT`  
**设备**: `1-1.1.1` = 相机（Dev 5）  
**含义**: 相机无法在规定时间内响应USB设备描述符读取请求  
**根本原因**: USB总线饱和，相机自身无法及时处理USB协议请求

---

## 4. 解决方案

### 🔥 方案1: 降低相机帧率（已实施）

**修改文件**: `src/bot_hardware/launch/sensor_bringup.launch.py`

```python
# 修改前
'color_fps': '15',
'depth_fps': '15',

# 修改后
'color_fps': '10',  # 降低33%带宽
'depth_fps': '10',  # 降低33%带宽
```

**理论带宽减少**:
- Color MJPEG 640x480@15fps → @10fps: ~12MB/s → ~8MB/s
- Depth Raw 640x480@15fps → @10fps: ~9MB/s → ~6MB/s
- **总带宽**: ~21MB/s → ~14MB/s (减少33%)

**预期效果**:
- 舵机写入成功率提升
- /odometry/filtered 稳定发布
- RTABMap能够同步数据

**测试方法**:
```bash
# 同步修改文件到树莓派
scp src/bot_hardware/launch/sensor_bringup.launch.py \
    lododo@192.168.2.120:~/lododo_bot/src/bot_hardware/launch/

# 重启系统（已修改launch文件）
ssh lododo@192.168.2.120 'pkill -9 -f ros2 && ./launch_remote_slam.sh'

# 监控10分钟（观察是否再出现错误）
watch -n 2 'ros2 topic hz /wheel/odom /camera/color/image_raw'
watch -n 2 'dmesg | grep -i "usb.*error" | tail -5'
```

---

### 🔧 方案2: 物理重新连接USB设备（备选）

**目标**: 将舵机控制器移到独立的USB端口

```bash
# 当前拓扑（所有设备共享Bus 01）
lsusb -t

# 理想拓扑（分散到不同Bus）
Bus 01: 舵机 ttyACM0（保证低延迟）
Bus 02: 相机（高带宽独占）
Bus 03: IMU ttyUSB0（低速独立）
```

**操作步骤**:
1. 关闭系统
2. 将相机USB线从当前端口拔出
3. 插入树莓派的 **后侧USB 3.0蓝色端口**（Bus 02）
4. 确认舵机控制器仍在前侧USB 2.0端口（Bus 01）
5. 启动系统并检查 `lsusb -t`

**验证方法**:
```bash
lsusb -t | grep -A5 "Bus 02"  # 相机应该出现在这里
lsusb -t | grep -A5 "Bus 01"  # 舵机应该出现在这里
```

**风险**: USB 3.0控制器可能不支持UVC相机（需测试）

---

### ⚙️ 方案3: 调整USB串口优先级（实验性）

**修改串口latency_timer**（降低舵机写入延迟）:

```bash
# 当前值（默认16ms）
cat /sys/bus/usb-serial/devices/ttyACM0/latency_timer

# 修改为1ms（最高优先级）
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyACM0/latency_timer

# 持久化修改（添加到启动脚本）
echo 'echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyACM0/latency_timer' \
    | sudo tee -a /etc/rc.local
```

**注意**: 此方案效果有限，因为根本问题是USB总线调度，而非串口驱动配置。

---

### 🛡️ 方案4: 启用USB autosuspend例外（预防性）

**问题**: 某些USB设备可能被自动挂起导致唤醒延迟

```bash
# 禁用相机自动挂起
echo 'on' | sudo tee /sys/bus/usb/devices/1-1.1.1/power/control

# 禁用舵机控制器自动挂起
echo 'on' | sudo tee /sys/bus/usb/devices/1-1.4/power/control

# 持久化（添加到udev规则）
sudo tee /etc/udev/rules.d/99-usb-no-autosuspend.rules <<'EOF'
# 禁用Astra相机自动挂起
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0501", ATTR{power/control}="on"

# 禁用ST3215舵机控制器自动挂起（CDC ACM）
SUBSYSTEM=="usb", DRIVER=="cdc_acm", ATTR{power/control}="on"
EOF

sudo udevadm control --reload-rules
```

---

## 5. 测试流程

### 测试1: 验证帧率降低是否解决问题

```bash
# 1. 同步修改后的文件
cd ~/workDisk/lododo_bot
scp src/bot_hardware/launch/sensor_bringup.launch.py \
    lododo@192.168.2.120:~/lododo_bot/src/bot_hardware/launch/

# 2. SSH到树莓派重启系统
ssh lododo@192.168.2.120
pkill -9 -f ros2
./launch_remote_slam.sh

# 3. 监控关键指标（持续5分钟）
# 终端1: 查看topic频率
watch -n 2 'ros2 topic hz /wheel/odom /camera/color/image_raw /odometry/filtered'

# 终端2: 查看USB错误
watch -n 2 'dmesg | grep -i "usb.*error\|servo.*failed" | tail -10'

# 终端3: 查看RTABMap状态
ros2 topic echo /map --once  # 应该有数据

# 4. 记录结果
# ✅ 成功标志:
#    - /wheel/odom 稳定50Hz
#    - /camera/color/image_raw 稳定10Hz
#    - /odometry/filtered 稳定5Hz
#    - dmesg无新USB错误
#    - /map 有数据发布
#
# ❌ 失败标志:
#    - 仍然出现 "Servo write_speed failed"
#    - dmesg仍有 error -110
#    - /odometry/filtered 间歇性中断
```

### 测试2: 如果测试1失败，执行物理重连

```bash
# 1. 关闭系统
ssh lododo@192.168.2.120 'sudo shutdown -h now'

# 2. 物理操作（在树莓派上）:
#    - 拔出相机USB线
#    - 插入后侧蓝色USB 3.0端口
#    - 保持舵机/IMU在原端口

# 3. 启动并验证拓扑
ssh lododo@192.168.2.120
lsusb -t | grep -E "Bus 01|Bus 02|Bus 03" -A10

# 期望看到:
# Bus 02: 相机（Dev 5, Dev 7）
# Bus 01: 舵机（Dev 6）
# Bus 01: IMU（Dev 4）

# 4. 重复测试1的监控步骤
```

---

## 6. 成功标准

**系统应达到以下指标**:

| 指标 | 目标值 | 验证方法 |
|------|--------|----------|
| /wheel/odom | 50Hz 稳定 | `ros2 topic hz` |
| /camera/color/image_raw | 10Hz 稳定 | `ros2 topic hz` |
| /odometry/filtered | 5Hz 稳定 | `ros2 topic hz` |
| /map | 有数据 | `ros2 topic echo --once` |
| USB错误 | 无error -110 | `dmesg | grep "usb.*error"` |
| 舵机写入 | 无failed日志 | 查看omni_hardware_node日志 |
| 连续运行 | >30分钟无故障 | 长期监控 |

---

## 7. 长期优化建议

1. **考虑使用外接USB Hub**: 
   - 带独立供电的USB 3.0 Hub
   - 将相机连接到Hub（隔离电源干扰）

2. **升级硬件**:
   - 替换为CSI接口相机（不占用USB带宽）
   - 或使用USB-CAN转换器控制舵机（更高优先级）

3. **软件优化**:
   - 在astra_camera驱动中实现自适应帧率
   - 根据系统负载动态调整帧率

---

## 8. 参考文档

- USB Error Codes: https://www.kernel.org/doc/html/latest/driver-api/usb/error-codes.html
- Raspberry Pi USB Issues: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#usb-device-boot-mode
- ST3215 Protocol: (查阅舵机驱动源码)

---

**创建时间**: 2026-02-02 13:50  
**作者**: AI Agent  
**状态**: 方案1已实施，等待测试验证
