# 文件描述符限制问题修复 / File Descriptor Limit Fix

**日期 / Date**: 2026-02-02  
**严重程度 / Severity**: 🔴 Critical  
**影响范围 / Impact**: 树莓派部署 / Raspberry Pi Deployment

---

## 问题描述 / Problem Description

### 症状 / Symptoms

在树莓派上运行完整SLAM系统时，出现以下错误：

```
Failed to create publisher: rcl node's context is invalid, at ./src/rcl/node.c:428
```

**表现**：
- ROS2节点进程运行正常
- 无法执行新的`ros2`命令（topic list, node info等）
- 系统报"context is invalid"错误

### 诊断结果 / Diagnosis

```bash
$ ulimit -n
1024                    # 系统默认限制

$ lsof | wc -l
76185                   # 实际使用远超限制（超限73倍）
```

**根本原因**：文件描述符（File Descriptor）耗尽。

---

## 原因分析 / Root Cause Analysis

### ROS2 FastDDS的FD消耗 / FastDDS FD Consumption

每个ROS2节点的FD使用：

| 组件 | 数量 | FD/单位 | 小计 |
|------|------|---------|------|
| 话题发布者 | ~5 | 15-20 | 75-100 |
| 话题订阅者 | ~3 | 15-20 | 45-60 |
| DDS Discovery | 1 | 10-15 | 10-15 |
| 服务/动作 | ~2 | 5-10 | 10-20 |
| **单节点总计** | - | - | **140-195 FD** |

### 系统总消耗 / System Total

```
19个节点 × 150 FD = 2850 FD（保守估计）
FastDDS共享内存文件 × 2 = 200 FD
网络socket + 系统文件 = 300 FD
-------------------------------------------
总计：3350+ FD >> 默认限制1024
```

### 触发条件 / Trigger Conditions

- ✅ 树莓派单独运行：节点较少，未超限
- ❌ **PC端RViz连接后**：
  - RViz订阅多个话题 → 新增DDS连接
  - FastDDS Discovery广播增加
  - **瞬间超过1024限制**

---

## 解决方案 / Solution

### 永久修复 / Permanent Fix

在树莓派上执行以下命令：

```bash
# 1. 修改系统限制配置
sudo tee -a /etc/security/limits.conf << EOF
* soft nofile 65536
* hard nofile 65536
EOF

# 2. 重启系统生效
sudo reboot

# 3. 验证
ulimit -n  # 应该显示 65536
```

### 临时修复（当前会话）/ Temporary Fix

```bash
ulimit -n 65536
```

**注意**：仅对当前终端会话有效，重启后失效。

---

## 验证步骤 / Verification

### 1. 检查限制是否生效

```bash
ulimit -n
# 期望输出: 65536
```

### 2. 启动完整系统

```bash
cd ~/lododo_bot
./launch_remote_slam.sh
```

### 3. 验证ROS2命令正常

```bash
ros2 node list      # 应正常显示节点列表
ros2 topic list     # 应正常显示话题列表
ros2 topic hz /odometry/filtered  # 应能监听数据
```

### 4. 监控FD使用

```bash
# 检查各进程FD使用
for pid in $(pgrep -f "ros2|rtabmap|nav2"); do
    echo "PID $pid: $(ls /proc/$pid/fd 2>/dev/null | wc -l) FDs"
done

# 应该看到每个进程FD在50-200之间（正常范围）
```

---

## 预防措施 / Prevention

### 新树莓派部署检查清单 / New Deployment Checklist

在部署ROS2 SLAM系统前，确保：

- [ ] 检查`ulimit -n`是否≥65536
- [ ] 配置`/etc/security/limits.conf`
- [ ] 重启系统验证配置生效
- [ ] 测试完整系统（硬件+导航+RViz远程连接）

### 长期监控 / Long-term Monitoring

```bash
# 添加到crontab，每小时检查一次
0 * * * * lsof | wc -l >> ~/fd_usage.log
```

如果FD使用持续增长超过50,000，需排查是否有节点泄漏。

---

## 相关配置 / Related Configurations

### RTABMap同步队列优化

为应对PC远程连接的网络延迟，已增大RTABMap同步队列：

```python
# src/bot_bringup/launch/real_robot_bringup.launch.py
parameters=[
    {
        'queue_size': 50,              # 增大队列（默认30）
        'approx_sync_max_interval': 0.5,  # 允许0.5秒时间差
    }
]
```

### 系统资源限制建议

```bash
# /etc/security/limits.conf
* soft nofile 65536    # 文件描述符
* hard nofile 65536
* soft nproc 4096      # 进程数（可选）
* hard nproc 4096
```

---

## 故障排查 / Troubleshooting

### 问题：修改limits.conf后仍显示1024

**原因**：未重启或会话未重新登录

**解决**：
```bash
# 方法1：完全重启
sudo reboot

# 方法2：重新登录SSH会话
exit
ssh lododo@192.168.2.120
ulimit -n  # 验证
```

### 问题：某个节点FD异常高（>1000）

**诊断**：
```bash
PID=<异常进程PID>
ls -l /proc/$PID/fd | head -20  # 查看FD类型
```

**常见原因**：
- FastDDS共享内存泄漏 → 清理`/dev/shm/fastrtps_*`
- Socket未关闭 → 重启节点
- 相机驱动问题 → 检查USB设备

---

## 技术背景 / Technical Background

### 为什么ROS2需要这么多FD？

ROS2使用DDS（Data Distribution Service）作为通信中间件：

1. **每个话题连接**需要：
   - 2个共享内存文件（发布/订阅）
   - 4-6个UDP socket（Discovery + Data）
   - 2-4个pipe（内部同步）
   - **总计：8-12 FD**

2. **FastDDS Discovery**需要：
   - 广播socket（每个domain一个）
   - 多播socket（自动发现）
   - 持久化连接（peer-to-peer）

3. **跨机器通信**（PC + 树莓派）额外需要：
   - 每个远程订阅者 +20-30 FD
   - RViz通常订阅10+话题 → **+200-300 FD**

### Linux默认限制来源

- **历史原因**：早期Linux为单用户桌面系统设计
- **现代需求**：ROS2这类分布式系统需要更高限制
- **行业标准**：服务器通常配置为65536-1048576

---

## 参考资料 / References

- [ROS2 FastDDS Tuning Guide](https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/tcp/tcp.html)
- [Linux ulimit Documentation](https://man7.org/linux/man-pages/man2/setrlimit.2.html)
- [Issue Discussion](https://github.com/ros2/rmw_fastrtps/issues/456)

---

**修复者 / Fixed by**: GitHub Copilot  
**验证者 / Verified by**: Hurry  
**状态 / Status**: ✅ 已解决 / Resolved
