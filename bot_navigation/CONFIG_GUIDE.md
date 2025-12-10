# Navigation Configuration Guide
# 导航配置指南

## 📁 配置文件组织 / Configuration File Organization

```
bot_navigation/config/
├── robot_localization_sim.yaml      # ⚠️ 仿真专用 / SIMULATION ONLY
├── robot_localization_real.yaml     # ⚠️ 真机专用 / REAL ROBOT ONLY
├── robot_localization_imu.yaml      # 🗑️ 已废弃，保留作参考
├── nav2_params_imu.yaml             # ✅ 通用导航参数
└── slam_toolbox_imu_official.yaml   # ✅ 通用SLAM参数
```

---

## 🤖 仿真配置 vs 真机配置

### ⚙️ robot_localization_sim.yaml (仿真专用)

**适用场景：**
- Gazebo仿真环境
- 开发和调试阶段
- 算法验证

**配置策略：**
```yaml
# 轮式里程计：位置 + 线速度 + 角度 + 角速度 (全部启用)
odom0_config: [true, true, false, false, false, true,   # 启用 yaw
               true, true, false, false, false, true,   # 启用 vyaw
               false, false, false]

# IMU：只用姿态参考，不用角速度
imu0_config: [false, false, false, true, true, false,   # yaw 禁用
              false, false, false, false, false, false, # vyaw 禁用
              false, false, false]
```

**为什么这样配置？**
- ✅ Gazebo轮式里程计完美准确（无打滑、无噪声）
- ✅ Ground-truth数据，定位精度高
- ❌ Gazebo IMU角速度信号很弱（~10^-7量级）
- ❌ 不能代表真机实际情况

**优点：**
- 快速开发迭代
- 易于调试
- 性能可预测

**缺点：**
- 不能直接用于真机
- 需要维护两套配置

---

### ⚙️ robot_localization_real.yaml (真机专用)

**适用场景：**
- 实际机器人部署
- 现场测试
- 生产环境

**配置策略：**
```yaml
# 轮式里程计：只用位置 + 线速度 (禁用角度)
odom0_config: [true, true, false, false, false, false,  # 禁用 yaw
               true, true, false, false, false, false,  # 禁用 vyaw
               false, false, false]

# IMU：姿态 + 角速度 (核心传感器)
imu0_config: [false, false, false, true, true, true,    # 启用 yaw
              false, false, false, false, false, true,  # 启用 vyaw
              false, false, false]
```

**为什么这样配置？**

**三轮全向轮的特殊问题：**
```
旋转运动学：w = (v1 + v2 + v3) / (3 * R)

问题：
├── 三个轮子的打滑程度不同
├── 任何一个轮子打滑 → 角速度计算错误
├── 误差累积：每360°旋转可能偏差 20-50°
└── 长时间运行后yaw完全漂移

解决：
└── IMU陀螺仪直接测量实际旋转角速度
    └── 不受打滑影响 ✅
```

**实际场景示例：**

| 场景 | 轮式里程计表现 | IMU陀螺仪表现 |
|------|--------------|--------------|
| **原地旋转360°** | 误差20-50° ❌ | 误差<5° ✅ |
| **地毯/瓷砖过渡** | yaw突然跳变 ❌ | 不受影响 ✅ |
| **轮子悬空** | 继续错误累加 ❌ | 测量实际旋转 ✅ |
| **斜坡行驶** | 重力影响轮速 ❌ | 测量真实角速度 ✅ |

---

## 🚀 真机部署检查清单

上真机前必须完成以下步骤：

### 1. 硬件检查
- [ ] IMU硬件正确连接
- [ ] IMU驱动程序正常工作
- [ ] `/imu/data` 话题正常发布
- [ ] 角速度数据有合理值（不是10^-7量级）

### 2. IMU标定
```bash
# 静止放置10分钟，记录零偏
ros2 topic echo /imu/data --field angular_velocity

# 记录静止时的平均值作为零偏
# bias_x = avg(angular_velocity.x)
# bias_y = avg(angular_velocity.y)  
# bias_z = avg(angular_velocity.z)
```

### 3. 旋转测试
```bash
# 原地旋转360度 × 3次
# 使用键盘遥控或发送命令

# 检查累积误差
ros2 run tf2_ros tf2_echo odom base_link

# 期望：3次旋转后角度误差 < 15度 (5度/次)
```

### 4. 参数调整

**如果旋转反应太慢或滞后：**
```yaml
# robot_localization_real.yaml
imu0_twist_rejection_threshold: 1.0  # 降低到1.0-1.5
```

**如果旋转过于敏感或抖动：**
```yaml
imu0_twist_rejection_threshold: 3.0  # 增加到3.0-5.0
```

**如果角度漂移严重：**
1. 检查IMU零偏标定
2. 增加过程噪声：
```yaml
process_noise_covariance[11,11]: 0.04  # vyaw的过程噪声
```

### 5. 组合运动测试
```bash
# 测试直线+旋转组合
# 1. 直线前进2米
# 2. 原地旋转90度
# 3. 再直线前进2米
# 4. 检查最终位置偏差

# 期望：位置偏差 < 20cm，角度偏差 < 10度
```

---

## 📝 Launch文件配置

### 当前开发阶段（仿真）
```python
# simulation_nav2_imu_depthimage.launch.py
ekf_config = os.path.join(pkg_bot_navigation, 'config', 'robot_localization_sim.yaml')
```

### 真机部署阶段（待创建）
```python
# real_robot_nav2.launch.py (待创建)
ekf_config = os.path.join(pkg_bot_navigation, 'config', 'robot_localization_real.yaml')
```

---

## 🔧 故障排除

### 问题1：旋转时odom-map偏移严重

**症状：**
- RViz中看到机器人轨迹扭曲
- 地图变形或墙壁弯曲
- map→odom TF偏移超过20cm

**诊断：**
```bash
# 检查map→odom偏移
ros2 run tf2_ros tf2_echo map odom

# 检查IMU角速度
ros2 topic echo /imu/data --field angular_velocity.z

# 检查EKF输出
ros2 topic echo /odometry/filtered --field twist.twist.angular.z
```

**解决方案：**
- 如果是仿真：使用 `robot_localization_sim.yaml` ✅
- 如果是真机：使用 `robot_localization_real.yaml` ✅
- 检查IMU数据是否正常（不应该是10^-7量级）
- 调整 `imu0_twist_rejection_threshold`

### 问题2：直线运动漂移

**症状：**
- 直线前进，但odom路径是曲线
- 位置累积误差大

**诊断：**
```bash
# 检查轮式里程计
ros2 topic echo /odom --field twist.twist

# 检查EKF配置
ros2 param get /ekf_filter_node odom0_config
```

**解决方案：**
- 检查轮子是否正常工作
- 检查地面是否平整
- 检查odom0_config中x,y,vx,vy是否启用

### 问题3：编译找不到配置文件

**错误信息：**
```
error: [Errno 2] No such file or directory: '.../robot_localization_sim.yaml'
```

**解决方案：**
```bash
# 清理并重新编译
cd ~/lododo_bot
rm -rf build/bot_navigation install/bot_navigation
colcon build --packages-select bot_navigation --symlink-install
```

---

## 📚 参考资料

### 相关文档
- [TECHNICAL_DESIGN.md](../TECHNICAL_DESIGN.md) - 技术设计文档
- [robot_localization官方文档](http://docs.ros.org/en/humble/p/robot_localization/)
- [Nav2官方文档](https://navigation.ros.org/)

### 配置示例
- Gazebo仿真：`robot_localization_sim.yaml`
- 真机部署：`robot_localization_real.yaml`
- 旧版参考：`robot_localization_imu.yaml` (已废弃)

### 关键概念
- **EKF融合**: 扩展卡尔曼滤波器融合多传感器
- **Wheel Slip**: 轮子打滑导致里程计误差
- **Gyroscope**: 陀螺仪测量角速度，不受打滑影响
- **Three Omni-Wheel**: 三轮全向轮特殊运动学

---

## ⚠️ 重要提醒

1. **仿真和真机配置不能混用！**
   - 仿真用sim配置
   - 真机用real配置

2. **真机必须使用IMU角速度！**
   - 三轮全向轮打滑严重
   - 轮式yaw不可靠

3. **上真机前必须完成检查清单！**
   - IMU标定
   - 旋转测试
   - 参数调整

4. **保留旧配置文件作为参考**
   - robot_localization_imu.yaml
   - 记录开发历史

---

**最后更新**: 2025年12月10日
**维护者**: LeKiwi机器人项目组
