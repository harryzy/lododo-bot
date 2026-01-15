# lododo机器人真机硬件部署需求文档

**项目名称**: lododo三轮全向移动机器人  
**目标平台**: Raspberry Pi 4B + Ubuntu 22.04  
**ROS版本**: ROS2 Humble  
**创建日期**: 2026-01-15  
**状态**: ⚙️ 规划阶段

---

## 1. 项目概述

将仿真环境中验证通过的自主导航系统部署到真实硬件平台，实现ST3215舵机驱动、传感器集成、EKF配置切换和硬件专用启动文件。

---

## 2. 硬件清单

| 组件 | 型号 | 数量 | 接口 |
|------|------|------|------|
| 主控 | Raspberry Pi 4B (4GB+) | 1 | - |
| 驱动电机 | ST3215舵机 | 3 | TTL串口 (1Mbps) |
| 串口转换器 | USB转TTL | 1 | /dev/ttyACM0 |
| 深度相机 | 奥比中光 Astra Pro | 1 | USB 3.0 |
| IMU | 亚博６轴IMU | 1 | 串口 |
| 电池 | 12V 5.2Ah锂电池 | 1 | DC5521 |

---

## 3. 开发任务清单

### 3.1 ST3215舵机驱动开发

**文件路径**: `bot_hardware/bot_hardware/drivers/st3215_driver.py`

**功能要求**:
- 串口通信，波特率1Mbps（115200备选）
- 指令实现：写速度、读位置、读电压
- 数据包结构：帧头(0xFF 0xFF) + ID + 长度 + 命令 + 数据 + 校验和
- 错误处理：超时重试、校验和验证、异常捕获

**预计工作量**: 1天

---

### 3.2 ros2_control硬件接口实现

**文件路径**: `bot_hardware/bot_hardware/omni_hardware_interface.py`

**功能要求**:
- 实现ros2_control的`SystemInterface`接口
- 订阅 `/omni_wheel_controller/commands`（轮速指令）
- 发布 `/wheel/odom`（基于编码器的里程计，50Hz）
- 正向运动学：编码器位置 → (x, y, θ)姿态估计
- 速度单位转换：rad/s ↔ RPM

**预计工作量**: 2天

---

### 3.3 传感器集成

**Astra Pro深度相机**:
- 使用官方驱动：`astra_camera/astra_pro.launch.py`
- 话题输出：`/camera/color/image_raw`, `/camera/depth/image_raw`
- 验证RGB-D对齐和深度范围(0.6m-8m)

**IMU传感器**:
- 驱动：根据具体硬件型号确定
- 话题输出：`/imu/data` (建议50-100Hz)
- 静态零偏标定：机器人静止10分钟记录偏差
- **关键要求**：必须提供准确的角速度(yaw rate)用于EKF融合

**预计工作量**: 1天

---

### 3.4 EKF配置切换（真机专用）

**关键说明**：真机必须使用IMU角速度，因为三轮全向轮旋转时打滑严重，轮式里程计无法可靠估计yaw角度。

**配置文件**: 
- `bot_navigation/config/localization/robot_localization.yaml` (真机配置)
- `bot_navigation/config/localization/robot_localization_sim.yaml` (仿真配置)

**真机配置修改**:
```yaml
# 轮式里程计：仅使用位置和线速度，禁用yaw和vyaw
odom0_config: [true, true, false, false, false, false,
               true, true, false, false, false, false,
               false, false, false]
               
# IMU：启用yaw和vyaw，作为旋转信息的主要来源
imu0_config:  [false, false, false, true, true, true,
               false, false, false, false, false, true,
               false, false, false]
```

**原因分析**：三轮全向轮机器人原地旋转时，三个轮子打滑不一致，导致轮式里程计计算的角速度误差巨大（360°旋转可能产生20-50度误差）。IMU陀螺仪直接测量真实角速度，不受轮子打滑影响。

**预计工作量**: 0.5天

---

### 3.5 真机启动文件创建

**文件路径**: `bot_bringup/launch/real_robot_*.launch.py`

**启动文件清单**:
- `real_robot_bringup.launch.py`: 硬件节点 + 控制器加载
- `real_robot_navigation.launch.py`: Nav2 + RTABMap定位模式
- `real_robot_slam.launch.py`: Nav2 + RTABMap建图模式
- 所有节点必须设置 `use_sim_time:=false`

**预计工作量**: 1天

---

## 4. 测试与校准流程

### 4.1 硬件连接测试
```bash
ros2 topic echo /imu/data                    # 验证IMU数据输出
ros2 topic echo /camera/depth/image_raw      # 验证相机深度图
ros2 run bot_hardware test_servo             # 测试舵机响应
```

### 4.2 运动学校准
- **直线运动测试**：1米直线，调整线速度系数
- **旋转测试**：360° × 3次，验证yaw累积误差 < 5°
- **全向运动测试**：对角线移动，微调运动学雅可比矩阵

### 4.3 导航功能验证
- **短距离导航**：2-3米路点目标
- **SLAM建图**：小场地10分钟建图测试
- **长时间稳定性**：连续运行2小时以上

---

## 5. 代码开发规范

### 5.1 双语注释要求

**所有代码必须使用中英文双语注释**：

```python
# 初始化舵机驱动 / Initialize servo driver
self.driver = ST3215Driver('/dev/ttyACM0')

# 计算三轮全向轮速度指令 / Calculate 3-wheel omnidirectional velocity commands
wheel_velocities = self._inverse_kinematics(vx, vy, omega)

# 读取编码器位置并计算里程计 / Read encoder positions and compute odometry
positions = self._read_all_encoders()
odom = self._forward_kinematics(positions)
```

### 5.2 日志输出要求

**所有日志输出必须使用纯英文**：

```python
# ✅ 正确示例
self.get_logger().info('Servo driver initialized successfully')
self.get_logger().error(f'Failed to read encoder from servo ID: {servo_id}')
self.get_logger().warn('IMU data timeout, using last known value')

# ❌ 错误示例（禁止中文日志）
self.get_logger().info('舵机驱动初始化成功')
self.get_logger().error(f'读取编码器失败，舵机ID: {servo_id}')
```

### 5.3 命名规范
- 类名：`PascalCase` (例如 `ST3215Driver`)
- 函数名：`snake_case` (例如 `read_encoder_position`)
- ROS节点名：`snake_case` (例如 `omni_hardware_interface`)
- 私有方法：`_snake_case` (例如 `_calculate_checksum`)

---

## 6. 验收标准

- [ ] ST3215舵机响应速度指令，延迟 < 50ms
- [ ] 轮式里程计以50Hz频率发布，包含编码器反馈
- [ ] IMU静止时角速度方差 < 0.01 rad/s
- [ ] 机器人直线移动1米，位置误差 < 5cm
- [ ] 机器人原地旋转360°，角度误差 < 5°
- [ ] Nav2成功导航到2米目标点，无碰撞
- [ ] RTABMap在已建图区域重定位成功率 > 95%

---

## 7. 开发时间计划

| 阶段 | 任务内容 | 预计时间 |
|------|---------|---------|
| 第1周 | ST3215驱动 + 硬件接口 | 3天 |
| 第1周 | 传感器集成 + EKF配置 | 2天 |
| 第2周 | 启动文件 + 连接测试 | 2天 |
| 第2周 | 运动学校准 + 功能验证 | 3天 |

**总预计工作量**: 10天 (2周)

---

## 8. 参考文档

- [TECHNICAL_DESIGN.md](../../TECHNICAL_DESIGN.md) - 系统架构设计
- [TODO.md](../../../TODO.md) - 开发路线图
- [AUTONOMOUS_PATROL_DESIGN.md](../../AUTONOMOUS_PATROL_DESIGN.md) - 巡航系统设计
- [omni_controller_node.py](../../bot_control/bot_control/omni_controller_node.py) - 运动学参考实现

---

**文档状态**: 草稿 v1.0  
**下次审阅**: 2026-01-20  
**批准条件**: 系统集成测试完成
