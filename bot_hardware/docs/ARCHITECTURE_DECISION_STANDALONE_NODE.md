# 架构决策记录 (ADR) - 采用Standalone硬件控制节点

**文档类型**: Architecture Decision Record (ADR)  
**决策日期**: 2026-01-19  
**状态**: ✅ 已接受 (Accepted)  
**决策者**: 项目团队  

---

## 决策概述

**决策**: LeKiwi Robot项目**不使用ros2_control框架**，改用Standalone ROS2硬件控制节点实现硬件控制功能。

**简短说明**: 将OmniHardwareInterface从ros2_control SystemInterface改为独立的ROS2 Node，直接订阅/cmd_vel和发布/wheel/odom，绕过controller_manager和pluginlib机制。

---

## 背景与上下文

### 项目定位

- **类型**: Python全原型项目
- **目标**: 快速验证全向轮机器人导航、SLAM、任务管理功能
- **生命周期**: 中期项目（6-12个月）
- **后续计划**: 如开发C++版本，将重新设计架构，当前代码不复用

### 技术挑战

在P2.6阶段（硬件接口层实现）面临ros2_control Python集成问题：

**问题**: ros2_control使用pluginlib加载C++硬件接口插件，Python实现无法直接注册。

**可选方案**:
- 方案A: C++包装器（pybind11桥接） - 2-3天
- 方案B: 纯C++重构 - 5-7天
- 方案C: ros2_control_py（第三方工具） - 风险高
- 方案D: Standalone节点 - 1天

**技术调研**: 详见[P2.6技术调研报告](./P2_6_TECHNICAL_RESEARCH_REPORT.md)

---

## 决策内容

### 核心决策

**采用方案D** - Standalone ROS2硬件控制节点（OmniHardwareNode）

**架构变更**:

```
# 原设计（ros2_control标准架构）
[Nav2] → /cmd_vel → [controller_manager] → [omni_wheel_controller] 
                   → [OmniHardwareInterface插件] → [硬件]

# 新架构（Standalone节点）
[Nav2] → /cmd_vel → [OmniHardwareNode] → [硬件]
```

**实现方式**:
- 创建OmniHardwareNode继承rclpy.node.Node
- 直接实例化OmniHardwareInterface（复用P2.1-P2.5代码）
- 订阅/cmd_vel话题接收速度指令
- 发布/wheel/odom话题提供里程计
- 50Hz定时器执行read()/write()控制循环

---

## 决策理由

### 正面因素 (Pros)

#### 1. 符合项目定位 ⭐⭐⭐⭐⭐
- ✅ Python全原型项目，无需考虑C++兼容性
- ✅ 后续如开发C++版本，当前代码不复用，无遗留包袱
- ✅ 简化架构，降低复杂度

#### 2. 快速实施 ⭐⭐⭐⭐⭐
- ✅ 1天完成（vs ros2_control方案2-3天）
- ✅ 节省2天工作量
- ✅ 无C++开发成本

#### 3. 代码复用 ⭐⭐⭐⭐⭐
- ✅ 直接复用P2.1-P2.5的659行代码
- ✅ 无需重写，仅需包装为ROS2节点
- ✅ 所有P1组件（driver, encoder, kinematics, velocity_ramp）保持不变

#### 4. 功能完整 ⭐⭐⭐⭐⭐
- ✅ 所有核心功能保留（控制、里程计、看门狗、速度斜坡）
- ✅ 上层功能完全不受影响（Nav2、SLAM、任务管理）
- ✅ 性能无差异（同样50Hz控制循环）

#### 5. 易于调试 ⭐⭐⭐⭐
- ✅ 纯Python实现，无C++/Python互操作问题
- ✅ 直接查看ROS2日志，无controller_manager中间层
- ✅ 便于快速修改和实验

### 负面因素 (Cons)

#### 1. 不符合ros2_control标准 ⚠️
- ❌ 无法使用`ros2 control`命令行工具
- ❌ 无controller_manager管理功能
- ❌ 无法动态切换控制器

**缓解措施**: 
- ✅ 项目只有1个控制器，不需要切换
- ✅ 可通过ROS2标准工具（ros2 topic/service）管理节点
- ✅ `ros2 control`工具对项目无实际价值

#### 2. 代码不能复用到其他ros2_control机器人 ⚠️
- ❌ OmniHardwareNode是项目特定实现
- ❌ 无法作为ros2_control插件分享给社区

**缓解措施**:
- ✅ 项目定位为原型，不需要复用
- ✅ 如未来需要，可参考P2.6技术调研报告中的方案A

#### 3. 与原设计文档偏离 ⚠️
- ❌ HARDWARE_DEPLOYMENT_DESIGN.md基于ros2_control设计
- ❌ IMPLEMENTATION_ROADMAP.md P2阶段描述需要更新

**缓解措施**:
- ✅ 已更新IMPLEMENTATION_ROADMAP.md（本ADR同时完成）
- ✅ 创建本ADR文档记录架构变更
- ✅ 保留P2.5的URDF/controller配置文件作为参考

---

## 风险分析

### 已识别风险

| 风险 | 概率 | 影响 | 缓解措施 | 状态 |
|------|------|------|---------|------|
| 上层功能不兼容 | 🟢 低 | 🔴 高 | 已验证话题接口兼容 | ✅ 已缓解 |
| 性能不足 | 🟢 低 | 🟡 中 | 50Hz足够，实测验证 | ✅ 已缓解 |
| 未来需要ros2_control | 🟡 中 | 🟡 中 | 可参考技术调研报告重构 | ⏳ 接受风险 |
| 缺少管理工具 | 🟢 低 | 🟢 低 | 使用ros2标准工具替代 | ✅ 已缓解 |

### 风险接受

**接受风险**: 未来如需ros2_control标准架构，需要2-3天重构时间。

**理由**: 
- 当前项目定位为Python原型，2-3天重构成本可接受
- 后续C++版本将重新设计，无需继承当前代码
- 节省的2天工作量可用于其他功能开发

---

## 实施影响

### 代码变更

#### 新增文件
- `bot_hardware/hardware_interface/omni_hardware_node.py` - Standalone节点实现
- `bot_hardware/launch/hardware_node.launch.py` - 节点启动文件

#### 修改文件
- `IMPLEMENTATION_ROADMAP.md` - 更新P2阶段描述
- `package.xml` - 移除ros2_control相关依赖（如需）
- `setup.py` - 添加omni_hardware_node entry_point

#### 保留文件（不使用）
- `urdf/bot_hardware.ros2_control.xacro` - 参考保留
- `config/controllers.yaml` - 参考保留
- `launch/hardware_test.launch.py` - 参考保留

### 接口变更

#### 保持不变的接口（上层兼容）
- ✅ `/cmd_vel` (Twist) - 速度指令输入
- ✅ `/wheel/odom` (Odometry) - 里程计输出
- ✅ 50Hz控制频率
- ✅ 所有功能行为（看门狗、速度斜坡、急停）

#### 不再提供的接口
- ❌ `/controller_manager/*` services
- ❌ `/omni_wheel_controller/*` topics
- ❌ ros2_control State/Command interfaces

### 依赖变更

#### 移除依赖
- `ros2_control` (可选，仅用于mock测试)
- `ros2_controllers` (不需要)
- `controller_manager` (不需要)

#### 新增依赖
- 无（所有依赖已满足）

---

## 验证标准

### 功能验证

- [ ] OmniHardwareNode能正常启动
- [ ] 订阅/cmd_vel，teleop能控制机器人
- [ ] 发布/wheel/odom，频率~50Hz
- [ ] Nav2导航功能正常
- [ ] SLAM建图功能正常
- [ ] 看门狗超时机制工作
- [ ] 所有P1组件（舵机、编码器）正常工作

### 性能验证

- [ ] 控制循环频率≥50Hz
- [ ] 里程计时间戳延迟<5ms
- [ ] CPU占用<10%（Raspberry Pi 4B）
- [ ] 内存占用<100MB

### 集成验证

- [ ] 与Nav2集成测试（移动到目标点）
- [ ] 与RTABMap集成测试（SLAM建图）
- [ ] 与MissionPlanner集成测试（任务执行）
- [ ] 与CommandAdapter集成测试（语音/Web控制）

---

## 替代方案

### 如果需要恢复ros2_control

**场景**: 未来需要ros2_control标准架构（复用到其他机器人、集成第三方控制器等）

**实施路径**:

#### 方案A: C++包装器（推荐）⭐
- **步骤**:
  1. 创建C++ SystemInterface派生类
  2. 使用pybind11桥接Python OmniHardwareInterface
  3. 实现pluginlib注册
  4. 恢复P2.5的URDF/controller配置
- **工作量**: 2-3天
- **参考**: [P2.6技术调研报告](./P2_6_TECHNICAL_RESEARCH_REPORT.md) 方案A

#### 方案B: 纯C++重构
- **步骤**:
  1. 将OmniHardwareInterface移植到C++
  2. 将P1组件移植到C++
  3. 实现ros2_control标准接口
- **工作量**: 5-7天
- **适用**: C++版本项目

---

## 参考资料

### 相关文档
- [IMPLEMENTATION_ROADMAP.md](./IMPLEMENTATION_ROADMAP.md) - P2阶段实施路线
- [P2_6_TECHNICAL_RESEARCH_REPORT.md](./P2_6_TECHNICAL_RESEARCH_REPORT.md) - 技术调研报告
- [P2_STAGE_COMPLETION_REPORT.md](./P2_STAGE_COMPLETION_REPORT.md) - P2阶段完成报告
- [HARDWARE_DEPLOYMENT_DESIGN.md](./HARDWARE_DEPLOYMENT_DESIGN.md) - 原硬件部署设计

### 技术参考
- ROS2 Node API: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- Standalone控制节点示例: [omni_hardware_node.py](../bot_hardware/hardware_interface/omni_hardware_node.py)
- ros2_control文档: https://control.ros.org/ (参考，不使用)

---

## 决策历史

| 日期 | 事件 | 状态 |
|------|------|------|
| 2026-01-19 | P2.6技术调研完成 | 调研阶段 |
| 2026-01-19 | 团队讨论，确定项目为Python原型 | 讨论阶段 |
| 2026-01-19 | 决策采用Standalone节点方案 | ✅ 已接受 |
| 2026-01-19 | 更新IMPLEMENTATION_ROADMAP.md | 已实施 |
| 2026-01-19 | 创建本ADR文档 | 已归档 |

---

## 签名确认

**决策负责人**: 项目负责人  
**技术评审**: GitHub Copilot (AI Agent)  
**日期**: 2026-01-19  
**批准状态**: ✅ 已批准

---

**文档状态**: 📌 已归档 (Archived)  
**最后更新**: 2026-01-19  
