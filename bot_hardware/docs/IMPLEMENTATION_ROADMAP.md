# LeKiwi Robot 硬件部署实现路线图

**文档版本**: v1.0  
**创建日期**: 2026-01-19  
**基于设计文档**: HARDWARE_DEPLOYMENT_DESIGN.md v0.7  
**项目阶段**: 代码实现阶段  

---

## 文档说明

本路线图将硬件部署项目划分为5个阶段，每个阶段包含明确的子目标和验收标准。所有章节行号引用均基于设计文档v0.7。

**阅读指南**:
- 🔴 = Critical优先级（必须完成）
- 🟡 = Important优先级（核心功能）
- 🟢 = Low优先级（优化项）
- 📖 = 参考设计文档的具体行号

**实施原则**:
1. 严格按阶段顺序实施，下一阶段依赖上一阶段的交付物
2. 每个阶段结束时必须通过验收测试
3. 遵循设计文档§1.4核心编码规范（行498-1646）
4. 所有参数从hardware_config.yaml读取，禁止硬编码

---

## 阶段概览

| 阶段 | 名称 | 预计工作量 | 交付物 | 依赖关系 |
|------|------|-----------|--------|---------|
| **P0** | 环境准备与配置 | 0.5天 | 项目结构、配置文件 | 无 |
| **P1** | 基础驱动层实现 | 3天 | ST3215Driver、工具类 | P0 |
| **P2** | 硬件接口层实现 | 5天 | OmniHardwareInterface、ros2_control集成 | P1 |
| **P3** | 传感器集成 | 3天 | IMU、相机驱动适配 | P2 |
| **P4** | 启动与测试 | 2天 | Launch文件、集成测试 | P3 |
| **P5** | 优化与文档 | 1.5天 | 性能优化、部署文档 | P4 |

**总预计**: 15天（不含硬件调试时间）

---

## P0阶段: 环境准备与配置 📦

**目标**: 建立项目基础框架和配置文件

**优先级**: 🔴 Critical

**预计工作量**: 0.5天

### P0.1 创建包结构

**参考设计文档**: §1.4 包结构设计（行447-497）

**子目标**:
- [ ] 创建bot_hardware包目录结构
  ```
  bot_hardware/
  ├── bot_hardware/          # Python源代码
  │   ├── __init__.py
  │   ├── drivers/           # 硬件驱动
  │   ├── hardware_interface/# ros2_control接口
  │   ├── utils/             # 工具类
  │   ├── tools/             # 命令行工具
  │   └── imu_ros2_device/   # IMU驱动
  ├── config/
  ├── launch/
  ├── test/
  ├── package.xml
  └── setup.py
  ```
- [ ] 配置package.xml依赖声明
- [ ] 配置setup.py入口点（参考§1.4.5，行1424-1646）

**验收标准**:
- `colcon build --packages-select bot_hardware` 成功
- 包结构符合设计文档要求

---

### P0.2 创建hardware_config.yaml

**参考设计文档**: §1.4.1 统一配置管理（行500-841）

**子目标**:
- [ ] 创建config/hardware_config.yaml
- [ ] 配置serial段（舵机/IMU串口参数）
- [ ] 配置servo段（基本参数+health_monitor子区域，参考行594-659）
- [ ] 配置imu段（坐标系转换参数）
- [ ] 配置kinematics段（雅可比矩阵）
- [ ] 配置motion段（速度限制+velocity_ramp高级配置）
- [ ] 配置odometry段（协方差矩阵）

**验收标准**:
- YAML文件符合设计文档完整结构
- 所有路径使用相对路径（§1.4.4，行1075-1422）

---

### P0.3 实现PathManager工具类

**参考设计文档**: §1.4.4 相对路径管理（行1075-1422）

**子目标**:
- [ ] 实现PathManager类（bot_hardware/utils/path_manager.py）
- [ ] 实现resolve_path()方法（相对路径解析）
- [ ] 实现validate_config_paths()方法（启动时验证）
- [ ] 编写单元测试（test/test_path_manager.py）

**验收标准**:
- 单元测试通过
- 能正确解析相对路径到绝对路径
- 能检测绝对路径并抛出异常

---

## P1阶段: 基础驱动层实现 🔧

**目标**: 实现与硬件直接交互的驱动类

**优先级**: 🔴 Critical

**预计工作量**: 3天

### P1.1 实现ST3215Driver舵机驱动

**参考设计文档**: §3.1 ST3215舵机驱动模块（行1869-2233）

**子目标**:
- [ ] 实现ST3215Driver类（bot_hardware/drivers/st3215_driver.py）
- [ ] 封装scservo_sdk基本操作（参考§3.1.3，行1885-1955）
  - write_speed() - 速度控制
  - read_position() - 位置读取
  - read_status() - 状态查询
  - initialize() - 初始化连接
- [ ] 实现错误处理和重试机制（参考§3.1.4 Q2，行1972-1997）
- [ ] 实现日志输出（纯英文，参考§1.4.2，行842-1050）

**验收标准**:
- 能成功连接舵机（ID 7/8/9）
- 能发送速度指令并读取位置反馈
- 通信失败时能自动重试（最多3次）

---

### P1.2 实现EncoderHandler编码器处理器

**参考设计文档**: §3.2.4 Q3 编码器溢出处理（行2315-2591）

**子目标**:
- [ ] 实现EncoderHandler类（bot_hardware/utils/encoder_handler.py）
- [ ] 实现构造函数（从config读取encoder_resolution等参数）
- [ ] 实现get_position_delta()方法（溢出检测+周数统计）
  - 🆕 Round 7增强: None输入处理
  - 🆕 Round 7增强: 首次初始化标志position_initialized
  - 🆕 Round 7增强: 异常delta合理性检查（容差100 ticks）
- [ ] 实现ticks_to_radians()方法（使用预计算因子）
- [ ] 🆕 实现get_velocity_rad_s()方法（Round 7新增，方案A一步计算）
- [ ] 实现get_absolute_position()方法（调试用）
- [ ] 编写单元测试（参考§6.1.2，行5360-5487）
  - test_none_input_handling（Round 7新增）
  - test_initialization_at_overflow_boundary（Round 7新增）
  - test_packet_loss_scenario（Round 7新增）

**验收标准**:
- 单元测试全部通过
- 能正确处理编码器0→4095和4095→0跨越
- 连续5次None输入后抛出异常

---

### P1.3 实现VelocityRamp速度斜坡限制器

**参考设计文档**: §3.2.4 Q4 VelocityRamp设计（行2411-2591，行3000-3200）

**子目标**:
- [ ] 实现VelocityRamp类（bot_hardware/utils/velocity_ramp.py）
- [ ] 实现构造函数（从config读取加速度限制参数）
- [ ] 实现limit()方法（分别限制线速度和角速度）
- [ ] 实现急停功能（触发+自动恢复，参考§3.1.4 Q5，行2061-2209）
- [ ] 实现ServoHealthMonitor集成（从config['servo']['health_monitor']读取配置）

**验收标准**:
- 能将速度突变（0→0.5m/s）平滑到0.5m/s²加速度
- 急停触发后15秒自动恢复
- 日志记录使用正确级别（WARN/ERROR/CRITICAL，参考§1.4.2，行906-1006）

---

### P1.4 实现OmniKinematics运动学工具

**参考设计文档**: §3.2.3 运动学设计（行2272-2314）

**子目标**:
- [ ] 实现OmniKinematics类（bot_hardware/utils/omni_kinematics.py）
- [ ] 实现inverse_kinematics()方法（机器人速度→轮子速度）
- [ ] 实现forward_kinematics()方法（轮子速度→机器人速度）
- [ ] 从config读取雅可比矩阵参数（L1/L2/L3）
- [ ] 编写单元测试（对比kinematics_test.yaml标准数据，参考§6.1.2，行5438-5487）

**验收标准**:
- 单元测试通过（误差<0.01 rad/s）
- 正逆运动学互为逆运算（round-trip test）

---

## P2阶段: 硬件接口层实现 🔌

**目标**: 实现ros2_control标准硬件接口

**优先级**: 🔴 Critical

**预计工作量**: 5天

### P2.1 实现OmniHardwareInterface核心框架

**参考设计文档**: §3.2.5 OmniHardwareInterface集成（行2592-2999）

**子目标**:
- [ ] 创建OmniHardwareInterface类（bot_hardware/hardware_interface/omni_hardware_interface.py）
- [ ] 实现on_init()方法（加载配置文件）
- [ ] 实现on_configure()方法（初始化驱动和工具类）
  - 🆕 Round 7修正: 正确的初始化顺序（driver→encoder→velocity_ramp→servo_health，参考行2756-2780）
- [ ] 实现on_activate()方法（启动控制循环）
- [ ] 实现on_deactivate()方法（停止控制）

**验收标准**:
- 节点能通过ros2_control生命周期状态机
- 能加载hardware_config.yaml配置

---

### P2.2 实现read()方法（里程计反馈）

**参考设计文档**: §2.2 里程计反馈流（行1680-1709），§3.2.5 read()实现（行2781-2831）

**子目标**:
- [ ] 实现read()方法
  - 🆕 Round 7优化: 在循环前记录时间戳（3个舵机共享，参考§3.5.4，行1775-1820）
  - 🆕 Round 7优化: 使用encoder_handler.get_velocity_rad_s()简化调用
  - 读取3个舵机编码器位置
  - 计算轮子角速度
  - 正向运动学计算机器人速度
  - 积分更新位姿
  - 发布/wheel/odom话题
- [ ] 实现协方差矩阵配置（从config读取）
- [ ] 实现TF发布（odom→base_link，可选）

**验收标准**:
- /wheel/odom话题发布频率~50Hz
- 时间戳延迟<5ms（使用check_timestamp_sync验证，参考§8.6）
- 手动转动轮子，里程计数据正确变化

---

### P2.3 实现write()方法（速度控制）

**参考设计文档**: §2.1 控制指令流（行1649-1679），§3.2.5 write()实现（行2832-2860）

**子目标**:
- [ ] 实现write()方法
  - 从HW command interface读取目标速度
  - VelocityRamp速度斜坡限制
  - 逆向运动学计算轮子速度
  - 转换rad/s→RPM（乘30/π）
  - 发送速度指令到舵机
- [ ] 实现看门狗超时检测（0.5s无cmd_vel则停止）

**验收标准**:
- 发送cmd_vel指令，机器人能平滑加速
- 速度突变被平滑限制（实测加速度≤0.5m/s²）

---

### P2.4 实现_read_current_wheel_velocities()

**参考设计文档**: §3.2.4 Q4 VelocityRamp初始化（行3000-3200）

**子目标**:
- [ ] 实现_read_current_wheel_velocities()方法
  - 🆕 Round 7完整实现: Step 0预初始化EncoderHandler基准
  - Step 1: 等待20ms精确计时
  - Step 2: 第二次读取计算速度
  - Step 3: 异常处理（初始化失败返回ERROR）

**验收标准**:
- 节点重启时能读取当前实际速度（非零）
- 初始化失败时节点无法启动（返回ERROR）

---

### P2.5 创建URDF模型

**参考设计文档**: §4.4 URDF设计（行5210-5274）

**子目标**:
- [ ] 创建lekiwi_bot_real.xacro（urdf/lekiwi_bot_real.xacro）
- [ ] 定义ros2_control硬件插件
- [ ] 配置3个轮子关节（command_interface: velocity, state_interface: position/velocity）
- [ ] 使用实测惯性参数（不复用仿真URDF）

**验收标准**:
- `ros2 launch bot_description display.launch.py model:=real` 能显示模型
- ros2_control能识别硬件插件

---

## P3阶段: 传感器集成 📡

**目标**: 集成IMU和相机传感器

**优先级**: 🟡 Important

**预计工作量**: 3天

### P3.1 适配ybimu_driver IMU驱动

**参考设计文档**: §3.5.2 IMU传感器（行4048-4139），§3.5.2.1 ybimu_driver现状（行4140-4247）

**子目标**:
- [ ] 确认ybimu_driver代码位置（imu_ros2_device/ybimu_driver.py）
- [ ] 验证数据发布到/imu/data_raw
- [ ] 配置串口设备（/dev/lekiwi_imu，udev规则）
- [ ] 验证数据格式（加速度+角速度+方向）

**验收标准**:
- /imu/data_raw话题发布频率~100Hz
- 数据单位正确（m/s², rad/s）

---

### P3.2 实现imu_filter_node滤波节点

**参考设计文档**: §3.5.2.2 imu_filter_node设计（行4248-4517）

**子目标**:
- [ ] 创建imu_filter_node.py（imu_ros2_device/imu_filter_node.py）
- [ ] 实现REP-103坐标系转换（NED→ENU）
  - 配置mounting_rotation参数（从config读取）
  - 实现旋转矩阵应用
- [ ] 实现滤波算法（滑动平均，窗口5）
- [ ] 🆕 Round 7关键设计: 保留ybimu_driver原始时间戳（方案B，参考§3.5.4，行1775-1820）
- [ ] 发布到/imu/data话题

**验收标准**:
- /imu/data话题数据经过滤波和坐标转换
- 时间戳延迟<5ms（使用check_timestamp_sync验证）
- 静止时重力向量指向z轴正方向（[0, 0, 9.81]）

---

### P3.3 实现test_imu_coordinate验证工具

**参考设计文档**: §3.5.2.3 IMU坐标系验证工具（行4518-4943）

**子目标**:
- [ ] 创建test_imu_coordinate.py（tools/test_imu_coordinate.py）
- [ ] 订阅/imu/data，收集100个样本
- [ ] 计算重力向量平均值和标准差
- [ ] 判定标定质量（GOOD/WARN/FAIL）
- [ ] 在setup.py中注册entry_points（参考§1.4.5，行1513-1515）

**验收标准**:
- `ros2 run bot_hardware test_imu_coordinate` 能运行
- 输出重力向量误差<0.2 m/s²（参考§8.4.4，行6043-6051）
- setup.py已注册entry_points（参考§1.4.5，行1513-1515）

---

### P3.4 集成Astra Pro相机驱动

**参考设计文档**: §3.5.1 Astra Pro相机（行3924-3936）

**子目标**:
- [ ] 安装astra_camera驱动（ROS2 Humble版本）
- [ ] 配置USB权限（udev规则）
- [ ] 验证话题发布
  - /camera/color/image_raw
  - /camera/depth/image_raw
  - /camera/color/camera_info

**验收标准**:
- 相机话题正常发布（~30fps）
- RGB和Depth时间戳同步（误差<10ms）

---

### P3.5 执行相机标定

**参考设计文档**: §3.5.3.1 相机内参标定流程（行3937-4047）

**子目标**:
- [ ] 准备棋盘格标定板（8x6，方格25mm）
- [ ] 运行camera_calibration工具
- [ ] 保存标定文件到config/camera_info.yaml
- [ ] 配置相机驱动加载标定文件

**验收标准**:
- 重投影误差<0.5像素
- /camera/color/camera_info包含标定参数

---

## P4阶段: 启动与测试 🚀

**目标**: 创建launch文件并执行集成测试

**优先级**: 🔴 Critical

**预计工作量**: 2天

### P4.1 创建hardware_bringup.launch.py

**参考设计文档**: §3.4.1 启动文件清单（行3350-3358），§3.4.2 启动顺序设计（行3359-3673）

**子目标**:
- [ ] 创建hardware_bringup.launch.py（launch/hardware_bringup.launch.py）
- [ ] 按顺序启动节点（参考§3.4.2事件驱动启动，行3388-3636）
  1. controller_manager
  2. robot_state_publisher
  3. joint_state_broadcaster（使用spawner）
  4. omni_wheel_controller（使用spawner）
  5. ybimu_driver
  6. imu_filter_node
- [ ] 配置参数传递（use_sim_time=false）
- [ ] 添加事件监听器（状态监听，失败处理）

**验收标准**:
- `ros2 launch bot_hardware hardware_bringup.launch.py` 成功启动
- 所有节点正常运行（`ros2 node list`）
- 所有话题正常发布（/wheel/odom, /imu/data）

---

### P4.2 创建real_robot_bringup.launch.py

**参考设计文档**: §3.4.1 启动文件清单（行3350-3358）

**子目标**:
- [ ] 创建real_robot_bringup.launch.py（完整系统启动）
- [ ] 包含hardware_bringup.launch.py
- [ ] 启动robot_localization（EKF融合）
  - 🆕 验证EKF真机配置参数（参考§3.3，行3201-3347）
  - 确认使用robot_localization.yaml（非_sim版本）
  - 验证IMU权重配置正确（处理全向轮滑移）
- [ ] 启动astra_camera
- [ ] 可选：启动RTABMap或Nav2

**验收标准**:
- 完整系统能一键启动
- /odometry/filtered话题发布（EKF融合输出）

---

### P4.3 执行硬件连通性测试

**参考设计文档**: §6.2.1 集成测试清单（行5512-5552）

**子目标**:
- [ ] 测试1.1: 舵机通信（ping响应时间<50ms）
- [ ] 测试1.2: 舵机速度控制（速度误差<10%）
- [ ] 测试1.3: 编码器读取（成功率>99%）
- [ ] 测试1.4: IMU数据读取（频率48-52Hz）
- [ ] 测试1.5: 相机数据读取（频率28-32fps）
- [ ] 🆕 测试2.5: 编码器溢出边界测试（Round 7新增，行5539）
- [ ] 🆕 测试3.5: 时间戳同步验证（Round 7新增，行5544-5545）

**验收标准**:
- 所有测试项通过（参考§6.2.1测试表格）
- 填写测试报告（参考§6.2.2模板，行5553-5652）

---

### P4.4 执行运动学校准测试

**参考设计文档**: §6.2.1 运动学校准（行5525-5528）

**子目标**:
- [ ] 测试2.1: 1米直线前进（位置误差<5cm）
- [ ] 测试2.2: 360°原地旋转（角度误差<5°）
- [ ] 测试2.3: 对角线移动到(1,1)点（误差<10cm）
- [ ] 测试2.4: 8字形路径返回起点（误差<15cm）

**验收标准**:
- 重复测试10次，平均误差满足标准
- 如果超标，调整雅可比矩阵参数

---

## P5阶段: 优化与文档 ✨

**目标**: 性能优化和部署文档完善

**优先级**: 🟢 Low

**预计工作量**: 1.5天

### P5.1 执行IMU标定流程

**参考设计文档**: §8.4 IMU标定流程（行5990-6052）

**子目标**:
- [ ] 按照§8.4.2步骤执行10分钟静止标定
- [ ] 转换标定结果为imu_bias.yaml
- [ ] 保存到calibration/目录（参考§8.4.3，行6029-6042）
- [ ] 使用test_imu_coordinate验证标定质量（参考§8.4.4，行6043-6051）

**验收标准**:
- 重力向量误差<0.2 m/s²
- 陀螺仪零偏<0.01 rad/s

---

### P5.2 实现check_timestamp_sync验证工具

**参考设计文档**: §8.6 时间戳同步验证工具（行6070-6289）

**子目标**:
- [ ] 创建check_timestamp_sync.py（tools/check_timestamp_sync.py）
- [ ] 实现TimestampSyncChecker类（完整代码参考行6112-6269）
- [ ] 订阅/wheel/odom和/imu/data
- [ ] 计算发布延迟统计（滑动窗口100个样本）
- [ ] 每5秒生成报告（PASS/WARN/FAIL）
- [ ] 在setup.py中注册entry_points

**验收标准**:
- `ros2 run bot_hardware check_timestamp_sync` 能运行
- 报告显示延迟<5ms（PASS状态）
- setup.py已注册entry_points

---

### P5.3 性能优化

**参考设计文档**: §6.3 性能测试（行5675-5708）

**子目标**:
- [ ] 测量控制循环延迟（目标<20ms）
- [ ] 测量CPU占用率（目标<70%）
- [ ] 测量内存占用（目标<2GB）
- [ ] 测量串口通信成功率（目标>99%）
- [ ] 如果性能不达标，进行优化
  - 降低不必要的日志输出
  - 关闭调试功能
  - 优化计算密集代码

**验收标准**:
- 所有性能指标达标（参考§6.2.2性能指标表，行5622-5630）

---

### P5.4 编写部署脚本

**参考设计文档**: §8.1 系统依赖安装（行5853-5953）

**子目标**:
- [ ] 创建scripts/deploy_hardware.sh（自动化部署脚本）
- [ ] 实现依赖安装（ROS2、Python库、udev规则）
- [ ] 实现工作空间构建
- [ ] 实现环境变量配置
- [ ] 编写使用说明

**验收标准**:
- 在全新Ubuntu 22.04系统上运行脚本能完成部署
- 脚本执行无错误

---

### P5.5 完善文档

**参考设计文档**: §8 部署与维护指南（行5851-6289）

**子目标**:
- [ ] 编写README.md（快速上手指南）
- [ ] 编写TROUBLESHOOTING.md（常见问题排查，参考§8.3，行5978-5987）
- [ ] 编写MAINTENANCE.md（维护计划，参考§8.5，行6053-6068）
- [ ] 更新setup.py注释和文档字符串

**验收标准**:
- 文档完整，新用户能独立完成部署
- 所有命令可复制执行

---

## 验收标准总结

### P0阶段验收（环境准备）
- [ ] 包结构完整，colcon build成功
- [ ] hardware_config.yaml包含所有必需参数
- [ ] PathManager单元测试通过

### P1阶段验收（基础驱动）
- [ ] ST3215Driver能控制舵机
- [ ] EncoderHandler单元测试全部通过（包括Round 7新增3个边界测试）
- [ ] VelocityRamp能平滑限制速度突变
- [ ] OmniKinematics运动学精度<0.01 rad/s

### P2阶段验收（硬件接口）
- [ ] OmniHardwareInterface通过ros2_control生命周期
- [ ] /wheel/odom话题发布正常（50Hz）
- [ ] 发送cmd_vel能控制机器人运动
- [ ] 速度斜坡限制生效（加速度≤0.5m/s²）

### P3阶段验收（传感器集成）
- [ ] IMU数据坐标系正确（重力向量[0,0,9.81]）
- [ ] IMU时间戳延迟<5ms
- [ ] 相机RGB+Depth正常发布（30fps）
- [ ] test_imu_coordinate输出GOOD状态

### P4阶段验收（启动与测试）
- [ ] hardware_bringup.launch.py能一键启动硬件层
- [ ] real_robot_bringup.launch.py能启动完整系统
- [ ] 集成测试清单21项全部通过（参考§6.2.1）
- [ ] 运动学校准测试达标（直线误差<5cm，旋转误差<5°）

### P5阶段验收（优化与文档）
- [ ] IMU标定质量达标（重力误差<0.2 m/s²）
- [ ] check_timestamp_sync报告PASS
- [ ] 性能指标全部达标（CPU<70%, 延迟<20ms）
- [ ] 部署脚本在全新系统上测试通过
- [ ] 文档完整可用

---

## 风险预警

**参考设计文档**: §5 风险点与应对策略（行5275-5304）

### 高风险项 🔴
1. **舵机通信不稳定**（§5.1）
   - 影响: 机器人抖动/停止
   - 应对: P1.1实现重试机制，P4.3测试通信成功率

2. **编码器溢出误判**（§3.2.4 Q3）
   - 影响: 里程计跳变
   - 应对: P1.2实现完整边界测试，P4.3执行溢出边界测试

3. **ros2_control节点崩溃**（§5.2）
   - 影响: 失去控制
   - 应对: P2.1实现异常处理，P4.3测试节点重启恢复

### 中风险项 🟡
1. **IMU零偏漂移**（§5.1）
   - 影响: yaw累积误差
   - 应对: P5.1执行标定，P5.5添加定期标定提醒

2. **EKF发散**（§5.2）
   - 影响: 位姿估计错误
   - 应对: P3.2正确配置协方差矩阵，P4.4测试融合效果

3. **时间戳同步失败**（Round 7新增）
   - 影响: EKF融合精度下降
   - 应对: P2.2/P3.2严格遵循时间戳策略，P5.2验证同步性

---

## 开发注意事项

### 必须遵守的规范
1. **配置管理**（§1.4.1）: 所有参数从hardware_config.yaml读取，禁止硬编码
2. **双语注释**（§1.4.2）: 中文描述+英文日志
3. **相对路径**（§1.4.4）: 配置文件中只能使用相对路径
4. **日志级别**（§1.4.2, Round 7扩展）: 严格区分WARN/ERROR/CRITICAL使用场景
5. **setup.py注册**（§1.4.5）: 所有命令行工具必须注册entry_points

### Round 7关键设计要求
1. **EncoderHandler增强**（§3.2.4 Q3）: 必须实现None处理、初始化标志、异常delta检查
2. **VelocityRamp预初始化**（§3.2.4 Q4）: _read_current_wheel_velocities()必须包含Step 0
3. **组件初始化顺序**（§3.2.5）: 严格按driver→encoder→velocity_ramp→servo_health顺序
4. **时间戳策略**（§3.5.4）: OmniHardwareInterface循环前记录，imu_filter_node保留原始
5. **单元测试覆盖**（§6.1.2）: EncoderHandler必须包含3个Round 7新增边界测试

### 开发工具
- **测试工具**: pytest（单元测试）、colcon test（集成测试）
- **调试工具**: rqt_graph（数据流）、ros2 topic echo（话题监控）
- **验证工具**: test_imu_coordinate（IMU）、check_timestamp_sync（时间戳）

---

## 文档更新记录

| 日期 | 版本 | 变更说明 | 作者 |
|------|------|---------|------|
| 2026-01-19 | v1.0 | 初始版本，基于设计文档v0.7创建 | Hurry |

---

## 附录: 关键行号索引

**方便快速定位设计文档关键章节**

| 章节 | 行号范围 | 描述 |
|------|---------|------|
| §1.4 核心编码规范 | 498-1646 | 配置管理、双语注释、路径管理、setup.py |
| §1.4.1 hardware_config.yaml | 500-841 | 完整配置文件结构 |
| §1.4.5 setup.py配置 | 1424-1646 | entry_points注册示例 |
| §2.1-2.3 数据流设计 | 1649-1738 | 控制/里程计/传感器数据流 |
| §3.1 ST3215Driver | 1869-2233 | 舵机驱动设计 |
| §3.2.4 Q3 EncoderHandler | 2315-2591 | 编码器溢出+Round 7增强 |
| §3.2.4 Q4 VelocityRamp | 2411-2591, 3000-3200 | 速度斜坡+初始化策略 |
| §3.2.5 OmniHardwareInterface | 2592-2999 | 硬件接口集成示例 |
| §3.4.2 启动顺序设计 | 3359-3673 | 事件驱动launch机制 |
| §3.5.2 IMU传感器 | 4048-4943 | ybimu_driver+imu_filter_node+验证工具 |
| §3.5.4 时间戳同步 | 1739-1866 | Round 7时间戳策略 |
| §6.1.2 单元测试 | 5360-5487 | EncoderHandler边界测试 |
| §6.2.1 集成测试 | 5512-5552 | 测试清单+Round 7新增项 |
| §8.4 IMU标定流程 | 5990-6052 | 标定步骤+质量评估 |
| §8.6 时间戳验证工具 | 6070-6289 | check_timestamp_sync完整代码 |

---

**END OF ROADMAP**
