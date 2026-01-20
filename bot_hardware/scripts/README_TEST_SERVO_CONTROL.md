# 单舵机控制测试使用指南

## 功能说明

通过ROS2话题 `/wheel/direct_speeds` 直接控制单个轮子，用于验证驱动是否正确控制了指定的舵机。

## 使用流程

### 1. 启动硬件节点（终端1）

```bash
cd ~/workDisk/lododo_bot
source install/setup.bash
ros2 launch bot_hardware hardware_bringup.launch.py
```

**预期输出**：
```
[INFO] [omni_hardware_node]: === OmniHardwareNode Initialized Successfully ===
[INFO] [omni_hardware_node]: Control loop started at 50Hz
```

### 2. 运行测试脚本（终端2）

```bash
cd ~/workDisk/lododo_bot
source install/setup.bash

# 测试1号轮（舵机7），速度10 rad/s，运行5秒
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 1 --speed 10 --duration 5
```

## 轮子ID映射

| 轮子ID | 舵机ID | 位置 |
|--------|--------|------|
| 1 | 7 | Wheel 1 |
| 2 | 8 | Wheel 2 |
| 3 | 9 | Wheel 3 |

## 测试示例

### 单独测试每个轮子

```bash
# 1号轮正向旋转
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 1 --speed 10 --duration 5

# 2号轮正向旋转
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 2 --speed 10 --duration 5

# 3号轮正向旋转
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 3 --speed 10 --duration 5
```

### 测试反向旋转

```bash
# 1号轮反向旋转
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 1 --speed -10 --duration 5
```

### 自动测试所有轮子

```bash
# 依次测试三个轮子
python3 src/bot_hardware/scripts/test_servo_control.py --test-all --speed 10 --duration 3
```

## 预期输出

### 终端1（硬件节点）

```
[INFO] [omni_hardware_node]: Direct wheel speeds: [10.00, 0.00, 0.00] rad/s
```

### 终端2（测试脚本）

```
======================================================================
单舵机控制测试开始 / Single Servo Control Test Started
======================================================================

[1] 发送速度指令...
    轮子速度: [10.00, 0.00, 0.00] rad/s
    控制轮子1: 10.00 rad/s

[2] 运行监控 (5秒)...
----------------------------------------------------------------------
[INFO] [servo_control_tester]: [0.5s] X位移: 0.023m, 瞬时变化: 0.0023m, 速度: 0.045m/s
[INFO] [servo_control_tester]: [1.0s] X位移: 0.046m, 瞬时变化: 0.0023m, 速度: 0.045m/s
[INFO] [servo_control_tester]: [1.5s] X位移: 0.069m, 瞬时变化: 0.0023m, 速度: 0.045m/s
...

[3] 停止舵机...

[4] 测试统计:
    总位移: 0.225m
    平均速度: 0.045m/s

======================================================================
测试完成 / Test Completed
======================================================================
```

## 如何判断测试成功

✅ **成功标志**：
1. 指定的轮子开始旋转
2. 其他轮子保持静止
3. /wheel/odom 显示位移变化
4. 停止命令后轮子停止

❌ **失败情况**：
1. 错误的轮子旋转 → 舵机ID接线错误
2. 多个轮子同时旋转 → 可能是ID冲突
3. 轮子不转 → 检查速度值是否过小或硬件连接
4. 旋转方向相反 → 正常（取决于安装方向）

## 高级测试

### 测试不同速度

```bash
# 低速测试
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 1 --speed 5 --duration 3

# 中速测试
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 1 --speed 15 --duration 3

# 高速测试
python3 src/bot_hardware/scripts/test_servo_control.py --wheel-id 1 --speed 30 --duration 3
```

### 验证速度响应

观察不同速度下的实际速度响应，验证：
- 速度线性关系
- 加速响应时间
- 最大速度限制

## 故障排查

### 问题1: "话题不存在"

```bash
# 检查话题
ros2 topic list | grep direct_speeds
```

**解决**: 确保硬件节点已启动且版本正确（需要包含 `/wheel/direct_speeds` 订阅）

### 问题2: 轮子不转

1. 检查速度值是否足够大（建议 ≥5 rad/s）
2. 查看硬件节点日志是否有错误
3. 确认串口通信正常

### 问题3: 错误的轮子旋转

- 检查物理接线
- 验证 `hardware_config.yaml` 中的舵机ID配置

## 技术细节

### 话题格式

```bash
# 话题: /wheel/direct_speeds
# 类型: std_msgs/Float64MultiArray
# 数据: [wheel1_rad/s, wheel2_rad/s, wheel3_rad/s]

# 示例：只转1号轮
ros2 topic pub /wheel/direct_speeds std_msgs/msg/Float64MultiArray \
  "{data: [10.0, 0.0, 0.0]}" --once
```

### 控制模式

- **直接速度模式**: 发送到 `/wheel/direct_speeds` 后激活
- **正常模式**: 发送到 `/cmd_vel` 后自动切换回正常模式
- **看门狗超时**: 2秒无命令自动停止并退出直接速度模式

## 下一步

测试通过后，可以进行：
1. 综合运动测试（/cmd_vel）
2. 里程计精度测试
3. Nav2导航集成测试
