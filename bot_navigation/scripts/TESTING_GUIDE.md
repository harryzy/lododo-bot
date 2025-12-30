# 任务状态控制改进 - 快速开始

## 本次改进内容

✅ **任务1**: 统一所有handler的状态响应
- NavigationServiceHandler ✓
- ExplorationServiceHandler ✓  
- PatrolServiceHandler ✓
- WaypointServiceHandler ✓（服务控制，无需修改）

✅ **任务2**: 测试脚本命令行参数支持
- 支持 `--test` 指定运行特定case
- 支持 `--list` 列出所有测试
- 支持 `--verbose` 详细模式

✅ **任务3**: 自动状态检测
- 订阅 `/cmd_vel` 检测速度命令
- 订阅 `/wheel/odom` 检测位移
- 自动验证机器人是否真的停止/移动

---

## 快速测试

### 方法1: 使用交互式脚本（推荐）

```bash
cd ~/lododo_bot
source install/setup.bash

# 运行交互式测试菜单
./quick_test.sh
```

### 方法2: 直接运行测试

```bash
# 列出所有测试
python3 src/bot_navigation/scripts/test_mission_integration_v2.py --list

# 运行TEST 1（导航暂停/恢复）
python3 src/bot_navigation/scripts/test_mission_integration_v2.py --test 1

# 详细模式
python3 src/bot_navigation/scripts/test_mission_integration_v2.py --test 1 --verbose

# 按名称运行
python3 src/bot_navigation/scripts/test_mission_integration_v2.py --test test_navigation_pause_resume
```

---

## 测试前准备

**终端1**: 启动仿真环境
```bash
cd ~/lododo_bot
source install/setup.bash
ros2 launch bot_bringup simple_simulation_nav2_rtabmap.launch.py
```

**终端2**: 等待仿真环境完全启动（约30秒），然后运行测试

---

## 验证要点

### TEST 1: Navigation Pause/Resume

测试会自动验证以下7个检查点：

1. ✓ **Navigation Start** - 导航任务启动成功
2. ✓ **Robot Moving After Start** - 机器人开始移动（自动检测速度）
3. ✓ **Pause Task** - 暂停命令发送成功
4. ✓ **Robot Stopped After Pause** - 机器人实际停止（自动检测速度）
5. ✓ **Robot Stationary During Pause** - 暂停期间保持静止（检测位移<10cm）
6. ✓ **Resume Task** - 恢复命令发送成功
7. ✓ **Robot Moving After Resume** - 机器人重新开始移动（自动检测速度）

**预期结果**: 所有7个检查点都应该显示 `✓ PASS`

---

## 自动检测原理

### RobotStateMonitor 监控机制

```python
# 订阅速度命令
/cmd_vel → 检测是否有非零速度命令

# 订阅里程计
/wheel/odom → 计算实际位移距离

# 判断标准
is_moving = 最近0.5秒内有速度命令 > 0.01 m/s
is_stopped = 最近0.5秒内速度命令 < 0.01 m/s
position_change = 计算3秒内位移距离
```

### 为什么不只检查任务状态？

| 检查方式 | 问题 |
|---------|------|
| ❌ 只检查 task.state | 状态可能变为PAUSED，但机器人仍在移动 |
| ✅ 检查 /cmd_vel + task.state | 验证Nav2实际停止发送速度命令 |
| ✅ 检查 /odom + task.state | 验证机器人实际停止移动 |

---

## 测试输出示例

```bash
========================================================
TEST 1: Navigation Pause/Resume Control
========================================================

[Step 1] 启动导航到目标点 (3.0, 3.0)
✓ PASS Navigation Start: Task ID: navigation_20251230_143022

[Step 2] 等待机器人开始移动...
✓ PASS Robot Moving After Start: 机器人已开始移动
     Details: Linear: 0.254 m/s, Angular: 0.123 rad/s

[Step 3] 暂停任务
✓ PASS Pause Task: 暂停命令发送成功

[Step 4] 验证机器人是否停止...
✓ PASS Robot Stopped After Pause: 机器人已停止
     Details: Linear: 0.001 m/s, Angular: 0.000 rad/s

[Step 5] 保持暂停3秒，验证机器人保持静止...
✓ PASS Robot Stationary During Pause: 位移: 0.0023 m

[Step 6] 恢复任务
✓ PASS Resume Task: 恢复命令发送成功

[Step 7] 验证机器人是否重新开始移动...
✓ PASS Robot Moving After Resume: 机器人重新开始移动
     Details: Linear: 0.268 m/s, Angular: 0.145 rad/s

========================================================
Test Summary
========================================================
Total Tests:  7
Passed:       7
Failed:       0
Pass Rate:    100.0%
```

---

## 详细文档

- [MISSION_STATE_CONTROL_IMPROVEMENT.md](MISSION_STATE_CONTROL_IMPROVEMENT.md) - 完整改进说明
- [PAUSE_RESUME_FIX.md](PAUSE_RESUME_FIX.md) - 暂停/恢复修复详情
- [test_mission_integration_v2.py](src/bot_navigation/scripts/test_mission_integration_v2.py) - 测试脚本源码

---

## 故障排查

### 测试失败: "Service not available"

**原因**: MissionPlanner未启动或仿真环境未完全加载

**解决**:
```bash
# 检查MissionPlanner是否运行
ros2 node list | grep mission_planner

# 检查服务是否可用
ros2 service list | grep mission
```

### 测试失败: "Robot not moving"

**原因**: Nav2导航未正确启动或地图未加载

**解决**:
```bash
# 检查Nav2状态
ros2 topic echo /cmd_vel --once

# 检查地图是否发布
ros2 topic echo /map --once

# 在RVIZ中确认机器人位置正确初始化
```

### 详细模式不显示额外信息

**原因**: `--verbose` 标志未传递或stdout被重定向

**解决**: 确保使用 `--verbose` 标志并直接在终端运行（不要重定向输出）

---

## 下一步

1. **运行完整测试**: `./quick_test.sh` 选择选项2
2. **查看详细日志**: 添加 `--verbose` 标志
3. **实现其他测试**: TEST 2-5尚未完整实现
4. **集成到CI**: 考虑自动化测试流程

---

*创建日期: 2025-12-30*
*测试环境: ROS2 Humble + Gazebo + Nav2 + RTABMap*
