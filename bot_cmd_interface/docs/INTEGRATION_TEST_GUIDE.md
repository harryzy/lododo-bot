# CommandAdapter 集成测试指南
# Command Adapter Integration Test Guide

## 测试概览 / Test Overview

集成测试套件验证CommandAdapter与MissionPlanner之间的完整交互流程，包括：

- ✅ 导航功能 (navigate_to_pose)
- ✅ 任务管理 (exploration, patrol, pause/resume/cancel)
- ✅ 查询功能 (task_status, robot_status)
- ✅ 异常处理 (invalid JSON, missing params, deduplication)

---

## 快速开始 / Quick Start

### 1. 启动测试环境 / Launch Test Environment

**终端 1 - Terminal 1: 启动仿真+MissionPlanner+CommandAdapter**

```bash
# 定位模式（用于导航测试）
ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
  slam:=false \
  map_name:=office_floor1

# 或者 SLAM模式（用于探索测试）
ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
  slam:=true
```

**等待所有节点启动完成（~10-15秒）**

### 2. 运行集成测试 / Run Integration Tests

**终端 2 - Terminal 2: 运行测试**

```bash
cd ~/lododo_bot/src/bot_cmd_interface

# 方法1: 交互式菜单（推荐）
./scripts/run_integration_tests.sh

# 方法2: 运行所有测试
./scripts/run_integration_tests.sh --all

# 方法3: 运行特定测试
./scripts/run_integration_tests.sh --test 1  # 按编号
./scripts/run_integration_tests.sh --test test_navigate_to_pose  # 按名称

# 方法4: 直接调用Python脚本
python3 test/test_integration.py --verbose
python3 test/test_integration.py --test 1
python3 test/test_integration.py --list
```

---

## 测试用例详解 / Test Cases

### 测试1: Navigate to Pose - 导航到目标点

**测试目标**: 验证完整的导航流程

```python
# 请求
action: NAVIGATE_TO_POSE
params: {x: 2.0, y: 3.0, yaw: 1.57}

# 期望响应流程
queued → executing → completed (with task_id)
```

**验证点**:
- ✅ 接收到3个响应（queued, executing, completed）
- ✅ completed响应包含task_id
- ✅ 任务成功提交到MissionPlanner

---

### 测试2: Emergency Stop - 紧急停止

**测试目标**: 验证紧急停止功能

```python
action: EMERGENCY_STOP
params: {clear_tasks: true}
priority: 1  # 最高优先级
```

**验证点**:
- ✅ 立即执行（高优先级）
- ✅ 成功调用emergency_stop服务
- ✅ 清除任务队列（如果clear_tasks=true）

---

### 测试3: Start Exploration - 开始探索

**测试目标**: 验证探索任务启动

```python
action: START_EXPLORATION
params: {
  map_name: 'test_map',
  save_map: false,
  max_duration: 60.0
}
```

**验证点**:
- ✅ 返回task_id
- ✅ MissionPlanner开始执行探索
- ✅ 参数正确传递

**注意**: 需要SLAM模式启动环境

---

### 测试4: Get Task Status - 获取任务状态

**测试目标**: 验证任务状态查询

**流程**:
1. 创建导航任务
2. 获取task_id
3. 查询任务状态

**验证点**:
- ✅ 返回任务状态（state, progress）
- ✅ task_id匹配
- ✅ 状态正确更新

---

### 测试5: Pause/Resume/Cancel Task - 任务控制

**测试目标**: 验证任务暂停/恢复/取消

**流程**:
1. 创建导航任务
2. 暂停任务
3. 恢复任务
4. 取消任务

**验证点**:
- ✅ 每个操作都成功返回
- ✅ 任务状态正确变化
- ✅ MissionPlanner执行相应操作

---

### 测试6: Invalid JSON Format - 非法JSON

**测试目标**: 验证错误处理

```python
# 发送非法JSON字符串
"{ invalid json }"
```

**验证点**:
- ✅ 不崩溃
- ✅ 返回failed响应
- ✅ 错误消息清晰

---

### 测试7: Missing Parameters - 参数缺失

**测试目标**: 验证参数验证

```python
action: NAVIGATE_TO_POSE
params: {y: 2.0, yaw: 0.0}  # 缺少x参数
```

**验证点**:
- ✅ 检测到参数缺失
- ✅ 返回错误或使用默认值
- ✅ 错误消息包含缺失参数信息

---

### 测试8: Request Deduplication - 请求去重

**测试目标**: 验证重复请求处理

**流程**:
1. 发送request_id=A的请求
2. 再次发送request_id=A的请求
3. 验证第二个请求被拒绝

**验证点**:
- ✅ 第一个请求正常处理
- ✅ 第二个请求被拒绝或忽略
- ✅ 不影响第一个请求的执行

---

## 测试模式 / Test Modes

### Mock模式（快速测试）

```bash
# 启动Mock模式
ros2 launch bot_bringup cmd_adapter.launch.py use_mock:=true

# 运行测试
python3 scripts/test_command_adapter.py
```

**特点**:
- ⚡ 快速执行（无需等待真实服务）
- 🔍 验证基本逻辑和消息格式
- ❌ 不验证真实服务交互

---

### Service模式（完整测试）

```bash
# 启动完整环境
ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=office_floor1

# 运行集成测试
./scripts/run_integration_tests.sh --all
```

**特点**:
- ✅ 验证真实服务交互
- ✅ 验证端到端流程
- ⏱️ 执行时间较长（每个测试5-10秒）

---

## 故障排除 / Troubleshooting

### 问题1: 测试超时 "Timeout waiting for completion"

**原因**:
- MissionPlanner未启动
- 服务不可用
- 网络延迟

**解决**:
```bash
# 检查服务是否可用
ros2 service list | grep mission

# 应该看到:
# /mission/emergency_stop
# /mission/navigate_to_pose
# /mission/start_exploration
# ...

# 检查节点是否运行
ros2 node list | grep mission_planner
```

---

### 问题2: "No response received"

**原因**:
- CommandAdapter未启动
- Topic订阅失败
- ROS2环境未source

**解决**:
```bash
# 检查Topic
ros2 topic list | grep /cmd/

# 应该看到:
# /cmd/request
# /cmd/response

# 检查节点
ros2 node list | grep command_adapter
```

---

### 问题3: 测试失败 "Missing statuses"

**原因**:
- 任务执行太快，中间状态未捕获
- 响应回调延迟

**解决**:
- 增加等待时间: `timeout=15.0` → `timeout=20.0`
- 检查网络延迟
- 查看日志: `ros2 launch ... log_level:=debug`

---

## 性能指标 / Performance Metrics

### 响应时间基准

| 测试用例 | 预期时间 | 最长时间 |
|---------|---------|---------|
| Navigate to Pose | 3-5s | 10s |
| Emergency Stop | 1-2s | 5s |
| Start Exploration | 3-5s | 10s |
| Get Task Status | 1-2s | 5s |
| Pause/Resume/Cancel | 2-4s each | 10s |
| Invalid JSON | <1s | 2s |
| Missing Parameters | <1s | 2s |
| Deduplication | 2-4s | 10s |

### 系统资源

- **内存使用**: ~200MB (包括Gazebo)
- **CPU使用**: ~15-25% (4核)
- **网络延迟**: <10ms (本地)

---

## 持续集成 / CI Integration

### GitHub Actions示例

```yaml
name: CommandAdapter Integration Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup ROS2 Humble
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble
      
      - name: Build workspace
        run: |
          cd ~/lododo_bot
          colcon build --packages-select bot_cmd_interface
          source install/setup.bash
      
      - name: Run tests
        run: |
          # 启动测试环境（后台）
          ros2 launch bot_bringup simulation_cmd_interface_test.launch.py &
          sleep 15
          
          # 运行测试
          cd ~/lododo_bot/src/bot_cmd_interface
          python3 test/test_integration.py --verbose
          
          # 检查退出码
          exit $?
```

---

## 下一步 / Next Steps

1. **Phase 3.2: 性能测试**
   - 吞吐量测试（100 requests/s）
   - 并发测试（10 threads）
   - 内存泄漏检测

2. **Phase 3.3: Mock终端**
   - 交互式命令行工具
   - 展示SDK使用示例
   - 用于演示和调试

3. **Phase 4: 文档完善**
   - API文档
   - 架构设计文档
   - 用户手册

---

## 相关文档 / Related Docs

- [IMPLEMENTATION_TODO.md](../IMPLEMENTATION_TODO.md) - 开发任务清单
- [MISSION_PLANNER_USAGE.md](../../bot_bringup/launch/MISSION_PLANNER_USAGE.md) - MissionPlanner使用指南
- [QUICKSTART.md](../../../QUICKSTART.md) - 项目快速开始

---

**Last Updated**: 2026-01-07  
**Status**: Phase 3.1 Complete ✅
