# CMD Terminal - Interactive Command Interface

交互式命令终端，提供友好的命令行界面来控制LeKiwi机器人。

## Features / 功能特性

- ✅ **Interactive CLI**: 交互式命令行界面
- ✅ **Auto-completion**: Tab键自动补全命令
- ✅ **Command History**: 上下键浏览历史命令
- ✅ **Colored Output**: 彩色输出，状态一目了然
- ✅ **Real-time Feedback**: 实时显示请求响应状态
- ✅ **Help System**: 内置帮助系统

---

## Quick Start / 快速开始

### 1. Start Test Environment (Terminal 1)

```bash
cd ~/lododo_bot
source install/setup.bash
./src/bot_cmd_interface/scripts/start_cmd_terminal.sh --launch
```

或手动启动：
```bash
ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
  slam:=false \
  gui:=false \
  map_name:=exploration_test
```

### 2. Start CMD Terminal (Terminal 2)

```bash
cd ~/lododo_bot
source install/setup.bash
./src/bot_cmd_interface/scripts/start_cmd_terminal.sh
```

或直接运行：
```bash
cd ~/lododo_bot/src/bot_cmd_interface/test
python3 cmd_terminal.py
```

---

## Available Commands / 可用命令

### Navigation Commands (导航命令)

#### `nav` / `navigate` / `goto`
导航到指定位置

**Syntax**:
```
nav <x> <y> [yaw]
```

**Examples**:
```
nav 2.0 3.0           # 导航到 (2.0, 3.0)，默认朝向0
nav 1.5 2.5 1.57      # 导航到 (1.5, 2.5)，朝向90度
goto -1.0 1.0         # 使用别名命令
```

**Response**:
```
➜ Sending navigation request to (2.0, 3.0, 0.0)...
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    task_id: nav_20260108_123456
    Navigation started
```

---

#### `explore`
开始自主探索建图

**Syntax**:
```
explore [map_name]
```

**Examples**:
```
explore                  # 使用时间戳作为地图名
explore office_floor1    # 指定地图名
```

**Response**:
```
➜ Starting exploration (map: office_floor1)...
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    task_id: exploration_20260108_123456
    Exploration task started
```

---

#### `patrol`
沿路径点巡航

**Syntax**:
```
patrol <waypoint_file> [mode]
```

**Modes**:
- `once`: 单次巡航
- `loop`: 循环巡航（默认）
- `pingpong`: 往返巡航

**Examples**:
```
patrol route1.yaml              # 循环巡航
patrol route1.yaml once         # 单次巡航
patrol route1.yaml pingpong     # 往返巡航
```

---

#### `stop` / `emergency`
紧急停止

**Syntax**:
```
stop [clear]
```

**Examples**:
```
stop          # 紧急停止当前任务
stop clear    # 紧急停止并清空任务队列
```

**Response**:
```
➜ EMERGENCY STOP!
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    Emergency stop executed (HARD_STOP)
    Cancelled 1 active tasks, cleared 1 from queue
```

---

### Task Control Commands (任务控制)

#### `pause`
暂停任务

**Syntax**:
```
pause <task_id>
```

**Example**:
```
pause nav_20260108_123456
```

---

#### `resume`
恢复任务

**Syntax**:
```
resume <task_id>
```

**Example**:
```
resume nav_20260108_123456
```

---

#### `cancel`
取消任务

**Syntax**:
```
cancel <task_id>
```

**Example**:
```
cancel nav_20260108_123456
```

---

### Query Commands (查询命令)

#### `status`
查询机器人状态

**Syntax**:
```
status
```

**Response**:
```
➜ Querying robot status...
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    battery: 95.5
    position: (1.2, 3.4, 0.5)
    velocity: (0.1, 0.0, 0.0)
```

---

#### `task`
查询任务状态

**Syntax**:
```
task <task_id>
```

**Example**:
```
task nav_20260108_123456
```

**Response**:
```
➜ Querying task status: nav_20260108_123456...
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    task_id: nav_20260108_123456
    task_type: NAVIGATE_TO_POSE
    state: RUNNING
    progress: 0.65
```

---

#### `stats`
显示统计信息

**Syntax**:
```
stats
```

**Output**:
```
==================================================
  Statistics / 统计信息
==================================================
Requests Sent:     15
Responses Received: 45
Completed:         14
Failed:            1
Pending:           0

Success Rate:      93.3%
```

---

### Utility Commands (实用工具)

#### `help`
显示帮助信息

**Syntax**:
```
help              # 显示所有命令
help <command>    # 显示特定命令的帮助
```

**Examples**:
```
help              # 显示所有可用命令
help nav          # 显示nav命令的详细用法
help explore      # 显示explore命令的详细用法
```

---

#### `history`
显示命令历史

**Syntax**:
```
history [n]
```

**Examples**:
```
history        # 显示最近10条命令
history 20     # 显示最近20条命令
```

---

#### `clear`
清空屏幕

**Syntax**:
```
clear
```

---

#### `exit` / `quit`
退出终端

**Syntax**:
```
exit
quit
```

---

## Tips & Tricks / 使用技巧

### 1. Tab Completion (Tab补全)
按Tab键自动补全命令：
```
cmd> na<Tab>       →  cmd> navigate
cmd> ex<Tab>       →  cmd> explore
```

### 2. Command History (命令历史)
- **↑** (上箭头): 浏览上一条命令
- **↓** (下箭头): 浏览下一条命令
- 命令历史保存在 `~/.cmd_terminal_history`

### 3. Keyboard Shortcuts (键盘快捷键)
- **Ctrl+C**: 取消当前输入（不退出）
- **Ctrl+D**: 退出终端
- **Ctrl+L**: 清屏（同clear命令）
- **Ctrl+A**: 光标移到行首
- **Ctrl+E**: 光标移到行尾

### 4. Command Aliases (命令别名)
某些命令有多个别名：
- `nav` = `navigate` = `goto`
- `stop` = `emergency`
- `exit` = `quit`

### 5. Quick Navigation (快速导航)
使用简短参数快速导航：
```
cmd> nav 0 0          # 返回原点
cmd> nav 1 1          # 快速到达 (1,1)
cmd> nav 2 0 1.57     # 带朝向导航
```

---

## Example Session / 示例会话

```
============================================================
  LeKiwi Robot - Unified Command Interface
  统一命令接口 - 交互式终端
============================================================

Connected to CommandAdapter
Type 'help' for available commands
Type 'exit' or 'quit' to exit

cmd> help
==================================================
  Available Commands / 可用命令
==================================================

Navigation:
  nav, goto       - Navigate to position (导航)
  explore         - Start exploration (探索)
  patrol          - Start patrol (巡航)
  stop, emergency - Emergency stop (紧急停止)

Task Control:
  pause           - Pause task (暂停)
  resume          - Resume task (恢复)
  cancel          - Cancel task (取消)

Query:
  status          - Robot status (状态)
  task            - Task status (任务状态)
  stats           - Statistics (统计)

Utility:
  help            - Show help (帮助)
  history         - Show command history (历史)
  clear           - Clear screen (清屏)
  exit, quit      - Exit terminal (退出)

Tip: Use Tab for auto-completion
Tip: Type 'help <command>' for detailed usage

cmd> nav 2 3
➜ Sending navigation request to (2.0, 3.0, 0.0)...
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    task_id: nav_20260108_110245
    Navigation started

cmd> task nav_20260108_110245
➜ Querying task status: nav_20260108_110245...
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    task_id: nav_20260108_110245
    task_type: NAVIGATE_TO_POSE
    state: RUNNING
    progress: 0.35

cmd> stop
➜ EMERGENCY STOP!
  ⟳ Queued
  ⟳ Executing
  ✓ Completed
    Emergency stop executed

cmd> stats
==================================================
  Statistics / 统计信息
==================================================
Requests Sent:     3
Responses Received: 9
Completed:         3
Failed:            0
Pending:           0

Success Rate:      100.0%

cmd> exit
Goodbye! 再见！
```

---

## Troubleshooting / 故障排除

### Problem: "CommandAdapter not found"

**Solution**:
确保测试环境已启动：
```bash
ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
  slam:=false map_name:=exploration_test
```

验证节点运行：
```bash
ros2 node list | grep command_adapter
```

---

### Problem: Commands not responding

**Solution**:
1. 检查CommandAdapter日志
2. 检查网络连接
3. 尝试简单命令测试：`stats`

---

### Problem: Tab completion not working

**Solution**:
确保安装了readline：
```bash
pip3 install readline --user
```

---

## Advanced Usage / 高级用法

### Scripted Commands (脚本化命令)

可以通过管道输入命令：
```bash
echo -e "nav 1 1\nstats\nexit" | python3 cmd_terminal.py
```

### Batch Operations (批量操作)

创建命令文件 `commands.txt`:
```
nav 1 1
task nav_20260108_123456
nav 2 2
stats
exit
```

执行：
```bash
cat commands.txt | python3 cmd_terminal.py
```

---

## Architecture / 架构说明

CMD Terminal的工作原理：

```
[User Input]
     ↓
[Command Parser]
     ↓
[Command Handler]
     ↓
[ROS2 Topic: /cmd/request]
     ↓
[CommandAdapter]
     ↓
[ServiceAdapter → MissionPlanner]
     ↓
[ROS2 Topic: /cmd/response]
     ↓
[Response Handler]
     ↓
[Display to User]
```

- **异步处理**: 后台线程处理ROS2消息
- **响应跟踪**: 自动关联request_id和响应
- **状态显示**: 实时显示queued → executing → completed流程

---

## Related Documentation / 相关文档

- [ARCHITECTURE.md](../ARCHITECTURE.md) - 系统架构设计
- [API_REFERENCE.md](../API_REFERENCE.md) - API接口文档
- [QUICKSTART.md](../QUICKSTART.md) - 快速入门指南

---

**Version**: 1.0.0  
**Last Updated**: 2026-01-08  
**Author**: LeKiwi Bot Development Team
