我# Mission Planner Launch 使用指南 / Usage Guide

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## English

### Overview

`simulation_mission_planner.launch.py` is the unified launch file for the LeKiwi robot mission planning system. It integrates:

- **Gazebo Simulation** - Configurable world environments
- **Nav2 Navigation Stack** - Path planning and obstacle avoidance
- **RTABMap SLAM/Localization** - Mapping or map-based localization
- **Mission Planner** - Task scheduling and execution management
- **RViz Visualization** - Real-time monitoring

### Architecture Layers

```
┌─────────────────────────────────────────┐
│  Launch Parameters (Static Config)      │
├─────────────────────────────────────────┤
│ • Gazebo world selection                │
│ • Simulation/Hardware mode              │
│ • RTABMap mode (SLAM/Localization)      │
│ • Initial map loading                   │
└─────────────────────────────────────────┘
              ↓ Launch
┌─────────────────────────────────────────┐
│  Mission Services (Dynamic Control)     │
├─────────────────────────────────────────┤
│ • /mission/start_patrol                 │
│ • /mission/start_exploration            │
│ • /mission/navigate_to_pose             │
│ • /mission/pause_task                   │
│ • /mission/resume_task                  │
│ • /mission/emergency_stop               │
└─────────────────────────────────────────┘
```

---

## Usage Scenarios

### Scenario 1: Autonomous Exploration & Mapping

**Objective**: Explore unknown environment and build map

**Launch Command**:
```bash
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=new_office.world \
    use_sim_time:=true \
    use_rviz:=true
```

**Note**: Launches in SLAM mode (default when no map specified)

**Runtime Commands**:
```bash
# Start autonomous exploration
ros2 service call /mission/start_exploration \
    bot_navigation_msgs/srv/StartExploration \
    "{auto_save: true, map_name: 'new_office_map', max_duration: 600.0}"

# Monitor progress in RViz
# Map will be auto-saved to ~/lododo_bot/maps/new_office_map/
```

**Key Parameters**:
- `world` - Select Gazebo environment (default: `navigation_5x5_rgbd.world`)
- `auto_save` - Auto-save map when exploration completes
- `map_name` - Name for saved map
- `max_duration` - Exploration timeout (seconds)

---

### Scenario 2: Navigation with Existing Map

**Objective**: Navigate to waypoints in known environment

**Launch Command**:
```bash
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=office.world \
    map_name:=office_floor1 \
    rtabmap_db_path:=~/lododo_bot/maps/office_floor1/rtabmap.db \
    use_sim_time:=true
```

**Note**: RTABMap loads database for localization mode

**Runtime Commands**:
```bash
# Single point navigation
ros2 service call /mission/navigate_to_pose \
    bot_navigation_msgs/srv/NavigateToPose \
    "{x: 2.5, y: 3.0, yaw: 1.57, frame_id: 'map'}"

# Check task status
ros2 service call /mission/get_task_status \
    bot_navigation_msgs/srv/GetTaskStatus \
    "{task_id: 'nav_xxxxxx'}"
```

**Key Parameters**:
- `map_name` - Map identifier from map library
- `rtabmap_db_path` - Path to RTABMap database file
- `x`, `y`, `yaw` - Target pose in map frame

---

### Scenario 3: Patrol with Waypoints

**Objective**: Patrol along predefined route

**Step 1: Record Waypoints** (if not already done)
```bash
# Launch with map for localization
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=office.world \
    map_name:=office_floor1 \
    rtabmap_db_path:=~/lododo_bot/maps/office_floor1/rtabmap.db

# Record waypoints (in separate terminal)
ros2 run bot_navigation waypoint_recorder \
    --ros-args -p persistence_dir:=~/lododo_bot/waypoints

# Use interactive CLI to save waypoints
# File will be saved as ~/lododo_bot/waypoints/<route_name>.yaml
```

**Step 2: Start Patrol**
```bash
# Start patrol with waypoint file
ros2 service call /mission/start_patrol \
    bot_navigation_msgs/srv/StartPatrol \
    "{route_file: '/home/hurry/lododo_bot/waypoints/office_route1.yaml', loop: true}"
```

**Control Commands**:
```bash
# Pause patrol
ros2 service call /mission/pause_task \
    bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'patrol_xxxxxx'}"

# Resume patrol
ros2 service call /mission/resume_task \
    bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'patrol_xxxxxx'}"

# Cancel patrol
ros2 service call /mission/cancel_task \
    bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'patrol_xxxxxx'}"
```

**Key Parameters**:
- `route_file` - Path to waypoint YAML file
- `loop` - Continuous loop mode (true/false)
- `waypoint_file` - (Launch arg) Default waypoint file for testing

---

### Scenario 4: Emergency Stop

**Objective**: Immediately stop all robot motion and tasks

**Command**:
```bash
ros2 service call /mission/emergency_stop \
    bot_navigation_msgs/srv/EmergencyStop \
    "{clear_tasks: true}"
```

**Behavior**:
- Sends zero velocity command (`cmd_vel = 0`)
- Cancels all active navigation goals
- Optionally clears task queue (`clear_tasks: true`)

---

## Launch Parameters Reference

### Environment Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `world` | string | `navigation_5x5_rgbd.world` | Gazebo world file name |
| `use_sim_time` | bool | `true` | Enable simulation time synchronization |
| `use_rviz` | bool | `true` | Launch RViz visualization |

### Map & Localization

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_name` | string | `new2_map` | Map identifier from library |
| `rtabmap_db_path` | string | `~/lododo_bot/maps/new2_map/rtabmap.db` | RTABMap database path |

**Note**: If `rtabmap_db_path` points to existing database, RTABMap launches in **localization mode**. Otherwise, **SLAM mode**.

### Mission Planner

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_rate` | float | `10.0` | Task execution check frequency (Hz) |
| `task_timeout` | float | `300.0` | Default task timeout (seconds) |
| `enable_auto_recovery` | bool | `true` | Enable automatic recovery on failure |
| `persistence_dir` | string | `(workspace)/mission` | Task persistence directory |

### Testing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `run_tests` | bool | `false` | Launch automated integration tests |
| `waypoint_file` | string | `~/lododo_bot/waypoints/new2_map.yaml` | Default waypoint file for patrol tests |

---

## Service Interfaces

### Task Creation

| Service | Message Type | Description |
|---------|-------------|-------------|
| `/mission/navigate_to_pose` | `NavigateToPose` | Single point navigation |
| `/mission/start_patrol` | `StartPatrol` | Start waypoint patrol |
| `/mission/start_exploration` | `StartExploration` | Start autonomous exploration |

### Task Control

| Service | Message Type | Description |
|---------|-------------|-------------|
| `/mission/pause_task` | `TaskControl` | Pause running task |
| `/mission/resume_task` | `TaskControl` | Resume paused task |
| `/mission/cancel_task` | `TaskControl` | Cancel task |
| `/mission/emergency_stop` | `EmergencyStop` | Emergency stop all tasks |

### Task Monitoring

| Service | Message Type | Description |
|---------|-------------|-------------|
| `/mission/list_tasks` | `ListTasks` | List all tasks (pending/running/paused) |
| `/mission/get_task_status` | `GetTaskStatus` | Query task status |
| `/mission/clear_completed_tasks` | `ClearTasks` | Clear completed/failed tasks |

---

## Best Practices

### 1. Configuration Layer Selection

- **Launch Parameters** → Environment setup (world, map, hardware/sim)
- **Service Calls** → Task logic (navigation, patrol, exploration)

### 2. Waypoint File Management

- Store waypoints in `~/lododo_bot/waypoints/`
- Use descriptive names: `office_floor1_route.yaml`
- Version control waypoint files for different maps

### 3. Map Library Organization

```
~/lododo_bot/maps/
├── office_floor1/
│   ├── rtabmap.db          # RTABMap database (primary)
│   ├── map.pgm             # Occupancy grid image
│   ├── map.yaml            # Map metadata
│   └── metadata.yaml       # Custom metadata
├── warehouse/
│   └── ...
```

### 4. Testing Workflow

```bash
# 1. Exploration phase
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=test_env.world

ros2 service call /mission/start_exploration \
    bot_navigation_msgs/srv/StartExploration \
    "{auto_save: true, map_name: 'test_env_map'}"

# 2. Record waypoints
ros2 run bot_navigation waypoint_recorder \
    --ros-args -p persistence_dir:=~/lododo_bot/waypoints

# 3. Test patrol
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=test_env.world \
    map_name:=test_env_map \
    rtabmap_db_path:=~/lododo_bot/maps/test_env_map/rtabmap.db

ros2 service call /mission/start_patrol \
    bot_navigation_msgs/srv/StartPatrol \
    "{route_file: '~/lododo_bot/waypoints/test_route.yaml', loop: true}"
```

---

## Troubleshooting

### Issue: Robot doesn't stop after pause

**Cause**: Nav2 controller may still send velocity commands after cancel

**Solution**: Fixed in latest version - `stop_robot()` continuously sends zero velocity in PAUSED state

### Issue: Second consecutive test fails

**Cause**: Async Nav2 callbacks may set NavigationExecutor to FAILED state after release

**Solution**: Fixed - `release_executor()` now force-resets state to IDLE

### Issue: Waypoint recorder CLI unresponsive

**Cause**: Launch files redirect stdin

**Solution**: Always use `ros2 run` for interactive nodes, NOT `ros2 launch`

---

## Related Documentation

- [AUTONOMOUS_PATROL_DESIGN.md](../../src/AUTONOMOUS_PATROL_DESIGN.md) - Patrol system architecture
- [TECHNICAL_DESIGN.md](../../src/TECHNICAL_DESIGN.md) - Overall system design
- [QUICKSTART.md](../../src/QUICKSTART.md) - Quick start guide
- [TODO.md](../../TODO.md) - Development roadmap

---

<a name="中文"></a>
## 中文

### 概述

`simulation_mission_planner.launch.py` 是 LeKiwi 机器人任务规划系统的统一启动文件，集成了：

- **Gazebo 仿真** - 可配置的世界环境
- **Nav2 导航栈** - 路径规划和避障
- **RTABMap SLAM/定位** - 建图或基于地图的定位
- **任务规划器** - 任务调度和执行管理
- **RViz 可视化** - 实时监控

### 架构分层

```
┌─────────────────────────────────────────┐
│  Launch 参数 (静态配置)                   │
├─────────────────────────────────────────┤
│ • Gazebo 世界选择                        │
│ • 仿真/硬件模式                          │
│ • RTABMap 模式 (SLAM/定位)              │
│ • 初始地图加载                           │
└─────────────────────────────────────────┘
              ↓ 启动
┌─────────────────────────────────────────┐
│  Mission 服务 (动态控制)                  │
├─────────────────────────────────────────┤
│ • /mission/start_patrol                 │
│ • /mission/start_exploration            │
│ • /mission/navigate_to_pose             │
│ • /mission/pause_task                   │
│ • /mission/resume_task                  │
│ • /mission/emergency_stop               │
└─────────────────────────────────────────┘
```

---

## 使用场景

### 场景 1：自主探索建图

**目标**：探索未知环境并构建地图

**启动命令**：
```bash
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=new_office.world \
    use_sim_time:=true \
    use_rviz:=true
```

**说明**：默认以 SLAM 模式启动（未指定地图时）

**运行时命令**：
```bash
# 启动自主探索
ros2 service call /mission/start_exploration \
    bot_navigation_msgs/srv/StartExploration \
    "{auto_save: true, map_name: 'new_office_map', max_duration: 600.0}"

# 在 RViz 中监控进度
# 地图将自动保存到 ~/lododo_bot/maps/new_office_map/
```

**关键参数**：
- `world` - 选择 Gazebo 环境（默认：`navigation_5x5_rgbd.world`）
- `auto_save` - 探索完成时自动保存地图
- `map_name` - 保存的地图名称
- `max_duration` - 探索超时时间（秒）

---

### 场景 2：已知地图导航

**目标**：在已知环境中导航到路点

**启动命令**：
```bash
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=office.world \
    map_name:=office_floor1 \
    rtabmap_db_path:=~/lododo_bot/maps/office_floor1/rtabmap.db \
    use_sim_time:=true
```

**说明**：RTABMap 加载数据库进入定位模式

**运行时命令**：
```bash
# 单点导航
ros2 service call /mission/navigate_to_pose \
    bot_navigation_msgs/srv/NavigateToPose \
    "{x: 2.5, y: 3.0, yaw: 1.57, frame_id: 'map'}"

# 检查任务状态
ros2 service call /mission/get_task_status \
    bot_navigation_msgs/srv/GetTaskStatus \
    "{task_id: 'nav_xxxxxx'}"
```

**关键参数**：
- `map_name` - 地图库中的地图标识
- `rtabmap_db_path` - RTABMap 数据库文件路径
- `x`, `y`, `yaw` - 地图坐标系中的目标位姿

---

### 场景 3：路点巡航

**目标**：沿预定义路线巡航

**步骤 1：录制路点**（如果尚未录制）
```bash
# 启动定位模式
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=office.world \
    map_name:=office_floor1 \
    rtabmap_db_path:=~/lododo_bot/maps/office_floor1/rtabmap.db

# 录制路点（新终端）
ros2 run bot_navigation waypoint_recorder \
    --ros-args -p persistence_dir:=~/lododo_bot/waypoints

# 使用交互式 CLI 保存路点
# 文件将保存为 ~/lododo_bot/waypoints/<路线名>.yaml
```

**步骤 2：启动巡航**
```bash
# 使用路点文件启动巡航
ros2 service call /mission/start_patrol \
    bot_navigation_msgs/srv/StartPatrol \
    "{route_file: '/home/hurry/lododo_bot/waypoints/office_route1.yaml', loop: true}"
```

**控制命令**：
```bash
# 暂停巡航
ros2 service call /mission/pause_task \
    bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'patrol_xxxxxx'}"

# 恢复巡航
ros2 service call /mission/resume_task \
    bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'patrol_xxxxxx'}"

# 取消巡航
ros2 service call /mission/cancel_task \
    bot_navigation_msgs/srv/TaskControl \
    "{task_id: 'patrol_xxxxxx'}"
```

**关键参数**：
- `route_file` - 路点 YAML 文件路径
- `loop` - 循环模式（true/false）
- `waypoint_file` - （启动参数）测试用的默认路点文件

---

### 场景 4：紧急停止

**目标**：立即停止所有机器人运动和任务

**命令**：
```bash
ros2 service call /mission/emergency_stop \
    bot_navigation_msgs/srv/EmergencyStop \
    "{clear_tasks: true}"
```

**行为**：
- 发送零速度命令（`cmd_vel = 0`）
- 取消所有活动的导航目标
- 可选清空任务队列（`clear_tasks: true`）

---

## Launch 参数参考

### 环境配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `world` | string | `navigation_5x5_rgbd.world` | Gazebo 世界文件名 |
| `use_sim_time` | bool | `true` | 启用仿真时间同步 |
| `use_rviz` | bool | `true` | 启动 RViz 可视化 |

### 地图与定位

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `map_name` | string | `new2_map` | 地图库中的地图标识 |
| `rtabmap_db_path` | string | `~/lododo_bot/maps/new2_map/rtabmap.db` | RTABMap 数据库路径 |

**注意**：如果 `rtabmap_db_path` 指向已存在的数据库，RTABMap 以**定位模式**启动，否则为 **SLAM 模式**。

### 任务规划器

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `update_rate` | float | `10.0` | 任务执行检查频率（Hz）|
| `task_timeout` | float | `300.0` | 默认任务超时时间（秒）|
| `enable_auto_recovery` | bool | `true` | 启用失败自动恢复 |
| `persistence_dir` | string | `(工作空间)/mission` | 任务持久化目录 |

### 测试

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `run_tests` | bool | `false` | 启动自动化集成测试 |
| `waypoint_file` | string | `~/lododo_bot/waypoints/new2_map.yaml` | 巡航测试的默认路点文件 |

---

## 服务接口

### 任务创建

| 服务 | 消息类型 | 说明 |
|------|---------|------|
| `/mission/navigate_to_pose` | `NavigateToPose` | 单点导航 |
| `/mission/start_patrol` | `StartPatrol` | 启动路点巡航 |
| `/mission/start_exploration` | `StartExploration` | 启动自主探索 |

### 任务控制

| 服务 | 消息类型 | 说明 |
|------|---------|------|
| `/mission/pause_task` | `TaskControl` | 暂停运行中的任务 |
| `/mission/resume_task` | `TaskControl` | 恢复暂停的任务 |
| `/mission/cancel_task` | `TaskControl` | 取消任务 |
| `/mission/emergency_stop` | `EmergencyStop` | 紧急停止所有任务 |

### 任务监控

| 服务 | 消息类型 | 说明 |
|------|---------|------|
| `/mission/list_tasks` | `ListTasks` | 列出所有任务（待执行/运行中/暂停）|
| `/mission/get_task_status` | `GetTaskStatus` | 查询任务状态 |
| `/mission/clear_completed_tasks` | `ClearTasks` | 清除已完成/失败的任务 |

---

## 最佳实践

### 1. 配置层选择原则

- **Launch 参数** → 环境设置（世界、地图、硬件/仿真）
- **服务调用** → 任务逻辑（导航、巡航、探索）

### 2. 路点文件管理

- 存储在 `~/lododo_bot/waypoints/`
- 使用描述性名称：`office_floor1_route.yaml`
- 为不同地图的路点文件进行版本控制

### 3. 地图库组织

```
~/lododo_bot/maps/
├── office_floor1/
│   ├── rtabmap.db          # RTABMap 数据库（主要）
│   ├── map.pgm             # 占用栅格图像
│   ├── map.yaml            # 地图元数据
│   └── metadata.yaml       # 自定义元数据
├── warehouse/
│   └── ...
```

### 4. 测试工作流程

```bash
# 1. 探索阶段
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=test_env.world

ros2 service call /mission/start_exploration \
    bot_navigation_msgs/srv/StartExploration \
    "{auto_save: true, map_name: 'test_env_map'}"

# 2. 录制路点
ros2 run bot_navigation waypoint_recorder \
    --ros-args -p persistence_dir:=~/lododo_bot/waypoints

# 3. 测试巡航
ros2 launch bot_bringup simulation_mission_planner.launch.py \
    world:=test_env.world \
    map_name:=test_env_map \
    rtabmap_db_path:=~/lododo_bot/maps/test_env_map/rtabmap.db

ros2 service call /mission/start_patrol \
    bot_navigation_msgs/srv/StartPatrol \
    "{route_file: '~/lododo_bot/waypoints/test_route.yaml', loop: true}"
```

---

## 故障排查

### 问题：暂停后机器人不停止

**原因**：Nav2 控制器在取消后可能仍发送速度命令

**解决**：最新版本已修复 - 在 PAUSED 状态时 `stop_robot()` 持续发送零速度

### 问题：连续第二次测试失败

**原因**：异步 Nav2 回调可能在释放后将 NavigationExecutor 设为 FAILED 状态

**解决**：已修复 - `release_executor()` 现在强制重置状态为 IDLE

### 问题：路点录制器 CLI 无响应

**原因**：Launch 文件重定向 stdin

**解决**：交互式节点始终使用 `ros2 run`，而非 `ros2 launch`

---

## 相关文档

- [AUTONOMOUS_PATROL_DESIGN.md](../../src/AUTONOMOUS_PATROL_DESIGN.md) - 巡航系统架构
- [TECHNICAL_DESIGN.md](../../src/TECHNICAL_DESIGN.md) - 整体系统设计
- [QUICKSTART.md](../../src/QUICKSTART.md) - 快速启动指南
- [TODO.md](../../TODO.md) - 开发路线图

---

**最后更新 / Last Updated**: 2025-12-31  
**版本 / Version**: 1.0  
**维护者 / Maintainer**: LeKiwi Bot Development Team
