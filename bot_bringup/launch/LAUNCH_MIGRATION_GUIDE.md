# Launch 文件迁移指南 / Launch File Migration Guide

**更新日期**: 2025-12-30  
**原因**: Phase 4 架构统一 - 废弃独立节点，统一到 MissionPlanner

---

## 📦 归档的 Launch 文件

以下 launch 文件已移至 `deprecated/` 目录，因为它们使用已废弃的独立节点架构：

### bot_bringup/launch/deprecated/
1. **exploration_with_map_save.launch.py**
   - 使用废弃的 `exploration_mapper` 节点
   - 功能：自动探索建图并保存

2. **patrol_with_map.launch.py**
   - 间接依赖废弃的 `patrol_node`
   - 功能：加载地图并启动巡航

### bot_navigation/launch/deprecated/
3. **patrol.launch.py**
   - 使用废弃的 `patrol_node` 节点
   - 功能：启动巡航节点

4. **test_exploration.launch.py**
   - 使用废弃的 `exploration_mapper` 节点
   - 功能：测试探索功能

---

## 🆕 新的使用方式

所有任务现在统一通过 **MissionPlanner** 服务接口启动。

### 1️⃣ 启动基础环境

```bash
# 启动 Gazebo + Nav2 + RTABMap + MissionPlanner
ros2 launch bot_bringup test_mission_planner.launch.py

# 或使用自定义地图
ros2 launch bot_bringup test_mission_planner.launch.py \
  map_name:=office_floor1 \
  rtabmap_db_path:=~/lododo_bot/maps/office_floor1/rtabmap.db
```

### 2️⃣ 通过服务启动任务

#### 探索任务（替代 exploration_with_map_save.launch.py）

```bash
# 启动探索并自动保存地图
ros2 service call /mission/start_exploration \
  bot_navigation_msgs/srv/StartExploration \
  "{
    map_name: 'auto_explored_map',
    save_map: true,
    max_duration: 600.0,
    coverage_threshold: 0.9
  }"
```

**功能对比**:
| 旧方式 | 新方式 |
|--------|--------|
| ❌ Launch文件启动 | ✅ 服务调用启动 |
| ❌ 独立节点 | ✅ 统一任务管理 |
| ❌ 无任务队列 | ✅ 支持多任务排队 |
| ❌ 难以控制 | ✅ 暂停/恢复/取消 |

---

#### 巡航任务（替代 patrol_with_map.launch.py）

```bash
# 启动巡航任务
ros2 service call /mission/start_patrol \
  bot_navigation_msgs/srv/StartPatrol \
  "{
    waypoint_file: '~/lododo_bot/waypoints/route1.yaml',
    mode: 'loop',
    loop_count: 0
  }"
```

**功能对比**:
| 旧方式 | 新方式 |
|--------|--------|
| ❌ 需要单独launch文件 | ✅ 服务调用即可 |
| ❌ 参数固定 | ✅ 动态配置 |
| ❌ 无任务状态 | ✅ 实时状态查询 |
| ❌ 停止需重启 | ✅ 暂停/恢复支持 |

---

#### 导航任务

```bash
# 导航到指定位置
ros2 service call /mission/navigate_to_pose \
  bot_navigation_msgs/srv/NavigateToPose \
  "{
    x: 2.0,
    y: 3.0,
    yaw: 0.0,
    frame_id: 'map'
  }"
```

---

### 3️⃣ 任务管理命令

#### 查询所有任务
```bash
ros2 service call /mission/list_tasks \
  bot_navigation_msgs/srv/ListTasks \
  "{filter: 'all'}"
```

#### 查询任务状态
```bash
ros2 service call /mission/get_task_status \
  bot_navigation_msgs/srv/GetTaskStatus \
  "{task_id: '<task_id>'}"
```

#### 暂停任务
```bash
ros2 service call /mission/pause_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<task_id>'}"
```

#### 恢复任务
```bash
ros2 service call /mission/resume_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<task_id>'}"
```

#### 取消任务
```bash
ros2 service call /mission/cancel_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<task_id>'}"
```

#### 紧急停止所有任务
```bash
ros2 service call /mission/emergency_stop \
  bot_navigation_msgs/srv/EmergencyStop \
  "{reason: 'manual stop'}"
```

---

## 🔄 完整迁移示例

### 场景1: 探索建图

**旧方式（已废弃）**:
```bash
# ❌ 使用独立节点
ros2 launch bot_bringup exploration_with_map_save.launch.py \
  map_name:=office \
  description:="办公室地图" \
  completion_threshold:=0.8
```

**新方式（推荐）**:
```bash
# 1. 启动环境
ros2 launch bot_bringup test_mission_planner.launch.py

# 2. 在另一个终端启动探索
ros2 service call /mission/start_exploration \
  bot_navigation_msgs/srv/StartExploration \
  "{
    map_name: 'office',
    save_map: true,
    max_duration: 600.0,
    coverage_threshold: 0.8
  }"

# 3. 查看进度
ros2 service call /mission/get_task_status \
  bot_navigation_msgs/srv/GetTaskStatus \
  "{task_id: '<从返回结果获取>'}"

# 4. 如需暂停
ros2 service call /mission/pause_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<task_id>'}"
```

---

### 场景2: 巡航导航

**旧方式（已废弃）**:
```bash
# ❌ 使用独立节点 + 多个launch文件
ros2 launch bot_bringup patrol_with_map.launch.py \
  map_name:=office \
  waypoint_file:=~/waypoints/route1.yaml \
  patrol_mode:=loop \
  max_loops:=-1
```

**新方式（推荐）**:
```bash
# 1. 启动环境（带地图定位）
ros2 launch bot_bringup test_mission_planner.launch.py \
  map_name:=office \
  rtabmap_db_path:=~/lododo_bot/maps/office/rtabmap.db

# 2. 启动巡航任务
ros2 service call /mission/start_patrol \
  bot_navigation_msgs/srv/StartPatrol \
  "{
    waypoint_file: '~/lododo_bot/waypoints/route1.yaml',
    mode: 'loop',
    loop_count: 0
  }"

# 3. 实时控制
ros2 service call /mission/pause_task ...   # 暂停
ros2 service call /mission/resume_task ...  # 恢复
ros2 service call /mission/cancel_task ...  # 取消
```

---

## 🎯 新架构的优势

### 1. 统一管理
- ✅ 所有任务通过 MissionPlanner 统一调度
- ✅ 任务队列自动管理
- ✅ 资源冲突自动避免（NavigationExecutor 互斥）

### 2. 灵活控制
- ✅ 动态启动/停止任务
- ✅ 支持暂停/恢复
- ✅ 紧急停止所有任务
- ✅ 实时状态查询

### 3. 易于扩展
- ✅ 新增任务类型只需添加 Handler
- ✅ 统一的服务接口
- ✅ 标准化的任务生命周期

### 4. 更好的可测试性
- ✅ 服务接口易于自动化测试
- ✅ 任务状态可验证
- ✅ 集成测试脚本支持

---

## 📚 相关文档

- **快速入门**: `docs/QUICKSTART.md`
- **任务管理架构**: `src/bot_navigation/REFACTORING_PHASE4_REPORT.md`
- **测试指南**: `src/bot_navigation/scripts/test_mission_integration_v2.py`
- **服务接口文档**: 查看 `bot_navigation_msgs` 包

---

## ⚠️ 重要提醒

### 归档文件仅供参考
- `deprecated/` 目录中的文件不再维护
- 不保证与当前系统兼容
- **生产环境禁止使用归档代码**

### 如需使用旧代码
如果需要查看旧的实现方式：
```bash
# 查看重构前的标签
git checkout v0.1.0-pre-phase4-refactor

# 返回当前版本
git checkout main
```

---

## 🔗 服务接口速查

| 功能 | 服务名称 | 消息类型 |
|------|----------|----------|
| 创建导航任务 | `/mission/navigate_to_pose` | `NavigateToPose` |
| 创建探索任务 | `/mission/start_exploration` | `StartExploration` |
| 创建巡航任务 | `/mission/start_patrol` | `StartPatrol` |
| 列出所有任务 | `/mission/list_tasks` | `ListTasks` |
| 查询任务状态 | `/mission/get_task_status` | `GetTaskStatus` |
| 暂停任务 | `/mission/pause_task` | `TaskControl` |
| 恢复任务 | `/mission/resume_task` | `TaskControl` |
| 取消任务 | `/mission/cancel_task` | `TaskControl` |
| 紧急停止 | `/mission/emergency_stop` | `EmergencyStop` |

---

**更新时间**: 2025-12-30  
**维护者**: LeKiwi Bot Development Team  
**反馈**: 如有问题请提交 Issue
