# 归档文件说明 / Archive Documentation

**归档日期**: 2025-12-30  
**重构版本**: Phase 4 - Unified Task Management Architecture  
**归档原因**: 完成从独立节点架构到统一任务管理架构的迁移

---

## 📦 归档目录结构

```
bak/
├── deprecated_nodes/                    # 废弃的独立节点
│   ├── exploration_mapper.py           # 旧的探索节点（1176行）
│   └── patrol_node.py                  # 旧的巡航节点（569行）
│
├── deprecated/                          # 早期废弃代码
│   ├── exploration_mapper_backup.py    # 探索节点早期备份
│   └── __init__.py
│
├── deprecated_task_handler.py           # 旧的任务处理器
├── navigation_service_handler.py        # 旧的服务处理器
├── mission_planner_backup.py.bak       # MissionPlanner早期备份
│
├── FILELIST.txt                         # 文件清单
├── README.md                            # 早期归档说明
└── ARCHIVE_README.md                    # 本文档
```

---

## 🗂️ 归档文件清单

### 1. deprecated_nodes/exploration_mapper.py
**原始位置**: `bot_navigation/exploration/exploration_mapper.py`  
**文件大小**: 1176 行  
**废弃日期**: 2025-12-30

**功能描述**:
- 独立的自主探索建图节点
- 基于 Frontier 的探索策略
- 拥有独立的 NavigationExecutor 实例
- 独立的 ROS2 节点架构

**废弃原因**:
1. ❌ **架构不统一**: 作为独立节点运行，无法与其他任务协同
2. ❌ **资源冲突**: 拥有独立的 NavigationExecutor，可能与其他任务冲突
3. ❌ **功能缺失**: 不支持任务队列、优先级、暂停/恢复等高级功能
4. ❌ **维护成本**: 与新架构功能重复，需要双重维护

**替代方案**:
- **服务入口**: `MissionPlanner` 的 `/mission/start_exploration` 服务
- **任务处理器**: `ExplorationHandler` (mission/service_handlers/handlers/exploration_handler.py)
- **策略模块**: 复用 `exploration_strategy.py`, `frontier_detector.py`, `exploration_utils.py`

**迁移示例**:
```bash
# ❌ 旧方式（已废弃）
ros2 run bot_navigation exploration_mapper

# ✅ 新方式（推荐）
ros2 service call /mission/start_exploration \
  bot_navigation_msgs/srv/StartExploration \
  "{map_name: 'office', save_map: true, max_duration: 300.0}"
```

---

### 2. deprecated_nodes/patrol_node.py
**原始位置**: `bot_navigation/patrol/patrol_node.py`  
**文件大小**: 569 行  
**废弃日期**: 2025-12-30

**功能描述**:
- 独立的路径巡航节点
- 支持 loop, ping_pong, once, random 模式
- 继承 NavigationExecutor 基类
- YAML 航点配置支持

**废弃原因**:
1. ❌ **架构不统一**: 独立节点架构，无法与探索、导航任务协同
2. ❌ **服务缺失**: 缺少标准化的服务接口（启动/暂停/恢复/取消）
3. ❌ **任务管理**: 无法通过 MissionPlanner 统一调度
4. ❌ **状态同步**: 状态管理独立，无法与任务系统集成

**替代方案**:
- **服务入口**: `MissionPlanner` 的 `/mission/start_patrol` 服务
- **任务处理器**: `PatrolHandler` (mission/service_handlers/handlers/patrol_handler.py)
- **核心逻辑**: 复用 `PatrolManager` (patrol/patrol_manager.py)

**迁移示例**:
```bash
# ❌ 旧方式（已废弃）
ros2 run bot_navigation patrol_node \
  --ros-args -p waypoint_file:=~/waypoints/route1.yaml

# ✅ 新方式（推荐）
ros2 service call /mission/start_patrol \
  bot_navigation_msgs/srv/StartPatrol \
  "{waypoint_file: '~/waypoints/route1.yaml', mode: 'loop'}"
```

---

### 3. navigation_service_handler.py
**原始位置**: `mission/service_handlers/navigation_service_handler.py`  
**废弃日期**: 2025-12-30（重构早期）

**废弃原因**:
- 服务回调功能已移至 `MissionPlanner` 内部（统一服务入口）
- `execute_navigation_task()` 方法已由 `NavigationHandler` 完全替代

**替代方案**:
- 服务处理: `MissionPlanner._handle_navigate_to_pose()`
- 任务执行: `NavigationHandler.execute()`

---

### 4. deprecated_task_handler.py
**原始位置**: `mission/service_handlers/task_execution_handler.py` (重构前)  
**废弃日期**: 2025-12-30（重构早期）

**废弃原因**:
- 旧版本的任务执行处理器
- 架构不统一，未使用基类模式
- 功能已被新的 Handler 架构完全替代

**替代方案**:
- 基类: `TaskExecutionHandler` (mission/service_handlers/task_execution_handler.py)
- 具体实现: `ExplorationHandler`, `NavigationHandler`, `PatrolHandler`

---

### 5. deprecated/exploration_mapper_backup.py
**原始位置**: 早期备份  
**说明**: `exploration_mapper.py` 在重构前的早期备份版本

---

## 🏗️ 架构变更对比

### 旧架构（已废弃）
```
独立节点模式：

exploration_mapper (独立节点)
    ├── NavigationExecutor (独立实例)
    ├── FrontierDetector
    └── ExplorationStrategy

patrol_node (独立节点)
    ├── NavigationExecutor (继承)
    └── PatrolManager

navigation_service (独立服务)
    └── execute_navigation_task()
```

**问题**:
- ❌ 多个 NavigationExecutor 实例可能冲突
- ❌ 任务之间无法协调（无队列、无优先级）
- ❌ 无统一的任务状态管理
- ❌ 服务接口不一致

---

### 新架构（当前）
```
统一任务管理模式：

MissionPlanner (统一服务入口)
    │
    ├── TaskManager (任务状态管理)
    │   └── Task Queue (WAITING_EXECUTION → RUNNING → COMPLETED)
    │
    ├── NavigationExecutor (共享资源，互斥访问)
    │
    └── TaskExecutionHandler (基类)
        ├── ExplorationHandler
        │   ├── FrontierDetector
        │   └── ExplorationStrategy
        │
        ├── PatrolHandler
        │   └── PatrolManager
        │
        └── NavigationHandler
```

**优势**:
- ✅ 单一 NavigationExecutor，资源互斥管理
- ✅ 统一任务队列，支持多任务调度
- ✅ 标准化的任务生命周期（execute/pause/resume/cancel）
- ✅ 统一的服务接口（/mission/* 命名空间）
- ✅ 可扩展架构（新增 handler 即可添加任务类型）

---

## 🔄 重命名记录

### navigation/ → mission/
**重命名日期**: 2025-12-30  
**原因**: 
- `navigation` 命名不准确，容易与 Nav2 导航混淆
- `mission` 更准确地表达"任务调度和管理"的职责
- 与 ROS 社区约定一致（mission planning）

**影响范围**:
- 目录: `bot_navigation/navigation/` → `bot_navigation/mission/`
- 导入: `from ..navigation.` → `from ..mission.`
- setup.py: `bot_navigation.navigation.` → `bot_navigation.mission.`

---

## 📋 setup.py 入口点变更

### 移除的入口点（已废弃）
```python
# ❌ Removed
'exploration_mapper = bot_navigation.exploration.exploration_mapper:main',
'patrol_node = bot_navigation.patrol.patrol_node:main',
```

### 当前入口点（推荐使用）
```python
# ✅ Core Services (Unified Task Management)
'mission_planner = bot_navigation.mission.mission_planner:main',
'waypoint_recorder = bot_navigation.mission.waypoint_recorder_node:main',

# ✅ Tool Services
'map_saver_node = bot_navigation.map.map_saver_node:main',
'map_loader_node = bot_navigation.map.map_loader_node:main',
```

---

## 🚀 迁移指南

### 1. 从 exploration_mapper 迁移

#### 启动探索任务
```bash
# ❌ 旧方式
ros2 run bot_navigation exploration_mapper \
  --ros-args \
  -p map_name:='office' \
  -p save_map:=true

# ✅ 新方式
ros2 service call /mission/start_exploration \
  bot_navigation_msgs/srv/StartExploration \
  "{map_name: 'office', save_map: true, max_duration: 600.0}"
```

#### 控制探索过程
```bash
# 暂停
ros2 service call /mission/pause_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<exploration_task_id>'}"

# 恢复
ros2 service call /mission/resume_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<exploration_task_id>'}"

# 取消
ros2 service call /mission/cancel_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<exploration_task_id>'}"
```

#### 查询任务状态
```bash
# 列出所有任务
ros2 service call /mission/list_tasks \
  bot_navigation_msgs/srv/ListTasks "{}"

# 查询特定任务
ros2 service call /mission/get_task_status \
  bot_navigation_msgs/srv/GetTaskStatus \
  "{task_id: '<task_id>'}"
```

---

### 2. 从 patrol_node 迁移

#### 启动巡航任务
```bash
# ❌ 旧方式
ros2 run bot_navigation patrol_node \
  --ros-args \
  -p waypoint_file:=~/waypoints/route1.yaml \
  -p mode:=loop

# ✅ 新方式
ros2 service call /mission/start_patrol \
  bot_navigation_msgs/srv/StartPatrol \
  "{waypoint_file: '~/waypoints/route1.yaml', mode: 'loop', loop_count: 0}"
```

#### 控制巡航过程
```bash
# 暂停巡航
ros2 service call /mission/pause_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<patrol_task_id>'}"

# 恢复巡航
ros2 service call /mission/resume_task \
  bot_navigation_msgs/srv/TaskControl \
  "{task_id: '<patrol_task_id>'}"
```

---

### 3. Launch 文件更新

#### 旧的独立节点 Launch
```python
# ❌ 已废弃
Node(
    package='bot_navigation',
    executable='exploration_mapper',
    name='exploration_mapper',
    parameters=[{'map_name': 'office'}]
)
```

#### 新的 MissionPlanner Launch
```python
# ✅ 推荐
Node(
    package='bot_navigation',
    executable='mission_planner',
    name='mission_planner',
    parameters=[mission_params_file]
)

# 然后通过服务启动探索
# ros2 service call /mission/start_exploration ...
```

---

## 🔍 代码查找指南

### 如果需要查看旧代码实现：

1. **探索功能**: `bak/deprecated_nodes/exploration_mapper.py`
   - 状态机逻辑
   - 边界检测流程
   - 旋转避障策略

2. **巡航功能**: `bak/deprecated_nodes/patrol_node.py`
   - 路点切换逻辑
   - 模式控制（loop/ping_pong/once/random）
   - 停留时间管理

3. **早期服务处理**: `bak/navigation_service_handler.py`
   - 导航服务回调
   - 目标验证逻辑

### 对应的新实现位置：

1. **探索功能**: 
   - Handler: `mission/service_handlers/handlers/exploration_handler.py`
   - 策略模块: `exploration/frontier_detector.py`, `exploration/exploration_strategy.py`

2. **巡航功能**:
   - Handler: `mission/service_handlers/handlers/patrol_handler.py`
   - 核心逻辑: `patrol/patrol_manager.py` (保留复用)

3. **任务管理**:
   - 服务入口: `mission/mission_planner.py`
   - 状态管理: `mission/task_manager.py`
   - 基类: `mission/service_handlers/task_execution_handler.py`

---

## ⚠️ 注意事项

### 1. 不要再使用旧的入口点
```bash
# ❌ 这些命令已失效（setup.py 中已移除）
ros2 run bot_navigation exploration_mapper
ros2 run bot_navigation patrol_node
```

### 2. 更新现有 Launch 文件
- 所有使用 `exploration_mapper` 或 `patrol_node` 的 launch 文件需要更新
- 参考 `launch/` 目录中的最新示例

### 3. 导入路径变更
如果有自定义代码导入这些模块：
```python
# ❌ 旧导入（已失效）
from bot_navigation.navigation.mission_planner import MissionPlanner

# ✅ 新导入（正确）
from bot_navigation.mission.mission_planner import MissionPlanner
```

### 4. 归档代码不再维护
- `bak/` 目录中的代码仅供参考，不再更新
- 不保证归档代码与当前系统兼容
- 生产环境禁止使用归档代码

---

## 📚 相关文档

- **任务管理架构**: `docs/TECHNICAL_DESIGN.md`
- **快速入门**: `docs/QUICKSTART.md`
- **测试指南**: `scripts/test_mission_integration_v2.py`
- **TODO 列表**: `TODO.md`

---

## 📞 联系信息

**开发团队**: LeKiwi Bot Development Team  
**维护者**: hurry <jeladeer@msn.com>  
**项目仓库**: https://github.com/your-repo/lododo_bot

---

*最后更新: 2025-12-30*  
*归档版本: Phase 4 - Unified Architecture*
