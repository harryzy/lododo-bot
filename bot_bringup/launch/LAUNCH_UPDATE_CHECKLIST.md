# Launch 文件更新清单

**更新日期**: 2025-12-30  
**原因**: 架构重构 - 废弃独立节点，统一到 MissionPlanner

---

## 🔍 检查结果

### ✅ 无需更新的文件

1. **test_mission_planner.launch.py** ✅
   - 已经使用 `mission_planner` 节点
   - 所有服务接口正确
   - 无废弃节点引用

2. **test_waypoint_recorder.launch.py** ✅
   - 仅提供基础环境
   - 提示用户单独运行 `ros2 run bot_navigation waypoint_recorder`
   - waypoint_recorder 路径已更新到 waypoint/ 模块

3. **simple_simulation_*.launch.py** ✅
   - 基础环境启动文件
   - 不包含任务节点

---

## ⚠️ 需要更新的文件

### 1. exploration_with_map_save.launch.py ❌
**位置**: `bot_bringup/launch/exploration_with_map_save.launch.py`

**问题**:
- 第113行: `executable='exploration_mapper'` - 使用废弃节点
- 该launch文件整体基于独立节点架构

**解决方案**: 
- **选项A（推荐）**: 废弃该文件，迁移到归档
- **选项B**: 重写为基于 MissionPlanner 的版本

**理由**: 
- 探索任务应通过 `/mission/start_exploration` 服务启动
- 独立节点已废弃
- 保留该文件会误导用户

---

### 2. bot_navigation/launch/patrol.launch.py ❌
**位置**: `bot_navigation/launch/patrol.launch.py`

**问题**:
- 第63行: `executable='patrol_node'` - 使用废弃节点
- 启动独立的 patrol_node

**解决方案**:
- **选项A（推荐）**: 废弃该文件，迁移到归档
- **选项B**: 重写为基于 MissionPlanner 的版本

**理由**:
- 巡航任务应通过 `/mission/start_patrol` 服务启动
- 独立节点已废弃
- PatrolManager 现在由 PatrolHandler 调用

---

### 3. bot_navigation/launch/test_exploration.launch.py ❌
**位置**: `bot_navigation/launch/test_exploration.launch.py`

**问题**:
- 第96行: `executable='exploration_mapper'` - 使用废弃节点

**解决方案**:
- **选项A（推荐）**: 废弃该文件，迁移到归档
- **选项B**: 重写为测试 MissionPlanner 探索服务的版本

---

### 4. patrol_with_map.launch.py ⚠️
**位置**: `bot_bringup/launch/patrol_with_map.launch.py`

**问题**:
- 该文件通过 IncludeLaunchDescription 引用 `patrol.launch.py`
- 间接依赖废弃的 patrol_node

**解决方案**:
- 移除对 patrol.launch.py 的引用
- 改为启动 mission_planner + 服务调用示例

---

## 📋 推荐处理方案

### 方案A: 激进清理（推荐）

将所有使用废弃节点的 launch 文件归档：

```bash
# 创建归档目录
mkdir -p src/bot_bringup/launch/deprecated
mkdir -p src/bot_navigation/launch/deprecated

# 归档文件
mv src/bot_bringup/launch/exploration_with_map_save.launch.py \
   src/bot_bringup/launch/deprecated/

mv src/bot_bringup/launch/patrol_with_map.launch.py \
   src/bot_bringup/launch/deprecated/

mv src/bot_navigation/launch/patrol.launch.py \
   src/bot_navigation/launch/deprecated/

mv src/bot_navigation/launch/test_exploration.launch.py \
   src/bot_navigation/launch/deprecated/
```

**创建替代文档**: `LAUNCH_MIGRATION.md`

---

### 方案B: 重写为 MissionPlanner 版本

创建新的 launch 文件使用 MissionPlanner：

#### 新文件: mission_exploration.launch.py
```python
# 启动环境 + MissionPlanner
# 提供服务调用示例启动探索
```

#### 新文件: mission_patrol.launch.py
```python
# 启动环境 + MissionPlanner
# 提供服务调用示例启动巡航
```

**优点**: 
- 提供迁移示例
- 用户友好

**缺点**:
- 增加维护成本
- 与推荐的服务调用方式不一致

---

## 🎯 最终建议

### 立即执行（方案A）:

1. ✅ **归档废弃 launch 文件**
   - exploration_with_map_save.launch.py
   - patrol_with_map.launch.py
   - patrol.launch.py
   - test_exploration.launch.py

2. ✅ **创建迁移文档**
   - LAUNCH_MIGRATION.md - 说明如何使用 MissionPlanner 替代

3. ✅ **更新文档引用**
   - 更新 QUICKSTART.md 和 README.md
   - 移除对废弃 launch 文件的引用

---

## 📝 迁移指南速查

### 旧方式（已废弃）:
```bash
# ❌ 探索
ros2 launch bot_bringup exploration_with_map_save.launch.py

# ❌ 巡航
ros2 launch bot_bringup patrol_with_map.launch.py
```

### 新方式（推荐）:
```bash
# ✅ 启动基础环境 + MissionPlanner
ros2 launch bot_bringup test_mission_planner.launch.py

# ✅ 在另一个终端调用探索服务
ros2 service call /mission/start_exploration \
  bot_navigation_msgs/srv/StartExploration \
  "{map_name: 'office', save_map: true, max_duration: 600.0}"

# ✅ 调用巡航服务
ros2 service call /mission/start_patrol \
  bot_navigation_msgs/srv/StartPatrol \
  "{waypoint_file: '~/waypoints/route1.yaml', mode: 'loop'}"
```

---

## ✅ 验证清单

归档完成后验证：
- [ ] 归档的 launch 文件已移动到 deprecated/ 目录
- [ ] 创建了 LAUNCH_MIGRATION.md 迁移文档
- [ ] test_mission_planner.launch.py 正常工作
- [ ] 更新了用户文档（README、QUICKSTART）
- [ ] 移除了对废弃 launch 文件的所有引用

---

**是否立即执行方案A？**
