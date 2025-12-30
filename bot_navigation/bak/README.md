# 备份和废弃文件说明

**归档日期**: 2025-12-30  
**重构版本**: Task Execution Architecture Refactoring  

---

## 📦 归档文件清单

### 1. navigation_service_handler.py
**原始位置**: `src/bot_navigation/bot_navigation/navigation/service_handlers/navigation_service_handler.py`

**废弃原因**: 
- 服务回调功能已移至 MissionPlanner 内部（统一服务入口）
- `execute_navigation_task()` 方法已由 NavigationHandler 完全替代
- 避免服务处理逻辑分散，统一架构

**替代方案**:
- 服务处理: `MissionPlanner._handle_navigate_to_pose()`
- 任务执行: `NavigationHandler.execute()`

**迁移位置**: `handlers/navigation_handler.py`

---

### 2. deprecated_task_handler.py
**原始位置**: `src/bot_navigation/bot_navigation/navigation/service_handlers/deprecated_task_handler.py`

**废弃原因**:
- 旧版本的任务执行处理器
- 架构不统一，未使用基类模式
- 功能已被新的 Handler 架构完全替代

**替代方案**:
- 基类: `TaskExecutionHandler` (service_handlers/task_execution_handler.py)
- 具体实现: `ExplorationHandler`, `NavigationHandler`, `PatrolHandler`

**原文件名**: `task_execution_handler.py` (在重构前被重命名为 deprecated_task_handler.py)

---

### 3. deprecated/exploration_mapper_backup.py
**原始位置**: `src/bot_navigation/bot_navigation/exploration/exploration_mapper.py`

**废弃原因**:
- 独立节点架构，无法与其他任务统一调度
- 拥有独立的 NavigationExecutor，可能与其他任务冲突
- 未集成到任务管理系统

**替代方案**:
- 新实现: `handlers/exploration_handler.py`
- 通过 MissionPlanner 统一管理
- 共享 NavigationExecutor，避免资源冲突

**核心功能保留**:
- 边界检测 (FrontierDetector)
- 探索策略 (ExplorationStrategy)
- 旋转控制 (RotationController)
- 安全管理 (SafetyManager)

**备份说明**: 
- 完整实现已备份，可用于参考
- 包含 1176 行完整代码
- 支持智能探索、安全检测等高级功能

---

### 4. deprecated/__init__.py
**原始位置**: `src/bot_navigation/bot_navigation/deprecated/__init__.py`

**说明**: 废弃模块的标记文件

---

## 🔄 架构变更对比

### 旧架构（已废弃）
```
exploration_mapper (独立节点)
  └─ 独立的 NavigationExecutor

navigation_service_handler
  └─ handle_navigate_to_pose()
  └─ execute_navigation_task()

deprecated_task_handler
  └─ handle_start_exploration()
  └─ execute_exploration_task()
```

### 新架构（当前）
```
MissionPlanner (统一调度中心)
├─ 统一服务入口
│  ├─ _handle_navigate_to_pose()
│  ├─ _handle_start_exploration()
│  └─ _handle_emergency_stop()
│
├─ TaskManager (任务队列管理)
├─ NavigationExecutor (共享资源)
│
└─ Task Handlers (任务执行)
   ├─ NavigationHandler
   ├─ PatrolHandler
   └─ ExplorationHandler
```

---

## 📝 重构要点

1. **统一入口**: 所有任务服务通过 MissionPlanner 统一管理
2. **资源互斥**: 共享 NavigationExecutor，使用 WAITING_EXECUTION 状态排队
3. **基类模式**: TaskExecutionHandler 定义统一接口
4. **清晰分工**: 
   - MissionPlanner: 服务入口 + 任务调度
   - Handlers: 任务执行逻辑
   - Tools: 独立工具功能（如 WaypointTools）

---

## 🔍 如何查找旧代码

如果需要参考旧实现的代码，请查看此目录：
- `bak/navigation_service_handler.py` - 旧的导航服务处理器
- `bak/deprecated_task_handler.py` - 旧的任务执行处理器
- `bak/deprecated/exploration_mapper_backup.py` - 完整的独立探索节点

---

## ⚠️ 注意事项

1. **不要直接使用备份文件**: 这些文件仅供参考，不应在生产环境中使用
2. **依赖关系已变更**: 旧文件的导入路径可能不再有效
3. **测试已更新**: 测试脚本已适配新架构，不兼容旧实现

---

## 📚 相关文档

- 架构设计: `docs/TASK_EXECUTION_ARCHITECTURE_REFACTOR.md`
- 快速开始: `src/QUICKSTART.md`
- TODO 列表: `TODO.md`

---

**维护者**: LeKiwi Bot Development Team  
**联系方式**: 参见项目 README.md
