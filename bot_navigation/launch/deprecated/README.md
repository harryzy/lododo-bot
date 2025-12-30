# 废弃的 Launch 文件 / Deprecated Launch Files

**归档日期**: 2025-12-30  
**原因**: Phase 4 架构统一 - 独立节点已废弃

---

## ⚠️ 重要警告

**本目录中的所有 launch 文件已废弃，不再维护。**

---

## 📦 归档文件清单

### 1. patrol.launch.py
**原功能**: 启动 PatrolNode 巡航节点

**使用的废弃节点**:
- `patrol_node` (bot_navigation.patrol.patrol_node)

**替代方案**:
```bash
# 使用 MissionPlanner 服务
ros2 launch bot_bringup test_mission_planner.launch.py

ros2 service call /mission/start_patrol \
  bot_navigation_msgs/srv/StartPatrol \
  "{waypoint_file: '~/waypoints/route1.yaml', mode: 'loop', loop_count: 0}"
```

---

### 2. test_exploration.launch.py
**原功能**: 测试探索建图功能

**使用的废弃节点**:
- `exploration_mapper` (bot_navigation.exploration.exploration_mapper)

**替代方案**:
```bash
# 使用 MissionPlanner 服务
ros2 launch bot_bringup test_mission_planner.launch.py

ros2 service call /mission/start_exploration \
  bot_navigation_msgs/srv/StartExploration \
  "{map_name: 'test_map', save_map: false, max_duration: 300.0}"
```

---

## 🔗 迁移指南

完整的迁移指南请参考：
**[../../bot_bringup/launch/LAUNCH_MIGRATION_GUIDE.md](../../bot_bringup/launch/LAUNCH_MIGRATION_GUIDE.md)**

---

## 📜 查看旧代码

如需查看重构前的完整代码：
```bash
cd ~/lododo_bot/src
git checkout v0.1.0-pre-phase4-refactor
```

---

**最后更新**: 2025-12-30  
**维护状态**: 已停止维护
