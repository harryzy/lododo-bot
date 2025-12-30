# 废弃的 Launch 文件 / Deprecated Launch Files

**归档日期**: 2025-12-30  
**原因**: Phase 4 架构统一 - 独立节点已废弃，launch文件命名规范化

---

## ⚠️ 重要警告

**本目录中的所有 launch 文件已废弃，不再维护。**

- ❌ 这些文件使用已废弃的独立节点或旧的命名约定
- ❌ 不保证与当前系统兼容
- ❌ 生产环境禁止使用
- ✅ 仅供参考旧实现方式

---

## 📦 归档文件清单

### 1. exploration_with_map_save.launch.py
**原功能**: 自动探索建图并保存到地图库

**废弃原因**: 使用独立的 `exploration_mapper` 节点

**替代方案**: 
```bash
# 使用 MissionPlanner 服务
ros2 launch bot_bringup simulation_mission_planner.launch.py

ros2 service call /mission/start_exploration \
  bot_navigation_msgs/srv/StartExploration \
  "{map_name: 'office', save_map: true, max_duration: 600.0}"
```

---

### 2. patrol_with_map.launch.py
**原功能**: 加载地图并启动巡航导航

**废弃原因**: 使用独立的 `patrol_node` 节点

**替代方案**:
```bash
# 使用 MissionPlanner 服务
ros2 launch bot_bringup simulation_mission_planner.launch.py \
  rtabmap_db_path:=~/lododo_bot/maps/office/rtabmap.db

ros2 service call /mission/start_patrol \
  bot_navigation_msgs/srv/StartPatrol \
  "{waypoint_file: '~/waypoints/route1.yaml', mode: 'loop'}"
```

---

### 3. test_waypoint_recorder.launch.py
**原功能**: 测试航点录制器（CLI模式环境准备）

**废弃原因**: 
- 命名不规范（不符合 `simulation_*` 约定）
- 功能描述不清晰（CLI vs Service模式混淆）

**替代方案**:
```bash
# 交互式CLI录制（手动操作）
./src/bot_bringup/scripts/start_waypoint_recorder_cli.sh

# Service模式（程序化控制）
ros2 launch bot_bringup simulation_waypoint_recorder.launch.py
```

---

### 4. simulation_gazebo.launch.py
**原功能**: 仅启动Gazebo仿真环境

**废弃原因**: 功能被更清晰命名的launch文件替代

**替代方案**:
```bash
# 基础仿真（Gazebo + 机器人spawn）
ros2 launch bot_bringup simple_simulation_gazebo.launch.py

# 完整系统（仿真 + 定位 + 导航 + 任务管理）
ros2 launch bot_bringup simulation_mission_planner.launch.py
```

---

### 5. simple_simulation_nav2_rtabmap.launch.py
**原功能**: 仿真 + RTABMap SLAM + Nav2导航（建图模式）

**废弃原因**: 功能被更简洁的launch文件替代

**替代方案**:
```bash
# 建图模式（SLAM）
ros2 launch bot_bringup simulation_nav2_rtabmap.launch.py

# 定位模式（纯定位，不建图）
ros2 launch bot_bringup simple_simulation_nav2_rtabmap_localization.launch.py

# 完整系统（包含任务管理）
ros2 launch bot_bringup simulation_mission_planner.launch.py
```

---

## 🔗 迁移指南

完整的迁移指南请参考：
**[../LAUNCH_MIGRATION_GUIDE.md](../LAUNCH_MIGRATION_GUIDE.md)**

---

## 📜 查看旧代码

如需查看重构前的完整代码（包括这些 launch 文件正常工作的版本）：

```bash
# 切换到重构前的标签
cd ~/lododo_bot/src
git checkout v0.1.0-pre-phase4-refactor

# 返回当前版本
git checkout main
```

---

## ❓ 为什么废弃

### 旧架构的问题:
1. ❌ **双轨并存**: 独立节点 + MissionPlanner 并存，用户混淆
2. ❌ **资源冲突**: 多个 NavigationExecutor 实例可能冲突
3. ❌ **功能缺失**: 独立节点不支持任务队列、暂停/恢复
4. ❌ **维护成本**: 需要同时维护两套实现
5. ❌ **命名不规范**: test_* 和 simulation_* 混用，功能不清晰

### 新架构的优势:
1. ✅ **统一入口**: 所有任务通过 MissionPlanner 服务启动
2. ✅ **资源管理**: 共享 NavigationExecutor，互斥访问
3. ✅ **任务队列**: 自动排队和调度
4. ✅ **灵活控制**: 支持暂停/恢复/取消
5. ✅ **易扩展**: 新增任务类型只需添加 Handler
6. ✅ **命名规范**: `simulation_*` 前缀表示仿真环境文件

---

## 📊 Launch文件组织结构

### 当前活跃文件（Active）
```
bot_bringup/launch/
├── simple_simulation_gazebo.launch.py          # 基础仿真
├── simulation_nav2_rtabmap.launch.py           # 建图模式
├── simple_simulation_nav2_rtabmap_localization.launch.py  # 定位模式
├── simulation_mission_planner.launch.py        # 完整系统（推荐）
├── simulation_waypoint_recorder.launch.py      # 航点录制（Service模式）
└── scripts/
    └── start_waypoint_recorder_cli.sh          # 航点录制（CLI模式）
```

### 归档文件（Deprecated）
```
bot_bringup/launch/deprecated/
├── exploration_with_map_save.launch.py         # 旧探索节点
├── patrol_with_map.launch.py                   # 旧巡航节点
├── test_waypoint_recorder.launch.py            # 旧测试文件
├── simulation_gazebo.launch.py                 # 被simple_simulation_gazebo替代
└── simple_simulation_nav2_rtabmap.launch.py    # 被simulation_nav2_rtabmap替代
```

---

**最后更新**: 2025-12-30  
**维护状态**: 已停止维护
