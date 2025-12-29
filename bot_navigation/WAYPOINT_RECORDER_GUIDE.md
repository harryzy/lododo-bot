# WaypointRecorder 使用指南

## 功能概述

WaypointRecorder 是一个交互式路点录制工具，用于：
- 手动录制机器人当前位置
- 自动录制机器人行驶轨迹
- 编辑和管理路点
- 保存和加载路点文件
- 在 RViz 中可视化路点

## 快速开始

### 1. 启动路点录制器

```bash
cd ~/lododo_bot
source install/setup.bash

# 方法1: 直接运行（推荐 - 交互式）
ros2 run bot_navigation waypoint_recorder

# 方法2: 使用 launch 文件（在独立终端）
ros2 launch bot_navigation waypoint_recorder.launch.py
```

**注意**: 
- 启动后会显示交互式菜单
- **需要手动输入命令**才会执行操作
- 不会自动开始录制，必须输入 `a` 启动自动录制或 `r` 手动录制

### 2. 命令列表

启动后会看到交互式菜单：

```
==================================================
LeKiwi 路点录制器 / Waypoint Recorder
==================================================
命令 / Commands:
  r - 录制当前位置 / Record current position
  a - 开始自动录制 / Start auto recording
  t - 停止自动录制 / Stop auto recording
  d - 删除最后一个路点 / Delete last waypoint
  c - 清空所有路点 / Clear all waypoints
  l - 列出所有路点 / List all waypoints
  p - 显示当前位姿 / Show current pose
  s - 保存路点到文件 / Save waypoints
  o - 从文件加载路点 / Load waypoints
  f - 列出已保存文件 / List saved files
  h - 显示帮助 / Show help
  q - 退出 / Quit
==================================================
```

## 使用场景

### 场景1: 手动录制巡航路径

1. 启动录制器
2. 使用键盘或手柄控制机器人移动到第一个路点
3. **输入 `r` 并按回车**录制当前位置
4. 继续移动到下一个路点
5. 重复步骤 3-4
6. 输入 `l` 查看所有路点
7. 输入 `s` 保存，命名为 `patrol_route1`

```bash
[0 waypoints] > r  ← 输入命令后按回车
✓ Manually recorded waypoint #1

[1 waypoints] > r  ← 再次输入并回车
✓ Manually recorded waypoint #2

[2 waypoints] > l  ← 查看路点列表
===== Waypoints (2) =====
  [1] x=1.230, y=2.450, yaw=0.000 rad
  [2] x=3.120, y=1.890, yaw=1.570 rad
==============================

[2 waypoints] > s  ← 保存
输入文件名 (不含扩展名): patrol_route1
✓ Saved waypoints to: patrol_route1.yaml
```

### 场景2: 自动录制行驶轨迹

1. 启动录制器
2. **输入 `a` 并回车**开始自动录制
3. 遥控机器人行驶（会自动按间隔录制）
4. **输入 `t` 并回车**停止自动录制
5. 输入 `s` 保存路点

```bash
[0 waypoints] > a  ← 输入命令启动自动录制
Started recording (interval=1.0s, min_distance=0.5m)

[0 waypoints] >    ← 机器人移动时自动录制
✓ Recorded waypoint #1
✓ Recorded waypoint #2
✓ Recorded waypoint #3
...

[15 waypoints] > t  ← 停止录制
Stopped recording. Total waypoints: 15

[15 waypoints] > s  ← 保存
输入文件名 (不含扩展名): exploration_route
✓ Saved waypoints to: exploration_route.yaml
```

### 场景3: 编辑路点

```bash
# 加载已有路点
[0 waypoints] > o
输入文件名 (不含扩展名): patrol_route1
✓ Loaded 5 waypoints from: patrol_route1.json

# 查看路点
[5 waypoints] > l
===== Waypoints (5) =====
  [1] x=1.230, y=2.450, yaw=0.000 rad
  [2] x=3.120, y=1.890, yaw=1.570 rad
  ...
==============================

# 删除最后一个路点
[5 waypoints] > d
Deleted last waypoint. Remaining: 4

# 添加新路点
[4 waypoints] > r
✓ Manually recorded waypoint #5

# 保存修改
[5 waypoints] > s
输入文件名 (不含扩展名): patrol_route1_edited
✓ Saved waypoints to: patrol_route1_edited.json
```

## 配置参数

### Launch 文件参数

```bash
ros2 launch bot_navigation waypoint_recorder.launch.py \
  pose_topic:=/rtabmap/localization_pose \
  recording_interval:=1.0 \
  min_distance:=0.5
```

参数说明：
- `pose_topic`: 位姿话题（默认: `/rtabmap/localization_pose`）
- `backup_pose_topic`: 备用里程计话题（默认: `/odom`）
- `use_odom_backup`: 是否使用里程计作为备用（默认: true）
- `recording_interval`: 自动录制间隔，秒（默认: 1.0）
- `min_distance`: 最小录制距离，米（默认: 0.5）
- `frame_id`: 路点坐标系（默认: map）

## RViz 可视化

路点会自动发布到 `/waypoint_markers` 话题，可以在 RViz 中添加 MarkerArray 显示：

1. 打开 RViz
2. Add → By topic → /waypoint_markers → MarkerArray
3. 绿色球体表示路点
4. 蓝色线条表示路径

## 文件存储

路点文件保存在：
```
~/.ros/lekiwi_bot/navigation/waypoints/
```

文件格式（YAML - 直接兼容 PatrolNode）：
```yaml
waypoints:
  - name: "点1"
    x: 1.23
    y: 2.45
    yaw: 0.0
    dwell_time: 2.0
  - name: "点2"
    x: 3.12
    y: 1.89
    yaw: 1.570
    dwell_time: 2.0
```

## 与 PatrolNode 集成

录制的路点可以直接用于巡航：

```bash
# 1. 录制路点
ros2 run bot_navigation waypoint_recorder
# ... 录制并保存为 my_patrol_route.yaml

# 2. 启动巡航
ros2 launch bot_bringup patrol_with_map.launch.py \
  waypoint_file:=my_patrol_route.yaml \
  patrol_mode:=loop
```

## 故障排除

### 问题1: 没有位姿信息

```
No pose available yet
```

**解决方案**:
- 确保机器人定位系统正在运行（RTABMap 或 AMCL）
- 检查话题: `ros2 topic echo /rtabmap/localization_pose`
- 如果使用 AMCL，修改 `pose_topic` 参数

### 问题2: 自动录制不工作

**原因**: 机器人移动距离太小或时间间隔未到

**解决方案**:
- 确保移动距离 > `min_distance`（默认 0.5m）
- 等待时间间隔（默认 1.0s）
- 调整参数：`recording_interval` 和 `min_distance`

### 问题3: 无法保存文件

**检查**:
- 目录权限: `~/.ros/lekiwi_bot/navigation/waypoints/`
- 磁盘空间
- 文件名不含特殊字符

## 高级用法

### 通过 ROS2 服务调用

WaypointRecorder 也可以通过 MissionPlanner 的服务接口使用：

```bash
# 开始录制
ros2 service call /mission/record_waypoints \
  bot_navigation_msgs/srv/RecordWaypoints \
  "{action: 'start'}"

# 录制当前位置
ros2 service call /mission/record_waypoints \
  bot_navigation_msgs/srv/RecordWaypoints \
  "{action: 'record_current'}"

# 停止录制
ros2 service call /mission/record_waypoints \
  bot_navigation_msgs/srv/RecordWaypoints \
  "{action: 'stop'}"

# 保存路点
ros2 service call /mission/waypoint_control \
  bot_navigation_msgs/srv/WaypointControl \
  "{action: 'save', filename: 'my_route'}"
```

## 下一步

- 查看 [PATROL_USAGE_GUIDE.md](PATROL_USAGE_GUIDE.md) 了解如何使用录制的路点进行巡航
- 查看 [MISSION_PLANNER_TEST.md](MISSION_PLANNER_TEST.md) 了解任务规划系统
