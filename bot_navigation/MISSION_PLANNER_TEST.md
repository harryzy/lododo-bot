# MissionPlanner 测试指南

## 快速测试

### 1. 启动 MissionPlanner

```bash
cd ~/lododo_bot
source install/setup.bash
ros2 launch bot_navigation mission_planner.launch.py
```

### 2. 查看可用服务

```bash
ros2 service list | grep mission
```

应该看到 13 个服务：
- /mission/create_task
- /mission/start_task
- /mission/pause_task
- /mission/resume_task
- /mission/cancel_task
- /mission/get_task_status
- /mission/list_tasks
- /mission/start_exploration
- /mission/start_patrol
- /mission/navigate_to_pose
- /mission/waypoint_control
- /mission/record_waypoints
- /mission/emergency_stop

### 3. 运行自动化测试

```bash
# 在另一个终端
cd ~/lododo_bot
source install/setup.bash
python3 src/bot_navigation/scripts/test_mission_planner.py
```

### 4. 手动测试示例

#### 创建任务
```bash
ros2 service call /mission/create_task bot_navigation_msgs/srv/CreateTask \
  "{task_id: 'nav_task_1', task_type: 'point_to_point', priority: 5, 
    parameters_keys: ['x', 'y', 'yaw'], parameters_values: ['2.0', '3.0', '1.57']}"
```

#### 列出所有任务
```bash
ros2 service call /mission/list_tasks bot_navigation_msgs/srv/ListTasks "{filter: 'all'}"
```

#### 获取任务状态
```bash
ros2 service call /mission/get_task_status bot_navigation_msgs/srv/GetTaskStatus \
  "{task_id: 'nav_task_1'}"
```

#### 导航到位姿
```bash
ros2 service call /mission/navigate_to_pose bot_navigation_msgs/srv/NavigateToPose \
  "{x: 1.5, y: 2.0, yaw: 0.0, frame_id: 'map'}"
```

#### 开始探索
```bash
ros2 service call /mission/start_exploration bot_navigation_msgs/srv/StartExploration \
  "{map_name: 'test_map', save_map: true, max_duration: 300.0, coverage_threshold: 0.9}"
```

#### 开始巡航
```bash
ros2 service call /mission/start_patrol bot_navigation_msgs/srv/StartPatrol \
  "{waypoint_file: 'patrol_waypoints.yaml', patrol_mode: 'loop', speed_factor: 1.0}"
```

#### 取消任务
```bash
ros2 service call /mission/cancel_task bot_navigation_msgs/srv/TaskControl \
  "{task_id: 'nav_task_1'}"
```

#### 紧急停止
```bash
ros2 service call /mission/emergency_stop bot_navigation_msgs/srv/EmergencyStop \
  "{clear_tasks: false}"
```

## 监控状态

### 查看系统状态
```bash
ros2 topic echo /mission_planner/state
```

### 查看任务状态
```bash
ros2 topic echo /mission_planner/task_status
```

## 已实现功能

✅ 任务创建和管理
✅ 任务状态查询
✅ 任务列表过滤
✅ 探索任务启动
✅ 巡航任务启动
✅ 点对点导航
✅ 路点控制接口
✅ 紧急停止

## 待完善功能

⏳ 任务执行完整流程（需要集成 ExplorationMapper 和 PatrolNode）
⏳ 任务超时处理
⏳ 任务自动恢复
⏳ WaypointRecorder 集成
⏳ 任务进度监控
⏳ 子节点生命周期管理

## 下一步

1. 实现 WaypointRecorder 路点录制器
2. 完善任务执行与组件集成
3. 添加任务完成回调
4. 实现任务持久化恢复
