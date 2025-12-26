# 巡航系统使用指南

## 快速开始

### 方案1: 自动探索 + 保存地图 + 巡航

#### 步骤1: 探索建图并保存
```bash
# 启动探索建图(会自动保存地图)
ros2 launch bot_bringup exploration_with_map_save.launch.py \
  map_name:=office_floor1 \
  description:="办公室1楼" \
  tags:="office,floor1,auto"

# 等待探索完成，地图会自动保存到 ~/lododo_bot/maps/office_floor1/
```

#### 步骤2: 使用保存的地图进行巡航
```bash
# 启动巡航(使用刚才保存的地图)
ros2 launch bot_bringup patrol_with_map.launch.py \
  use_map_library:=true \
  map_name:=office_floor1 \
  patrol_mode:=loop \
  auto_start:=false

# 在RViz中使用"2D Pose Estimate"设置初始位姿

# 开始巡航
ros2 service call /patrol/start std_srvs/srv/Trigger
```

---

## 详细使用

### 1. 探索建图 + 自动保存

**launch文件**: `exploration_with_map_save.launch.py`

**功能**:
- 启动Gazebo仿真 + RTABMap SLAM + Nav2
- 自动探索未知区域
- 探索完成后自动保存地图到地图库

**参数**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| world | cafe | Gazebo世界名称 |
| use_rviz | true | 是否启动RViz |
| exploration_radius | 8.0 | 探索半径(米) |
| completion_threshold | 0.78 | 完成度阈值(0-1) |
| map_name | auto_explored_map | 地图名称 |
| description | Automatically explored map | 地图描述 |
| tags | auto,exploration | 地图标签(逗号分隔) |
| auto_save | true | 自动保存地图 |

**使用示例**:
```bash
# 基础使用
ros2 launch bot_bringup exploration_with_map_save.launch.py

# 自定义地图信息
ros2 launch bot_bringup exploration_with_map_save.launch.py \
  map_name:=warehouse \
  description:="仓库地图" \
  tags:="warehouse,indoor" \
  completion_threshold:=0.85

# 不同的Gazebo世界
ros2 launch bot_bringup exploration_with_map_save.launch.py \
  world:=house \
  map_name:=house_map
```

**监控话题**:
```bash
# 探索状态
ros2 topic echo /exploration/status

# 探索完成信号
ros2 topic echo /exploration/complete

# 实时地图
ros2 topic echo /map
```

**手动控制**:
```bash
# 停止探索
ros2 service call /exploration/stop std_srvs/srv/Trigger

# 手动保存地图
ros2 service call /map_library/save_current_map std_srvs/srv/Trigger
```

---

### 2. 加载地图 + 巡航

**launch文件**: `patrol_with_map.launch.py`

**功能**:
- 启动Gazebo仿真 + AMCL定位 + Nav2
- 从地图库或文件加载地图
- 启动巡航节点
- 支持多种巡航模式

**参数**:

**地图相关**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| use_map_library | false | 是否从地图库加载 |
| map_name | auto_explored_map | 地图库中的地图名称 |
| map_version | 0 | 地图版本(0=最新) |
| map_file | "" | 直接指定地图文件路径 |

**巡航相关**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| waypoint_file | patrol_waypoints.yaml | 路点配置文件 |
| patrol_mode | loop | 巡航模式 |
| max_loops | -1 | 最大循环次数(-1=无限) |
| default_dwell_time | 2.0 | 默认停留时间(秒) |
| auto_start | false | 自动开始巡航 |

**巡航模式**:
- `loop`: 循环模式 (0→1→2→3→0→...)
- `ping_pong`: 往返模式 (0→1→2→3→2→1→0→...)
- `once`: 单次模式 (0→1→2→3→结束)
- `random`: 随机模式 (随机选择路点)

**使用示例**:

```bash
# 从地图库加载地图
ros2 launch bot_bringup patrol_with_map.launch.py \
  use_map_library:=true \
  map_name:=office_floor1 \
  patrol_mode:=loop \
  auto_start:=false

# 使用指定地图文件
ros2 launch bot_bringup patrol_with_map.launch.py \
  use_map_library:=false \
  map_file:=/path/to/my_map.yaml \
  patrol_mode:=ping_pong

# 自定义路点文件
ros2 launch bot_bringup patrol_with_map.launch.py \
  waypoint_file:=/path/to/custom_waypoints.yaml \
  patrol_mode:=once \
  max_loops:=3

# 自动开始巡航
ros2 launch bot_bringup patrol_with_map.launch.py \
  map_name:=warehouse \
  patrol_mode:=loop \
  auto_start:=true
```

**巡航控制**:
```bash
# 开始巡航
ros2 service call /patrol/start std_srvs/srv/Trigger

# 停止巡航
ros2 service call /patrol/stop std_srvs/srv/Trigger

# 暂停巡航
ros2 service call /patrol/pause std_srvs/srv/Trigger

# 恢复巡航
ros2 service call /patrol/resume std_srvs/srv/Trigger
```

**监控话题**:
```bash
# 巡航状态
ros2 topic echo /patrol/status

# 巡航完成信号
ros2 topic echo /patrol/complete

# 机器人定位
ros2 topic echo /amcl_pose

# 当前导航目标
ros2 topic echo /goal_pose
```

---

## 路点配置文件格式

**文件**: `patrol_waypoints.yaml`

```yaml
waypoints:
  - name: "point_1"
    x: 0.0
    y: 0.0
    yaw: 0.0
    dwell_time: 2.0
  
  - name: "point_2"
    x: 2.0
    y: 0.0
    yaw: 1.57  # 90度
    dwell_time: 3.0
  
  - name: "point_3"
    x: 2.0
    y: 2.0
    yaw: 3.14  # 180度
    dwell_time: 2.0
  
  - name: "point_4"
    x: 0.0
    y: 2.0
    yaw: -1.57  # -90度
    dwell_time: 2.0
```

**字段说明**:
- `name`: 路点名称(可选)
- `x, y`: 地图坐标系中的位置(米)
- `yaw`: 朝向角度(弧度)，0为+X方向
- `dwell_time`: 到达后停留时间(秒)

---

## 地图库管理

### 查看已保存的地图

地图保存在 `~/lododo_bot/maps/` 目录下:

```bash
# 查看地图库
ls -la ~/lododo_bot/maps/

# 每个地图有独立的目录
~/lododo_bot/maps/
├── map_library.yaml        # 地图索引
├── office_floor1/
│   ├── office_floor1_v1.yaml
│   ├── office_floor1_v1.pgm
│   ├── office_floor1_v2.yaml
│   └── office_floor1_v2.pgm
└── warehouse/
    ├── warehouse_v1.yaml
    └── warehouse_v1.pgm
```

### 地图版本管理

每次保存同名地图时，版本号会自动递增:
```bash
# 第一次保存
office_floor1_v1.yaml  # 版本1

# 第二次保存(更新)
office_floor1_v2.yaml  # 版本2

# 加载最新版本(默认)
ros2 launch bot_bringup patrol_with_map.launch.py \
  map_name:=office_floor1 \
  map_version:=0  # 0表示最新版本

# 加载指定版本
ros2 launch bot_bringup patrol_with_map.launch.py \
  map_name:=office_floor1 \
  map_version:=1  # 加载v1
```

---

## 常见问题

### Q1: 探索完成后地图没有自动保存?

**检查**:
1. 确认 `auto_save:=true`
2. 查看map_saver_node日志
3. 确认收到了 `/exploration/complete` 信号

```bash
# 手动触发保存
ros2 service call /map_library/save_current_map std_srvs/srv/Trigger
```

### Q2: 巡航时机器人不动?

**可能原因**:
1. 未调用 `/patrol/start` 服务
2. 未设置初始位姿(AMCL需要初始化)
3. 路点配置文件路径错误

**解决**:
```bash
# 1. 在RViz中设置初始位姿("2D Pose Estimate")
# 2. 开始巡航
ros2 service call /patrol/start std_srvs/srv/Trigger

# 3. 检查路点是否加载
ros2 topic echo /patrol/status
```

### Q3: 如何创建自定义路点?

**方法1: 手动记录位置**
1. 在RViz中查看机器人当前位姿
2. 记录 x, y, yaw
3. 编辑 `patrol_waypoints.yaml`

**方法2: 使用waypoint_recorder** (TODO)
```bash
# 启动录制器
ros2 run bot_navigation waypoint_recorder

# 驾驶机器人到各个位置
# 按'r'键记录当前位置
# 按's'键保存到文件
```

### Q4: 巡航路线不理想怎么办?

**优化方向**:
1. 调整路点顺序
2. 修改路点朝向(yaw)
3. 改变巡航模式
4. 调整停留时间

---

## 进阶使用

### 组合工作流

**完整流程: 探索 → 保存 → 巡航**

```bash
# 1. 探索并保存地图
ros2 launch bot_bringup exploration_with_map_save.launch.py \
  world:=cafe \
  map_name:=cafe_map

# 2. 等待探索完成(看到"✅ Map saved"日志)
# 3. Ctrl+C停止探索

# 4. 编辑路点文件(根据地图调整坐标)
nano ~/lododo_bot/src/bot_navigation/config/patrol_waypoints.yaml

# 5. 启动巡航
ros2 launch bot_bringup patrol_with_map.launch.py \
  use_map_library:=true \
  map_name:=cafe_map \
  patrol_mode:=loop

# 6. 设置初始位姿并开始巡航
ros2 service call /patrol/start std_srvs/srv/Trigger
```

---

## 文件位置

### Launch文件
- `src/bot_bringup/launch/exploration_with_map_save.launch.py`
- `src/bot_bringup/launch/patrol_with_map.launch.py`

### 配置文件
- `src/bot_navigation/config/patrol_waypoints.yaml`
- `src/bot_navigation/config/exploration/exploration_manager.yaml`

### 节点源码
- `src/bot_navigation/bot_navigation/patrol_node.py`
- `src/bot_navigation/bot_navigation/map_saver_node.py`
- `src/bot_navigation/bot_navigation/map_library_manager.py`
- `src/bot_navigation/bot_navigation/exploration_mapper.py`

### 地图存储
- `~/lododo_bot/maps/` - 地图库根目录

---

## 下一步开发

- [ ] 实现waypoint_recorder节点(实时录制路点)
- [ ] 创建使用AMCL的launch文件(patrol需要定位)
- [ ] 实现地图加载节点(从地图库加载到map_server)
- [ ] Web界面控制
- [ ] 任务持久化和恢复
