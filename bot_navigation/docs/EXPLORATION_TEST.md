# 自主探索建图测试 (Autonomous Exploration Mapping Test)

## 功能说明

这个测试程序实现了**相机朝向感知的自主探索建图**，专为60°窄FOV相机设计。

### 核心特性

1. **相机朝向保证** 🎥
   - `camera_optical_frame`的Z轴（光轴）指向`base_link`的+X方向（前方）
   - 机器人导航时会**先旋转到目标方向，然后前进**
   - 保证60°FOV始终覆盖运动方向，避免特征丢失

2. **智能探索策略** 🗺️
   - 基于**Frontier算法**：寻找已知/未知区域边界
   - 自动选择最优目标：优先近距离、大边界区域
   - 避免重复探索和死锁

3. **自动完成检测** ✅
   - 实时计算地图完成度（已知区域占比）
   - 达到阈值（默认88%）自动停止
   - 无更多边界时自动旋转扫描或结束

## 使用方法

### 1. 编译安装

```bash
cd /home/hurry/lododo_bot
colcon build --symlink-install --packages-select bot_navigation
source install/setup.bash
```

### 2. 启动探索建图

```bash
# 默认参数（cafe世界，88%完成度）
ros2 launch bot_navigation test_exploration.launch.py

# 自定义参数
ros2 launch bot_navigation test_exploration.launch.py \
    world:=cafe \
    exploration_radius:=8.0 \
    completion_threshold:=0.90 \
    use_rviz:=true
```

### 3. 监控进度

**终端输出：**
```
🗺️  探索建图节点已启动
   探索半径: 8.0m
   相机FOV: 60.0°
   完成阈值: 88.0%
   相机朝向策略: 始终保持朝向运动方向(+X)

📊 状态: 完成度=45.2%, 已知区域=12540, 总区域=27800
🎯 目标边界: 大小=25, 距离=3.45m, 位置=(2.15, -1.30)
📍 发送目标: (2.15, -1.30), yaw=125.3°

✅ 地图探索完成! 完成度: 88.5%
```

**RViz可视化：**
- **Map**: 实时地图（灰色=已知，黑色=障碍，白色=未知）
- **GlobalPlan**: 当前规划路径（绿色）
- **LocalCostmap**: 局部代价地图（红色=障碍）
- **RobotModel**: 机器人朝向（注意+X方向）

**话题监控：**
```bash
# 地图完成度
ros2 topic echo /exploration/complete

# 当前导航目标
ros2 topic echo /goal_pose

# 机器人速度
ros2 topic echo /cmd_vel
```

## 参数调整

### 探索参数文件

`config/exploration_mapper.yaml`:

```yaml
exploration_mapper:
  ros__parameters:
    exploration_radius: 8.0           # 探索半径(m)
    min_frontier_size: 15             # 最小边界尺寸
    map_completion_threshold: 0.88    # 完成度阈值
    rotation_speed: 0.6               # 旋转速度(rad/s)
    forward_speed: 0.20               # 前进速度(m/s)
```

### 关键参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `exploration_radius` | 8.0m | 探索半径，太大会导致路径规划失败 |
| `min_frontier_size` | 15 | 最小边界尺寸（像素），过小会频繁切换目标 |
| `map_completion_threshold` | 0.88 | 完成度阈值，cafe世界88%基本探索完毕 |
| `rotation_speed` | 0.6 rad/s | 原地旋转速度，用于扫描未知区域 |
| `forward_speed` | 0.20 m/s | 前进速度，匹配Nav2配置 |

## 工作原理

### 相机朝向策略

```
机器人坐标系 (base_link):
        +X (前方)
         ↑
         |
    +Y ←-○
        
相机光轴 (camera_optical_frame Z轴) = base_link +X

导航策略:
  1. 检测目标位置 (x_target, y_target)
  2. 计算目标yaw = atan2(y_target - y_robot, x_target - x_robot)
  3. 发送Nav2目标: (x_target, y_target, yaw_target)
  4. 机器人会先旋转到yaw_target（相机对准目标）
  5. 然后沿+X方向前进（相机持续看向目标）
```

### Frontier探索算法

```
1. 扫描地图，寻找边界点：
   - 边界点 = 未知区域(-1) 且 8邻域有自由空间(0-50)
   
2. BFS聚类相邻边界点
   
3. 过滤小边界（< min_frontier_size）
   
4. 评分选择最优边界：
   Score = frontier_size / distance
   
5. 发送导航目标到边界中心
   
6. 重复直到达到完成度阈值
```

## 测试场景

### Cafe世界（推荐）

```bash
ros2 launch bot_navigation test_exploration.launch.py world:=cafe
```

- **地图大小**: 约15m × 15m
- **障碍密度**: 中等（桌椅、墙壁）
- **完成时间**: 5-8分钟
- **推荐阈值**: 0.88

### Empty世界（快速测试）

```bash
ros2 launch bot_navigation test_exploration.launch.py world:=empty
```

- **地图大小**: 约10m × 10m
- **障碍密度**: 无障碍
- **完成时间**: 2-3分钟
- **推荐阈值**: 0.95

## 故障排查

### 问题1: 机器人不移动

**症状**: 探索节点启动，但机器人静止不动

**原因**:
- Nav2未完全启动
- 地图数据未接收

**解决**:
```bash
# 检查Nav2节点状态
ros2 node list | grep nav2

# 检查地图话题
ros2 topic hz /map

# 增加启动延迟（修改launch文件）
prefix=['bash -c "sleep 20 && $0 $@"']
```

### 问题2: 频繁切换目标

**症状**: 机器人频繁改变目标方向

**原因**:
- `min_frontier_size`太小
- 边界点噪声

**解决**:
```yaml
# 增加最小边界尺寸
min_frontier_size: 20  # 从15增加到20
```

### 问题3: 探索不完整

**症状**: 完成度60-70%就停止

**原因**:
- `exploration_radius`太小
- 远距离边界被过滤

**解决**:
```yaml
# 增加探索半径
exploration_radius: 10.0  # 从8.0增加到10.0

# 降低完成度阈值
map_completion_threshold: 0.85  # 从0.88降低到0.85
```

### 问题4: 路径规划失败

**症状**: 日志显示"Goal rejected"或"Failed to compute path"

**原因**:
- 目标点在障碍物内
- 目标距离超出costmap范围

**解决**:
```yaml
# 减小探索半径
exploration_radius: 6.0

# 增加安全距离
safe_distance: 0.45
```

## 性能优化

### 加快建图速度

1. **增加前进速度**（需同步修改Nav2配置）:
```yaml
forward_speed: 0.25  # 从0.20增加到0.25
```

2. **增加旋转速度**:
```yaml
rotation_speed: 0.8  # 从0.6增加到0.8
```

3. **降低完成度要求**:
```yaml
map_completion_threshold: 0.85  # 从0.88降低
```

### 提高建图质量

1. **降低运动速度**:
```yaml
forward_speed: 0.15
rotation_speed: 0.4
```

2. **增加RTABMap特征数**（修改`rtabmap.yaml`）:
```yaml
Vis/MaxFeatures: "2000"  # 从1500增加
```

3. **提高完成度要求**:
```yaml
map_completion_threshold: 0.92
```

## 代码结构

```
bot_navigation/
├── bot_navigation/
│   └── exploration_mapper.py      # 探索建图核心节点
├── launch/
│   └── test_exploration.launch.py # 测试启动文件
├── config/
│   └── exploration_mapper.yaml    # 配置参数
└── docs/
    └── EXPLORATION_TEST.md        # 本文档
```

## 扩展开发

### 添加新的探索策略

修改`select_best_frontier()`方法:

```python
def select_best_frontier(self, frontiers):
    """自定义评分策略"""
    for frontier in frontiers:
        # 距离权重
        distance_score = 1.0 / max(distance, 0.5)
        
        # 尺寸权重
        size_score = len(frontier) / 100.0
        
        # 朝向权重（优先前方目标）
        angle_diff = abs(target_yaw - current_yaw)
        orientation_score = 1.0 - (angle_diff / math.pi)
        
        # 综合评分
        score = distance_score * 0.4 + size_score * 0.4 + orientation_score * 0.2
```

### 集成Nav2 Behavior Tree

替换简单的Action Client为完整的BT控制:

```python
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import Path

# 规划路径
path = self.planner_client.send_goal(...)

# 跟随路径（相机朝向控制更精确）
self.controller_client.send_goal(path)
```

## 参考资料

- **Nav2文档**: https://navigation.ros.org/
- **RTABMap文档**: http://wiki.ros.org/rtabmap_ros
- **Frontier Exploration论文**: Yamauchi, B. (1997)

## 许可证

MIT License

---

**作者**: Auto-generated for LeKiwi Bot  
**日期**: 2025-12-11  
**版本**: 1.0
