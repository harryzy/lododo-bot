# 自主探索建图功能说明

## 功能概述

自动探索未知环境并建立地图，确保相机始终朝向运动方向以保证60度FOV覆盖。

## 使用方法

### 1. 启动仿真和导航系统

```bash
# 终端1: 启动RTABMap + Nav2系统
ros2 launch bot_bringup simulation_nav2_rtabmap.launch.py
```

### 2. 启动探索节点

```bash
# 终端2: 启动自主探索
ros2 launch bot_navigation test_exploration.launch.py
```

### 可选参数

```bash
ros2 launch bot_navigation test_exploration.launch.py \
  exploration_radius:=10.0 \          # 探索半径(米)
  completion_threshold:=0.90 \        # 地图完成度阈值(0-1)
  use_rviz:=true                      # 是否启动RViz
```

## 核心策略

### 相机朝向保证
- **camera_optical_frame的Z轴** 指向 **base_link的+X方向**（机器人前方）
- 机器人会自动**旋转**使+X方向对准目标，然后**前进**
- 这样保证60度FOV始终覆盖运动方向

### 探索算法
- **Frontier-based exploration**: 寻找已知区域与未知区域的边界
- **自动选择目标**: 优先近距离、大尺寸边界
- **避障导航**: 使用Nav2进行安全导航

### 完成条件
1. **地图完成度达到阈值**（默认88%）
2. **连续3次导航失败** - 自动终止并报告错误
3. **无更多可探索边界** - 地图探索完成

## 状态监控

### 监控话题

```bash
# 实时地图
ros2 topic echo /map

# 探索完成信号
ros2 topic echo /exploration/complete

# 当前导航目标
ros2 topic echo /goal_pose
```

### 日志输出

- ✅ **成功**: 导航目标完成
- ⚠️  **警告**: 目标被取消或未找到边界
- ❌ **错误**: 导航失败或action服务器未响应

## 配置参数

在 `config/exploration_mapper.yaml`:

```yaml
exploration_mapper:
  ros__parameters:
    exploration_radius: 8.0          # 探索半径(m)
    min_frontier_size: 15            # 最小边界点数
    goal_tolerance: 0.25             # 目标容差(m)
    rotation_speed: 0.6              # 旋转速度(rad/s)
    forward_speed: 0.20              # 前进速度(m/s)
    map_completion_threshold: 0.88   # 地图完成度阈值
    safe_distance: 0.35              # 安全距离(m)
    camera_fov: 60.0                 # 相机FOV(度)
```

## 故障排查

### 问题: "Nav2 action服务器未响应"

**原因**: Nav2导航系统未启动或未准备好

**解决**:
1. 确保先启动 `simulation_nav2_rtabmap.launch.py`
2. 等待约10-15秒，直到看到Nav2日志输出
3. 检查action服务器是否存在:
   ```bash
   ros2 action list
   # 应该看到 /navigate_to_pose
   ```

### 问题: 连续导航失败

**原因**: 
- 目标点不可达（被障碍物包围）
- 地图质量问题
- 导航参数配置问题

**解决**:
1. 检查RViz中的costmap是否正常显示障碍物
2. 调整 `exploration_radius` 参数（减小探索半径）
3. 增加 `min_frontier_size`（避免选择太小的边界）

### 问题: 相机朝向不正确

**原因**: TF树配置问题

**检查**:
```bash
# 查看TF关系
ros2 run tf2_tools view_frames

# 验证camera_optical_frame的Z轴指向base_link的+X
ros2 topic echo /tf_static
```

## 性能优化

### 提高建图速度
- 增加 `forward_speed` (最大0.22 m/s)
- 增加 `exploration_radius` (扩大搜索范围)
- 减小 `min_frontier_size` (更激进的探索)

### 提高建图质量
- 降低 `forward_speed` (给RTABMap更多时间处理)
- 减小 `exploration_radius` (更细致的探索)
- 增加 `map_completion_threshold` (更完整的地图)

## 技术细节

### Action客户端实现
- 使用异步回调处理goal状态
- `goal_response_callback`: 检查goal是否被接受
- `goal_result_callback`: 处理执行结果（成功/失败/取消）
- `goal_feedback_callback`: 监控导航进度（可选）

### 状态机
```
等待地图 → 寻找边界 → 选择目标 → 发送导航goal → 等待完成 → 循环
            ↓
         无边界 → 原地旋转扫描 → 继续寻找
            ↓
      连续失败3次 → 终止探索
```

### 失败处理
- **连续失败计数器**: 跟踪连续失败次数
- **最大失败次数**: 默认3次，超过则终止
- **自动重置**: 成功导航后重置计数器

## 作者

Auto-generated for LeKiwi Bot - 2025-12-11
