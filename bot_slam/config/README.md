# bot_slam 配置文件说明

## 📁 slam/
SLAM（同步定位与建图）和视觉里程计配置

### 配置文件

- **rtabmap.yaml** - RTABMap SLAM主配置
  - 用于导航+建图模式
  - 订阅: RGB图像、深度图像、相机信息
  - 发布: /rtabmap/mapData, /rtabmap/odom 等
  - 使用场景: simple_simulation_nav2_rtabmap.launch.py

- **rtabmap_odom.yaml** - RTABMap视觉里程计配置
  - 仅用于视觉里程计模式（不建图）
  - 订阅: RGB图像、深度图像、相机信息
  - 发布: /vo/odom (视觉里程计)
  - 使用场景: visual_odom.launch.py

## 使用示例

```bash
# 仿真导航 + RTABMap建图
ros2 launch bot_bringup simple_simulation_nav2_rtabmap.launch.py

# 纯视觉里程计模式
ros2 launch bot_navigation visual_odom.launch.py
```

## 未来扩展

Phase 4 自主巡航系统将在此目录添加：
- 地图库管理配置
- 探索参数配置
- 地图质量评估配置

详见：[AUTONOMOUS_PATROL_DESIGN.md](../../AUTONOMOUS_PATROL_DESIGN.md)
