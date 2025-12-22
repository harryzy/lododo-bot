# bot_navigation 配置文件目录结构

## 目录说明

### 📁 localization/
定位与里程计融合配置
- `robot_localization.yaml` - EKF融合配置（视觉里程计模式）
- `robot_localization_rtabmap.yaml` - EKF融合配置（RTABMap模式，融合轮式里程计+RTABMap里程计）

### 📁 nav2/
Nav2导航栈配置
- `nav2_params.yaml` - Nav2基础参数配置
- `nav2_params_imu.yaml` - Nav2参数配置（带IMU，当前主要使用）
- `exploration_bt.xml` - 探索行为树配置

### 📁 exploration/ (未来使用)
自主探索与巡航系统配置
- 预留给Phase 4自主巡航系统使用
- 将包含：任务配置、地图库配置、巡航路径等

### 📁 rviz/
RViz可视化配置
- `nav2.rviz` - 导航可视化配置
- `nav2_ros_dpimage.rviz` - 深度图像可视化配置

### 📁 archived_unused/
归档的未使用配置文件
- 调试开发期间创建但当前未使用的配置
- 保留以备后续参考，不建议直接使用

## 使用说明

### 当前主要启动配置
```bash
# 仿真导航 + RTABMap建图
ros2 launch bot_bringup simple_simulation_nav2_rtabmap.launch.py

# 使用配置：
# - bot_slam: slam/rtabmap.yaml
# - bot_navigation: localization/robot_localization_rtabmap.yaml  
# - bot_navigation: nav2/nav2_params_imu.yaml
```

### 配置文件选择原则
1. **仿真环境** - 使用Ground Truth里程计 + RTABMap
2. **真机环境** - 使用轮式里程计 + 视觉里程计融合
3. **导航参数** - 当前统一使用 `nav2_params_imu.yaml`

## 未来扩展

Phase 4自主巡航系统将在 `exploration/` 目录下添加：
- 任务类型配置
- 地图库管理配置
- 巡航路径配置
- 命令接口配置

详见：[AUTONOMOUS_PATROL_DESIGN.md](../../AUTONOMOUS_PATROL_DESIGN.md)
