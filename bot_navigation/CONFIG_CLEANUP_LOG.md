# bot_navigation 配置文件整理记录

**整理日期**: 2025年12月22日  
**原因**: 调试开发期间创建了大量配置文件变体，需要清理和组织

## ✅ 整理完成情况

### 📁 新的目录结构

```
src/bot_navigation/config/
├── localization/                      # 定位融合
│   ├── robot_localization.yaml        [使用中] EKF融合（视觉里程计）
│   └── robot_localization_rtabmap.yaml [使用中] EKF融合（RTABMap）
├── nav2/                              # 导航配置
│   ├── nav2_params.yaml               [使用中] Nav2基础参数
│   ├── nav2_params_imu.yaml           [使用中] Nav2参数（主要使用）
│   └── exploration_bt.xml             [使用中] 探索行为树
├── exploration/                       # 探索配置（预留）
│   └── (Phase 4自主巡航系统使用)
├── rviz/                              # RViz配置
│   ├── nav2.rviz                      [使用中]
│   └── nav2_ros_dpimage.rviz          [使用中]
├── archived_unused/                   # 归档未使用
│   ├── robot_localization_sim.yaml    [已归档]
│   ├── robot_localization_real.yaml   [已归档]
│   ├── robot_localization_imu.yaml    [已归档]
│   ├── slam_toolbox.yaml              [已归档]
│   ├── slam_toolbox_imu.yaml          [已归档]
│   ├── slam_toolbox_imu_official.yaml [已归档]
│   ├── slam_toolbox_vio.yaml          [已归档]
│   └── exploration_mapper.yaml        [已归档]
└── README.md                          # 配置说明文档
```

### 🔄 更新的启动文件

已更新以下启动文件中的配置路径：
1. `src/bot_bringup/launch/simple_simulation_nav2_rtabmap.launch.py`
   - rtabmap.yaml → bot_slam/config/slam/rtabmap.yaml
   - robot_localization_rtabmap.yaml → localization/robot_localization_rtabmap.yaml

2. `src/bot_bringup/launch/simulation_nav2_rtabmap.launch.py`
   - rtabmap.yaml → bot_slam/config/slam/rtabmap.yaml
   - robot_localization_rtabmap.yaml → localization/robot_localization_rtabmap.yaml

3. `src/bot_navigation/launch/visual_odom.launch.py`
   - rtabmap_odom.yaml → bot_slam/config/slam/rtabmap_odom.yaml
   - robot_localization.yaml → localization/robot_localization.yaml

**2025-12-22 更新**: SLAM配置已移至 bot_slam 功能包，符合模块化设计原则

### 📋 配置文件状态统计

| 状态 | 数量 | 位置 | 文件 |
|------|------|------|------|
| ✅ SLAM配置 | 2个 | bot_slam | rtabmap.yaml, rtabmap_odom.yaml |
| ✅ 定位配置 | 2个 | bot_navigation | robot_localization.yaml, robot_localization_rtabmap.yaml |
| ✅ 导航配置 | 3个 | bot_navigation | nav2_params.yaml, nav2_params_imu.yaml, exploration_bt.xml |
| ✅ RViz配置 | 2个 | bot_navigation | nav2.rviz, nav2_ros_dpimage.rviz |
| 📦 已归档 | 8个 | bot_navigation | 各种调试期间的配置变体 |
| 📁 预留目录 | 1个 | bot_navigation | exploration/ (Phase 4使用) |

## 🎯 整理效果

### 改进前
- 14个配置文件混杂在同一目录
- 无法区分哪些正在使用，哪些是调试遗留
- 文件命名混乱（sim、real、imu等后缀不统一）

### 改进后
- ✅ 按功能分类到4个子目录
- ✅ 正在使用的文件清晰可见
- ✅ 未使用文件归档保存，不影响日常使用
- ✅ 为Phase 4自主巡航系统预留 exploration/ 目录
- ✅ 添加README.md说明文档

## 📝 注意事项

1. **archived_unused/** 目录中的文件不应删除
   - 保留用于历史参考
   - 如需恢复可快速找到

2. **exploration/** 目录预留给Phase 4
   - 将用于自主巡航系统配置
   - 包括：任务配置、地图库配置、巡航路径等

3. **编译注意**
   - 已使用 `--symlink-install`，修改配置无需重新编译
   - 如移动配置文件，需要清理build目录重新编译

## 🔗 相关文档

- [配置目录README](./config/README.md)
- [自主巡航设计文档](../../AUTONOMOUS_PATROL_DESIGN.md)
- [总体技术设计](../../TECHNICAL_DESIGN.md)
