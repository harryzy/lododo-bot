#!/bin/bash
# 设置详细目录结构和初始配置文件

echo "=========================================="
echo "创建目录结构"
echo "=========================================="

# 1. bot_gazebo - 仿真环境（优先）
echo "[1/9] 设置 bot_gazebo..."
mkdir -p bot_gazebo/worlds
mkdir -p bot_gazebo/models
mkdir -p bot_gazebo/config
mkdir -p bot_gazebo/launch

# 2. bot_control - 运动控制
echo "[2/9] 设置 bot_control..."
mkdir -p bot_control/config
mkdir -p bot_control/launch

# 3. bot_bringup - 系统集成
echo "[3/9] 设置 bot_bringup..."
mkdir -p bot_bringup/launch
mkdir -p bot_bringup/config
mkdir -p bot_bringup/params

# 4. bot_navigation - 导航
echo "[4/9] 设置 bot_navigation..."
mkdir -p bot_navigation/config/nav2
mkdir -p bot_navigation/maps
mkdir -p bot_navigation/launch

# 5. bot_slam - SLAM
echo "[5/9] 设置 bot_slam..."
mkdir -p bot_slam/config
mkdir -p bot_slam/maps
mkdir -p bot_slam/launch

# 6. bot_perception - 感知
echo "[6/9] 设置 bot_perception..."
mkdir -p bot_perception/config
mkdir -p bot_perception/models
mkdir -p bot_perception/launch

# 7. bot_teleop - 遥控
echo "[7/9] 设置 bot_teleop..."
mkdir -p bot_teleop/web
mkdir -p bot_teleop/config
mkdir -p bot_teleop/launch

# 8. bot_hardware - 硬件（框架）
echo "[8/9] 设置 bot_hardware..."
mkdir -p bot_hardware/config
mkdir -p bot_hardware/launch

# 9. bot_voice - 语音（框架）
echo "[9/9] 设置 bot_voice..."
mkdir -p bot_voice/models
mkdir -p bot_voice/config
mkdir -p bot_voice/launch

# 为bot_description添加Gazebo相关目录
echo "设置 bot_description Gazebo支持..."
mkdir -p bot_description/config
mkdir -p bot_description/launch/py

echo ""
echo "✅ 目录结构创建完成！"
