#!/bin/bash
# LeKiwi机器人依赖安装脚本
# 日期: 2025-12-02

echo "=========================================="
echo "LeKiwi机器人依赖安装"
echo "=========================================="

# 更新包列表
echo "[1/6] 更新apt包列表..."
sudo apt update

# ROS2基础包
echo "[2/6] 安装ROS2基础包..."
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# 导航系统
echo "[3/6] 安装Nav2导航系统..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-behaviortree-cpp-v3

# SLAM建图
echo "[4/6] 安装SLAM工具..."
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-rtabmap-ros

# 相机和传感器
echo "[5/6] 安装相机和传感器支持..."
sudo apt install -y \
    ros-humble-depthimage-to-laserscan \
    ros-humble-robot-localization

# Web接口
echo "[6/6] 安装Web接口..."
sudo apt install -y \
    ros-humble-rosbridge-server

echo ""
echo "=========================================="
echo "Python依赖安装"
echo "=========================================="

# Python依赖
pip3 install \
    pyserial \
    numpy \
    scipy \
    transforms3d \
    opencv-python \
    pyyaml

echo ""
echo "=========================================="
echo "✅ 依赖安装完成！"
echo "=========================================="
echo ""
echo "注意事项："
echo "1. Astra Pro相机驱动可能需要单独编译"
echo "2. YOLOv8、Vosk等将在需要时安装"
echo "3. 请重启终端或运行: source ~/.bashrc"
echo ""
