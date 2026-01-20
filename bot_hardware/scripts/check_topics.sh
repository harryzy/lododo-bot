#!/bin/bash
# 检查硬件节点话题状态 / Check hardware node topics status

echo "=========================================="
echo "检查 ROS2 话题状态 / Check ROS2 Topics"
echo "=========================================="
echo ""

echo "[1] 检查 /wheel/odom 话题..."
ros2 topic info /wheel/odom
echo ""

echo "[2] 检查 /wheel/odom 发布频率..."
timeout 3 ros2 topic hz /wheel/odom
echo ""

echo "[3] 查看一条 /wheel/odom 消息..."
timeout 2 ros2 topic echo /wheel/odom --once
echo ""

echo "[4] 检查 /wheel/direct_speeds 话题..."
ros2 topic info /wheel/direct_speeds
echo ""

echo "=========================================="
echo "检查完成 / Check Completed"
echo "=========================================="
