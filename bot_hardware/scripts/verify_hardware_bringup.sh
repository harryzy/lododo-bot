#!/bin/bash
# hardware_bringup验证脚本

echo "=========================================="
echo "Hardware Bringup 完整验证"
echo "=========================================="

cd ~/workDisk/lododo_bot
source install/setup.bash

# 启动hardware_bringup
echo "启动 hardware_bringup.launch.py..."
ros2 launch bot_hardware hardware_bringup.launch.py > /tmp/hardware_test.log 2>&1 &
LAUNCH_PID=$!

sleep 8

echo ""
echo "=== 1. 运行中的节点 ==="
ros2 node list

echo ""
echo "=== 2. 关键话题 ==="
echo "控制话题:"
ros2 topic list | grep cmd_vel
echo "里程计话题:"
ros2 topic list | grep wheel/odom
echo "IMU话题:"
ros2 topic list | grep imu
echo "相机话题:"
ros2 topic list | grep camera | head -5

echo ""
echo "=== 3. 话题发布频率 ==="
echo "检查 /wheel/odom (应该~50Hz):"
timeout 3 ros2 topic hz /wheel/odom 2>&1 | grep "average rate"

echo ""
echo "检查 /imu/data (应该~10Hz):"
timeout 3 ros2 topic hz /imu/data 2>&1 | grep "average rate"

echo ""
echo "检查 /camera/depth/image_raw (应该~30Hz):"
timeout 3 ros2 topic hz /camera/depth/image_raw 2>&1 | grep "average rate"

echo ""
echo "=== 4. TF树验证 ==="
echo "base_link -> imu_link:"
timeout 2 ros2 run tf2_ros tf2_echo base_link imu_link 2>&1 | grep "Translation" | head -1

echo "base_link -> camera_link:"
timeout 2 ros2 run tf2_ros tf2_echo base_link camera_link 2>&1 | grep "Translation" | head -1

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="

# 清理
kill $LAUNCH_PID 2>/dev/null
sleep 1
pkill -9 -f hardware_bringup
pkill -9 -f astra_camera

echo ""
echo "日志文件: /tmp/hardware_test.log"
echo "查看完整日志: cat /tmp/hardware_test.log"
