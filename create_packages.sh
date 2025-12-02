#!/bin/bash
# 创建所有功能包框架
# 优先级：仿真相关 > 硬件相关

echo "=========================================="
echo "创建LeKiwi机器人功能包"
echo "=========================================="

# 1. bot_bringup - 系统启动与集成（最重要）
echo "[1/9] 创建 bot_bringup..."
ros2 pkg create --build-type ament_python bot_bringup \
    --dependencies rclpy std_msgs geometry_msgs nav_msgs sensor_msgs

# 2. bot_gazebo - Gazebo仿真（仿真优先）
echo "[2/9] 创建 bot_gazebo..."
ros2 pkg create --build-type ament_python bot_gazebo \
    --dependencies rclpy gazebo_ros gazebo_plugins

# 3. bot_control - 运动控制
echo "[3/9] 创建 bot_control..."
ros2 pkg create --build-type ament_python bot_control \
    --dependencies rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros

# 4. bot_navigation - 导航系统
echo "[4/9] 创建 bot_navigation..."
ros2 pkg create --build-type ament_python bot_navigation \
    --dependencies rclpy nav2_msgs nav_msgs geometry_msgs

# 5. bot_slam - SLAM建图
echo "[5/9] 创建 bot_slam..."
ros2 pkg create --build-type ament_python bot_slam \
    --dependencies rclpy sensor_msgs nav_msgs

# 6. bot_perception - 感知系统
echo "[6/9] 创建 bot_perception..."
ros2 pkg create --build-type ament_python bot_perception \
    --dependencies rclpy sensor_msgs cv_bridge image_transport

# 7. bot_teleop - 远程操控
echo "[7/9] 创建 bot_teleop..."
ros2 pkg create --build-type ament_python bot_teleop \
    --dependencies rclpy geometry_msgs

# 8. bot_hardware - 硬件驱动（后期实现）
echo "[8/9] 创建 bot_hardware..."
ros2 pkg create --build-type ament_python bot_hardware \
    --dependencies rclpy std_msgs sensor_msgs

# 9. bot_voice - 语音控制（后期实现）
echo "[9/9] 创建 bot_voice..."
ros2 pkg create --build-type ament_python bot_voice \
    --dependencies rclpy std_msgs

echo ""
echo "✅ 所有功能包创建完成！"
