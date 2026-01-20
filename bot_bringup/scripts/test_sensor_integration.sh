#!/bin/bash
#
# 传感器集成测试脚本
# Test script for sensor integration (P4)
#
# 功能：
# 1. 启动所有传感器（IMU + 相机）
# 2. 验证话题发布
# 3. 检查TF树
# 4. 测试数据质量
#
# 作者：hurry
# 日期：2026-01-20

set -e  # 遇错退出

echo "========================================="
echo "传感器集成测试 - Sensor Integration Test"
echo "========================================="
echo ""

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_passed() {
    echo -e "${GREEN}✓${NC} $1"
}

check_failed() {
    echo -e "${RED}✗${NC} $1"
}

check_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# ========================================
# 阶段1: 启动传感器
# ========================================

echo "阶段1: 启动传感器节点..."
echo "-----------------------------------"

# 确保工作空间已source
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "错误: ROS2工作空间未source!"
    echo "请执行: source ~/workDisk/lododo_bot/install/setup.bash"
    exit 1
fi

# 后台启动传感器
ros2 launch bot_hardware sensor_bringup.launch.py > /tmp/sensor_test.log 2>&1 &
LAUNCH_PID=$!

echo "传感器启动中... (PID: $LAUNCH_PID)"
echo "等待5秒初始化..."
sleep 5

# 检查进程是否存活
if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    check_failed "传感器启动失败，检查日志: /tmp/sensor_test.log"
    cat /tmp/sensor_test.log | tail -20
    exit 1
fi

check_passed "传感器节点已启动"
echo ""

# ========================================
# 阶段2: 验证话题发布
# ========================================

echo "阶段2: 验证话题发布..."
echo "-----------------------------------"

# 检查IMU话题
echo "检查IMU话题..."
if ros2 topic list | grep -q "/imu/data"; then
    check_passed "IMU话题存在: /imu/data"
    
    # 测试数据接收
    if timeout 3 ros2 topic echo /imu/data --once > /dev/null 2>&1; then
        check_passed "IMU数据正常接收"
    else
        check_warning "IMU话题存在但无数据"
    fi
else
    check_failed "IMU话题缺失: /imu/data"
fi

# 检查相机话题
echo ""
echo "检查相机话题..."
CAMERA_TOPICS=(
    "/camera/color/image_raw"
    "/camera/depth/image_raw"
    "/camera/depth/points"
    "/camera/color/camera_info"
    "/camera/depth/camera_info"
)

for topic in "${CAMERA_TOPICS[@]}"; do
    if ros2 topic list | grep -q "$topic"; then
        check_passed "相机话题存在: $topic"
    else
        check_failed "相机话题缺失: $topic"
    fi
done

# 测试深度数据
echo ""
if timeout 5 ros2 topic echo /camera/depth/image_raw --once > /dev/null 2>&1; then
    check_passed "深度数据正常接收"
else
    check_warning "深度话题存在但无数据（可能需要更长初始化时间）"
fi

echo ""

# ========================================
# 阶段3: 检查TF树
# ========================================

echo "阶段3: 检查TF变换..."
echo "-----------------------------------"

# 检查关键TF
TF_FRAMES=(
    "base_link"
    "imu_link"
    "camera_link"
    "camera_depth_optical_frame"
    "camera_color_optical_frame"
)

echo "检查TF帧..."
for frame in "${TF_FRAMES[@]}"; do
    if ros2 run tf2_ros tf2_echo base_link "$frame" 2>/dev/null | grep -q "At time"; then
        check_passed "TF存在: base_link → $frame"
    else
        check_warning "TF缺失: base_link → $frame"
    fi
done

echo ""

# ========================================
# 阶段4: 数据质量测试
# ========================================

echo "阶段4: 数据质量测试..."
echo "-----------------------------------"

# IMU频率测试
echo "测试IMU发布频率..."
IMU_HZ=$(timeout 5 ros2 topic hz /imu/data 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
if [ ! -z "$IMU_HZ" ]; then
    IMU_HZ_INT=$(echo $IMU_HZ | cut -d'.' -f1)
    if [ $IMU_HZ_INT -ge 8 ]; then
        check_passed "IMU频率: ${IMU_HZ} Hz (目标: 10Hz)"
    else
        check_warning "IMU频率偏低: ${IMU_HZ} Hz"
    fi
else
    check_warning "无法测量IMU频率"
fi

# 深度流频率测试
echo ""
echo "测试深度流发布频率..."
DEPTH_HZ=$(timeout 8 ros2 topic hz /camera/depth/image_raw --window 10 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
if [ ! -z "$DEPTH_HZ" ]; then
    DEPTH_HZ_INT=$(echo $DEPTH_HZ | cut -d'.' -f1)
    if [ $DEPTH_HZ_INT -ge 5 ]; then
        check_passed "深度流频率: ${DEPTH_HZ} Hz (可用)"
    else
        check_warning "深度流频率偏低: ${DEPTH_HZ} Hz"
    fi
else
    check_warning "无法测量深度流频率"
fi

echo ""

# ========================================
# 总结
# ========================================

echo "========================================="
echo "测试完成！"
echo "========================================="
echo ""
echo "命令参考："
echo "  查看IMU数据:    ros2 topic echo /imu/data"
echo "  查看相机话题:   ros2 topic list | grep camera"
echo "  查看TF树:      ros2 run tf2_tools view_frames"
echo "  RViz可视化:    rviz2"
echo ""
echo "停止传感器："
echo "  kill $LAUNCH_PID"
echo "  或执行: pkill -f sensor_bringup"
echo ""
echo "日志位置: /tmp/sensor_test.log"
echo ""

# 保持脚本运行，等待用户中断
echo "传感器正在运行中... (按Ctrl+C停止)"
wait $LAUNCH_PID
