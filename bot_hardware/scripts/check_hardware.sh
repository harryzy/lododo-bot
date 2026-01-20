#!/bin/bash
# 硬件连接检查脚本

echo "========================================="
echo "硬件连接状态检查"
echo "========================================="
echo ""

echo "1. 串口设备 (舵机 + IMU):"
ls -l /dev/tty{ACM,USB}* 2>/dev/null | awk '{print $NF}' || echo "  ❌ 未检测到串口设备"
echo ""

echo "2. USB相机设备 (Astra Pro):"
lsusb | grep -iE "(orbbec|astra|2bc5)" || echo "  ❌ 未检测到Astra Pro相机"
echo ""

echo "3. 用户权限检查:"
groups | grep -q dialout && echo "  ✓ dialout权限正常" || echo "  ⚠️  需要添加dialout权限: sudo usermod -aG dialout $USER"
echo ""

echo "4. udev规则检查:"
if [ -f /etc/udev/rules.d/56-orbbec-usb.rules ]; then
    echo "  ✓ Astra Pro udev规则已安装"
else
    echo "  ⚠️  Astra Pro udev规则未安装"
fi
echo ""

echo "========================================="
echo "建议测试命令："
echo "========================================="
echo "# 测试完整系统（所有传感器）："
echo "ros2 launch bot_hardware hardware_bringup.launch.py"
echo ""
echo "# 测试仅舵机："
echo "ros2 launch bot_hardware hardware_bringup.launch.py enable_sensors:=false"
echo ""
echo "# 测试仅IMU："
echo "ros2 launch bot_hardware hardware_bringup.launch.py enable_camera:=false"
echo ""
