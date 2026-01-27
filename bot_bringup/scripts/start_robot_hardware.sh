#!/bin/bash
set -e

cd ../../../
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动硬件驱动节点
ros2 launch bot_hardware hardware_bringup.launch.py \
  enable_servo:=true \
  enable_imu:=true \
  enable_camera:=true \
  publish_static_tf:=true