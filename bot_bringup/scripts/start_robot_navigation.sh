#!/bin/bash
set -e
# 启动导航和SLAM节点
ros2 launch bot_bringup remote_navigation.launch.py \
  slam:=true \
  use_rviz:=true