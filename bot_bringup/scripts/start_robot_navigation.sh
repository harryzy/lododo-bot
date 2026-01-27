#!/bin/bash
set -e

cd ../../../
source install/setup.bash

# 启动导航和SLAM节点
ros2 launch bot_bringup remote_navigation.launch.py \
  slam:=true \
  use_rviz:=true