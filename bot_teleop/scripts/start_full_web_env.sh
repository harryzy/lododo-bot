#!/bin/bash
# 一键启动完整 Web 控制环境
# 包含: Gazebo + Nav2 + MissionPlanner + Web服务器

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}======================================"
echo -e "  LeKiwi Web 完整环境启动"
echo -e "======================================${NC}"
echo ""

PROJECT_ROOT="$HOME/lododo_bot"

# 检查是否已有 ROS2 运行
if pgrep -x "gzserver" > /dev/null; then
    echo -e "${YELLOW}警告: 检测到 Gazebo 正在运行${NC}"
    read -p "是否继续? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 1. 启动仿真环境（后台）
echo -e "${GREEN}[1/3]${NC} 启动仿真环境..."
gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source $PROJECT_ROOT/install/setup.bash
    ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
        slam:=false \
        map_name:=exploration_test \
        gui:=true
    exec bash
" &

sleep 5

# 2. 启动 Web 后端（新终端）
echo -e "${GREEN}[2/3]${NC} 启动 Web 后端服务器..."
gnome-terminal -- bash -c "
    cd $PROJECT_ROOT/src/bot_teleop
    bash scripts/start_web_server.sh
    exec bash
" &

sleep 3

# 3. 启动前端（新终端）
echo -e "${GREEN}[3/3]${NC} 启动前端开发服务器..."
gnome-terminal -- bash -c "
    cd $PROJECT_ROOT/src/bot_teleop
    bash scripts/start_frontend.sh
    exec bash
" &

echo ""
echo -e "${GREEN}======================================"
echo -e "  启动完成！"
echo -e "======================================${NC}"
echo ""
echo -e "${GREEN}访问地址:${NC}"
echo -e "  Web UI:    http://localhost:3000"
echo -e "  REST API:  http://localhost:8000"
echo -e "  ROSBridge: ws://localhost:9090"
echo ""
echo -e "${YELLOW}提示: 各服务在独立终端窗口运行${NC}"
echo ""
