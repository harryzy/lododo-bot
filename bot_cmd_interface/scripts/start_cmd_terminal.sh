#!/bin/bash
#
# CMD Terminal 启动脚本
# Launch script for CMD Terminal
#
# 使用方法 / Usage:
#   Terminal 1: ./start_cmd_terminal.sh --launch    # 启动测试环境
#   Terminal 2: ./start_cmd_terminal.sh             # 启动终端
#

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# 检查参数
if [ "$1" == "--launch" ]; then
    echo -e "${CYAN}Starting test environment...${NC}"
    echo -e "${YELLOW}This will start Gazebo, Nav2, RTABMap, and CommandAdapter${NC}"
    echo ""
    
    cd ~/lododo_bot
    ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
        slam:=false \
        gui:=false \
        map_name:=exploration_test
    
    exit 0
fi

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS environment not sourced${NC}"
    echo "Please run: source ~/lododo_bot/install/setup.bash"
    exit 1
fi

# 检查CommandAdapter是否运行
echo -e "${CYAN}Checking system status...${NC}"

if ros2 node list | grep -q "/command_adapter"; then
    echo -e "${GREEN}✓ CommandAdapter is running${NC}"
else
    echo -e "${RED}✗ CommandAdapter is NOT running${NC}"
    echo ""
    echo -e "${YELLOW}Please start the test environment first:${NC}"
    echo "  Terminal 1: $0 --launch"
    echo ""
    echo -e "${YELLOW}Or manually:${NC}"
    echo "  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=exploration_test"
    echo ""
    exit 1
fi

# 启动终端
echo -e "${CYAN}Starting CMD Terminal...${NC}"
echo ""

cd ~/lododo_bot/src/bot_cmd_interface/test
python3 cmd_terminal.py
