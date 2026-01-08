#!/bin/bash
# 运行单个集成测试的便捷脚本
# Convenient script to run single integration test

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Single Test Runner${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# 检查是否提供了测试名称或编号
if [ $# -eq 0 ]; then
    echo -e "${YELLOW}Usage: $0 <test_name_or_number>${NC}"
    echo ""
    echo "Available tests:"
    python3 test_integration.py --list
    exit 1
fi

TEST_NAME=$1

# 检查必要的节点是否运行
echo -e "${CYAN}Checking system status...${NC}"

check_node() {
    local node_name=$1
    if ros2 node list | grep -q "$node_name"; then
        echo -e "${GREEN}✓${NC} $node_name is running"
        return 0
    else
        echo -e "${RED}✗${NC} $node_name is NOT running"
        return 1
    fi
}

all_nodes_ok=true

check_node "/command_adapter" || all_nodes_ok=false
check_node "/mission_planner" || all_nodes_ok=false

echo ""

if [ "$all_nodes_ok" = false ]; then
    echo -e "${RED}ERROR: Required nodes are not running!${NC}"
    echo ""
    echo -e "${YELLOW}Please start the test environment first:${NC}"
    echo "  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=exploration_test"
    echo ""
    exit 1
fi

# 运行测试
echo -e "${CYAN}Running test: ${TEST_NAME}${NC}"
echo ""

python3 test_integration.py --test "$TEST_NAME" --verbose

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Test completed${NC}"
echo -e "${CYAN}========================================${NC}"
