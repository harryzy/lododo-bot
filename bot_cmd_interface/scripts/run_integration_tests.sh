#!/bin/bash
#
# CommandAdapter集成测试运行脚本
# Command Adapter Integration Test Runner
#
# 前置条件: 启动测试环境
# ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=office_floor1
#
# 使用方法:
#   ./run_integration_tests.sh                   # 交互式菜单
#   ./run_integration_tests.sh --all             # 运行所有测试
#   ./run_integration_tests.sh --test 1          # 运行测试1
#   ./run_integration_tests.sh --test test_name  # 运行指定测试
#   ./run_integration_tests.sh --list            # 列出所有测试
#

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 检测ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS environment not sourced${NC}"
    echo "Please run: source ~/lododo_bot/install/setup.bash"
    exit 1
fi

echo -e "${CYAN}ROS Distribution: $ROS_DISTRO${NC}"

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TEST_SCRIPT="$SCRIPT_DIR/../test/test_integration.py"

# 检查测试脚本是否存在
if [ ! -f "$TEST_SCRIPT" ]; then
    echo -e "${RED}Error: Test script not found: $TEST_SCRIPT${NC}"
    exit 1
fi

# 检查是否有命令行参数
if [ $# -gt 0 ]; then
    if [ "$1" == "--list" ]; then
        python3 "$TEST_SCRIPT" --list
        exit 0
    elif [ "$1" == "--all" ]; then
        echo -e "${CYAN}Running all integration tests...${NC}\n"
        python3 "$TEST_SCRIPT" --verbose
        exit $?
    elif [ "$1" == "--test" ] && [ -n "$2" ]; then
        echo -e "${CYAN}Running test: $2${NC}\n"
        python3 "$TEST_SCRIPT" --test "$2" --verbose
        exit $?
    elif [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
        echo "Usage:"
        echo "  $0                           # Interactive menu"
        echo "  $0 --all                     # Run all tests"
        echo "  $0 --test <name|number>      # Run specific test"
        echo "  $0 --list                    # List all tests"
        echo ""
        echo "Prerequisites:"
        echo "  Terminal 1: ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=office_floor1"
        echo "  Terminal 2: $0"
        exit 0
    else
        echo -e "${RED}Invalid arguments${NC}"
        echo "Run '$0 --help' for usage"
        exit 1
    fi
fi

# 交互式菜单
show_menu() {
    echo ""
    echo -e "${CYAN}================================"
    echo "CommandAdapter Integration Tests"
    echo "================================${NC}"
    echo ""
    echo "Available Tests:"
    echo "  1) Navigate to Pose - 导航到目标点"
    echo "  2) Emergency Stop - 紧急停止"
    echo "  3) Start Exploration - 开始探索"
    echo "  4) Get Task Status - 获取任务状态"
    echo "  5) Pause/Resume/Cancel Task - 暂停/恢复/取消任务"
    echo "  6) Invalid JSON Format - 非法JSON格式"
    echo "  7) Missing Parameters - 参数缺失"
    echo "  8) Request Deduplication - 请求去重"
    echo ""
    echo "Options:"
    echo "  A) Run all tests"
    echo "  L) List test details"
    echo "  Q) Quit"
    echo ""
}

run_test() {
    local test_num=$1
    echo -e "${CYAN}Running test $test_num...${NC}\n"
    python3 "$TEST_SCRIPT" --test "$test_num" --verbose
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo -e "\n${GREEN}Test passed!${NC}"
    else
        echo -e "\n${RED}Test failed!${NC}"
    fi
    
    echo ""
    read -p "Press Enter to continue..."
}

run_all_tests() {
    echo -e "${CYAN}Running all integration tests...${NC}\n"
    python3 "$TEST_SCRIPT" --verbose
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo -e "\n${GREEN}All tests passed!${NC}"
    else
        echo -e "\n${YELLOW}Some tests failed!${NC}"
    fi
    
    echo ""
    read -p "Press Enter to continue..."
}

list_tests() {
    python3 "$TEST_SCRIPT" --list
    echo ""
    read -p "Press Enter to continue..."
}

# 主循环
while true; do
    show_menu
    read -p "Select option: " choice
    
    case $choice in
        [1-8])
            run_test "$choice"
            ;;
        [aA])
            run_all_tests
            ;;
        [lL])
            list_tests
            ;;
        [qQ])
            echo -e "${CYAN}Exiting...${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option${NC}"
            sleep 1
            ;;
    esac
done
