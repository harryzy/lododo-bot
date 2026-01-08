#!/bin/bash
#
# CommandAdapter性能测试运行脚本
# Command Adapter Performance Test Runner
#
# 使用方法:
#   ./run_performance_tests.sh                # 交互式菜单
#   ./run_performance_tests.sh --all          # 运行所有测试
#   ./run_performance_tests.sh --all --quick  # 快速模式
#   ./run_performance_tests.sh --test 1       # 运行测试1
#   ./run_performance_tests.sh --list         # 列出所有测试
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
TEST_SCRIPT="$SCRIPT_DIR/../test/test_performance.py"

# 检查测试脚本是否存在
if [ ! -f "$TEST_SCRIPT" ]; then
    echo -e "${RED}Error: Test script not found: $TEST_SCRIPT${NC}"
    exit 1
fi

# 检查依赖
echo -e "${CYAN}Checking dependencies...${NC}"
python3 -c "import psutil" 2>/dev/null || {
    echo -e "${YELLOW}Installing psutil...${NC}"
    pip3 install psutil --user
}

python3 -c "import colorama" 2>/dev/null || {
    echo -e "${YELLOW}Installing colorama...${NC}"
    pip3 install colorama --user
}

# 检查是否有命令行参数
if [ $# -gt 0 ]; then
    if [ "$1" == "--list" ]; then
        python3 "$TEST_SCRIPT" --list
        exit 0
    elif [ "$1" == "--all" ]; then
        shift
        echo -e "${CYAN}Running all performance tests...${NC}\n"
        python3 "$TEST_SCRIPT" --verbose "$@"
        exit $?
    elif [ "$1" == "--test" ] && [ -n "$2" ]; then
        shift
        test_arg="$1"
        shift
        echo -e "${CYAN}Running test: $test_arg${NC}\n"
        python3 "$TEST_SCRIPT" --test "$test_arg" --verbose "$@"
        exit $?
    elif [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
        echo "Usage:"
        echo "  $0                           # Interactive menu"
        echo "  $0 --all                     # Run all tests"
        echo "  $0 --all --quick             # Run all tests (quick mode)"
        echo "  $0 --test <name|number>      # Run specific test"
        echo "  $0 --list                    # List all tests"
        echo ""
        echo "Prerequisites:"
        echo "  Terminal 1: ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=exploration_test"
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
    echo "CommandAdapter Performance Tests"
    echo "================================${NC}"
    echo ""
    echo "Available Tests:"
    echo "  1) Throughput Test - 吞吐量测试"
    echo "  2) Concurrent Requests - 并发请求测试"
    echo "  3) Response Time Distribution - 响应时间分布"
    echo "  4) Queue Capacity - 队列容量测试"
    echo "  5) Sustained Load - 持续负载测试"
    echo ""
    echo "Options:"
    echo "  A) Run all tests (normal mode)"
    echo "  Q) Run all tests (quick mode)"
    echo "  L) List test details"
    echo "  X) Exit"
    echo ""
}

run_test() {
    local test_num=$1
    echo ""
    echo -e "${CYAN}Running test $test_num...${NC}"
    python3 "$TEST_SCRIPT" --test "$test_num" --verbose
    
    echo ""
    echo -e "${YELLOW}Press Enter to continue...${NC}"
    read
}

run_all_tests() {
    local mode=$1
    echo ""
    if [ "$mode" == "quick" ]; then
        echo -e "${CYAN}Running all tests (quick mode)...${NC}"
        python3 "$TEST_SCRIPT" --verbose --quick
    else
        echo -e "${CYAN}Running all tests (normal mode)...${NC}"
        python3 "$TEST_SCRIPT" --verbose
    fi
    
    echo ""
    echo -e "${YELLOW}Press Enter to continue...${NC}"
    read
}

list_tests() {
    python3 "$TEST_SCRIPT" --list
    echo ""
    echo -e "${YELLOW}Press Enter to continue...${NC}"
    read
}

# 主循环
while true; do
    show_menu
    read -p "Select option: " choice
    
    case $choice in
        1|2|3|4|5)
            run_test $choice
            ;;
        [Aa])
            run_all_tests "normal"
            ;;
        [Qq])
            run_all_tests "quick"
            ;;
        [Ll])
            list_tests
            ;;
        [Xx])
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option${NC}"
            sleep 1
            ;;
    esac
done
