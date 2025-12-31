#!/bin/bash
# Quick Test Script - Mission State Control Improvement Verification
# 快速测试脚本 - Mission State Control 改进验证

echo "=========================================="
echo "Mission State Control Test / 任务状态控制测试"
echo "=========================================="
echo ""

# Check if workspace is sourced / 检查是否已source工作空间
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: Please source ROS2 workspace first"
    echo "错误: 请先source ROS2工作空间"
    echo ""
    echo "Run / 运行:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source ~/lododo_bot/install/setup.bash"
    exit 1
fi

echo "Select Test Mode / 选择测试模式:"
echo "  1. List All Test Cases / 列出所有测试用例"
echo "  2. Run TEST 1: Navigation Pause/Resume (Recommended) / 运行 TEST 1: 导航暂停/恢复（推荐）"
echo "  3. Run TEST 1 (Verbose Mode) / 运行 TEST 1（详细模式）"
echo "  4. Run All Tests (TEST 1-5, 7-8, Requires Localization Mode) / 运行所有测试（TEST 1-5, 7-8，需要定位模式）"
echo "  5. Run TEST 6: Autonomous Exploration & Mapping (Requires SLAM Mode, Separate Launch) / 运行 TEST 6: 自主探索建图（需要 SLAM 模式，单独启动）"
echo "  6. Custom Test / 自定义测试"
echo ""
read -p "Please select / 请选择 (1-6): " choice

case $choice in
    1)
        echo ""
        echo "Listing all test cases... / 列出所有测试用例..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --list
        ;;
    2)
        echo ""
        echo "Running TEST 1: Navigation Pause/Resume / 运行 TEST 1: 导航暂停/恢复"
        echo "Make sure simulation environment is launched! / 请确保已启动仿真环境！"
        echo ""
        read -p "Press Enter to continue / 按Enter继续..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test 1
        ;;
    3)
        echo ""
        echo "Running TEST 1 (Verbose Mode) / 运行 TEST 1（详细模式）"
        echo "Make sure simulation environment is launched! / 请确保已启动仿真环境！"
        echo ""
        read -p "Press Enter to continue / 按Enter继续..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test 1 --verbose
        ;;
    4)
        echo ""
        echo "Running All Tests (TEST 1-5, 7-8) / 运行所有测试（TEST 1-5, 7-8）"
        echo "Make sure simulation environment is launched (Localization Mode + Existing Map)!"
        echo "请确保已启动仿真环境（定位模式 + 已有地图）！"
        echo ""
        echo "Launch Command Reference / 启动命令参考:"
        echo "  ros2 launch bot_bringup simulation_mission_planner.launch.py \\"
        echo "    world:=office.world \\"
        echo "    map_name:=office_floor1 \\"
        echo "    rtabmap_db_path:=~/lododo_bot/maps/office_floor1/rtabmap.db"
        echo ""
        read -p "Press Enter to continue / 按Enter继续..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test 1,2,3,4,5,7,8
        ;;
    5)
        echo ""
        echo "Running TEST 6: Autonomous Exploration & Mapping / 运行 TEST 6: 自主探索建图"
        echo ""
        echo "⚠️  WARNING: TEST 6 must be run separately in SLAM mode!"
        echo "⚠️  警告: TEST 6 需要在 SLAM 模式下单独运行！"
        echo ""
        echo "Please launch simulation environment with / 请使用以下命令启动仿真环境:"
        echo "  ros2 launch bot_bringup simulation_mission_planner.launch.py \\"
        echo "    world:=new_environment.world \\"
        echo "    use_sim_time:=true"
        echo ""
        echo "Note: Do not load a map, let RTABMap build map in SLAM mode"
        echo "说明: 不要加载地图，让 RTABMap 在 SLAM 模式下建图"
        echo ""
        read -p "Press Enter after confirming SLAM mode is launched / 确认已启动 SLAM 模式后按Enter继续..."
        
        echo ""
        echo "Enable verbose mode? (y/n) / 是否启用详细模式? (y/n)"
        read verbose
        
        if [ "$verbose" = "y" ]; then
            python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test 6 --verbose
        else
            python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test 6
        fi
        ;;
    6)
        echo ""
        echo "Enter test name or number / 输入测试名称或编号:"
        read test_name
        echo ""
        echo "Enable verbose mode? (y/n) / 是否启用详细模式? (y/n)"
        read verbose
        
        if [ "$verbose" = "y" ]; then
            python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test "$test_name" --verbose
        else
            python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test "$test_name"
        fi
        ;;
    *)
        echo "Invalid selection / 无效选择"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Test Completed / 测试完成"
echo "=========================================="
