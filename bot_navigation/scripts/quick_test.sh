#!/bin/bash
# 快速测试脚本 - Mission State Control 改进验证

echo "=========================================="
echo "Mission State Control 改进验证"
echo "=========================================="
echo ""

# 检查是否已source工作空间
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 请先source ROS2工作空间"
    echo "运行: source /opt/ros/humble/setup.bash"
    echo "      source ~/lododo_bot/install/setup.bash"
    exit 1
fi

echo "选择测试模式:"
echo "  1. 列出所有测试用例"
echo "  2. 运行 TEST 1: 导航暂停/恢复（推荐）"
echo "  3. 运行 TEST 1（详细模式）"
echo "  4. 运行所有测试"
echo "  5. 自定义测试"
echo ""
read -p "请选择 (1-5): " choice

case $choice in
    1)
        echo ""
        echo "列出所有测试用例..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --list
        ;;
    2)
        echo ""
        echo "运行 TEST 1: Navigation Pause/Resume"
        echo "请确保已启动仿真环境！"
        echo ""
        read -p "按Enter继续..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test 1
        ;;
    3)
        echo ""
        echo "运行 TEST 1（详细模式）"
        echo "请确保已启动仿真环境！"
        echo ""
        read -p "按Enter继续..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test 1 --verbose
        ;;
    4)
        echo ""
        echo "运行所有测试"
        echo "请确保已启动仿真环境！"
        echo "注意: 部分测试尚未实现"
        echo ""
        read -p "按Enter继续..."
        python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py
        ;;
    5)
        echo ""
        echo "输入测试名称或编号:"
        read test_name
        echo ""
        echo "是否启用详细模式? (y/n)"
        read verbose
        
        if [ "$verbose" = "y" ]; then
            python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test "$test_name" --verbose
        else
            python3 ~/lododo_bot/src/bot_navigation/scripts/test_mission_integration_v2.py --test "$test_name"
        fi
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
