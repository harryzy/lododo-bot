#!/bin/bash
# 运行所有集成测试的快速脚本

echo "=================================================="
echo "MissionPlanner 集成测试 - 全套测试"
echo "=================================================="
echo ""

cd ~/lododo_bot
source install/setup.bash

# 确保日志目录存在
mkdir -p log

# 运行所有测试
for test_num in {1..5}; do
    echo ""
    echo "=================================================="
    echo "运行测试 $test_num / 5"
    echo "=================================================="
    
    python3 src/bot_navigation/scripts/test_mission_integration_v2.py --test $test_num
    
    result=$?
    if [ $result -ne 0 ]; then
        echo ""
        echo "⚠️ 测试 $test_num 执行失败（退出码: $result）"
        echo "日志保存在: log/mission_test.log"
    fi
    
    # 测试之间等待一下，让系统稳定
    if [ $test_num -lt 5 ]; then
        echo ""
        echo "等待2秒后继续下一个测试..."
        sleep 2
    fi
done

echo ""
echo "=================================================="
echo "所有测试完成！"
echo "=================================================="
echo ""
echo "查看完整日志："
echo "  tail -100 log/mission_test.log"
echo ""
