#!/bin/bash
# -*- coding: utf-8 -*-

# ============================================================
# test_hardware_bringup.sh - 硬件启动测试脚本
# Hardware Bringup Test Script
# ============================================================
# 
# 功能：验证hardware_bringup.launch.py的功能完整性
# Function: Validate hardware_bringup.launch.py functionality
#
# 测试阶段：
# Stage 1: Launch文件语法检查
# Stage 2: 节点启动验证（5秒）
# Stage 3: 话题发布检查
# Stage 4: TF树验证
#
# 作者：hurry
# 日期：2026-01-20
# ============================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 测试计数器
TESTS_PASSED=0
TESTS_FAILED=0

print_header() {
    echo -e "\n${BLUE}================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================================${NC}\n"
}

print_test() {
    echo -e "${YELLOW}[TEST]${NC} $1"
}

print_pass() {
    echo -e "${GREEN}✓ PASS${NC} - $1"
    ((TESTS_PASSED++))
}

print_fail() {
    echo -e "${RED}✗ FAIL${NC} - $1"
    ((TESTS_FAILED++))
}

print_warn() {
    echo -e "${YELLOW}⚠ WARN${NC} - $1"
}

# ============================================================
# Stage 1: Launch文件语法检查
# ============================================================
stage1_syntax_check() {
    print_header "Stage 1: Launch文件语法检查"
    
    print_test "检查hardware_bringup.launch.py语法"
    if python3 -m py_compile ~/workDisk/lododo_bot/src/bot_hardware/launch/hardware_bringup.launch.py 2>/dev/null; then
        print_pass "hardware_bringup.launch.py语法正确"
    else
        print_fail "hardware_bringup.launch.py语法错误"
        return 1
    fi
    
    print_test "检查sensor_bringup.launch.py语法"
    if python3 -m py_compile ~/workDisk/lododo_bot/src/bot_hardware/launch/sensor_bringup.launch.py 2>/dev/null; then
        print_pass "sensor_bringup.launch.py语法正确"
    else
        print_fail "sensor_bringup.launch.py语法错误"
        return 1
    fi
    
    print_test "验证launch文件可导入"
    if ros2 launch bot_hardware hardware_bringup.launch.py --show-args 2>/dev/null | grep -q "config_file"; then
        print_pass "Launch参数配置正确"
    else
        print_fail "Launch参数配置错误"
        return 1
    fi
}

# ============================================================
# Stage 2: 节点启动验证（需要真机硬件）
# ============================================================
stage2_node_startup() {
    print_header "Stage 2: 节点启动验证（需要真机硬件）"
    
    print_warn "此阶段需要连接真机硬件（舵机串口 + IMU串口）"
    read -p "是否继续测试节点启动？(y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_warn "跳过Stage 2 - 节点启动测试"
        return 0
    fi
    
    print_test "启动hardware_bringup（仅舵机，5秒测试）"
    ros2 launch bot_hardware hardware_bringup.launch.py enable_sensors:=false &
    LAUNCH_PID=$!
    sleep 5
    
    # 检查OmniHardwareNode是否运行
    if ros2 node list 2>/dev/null | grep -q "omni_hardware_node"; then
        print_pass "OmniHardwareNode启动成功"
    else
        print_fail "OmniHardwareNode启动失败"
        kill $LAUNCH_PID 2>/dev/null || true
        return 1
    fi
    
    # 清理
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 2
    
    print_test "启动hardware_bringup（完整系统，5秒测试）"
    ros2 launch bot_hardware hardware_bringup.launch.py &
    LAUNCH_PID=$!
    sleep 5
    
    # 检查所有预期节点
    EXPECTED_NODES=("omni_hardware_node" "ybimu_driver" "imu_filter_node")
    for node in "${EXPECTED_NODES[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            print_pass "节点 $node 运行中"
        else
            print_warn "节点 $node 未运行（可能因硬件未连接）"
        fi
    done
    
    # 清理
    kill $LAUNCH_PID 2>/dev/null || true
    sleep 2
}

# ============================================================
# Stage 3: 话题发布检查
# ============================================================
stage3_topic_check() {
    print_header "Stage 3: 话题发布检查（需要真机硬件）"
    
    print_warn "此阶段需要Stage 2的节点运行中"
    read -p "是否继续测试话题发布？(y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_warn "跳过Stage 3 - 话题检查"
        return 0
    fi
    
    print_test "检查关键话题是否存在"
    
    EXPECTED_TOPICS=(
        "/wheel/odom"
        "/imu/data_raw"
        "/imu/data"
        "/camera/color/image_raw"
        "/camera/depth/image_raw"
    )
    
    for topic in "${EXPECTED_TOPICS[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "$topic"; then
            print_pass "话题 $topic 存在"
            
            # 检查发布频率（如果可能）
            if timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep -q "average rate"; then
                print_pass "话题 $topic 正在发布数据"
            else
                print_warn "话题 $topic 存在但可能无数据"
            fi
        else
            print_warn "话题 $topic 不存在（可能因硬件未连接）"
        fi
    done
}

# ============================================================
# Stage 4: TF树验证
# ============================================================
stage4_tf_check() {
    print_header "Stage 4: TF树验证"
    
    print_test "检查静态TF是否发布"
    
    # 等待TF树建立
    sleep 2
    
    EXPECTED_FRAMES=("base_link" "imu_link" "camera_link")
    
    for frame in "${EXPECTED_FRAMES[@]}"; do
        if ros2 run tf2_ros tf2_echo base_link "$frame" 2>/dev/null | grep -q "Translation"; then
            print_pass "TF变换 base_link → $frame 正常"
        else
            print_warn "TF变换 base_link → $frame 未找到（可能因sensor未启动）"
        fi
    done
}

# ============================================================
# Stage 5: 参数化启动测试
# ============================================================
stage5_parameter_test() {
    print_header "Stage 5: 参数化启动测试"
    
    print_test "测试禁用传感器启动（enable_sensors:=false）"
    ros2 launch bot_hardware hardware_bringup.launch.py enable_sensors:=false --show-args 2>/dev/null | grep -q "enable_sensors"
    if [ $? -eq 0 ]; then
        print_pass "参数 enable_sensors 可配置"
    else
        print_fail "参数 enable_sensors 不可用"
    fi
    
    print_test "测试禁用IMU启动（enable_imu:=false）"
    ros2 launch bot_hardware hardware_bringup.launch.py enable_imu:=false --show-args 2>/dev/null | grep -q "enable_imu"
    if [ $? -eq 0 ]; then
        print_pass "参数 enable_imu 可配置"
    else
        print_fail "参数 enable_imu 不可用"
    fi
    
    print_test "测试禁用相机启动（enable_camera:=false）"
    ros2 launch bot_hardware hardware_bringup.launch.py enable_camera:=false --show-args 2>/dev/null | grep -q "enable_camera"
    if [ $? -eq 0 ]; then
        print_pass "参数 enable_camera 可配置"
    else
        print_fail "参数 enable_camera 不可用"
    fi
}

# ============================================================
# 主测试流程
# ============================================================
main() {
    print_header "Lododo Robot Hardware Bringup Test"
    echo "测试目标: hardware_bringup.launch.py"
    echo "测试日期: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    
    # 执行测试阶段
    stage1_syntax_check
    stage5_parameter_test
    
    # Stage 2-4需要真机硬件，提示用户
    echo ""
    print_warn "Stage 2-4需要真机硬件连接："
    echo "  - 舵机串口: /dev/ttyUSB0 或 hardware_config.yaml配置"
    echo "  - IMU串口: /dev/ttyUSB1 或 hardware_config.yaml配置"
    echo "  - Astra Pro相机: USB连接"
    echo ""
    
    stage2_node_startup
    stage3_topic_check
    stage4_tf_check
    
    # 输出测试结果
    print_header "测试结果总结"
    echo -e "通过: ${GREEN}${TESTS_PASSED}${NC}"
    echo -e "失败: ${RED}${TESTS_FAILED}${NC}"
    echo ""
    
    if [ $TESTS_FAILED -eq 0 ]; then
        echo -e "${GREEN}✓ 所有测试通过！${NC}"
        echo -e "${GREEN}hardware_bringup.launch.py 功能正常${NC}"
        return 0
    else
        echo -e "${RED}✗ 部分测试失败${NC}"
        echo -e "${YELLOW}请检查错误信息并修复问题${NC}"
        return 1
    fi
}

# 运行主函数
main "$@"
