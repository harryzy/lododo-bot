#!/usr/bin/env python3
"""
测试舵机健康检测功能 / Test Servo Health Monitoring

这个脚本独立测试健康监控的单次检测功能，不依赖ROS2节点。
This script independently tests a single health check without ROS2 node dependencies.

用法 / Usage:
    python3 test_servo_health.py
"""

import sys
import os
import yaml
import time

# 添加bot_hardware到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bot_hardware.drivers.st3215_driver import ST3215Driver


def load_config():
    """加载配置文件 / Load configuration file"""
    config_path = os.path.join(
        os.path.dirname(__file__), 
        '../config/hardware_config.yaml'
    )
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def test_single_health_check():
    """测试单次健康检测 / Test single health check"""
    print("=" * 70)
    print("舵机健康检测测试 / Servo Health Check Test")
    print("=" * 70)
    
    # 1. 加载配置
    print("\n[1] 加载配置文件...")
    config = load_config()
    
    servo_ids = [
        config['servo']['wheel_1_id'],
        config['servo']['wheel_2_id'],
        config['servo']['wheel_3_id']
    ]
    print(f"    舵机ID列表: {servo_ids}")
    
    # 2. 初始化驱动
    print("\n[2] 初始化ST3215驱动...")
    driver = ST3215Driver(
        port=config['serial']['servo_port'],
        baudrate=config['serial']['servo_baudrate'],
        servo_ids=servo_ids,
        retry_count=3,
        retry_delay=0.01
    )
    
    # 3. 初始化串口
    print("\n[3] 打开串口...")
    if not driver.initialize():
        print("    ❌ 串口初始化失败！")
        return False
    print("    ✅ 串口初始化成功")
    
    # 4. 单次健康检测
    print("\n[4] 执行单次健康检测...")
    print("-" * 70)
    
    for servo_id in servo_ids:
        print(f"\n  检测舵机 {servo_id}:")
        
        # 4.1 Ping测试
        print(f"    [Ping测试]", end=" ")
        ping_result = driver._ping_servo(servo_id)
        if ping_result:
            print("✅ 在线")
        else:
            print("❌ 无响应")
            continue
        
        # 4.2 读取位置
        print(f"    [位置读取]", end=" ")
        position = driver.read_position(servo_id)
        if position is not None:
            print(f"✅ 位置={position} (0-4095)")
        else:
            print("❌ 读取失败")
        
        # 4.3 读取状态（温度、电压、负载）
        print(f"    [状态读取]", end=" ")
        status = driver.read_status(servo_id)
        if status is not None:
            print(f"✅ 状态数据:")
            print(f"        - 位置: {status.get('position', 'N/A')}")
            print(f"        - 速度: {status.get('speed', 'N/A')}")
            print(f"        - 温度: {status.get('temperature', 'N/A')}°C (TODO)")
            print(f"        - 电压: {status.get('voltage', 'N/A')}V (TODO)")
            print(f"        - 负载: {status.get('load', 'N/A')}% (TODO)")
        else:
            print("❌ 读取失败")
        
        # 4.4 通信统计
        success, failure, rate = driver.get_communication_stats(servo_id)
        print(f"    [通信统计]")
        print(f"        - 成功次数: {success}")
        print(f"        - 失败次数: {failure}")
        print(f"        - 成功率: {rate:.1f}%")
    
    print("\n" + "-" * 70)
    
    # 5. 关闭串口
    print("\n[5] 关闭串口...")
    driver.close()
    print("    ✅ 串口已关闭")
    
    print("\n" + "=" * 70)
    print("测试完成 / Test Completed")
    print("=" * 70)
    
    return True


def test_continuous_health_check(duration=10):
    """测试连续健康检测 / Test continuous health check"""
    print("=" * 70)
    print(f"连续健康检测测试 ({duration}秒) / Continuous Health Check Test ({duration}s)")
    print("=" * 70)
    
    # 1. 加载配置
    print("\n[1] 加载配置文件...")
    config = load_config()
    
    servo_ids = [
        config['servo']['wheel_1_id'],
        config['servo']['wheel_2_id'],
        config['servo']['wheel_3_id']
    ]
    
    # 2. 初始化驱动
    print("\n[2] 初始化ST3215驱动...")
    driver = ST3215Driver(
        port=config['serial']['servo_port'],
        baudrate=config['serial']['servo_baudrate'],
        servo_ids=servo_ids,
        retry_count=3,
        retry_delay=0.01
    )
    
    if not driver.initialize():
        print("    ❌ 初始化失败！")
        return False
    print("    ✅ 初始化成功")
    
    # 3. 连续检测
    print(f"\n[3] 开始连续检测 (每100ms一次)...")
    print("-" * 70)
    
    start_time = time.time()
    check_count = 0
    failure_count = {sid: 0 for sid in servo_ids}
    
    try:
        while time.time() - start_time < duration:
            check_count += 1
            print(f"\n第 {check_count} 次检测 (时间: {time.time() - start_time:.1f}s):")
            
            for servo_id in servo_ids:
                ping_result = driver._ping_servo(servo_id)
                if ping_result:
                    print(f"  舵机 {servo_id}: ✅ 在线")
                else:
                    print(f"  舵机 {servo_id}: ❌ 无响应")
                    failure_count[servo_id] += 1
            
            time.sleep(0.1)  # 100ms间隔
            
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    
    # 4. 统计结果
    print("\n" + "-" * 70)
    print(f"\n[4] 测试统计:")
    print(f"    - 总检测次数: {check_count}")
    print(f"    - 测试时长: {time.time() - start_time:.1f}秒")
    print(f"    - 失败统计:")
    for servo_id in servo_ids:
        success, failure, rate = driver.get_communication_stats(servo_id)
        print(f"        舵机 {servo_id}: 成功={success}, 失败={failure}, 成功率={rate:.1f}%")
    
    # 5. 关闭
    driver.close()
    print("\n" + "=" * 70)
    print("测试完成 / Test Completed")
    print("=" * 70)
    
    return True


def main():
    """主函数 / Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='测试舵机健康检测功能')
    parser.add_argument(
        '--mode', 
        choices=['single', 'continuous'], 
        default='single',
        help='测试模式: single=单次检测, continuous=连续检测'
    )
    parser.add_argument(
        '--duration', 
        type=int, 
        default=10,
        help='连续检测模式的持续时间(秒)'
    )
    
    args = parser.parse_args()
    
    try:
        if args.mode == 'single':
            success = test_single_health_check()
        else:
            success = test_continuous_health_check(args.duration)
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
