#!/usr/bin/env python3
"""
舵机连接诊断工具 - Servo Connection Diagnostic Tool
快速检测ST3215舵机连接状态
"""

import sys
import time

# 添加路径以导入驱动
sys.path.insert(0, '/home/hurry/workDisk/lododo_bot/src/bot_hardware')

from bot_hardware.drivers.st3215_driver import ST3215Driver

def test_baudrate(port, baudrate, servo_ids):
    """测试指定波特率"""
    print(f"\n尝试波特率: {baudrate}")
    try:
        driver = ST3215Driver(
            port=port,
            baudrate=baudrate,
            servo_ids=servo_ids,
            retry_count=1,
            retry_delay=0.01
        )
        
        if not driver.initialize():
            print(f"  ❌ 初始化失败")
            return False
        
        print(f"  ✓ 串口打开成功")
        
        # 扫描所有可能的舵机ID (1-253)
        found_servos = []
        print(f"  扫描舵机ID 1-20...")
        for servo_id in range(1, 21):
            try:
                pos = driver.read_position(servo_id)
                if pos is not None:
                    print(f"    ✓ 发现舵机 ID={servo_id}, 位置={pos}")
                    found_servos.append(servo_id)
            except:
                pass
        
        driver.close()
        
        if found_servos:
            print(f"  ✓ 找到 {len(found_servos)} 个舵机: {found_servos}")
            return True
        else:
            print(f"  ❌ 未发现任何舵机")
            return False
            
    except Exception as e:
        print(f"  ❌ 错误: {e}")
        return False

def main():
    print("=" * 60)
    print("ST3215舵机连接诊断")
    print("=" * 60)
    
    port = '/dev/ttyACM0'
    print(f"\n串口: {port}")
    
    # 常见波特率
    baudrates = [1000000, 115200, 57600, 38400, 19200, 9600]
    
    # 配置的舵机ID
    configured_ids = [7, 8, 9]
    print(f"配置的舵机ID: {configured_ids}")
    
    print("\n开始测试不同波特率...")
    
    for baudrate in baudrates:
        if test_baudrate(port, baudrate, configured_ids):
            print(f"\n✓ 成功！使用波特率 {baudrate}")
            break
    else:
        print("\n❌ 所有波特率测试失败")
        print("\n可能原因:")
        print("1. 舵机电源未开启")
        print("2. 串口连接错误")
        print("3. 舵机ID不在1-20范围内")
        print("4. 舵机硬件故障")
        
    print("\n" + "=" * 60)

if __name__ == '__main__':
    main()
