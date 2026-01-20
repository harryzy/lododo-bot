#!/usr/bin/env python3
"""
详细舵机诊断工具 - 底层SDK测试
"""

import sys
import time

sys.path.insert(0, '/home/hurry/workDisk/lododo_bot/src/bot_hardware')

from bot_hardware.ftservo.scservo_sdk import PortHandler, sms_sts

def test_low_level():
    """底层SDK测试"""
    print("=" * 60)
    print("ST3215舵机底层SDK测试")
    print("=" * 60)
    
    port = '/dev/ttyACM0'
    print(f"\n串口: {port}")
    
    for baudrate in [1000000, 115200]:
        print(f"\n{'='*50}")
        print(f"测试波特率: {baudrate}")
        print(f"{'='*50}")
        
        # 初始化SDK
        port_handler = PortHandler(port)
        packet_handler = sms_sts(port_handler)
        
        # 打开串口
        if not port_handler.openPort():
            print(f"  ❌ 无法打开串口")
            continue
        print(f"  ✓ 串口打开成功")
        
        # 设置波特率
        if not port_handler.setBaudRate(baudrate):
            print(f"  ❌ 无法设置波特率")
            port_handler.closePort()
            continue
        print(f"  ✓ 波特率设置成功")
        
        # 扫描舵机ID 1-20
        print(f"\n  扫描舵机ID 1-20...")
        found_servos = []
        
        for servo_id in range(1, 21):
            # 尝试读取位置
            try:
                # SMS/STS协议 PING指令
                scs_present_position_l = 56  # 位置寄存器地址
                
                # 读取2字节位置数据
                scs_present_position, scs_comm_result, scs_error = \
                    packet_handler.read2ByteTxRx(port_handler, servo_id, scs_present_position_l)
                
                if scs_comm_result == 0:  # COMM_SUCCESS
                    print(f"    ✓ 舵机 ID={servo_id} 在线, 位置={scs_present_position}, 通信结果={scs_comm_result}")
                    found_servos.append(servo_id)
                else:
                    # 打印通信失败的详细信息
                    if servo_id <= 10:  # 只打印前10个ID的详细信息
                        print(f"    · ID={servo_id}: 通信结果={scs_comm_result}, 错误={scs_error}")
            except Exception as e:
                if servo_id <= 3:
                    print(f"    · ID={servo_id}: 异常 {e}")
        
        port_handler.closePort()
        
        if found_servos:
            print(f"\n  ✓✓✓ 成功！找到 {len(found_servos)} 个舵机: {found_servos}")
            print(f"  建议配置: servo_ids: {found_servos}")
            return found_servos
        else:
            print(f"\n  ❌ 未找到任何舵机")
    
    return []

if __name__ == '__main__':
    found = test_low_level()
    
    if not found:
        print("\n" + "=" * 60)
        print("诊断建议:")
        print("=" * 60)
        print("1. 检查舵机电源是否开启（LED指示灯）")
        print("2. 确认舵机数据线连接到/dev/ttyACM0")
        print("3. 尝试重新给舵机上电")
        print("4. 使用万用表测试舵机电源电压（应为12V）")
