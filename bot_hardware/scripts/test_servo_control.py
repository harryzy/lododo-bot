#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_servo_control.py - 单独舵机控制测试脚本（ROS2版本）
Single Servo Control Test Script (ROS2 Version)

功能 / Features:
- 通过ROS2话题 /wheel/direct_speeds 直接控制单个舵机
- 需要先启动 hardware_bringup.launch.py
- 指定舵机ID(1/2/3)、速度和运行时长
- 实时订阅 /wheel/odom 监控反馈

前置条件 / Prerequisites:
    启动硬件节点:
    ros2 launch bot_hardware hardware_bringup.launch.py

使用示例 / Usage:
    # 测试1号轮（舵机7），速度10 rad/s，运行5秒
    python3 test_servo_control.py --wheel-id 1 --speed 10 --duration 5
    
    # 测试2号轮（舵机8），反向旋转
    python3 test_servo_control.py --wheel-id 2 --speed -15 --duration 3
    
    # 测试所有轮子（依次测试）
    python3 test_servo_control.py --test-all --speed 10 --duration 3

Author: Hurry
Created: 2026-01-20
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import argparse
import time
import sys
import os
import yaml


def load_servo_id_mapping():
    """
    从配置文件加载轮子ID到舵机ID的映射 / Load wheel ID to servo ID mapping from config
    
    Returns:
        dict: {1: servo_id_1, 2: servo_id_2, 3: servo_id_3}
    """
    config_path = os.path.expanduser(
        '~/workDisk/lododo_bot/src/bot_hardware/config/hardware_config.yaml'
    )
    
    if not os.path.exists(config_path):
        print(f"❌ 配置文件不存在: {config_path}")
        return None
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # 读取舵机ID映射
    servo_config = config.get('servo', {})
    mapping = {
        1: servo_config.get('wheel_1_id'),
        2: servo_config.get('wheel_2_id'),
        3: servo_config.get('wheel_3_id')
    }
    
    return mapping


class ServoControlTester(Node):
    """舵机控制测试节点 / Servo Control Test Node"""
    
    def __init__(self, wheel_id, speed_rad_s, duration, servo_mapping):
        super().__init__('servo_control_tester')
        
        self.wheel_id = wheel_id  # 1, 2, 或 3
        self.speed_rad_s = speed_rad_s
        self.duration = duration
        self.servo_mapping = servo_mapping  # 轮子ID到舵机ID的映射
        
        # 速度合法性检查 / Speed validity check
        rpm_value = abs(speed_rad_s) * 30 / 3.14159  # rad/s → RPM
        max_rpm = 45  # ST3215最大RPM
        if rpm_value > max_rpm:
            self.get_logger().warn(
                f'⚠️  速度超限: {rpm_value:.1f} RPM > {max_rpm} RPM (max)'
            )
            self.get_logger().warn(
                f'建议使用速度: {max_rpm * 3.14159 / 30:.1f} rad/s 以内'
            )
            self.get_logger().warn('舵机可能无法达到目标速度，将受限于硬件性能')
        
        # 速度合法性检查 / Speed validity check
        rpm_value = abs(speed_rad_s) * 30 / 3.14159  # rad/s → RPM
        max_rpm = 45  # ST3215最大RPM
        if rpm_value > max_rpm:
            self.get_logger().warn(
                f'⚠️  速度超限: {rpm_value:.1f} RPM > {max_rpm} RPM (max)'
            )
            self.get_logger().warn(
                f'建议使用速度: {max_rpm * 3.14159 / 30:.1f} rad/s 以内'
            )
            self.get_logger().warn('舵机可能无法达到目标速度，将受限于硬件性能')
        
        # 创建发布器 / Create publisher
        self.speed_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel/direct_speeds',
            10
        )
        
        # 创建订阅器监控里程计 / Create subscriber to monitor odometry
        # 使用系统默认 QoS，确保与硬件节点发布的话题兼容
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wheel/odom',
            self.odom_callback,
            10
        )
        
        # 状态变量 / State variables
        self.initial_position = None  # (x, y) 初始位置
        self.last_position = None     # (x, y) 上次位置
        self.start_time = None
        self.test_running = False
        self.odom_received_count = 0  # 接收到的里程计消息计数
        
        servo_id = self.servo_mapping[wheel_id]
        self.get_logger().info('ServoControlTester initialized')
        self.get_logger().info(f'Target: Wheel {wheel_id} (Servo {servo_id}), Speed: {speed_rad_s:.2f} rad/s, Duration: {duration}s')
    
    def odom_callback(self, msg):
        """里程计回调 / Odometry callback"""
        self.odom_received_count += 1
        
        if not self.test_running:
            return
        
        # 提取x, y位置 / Extract x, y position
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_pos = (current_x, current_y)
        
        if self.initial_position is None:
            self.initial_position = current_pos
            self.last_position = current_pos
            return
        
        # 计算总位移（2D距离） / Calculate total displacement (2D distance)
        dx = current_x - self.initial_position[0]
        dy = current_y - self.initial_position[1]
        total_displacement = (dx**2 + dy**2)**0.5
        
        # 计算瞬时变化 / Calculate instantaneous change
        delta_x = current_x - self.last_position[0]
        delta_y = current_y - self.last_position[1]
        delta_distance = (delta_x**2 + delta_y**2)**0.5
        
        # 计算瞬时速度（2D） / Calculate instantaneous velocity (2D)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        velocity_2d = (vx**2 + vy**2)**0.5
        
        elapsed = time.time() - self.start_time
        
        # 每0.5秒打印一次 / Print every 0.5s
        if int(elapsed * 2) != int((elapsed - 0.02) * 2):
            self.get_logger().info(
                f'[{elapsed:.1f}s] 总位移: {total_displacement:.3f}m (X:{dx:.3f}, Y:{dy:.3f}), '
                f'瞬时速度: {velocity_2d:.3f}m/s (vx:{vx:.3f}, vy:{vy:.3f})'
            )
        
        self.last_position = current_pos
    
    def run_test(self):
        """运行测试 / Run test"""
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('单舵机控制测试开始 / Single Servo Control Test Started')
        self.get_logger().info('=' * 70)
        
        # 构造速度命令 / Construct speed command
        # wheel_id: 1->index 0, 2->index 1, 3->index 2
        speeds = [0.0, 0.0, 0.0]
        speeds[self.wheel_id - 1] = self.speed_rad_s
        
        self.get_logger().info(f'\n[1] 发送速度指令...')
        self.get_logger().info(f'    轮子速度: [{speeds[0]:.2f}, {speeds[1]:.2f}, {speeds[2]:.2f}] rad/s')
        self.get_logger().info(f'    控制轮子{self.wheel_id}: {self.speed_rad_s:.2f} rad/s')
        
        # 创建速度消息 / Create speed message
        msg = Float64MultiArray()
        msg.data = speeds
        
        self.get_logger().info(f'\n[2] 等待里程计数据...')
        # 等待至少接收一条里程计消息
        wait_start = time.time()
        while self.odom_received_count == 0 and (time.time() - wait_start) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.odom_received_count == 0:
            self.get_logger().warn('    警告: 3秒内未收到里程计数据，测试可能无法正常统计位移')
            self.get_logger().warn('    请检查 /wheel/odom 话题是否正常发布')
        else:
            self.get_logger().info(f'    已接收 {self.odom_received_count} 条里程计消息')
        
        self.get_logger().info(f'\n[3] 运行监控 ({self.duration}秒)...')
        self.get_logger().info('    (每0.5秒发送一次速度命令以保持运动)')
        self.get_logger().info('-' * 70)
        
        # 开始测试 / Start test
        self.test_running = True
        self.start_time = time.time()
        
        # 持续发送速度命令并监控 / Continuously send speed commands and monitor
        # 每0.5秒发送一次命令，防止看门狗超时（超时时间为2秒）
        last_command_time = time.time()
        command_interval = 0.5  # 500ms
        
        while rclpy.ok() and (time.time() - self.start_time) < self.duration:
            # 定期重新发送速度命令 / Periodically resend speed command
            if time.time() - last_command_time >= command_interval:
                self.speed_pub.publish(msg)
                last_command_time = time.time()
            
            rclpy.spin_once(self, timeout_sec=0.02)
        
        # 停止舵机 / Stop servo
        self.get_logger().info('\n[4] 停止舵机...')
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0]
        self.speed_pub.publish(stop_msg)
        
        # 等待0.5秒确保停止命令生效 / Wait 0.5s for stop command to take effect
        time.sleep(0.5)
        
        # 测试结束 / Test finished
        self.test_running = False
        
        self.get_logger().info(f'\n[5] 测试统计:')
        self.get_logger().info(f'    收到里程计消息数: {self.odom_received_count}')
        
        if self.initial_position is not None and self.last_position is not None:
            # 计算总位移（2D距离） / Calculate total displacement (2D distance)
            dx = self.last_position[0] - self.initial_position[0]
            dy = self.last_position[1] - self.initial_position[1]
            final_displacement = (dx**2 + dy**2)**0.5
            avg_velocity = final_displacement / self.duration
            
            self.get_logger().info(f'    总位移: {final_displacement:.3f}m (X:{dx:.3f}m, Y:{dy:.3f}m)')
            self.get_logger().info(f'    平均速度: {avg_velocity:.3f}m/s')
        else:
            self.get_logger().warn(f'    警告: 未能获取有效的里程计数据')
            self.get_logger().warn(f'    请确认硬件节点正常运行并发布 /wheel/odom 话题')
        
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('测试完成 / Test Completed')
        self.get_logger().info('=' * 70 + '\n')


def test_single_wheel(wheel_id, speed_rad_s, duration):
    """测试单个轮子 / Test single wheel"""
    # 加载舵机ID映射
    servo_mapping = load_servo_id_mapping()
    if servo_mapping is None:
        return False
    
    rclpy.init()
    
    tester = ServoControlTester(wheel_id, speed_rad_s, duration, servo_mapping)
    
    try:
        # 等待1秒让节点初始化 / Wait 1s for node initialization
        time.sleep(1.0)
        
        # 运行测试 / Run test
        tester.run_test()
        
        return True
    
    except KeyboardInterrupt:
        tester.get_logger().warn('\n⚠️  测试被用户中断')
        return False
    
    except Exception as e:
        tester.get_logger().error(f'\n❌ 测试失败: {e}')
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()


def test_all_wheels(speed_rad_s, duration):
    """依次测试所有轮子 / Test all wheels sequentially"""
    print('\n' + '=' * 70)
    print('全轮子顺序测试 / All Wheels Sequential Test')
    print('=' * 70)
    print(f'\n将依次测试轮子: 1, 2, 3')
    print(f'每个轮子运行 {duration} 秒，速度 {speed_rad_s} rad/s')
    
    input('\n按Enter键开始...')
    
    results = {}
    for wheel_id in [1, 2, 3]:
        print(f'\n\n>>> 测试 {wheel_id}/3: 轮子 {wheel_id} <<<')
        success = test_single_wheel(wheel_id, speed_rad_s, duration)
        results[wheel_id] = success
        
        if wheel_id < 3:
            print(f'\n等待3秒后测试下一个轮子...')
            time.sleep(3)
    
    # 汇总结果 / Summary results
    print('\n\n' + '=' * 70)
    print('测试结果汇总 / Test Results Summary')
    print('=' * 70)
    for wheel_id, success in results.items():
        status = '✅ 通过' if success else '❌ 失败'
        print(f'  轮子 {wheel_id}: {status}')
    print('=' * 70)
    
    return all(results.values())


def main():
    """主函数 / Main function"""
    # 加载舵机ID映射用于显示帮助信息
    servo_mapping = load_servo_id_mapping()
    if servo_mapping is None:
        print("无法加载配置文件，使用默认帮助信息")
        mapping_text = """  --wheel-id 1 → 舵机? (从配置文件读取)
  --wheel-id 2 → 舵机? (从配置文件读取)
  --wheel-id 3 → 舵机? (从配置文件读取)"""
    else:
        mapping_text = f"""  --wheel-id 1 → 舵机{servo_mapping[1]} (Wheel 1 → Servo {servo_mapping[1]})
  --wheel-id 2 → 舵机{servo_mapping[2]} (Wheel 2 → Servo {servo_mapping[2]})
  --wheel-id 3 → 舵机{servo_mapping[3]} (Wheel 3 → Servo {servo_mapping[3]})"""
    
    parser = argparse.ArgumentParser(
        description='单舵机控制测试脚本（ROS2版本）/ Single Servo Control Test (ROS2)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
前置条件 / Prerequisites:
  必须先启动硬件节点:
  ros2 launch bot_hardware hardware_bringup.launch.py

轮子ID映射 / Wheel ID Mapping (从hardware_config.yaml读取):
{mapping_text}

示例 / Examples:
  # 测试1号轮，正向10 rad/s，5秒
  python3 test_servo_control.py --wheel-id 1 --speed 10 --duration 5
  
  # 测试2号轮，反向15 rad/s，3秒
  python3 test_servo_control.py --wheel-id 2 --speed -15 --duration 3
  
  # 依次测试所有轮子
  python3 test_servo_control.py --test-all --speed 10 --duration 3
        """
    )
    
    # 构造wheel-id参数的帮助文本
    if servo_mapping:
        wheel_id_help = f'轮子ID / Wheel ID (1=舵机{servo_mapping[1]}, 2=舵机{servo_mapping[2]}, 3=舵机{servo_mapping[3]})'
    else:
        wheel_id_help = '轮子ID / Wheel ID (从配置文件读取映射)'
    
    parser.add_argument(
        '--wheel-id',
        type=int,
        choices=[1, 2, 3],
        help=wheel_id_help
    )
    
    parser.add_argument(
        '--speed',
        type=float,
        default=10.0,
        help='目标速度(rad/s)，负数表示反向 / Target speed (rad/s), negative for reverse (default: 10.0) | 建议范围: -4.7 ~ 4.7 rad/s (≈±45 RPM)'
    )
    
    parser.add_argument(
        '--duration',
        type=float,
        default=5.0,
        help='运行时长(秒) / Duration (seconds, default: 5.0)'
    )
    
    parser.add_argument(
        '--test-all',
        action='store_true',
        help='依次测试所有轮子 / Test all wheels sequentially'
    )
    
    args = parser.parse_args()
    
    # 验证参数 / Validate arguments
    if not args.test_all and args.wheel_id is None:
        parser.error('必须指定 --wheel-id 或 --test-all / Must specify --wheel-id or --test-all')
    
    if args.duration <= 0:
        parser.error('运行时长必须大于0 / Duration must be greater than 0')
    
    # 执行测试 / Execute test
    try:
        if args.test_all:
            success = test_all_wheels(args.speed, args.duration)
        else:
            success = test_single_wheel(args.wheel_id, args.speed, args.duration)
        
        sys.exit(0 if success else 1)
    
    except KeyboardInterrupt:
        print('\n\n⚠️  测试被用户中断')
        sys.exit(130)
    except Exception as e:
        print(f'\n\n❌ 测试过程中发生异常: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
