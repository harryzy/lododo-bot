#!/usr/bin/env python3
"""
轮速监控和分析脚本
实时显示命令轮速和实际轮速，分析偏差
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class WheelSpeedMonitor(Node):
    def __init__(self):
        super().__init__('wheel_speed_monitor')
        
        # 配置QoS - 使用BEST_EFFORT以匹配joint_states的发布者
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅命令速度
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 订阅轮子命令
        self.wheel_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/omni_wheel_controller/commands',
            self.wheel_cmd_callback,
            10
        )
        
        # 订阅关节状态 - 使用BEST_EFFORT QoS
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        # 存储最新数据
        self.latest_cmd_vel = None
        self.latest_wheel_cmd = None
        self.latest_wheel_actual = None
        
        self.get_logger().info('轮速监控器已启动')
        self.get_logger().info('=' * 80)
    
    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.print_status()
    
    def wheel_cmd_callback(self, msg):
        self.latest_wheel_cmd = np.array(msg.data)
    
    def joint_state_callback(self, msg):
        # 提取三个轮子的速度
        try:
            rear_idx = msg.name.index('omni_wheel_mount-v5-2_to_wheel')
            right_idx = msg.name.index('omni_wheel_mount-v5-1_to_wheel')
            left_idx = msg.name.index('omni_wheel_mount-v5_to_wheel')
            
            self.latest_wheel_actual = np.array([
                msg.velocity[rear_idx],   # wheel1
                msg.velocity[right_idx],  # wheel2
                msg.velocity[left_idx]    # wheel3
            ])
        except (ValueError, IndexError):
            pass
    
    def print_status(self):
        if self.latest_cmd_vel is None:
            return
        
        vx = self.latest_cmd_vel.linear.x
        vy = self.latest_cmd_vel.linear.y
        omega = self.latest_cmd_vel.angular.z
        
        # 只在有运动命令时打印
        if abs(vx) < 0.001 and abs(vy) < 0.001 and abs(omega) < 0.001:
            return
        
        print('\n' + '=' * 80)
        print(f'📍 cmd_vel: vx={vx:.3f} m/s, vy={vy:.3f} m/s, omega={omega:.3f} rad/s')
        
        if self.latest_wheel_cmd is not None:
            print(f'📤 命令轮速: [{self.latest_wheel_cmd[0]:6.2f}, {self.latest_wheel_cmd[1]:6.2f}, {self.latest_wheel_cmd[2]:6.2f}] rad/s')
        
        if self.latest_wheel_actual is not None:
            print(f'📥 实际轮速: [{self.latest_wheel_actual[0]:6.2f}, {self.latest_wheel_actual[1]:6.2f}, {self.latest_wheel_actual[2]:6.2f}] rad/s')
            
            if self.latest_wheel_cmd is not None:
                error = self.latest_wheel_actual - self.latest_wheel_cmd
                error_percent = np.divide(error, self.latest_wheel_cmd, 
                                         out=np.zeros_like(error), 
                                         where=np.abs(self.latest_wheel_cmd) > 0.01) * 100
                
                print(f'⚠️  跟踪误差: [{error[0]:6.2f}, {error[1]:6.2f}, {error[2]:6.2f}] rad/s')
                print(f'📊 误差百分比: [{error_percent[0]:5.1f}%, {error_percent[1]:5.1f}%, {error_percent[2]:5.1f}%]')
                
                # 判断是否存在系统性偏差
                if np.max(np.abs(error_percent)) > 10:
                    print('⚠️  警告：存在较大跟踪误差！')
        
        print('=' * 80)


def main(args=None):
    rclpy.init(args=args)
    monitor = WheelSpeedMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
