#!/usr/bin/env python3
"""
LeKiwi三轮全向轮运动学控制器
实现cmd_vel到轮速的转换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import math


class OmniController(Node):
    """三轮全向轮运动学控制器"""

    def __init__(self):
        super().__init__('omni_controller')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.05),      # 轮子半径 (m)
                ('rear_wheel_dist', 0.105),  # 后轮到中心距离 (m)
                ('front_wheel_dist', 0.085), # 前轮到中心距离 (m)
                ('max_wheel_speed', 4.712),  # 最大轮速 (rad/s)
                ('publish_rate', 50.0),      # 发布频率 (Hz)
            ]
        )
        
        # 获取参数
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.L1 = self.get_parameter('rear_wheel_dist').value  # 后轮
        self.L2 = self.get_parameter('front_wheel_dist').value  # 右前轮
        self.L3 = self.get_parameter('front_wheel_dist').value  # 左前轮
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        
        # 轮子方向角 (弧度)
        self.theta1 = math.pi / 2        # 90°  - 后轮
        self.theta2 = 7 * math.pi / 6    # 210° - 右前轮
        self.theta3 = 11 * math.pi / 6   # 330° - 左前轮
        
        # 当前目标速度
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega_z = 0.0
        
        # 订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布关节状态 (仅用于仿真模式，真实硬件会有硬件接口)
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )
        
        # 定时发布关节命令
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_joint_commands)
        
        self.get_logger().info('Three-wheel omni-directional controller started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Rear wheel distance: {self.L1}m, Front wheel distance: {self.L2}m')

    def cmd_vel_callback(self, msg):
        """接收速度命令"""
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_omega_z = msg.angular.z

    def inverse_kinematics(self, vx, vy, omega_z):
        """
        逆运动学：机器人速度 -> 轮子角速度
        
        Args:
            vx: X方向线速度 (m/s)
            vy: Y方向线速度 (m/s)
            omega_z: 角速度 (rad/s)
            
        Returns:
            (w1, w2, w3): 三个轮子的角速度 (rad/s)
        """
        R = self.wheel_radius
        
        # 运动学方程
        # ω = (1/R) * [-sin(θ)*Vx + cos(θ)*Vy + L*ωz]
        
        w1 = (1/R) * (-np.sin(self.theta1) * vx + 
                      np.cos(self.theta1) * vy + 
                      self.L1 * omega_z)
        
        w2 = (1/R) * (-np.sin(self.theta2) * vx + 
                      np.cos(self.theta2) * vy + 
                      self.L2 * omega_z)
        
        w3 = (1/R) * (-np.sin(self.theta3) * vx + 
                      np.cos(self.theta3) * vy + 
                      self.L3 * omega_z)
        
        # 限制轮速
        w1 = np.clip(w1, -self.max_wheel_speed, self.max_wheel_speed)
        w2 = np.clip(w2, -self.max_wheel_speed, self.max_wheel_speed)
        w3 = np.clip(w3, -self.max_wheel_speed, self.max_wheel_speed)
        
        return w1, w2, w3

    def publish_joint_commands(self):
        """发布关节命令"""
        # 计算轮速
        w1, w2, w3 = self.inverse_kinematics(
            self.target_vx,
            self.target_vy,
            self.target_omega_z
        )
        
        # 创建关节状态消息
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'wheel_rear_joint',
            'wheel_right_joint',
            'wheel_left_joint'
        ]
        joint_state.velocity = [w1, w2, w3]
        
        # 发布
        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    controller = OmniController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
