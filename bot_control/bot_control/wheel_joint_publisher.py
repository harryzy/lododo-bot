#!/usr/bin/env python3
"""
轮子关节状态发布器
为3个全向轮的舵机旋转关节发布状态
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState


class WheelJointPublisher(Node):
    """发布轮子关节的默认状态"""
    
    def __init__(self):
        super().__init__('wheel_joint_publisher')
        
        # 设置QoS以匹配robot_state_publisher的订阅者
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 创建发布者
        self.publisher = self.create_publisher(JointState, 'joint_states', qos)
        
        # 创建定时器，以10Hz频率发布
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # 关节名称
        self.joint_names = [
            'ST3215_Servo_Motor-v1_Revolute',
            'ST3215_Servo_Motor-v1-1_Revolute',
            'ST3215_Servo_Motor-v1-2_Revolute',
        ]
        
        self.get_logger().info('Wheel joint publisher started')
    
    def publish_joint_states(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        # 所有关节位置初始为0
        msg.position = [0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]
        
        self.publisher.publish(msg)
        # self.get_logger().debug(f'Published joint states at {msg.header.stamp}')


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
