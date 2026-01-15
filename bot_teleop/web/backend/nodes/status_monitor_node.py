#!/usr/bin/env python3
"""
机器人状态监控节点
订阅机器人位置、速度等状态信息，通过WebSocket广播给前端

订阅话题：
- /rtabmap/localization_pose (geometry_msgs/PoseWithCovarianceStamped) - 机器人位置
- /cmd_vel (geometry_msgs/Twist) - 速度命令
- /odometry/filtered (nav_msgs/Odometry) - 里程计数据（备用）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import math
from typing import Optional, Dict, Any
import asyncio


class StatusMonitorNode(Node):
    """机器人状态监控节点"""
    
    def __init__(self, websocket_handler=None):
        super().__init__('web_status_monitor')
        
        # WebSocket处理器
        self.websocket_handler = websocket_handler
        
        # 机器人状态数据
        self.position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.velocity = {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0}
        self.battery = None  # 预留，暂无电池话题
        
        # 数据接收标志
        self.pose_received = False
        self.vel_received = False
        
        # 订阅话题（使用PoseWithCovarianceStamped）
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self.pose_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 定时广播状态（2 Hz）
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_status)
        
        self.get_logger().info('Status monitor node initialized')
        self.get_logger().info('Subscribed to /localization_pose (PoseWithCovarianceStamped) and /cmd_vel')
        self.get_logger().info('Broadcasting robot status at 2 Hz')
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """处理位置信息（PoseWithCovarianceStamped）"""
        # 提取位置（从pose.pose中获取）
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # 从四元数计算yaw角
        orientation = msg.pose.pose.orientation
        yaw = self._quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        self.position['yaw'] = yaw
        self.pose_received = True
    
    def cmd_vel_callback(self, msg: Twist):
        """处理速度信息"""
        self.velocity['vx'] = msg.linear.x
        self.velocity['vy'] = msg.linear.y
        self.velocity['vyaw'] = msg.angular.z
        self.vel_received = True
    
    def broadcast_status(self):
        """广播机器人状态"""
        if self.websocket_handler is None:
            return
        
        # 构造状态消息
        status_message = {
            'type': 'robot_status',
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'data_available': {
                'pose': self.pose_received,
                'velocity': self.vel_received
            }
        }
        
        # 如果有电池数据，添加到消息中
        if self.battery is not None:
            status_message['battery'] = self.battery
        
        # 通过WebSocket广播（使用同步版本，因为在ROS2回调中）
        try:
            self.websocket_handler.broadcast_status_sync(status_message)
        except Exception as e:
            self.get_logger().warn(f'Failed to broadcast status: {e}')
    
    def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        """
        将四元数转换为yaw角（弧度）
        
        Args:
            x, y, z, w: 四元数分量
            
        Returns:
            yaw角（弧度，范围 -π 到 π）
        """
        # 使用标准的四元数到欧拉角转换公式
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def spin(self):
        """运行节点"""
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
    
    def shutdown(self):
        """关闭节点"""
        self.get_logger().info('Status monitor node shutting down')
        self.destroy_node()
