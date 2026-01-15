#!/usr/bin/env python3
"""
Robot Status Monitor Node
Subscribe to robot position, velocity and other status information, broadcast to frontend via WebSocket

Subscribed topics:
- /rtabmap/localization_pose (geometry_msgs/PoseWithCovarianceStamped) - Robot position
- /cmd_vel (geometry_msgs/Twist) - Velocity commands
- /odometry/filtered (nav_msgs/Odometry) - Odometry data (backup)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import math
from typing import Optional, Dict, Any
import asyncio


class StatusMonitorNode(Node):
    """Robot status monitor node"""
    
    def __init__(self, websocket_handler=None):
        super().__init__('web_status_monitor')
        
        # WebSocket handler
        self.websocket_handler = websocket_handler
        
        # Robot status data
        self.position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.velocity = {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0}
        self.battery = None  # Reserved, no battery topic yet
        
        # Data reception flags
        self.pose_received = False
        self.vel_received = False
        
        # Subscribe to topics (using PoseWithCovarianceStamped)
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
        
        # Broadcast status periodically (2 Hz)
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_status)
        
        self.get_logger().info('Status monitor node initialized')
        self.get_logger().info('Subscribed to /localization_pose (PoseWithCovarianceStamped) and /cmd_vel')
        self.get_logger().info('Broadcasting robot status at 2 Hz')
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Process position information (PoseWithCovarianceStamped)"""
        # Extract position (from pose.pose)
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Calculate yaw angle from quaternion
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
        """Process velocity information"""
        self.velocity['vx'] = msg.linear.x
        self.velocity['vy'] = msg.linear.y
        self.velocity['vyaw'] = msg.angular.z
        self.vel_received = True
    
    def broadcast_status(self):
        """Broadcast robot status"""
        if self.websocket_handler is None:
            return
        
        # Build status message
        status_message = {
            'type': 'robot_status',
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'data_available': {
                'pose': self.pose_received,
                'velocity': self.vel_received
            }
        }
        
        # If battery data available, add to message
        if self.battery is not None:
            status_message['battery'] = self.battery
        
        # Broadcast via WebSocket (use sync version, as in ROS2 callback)
        try:
            self.websocket_handler.broadcast_status_sync(status_message)
        except Exception as e:
            self.get_logger().warn(f'Failed to broadcast status: {e}')
    
    def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        """
        Convert quaternion to yaw angle (radians)
        
        Args:
            x, y, z, w: Quaternion components
            
        Returns:
            Yaw angle (radians, range -π to π)
        """
        # Use standard quaternion to Euler angle conversion formula
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def spin(self):
        """Run node"""
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown node"""
        self.get_logger().info('Status monitor node shutting down')
        self.destroy_node()
