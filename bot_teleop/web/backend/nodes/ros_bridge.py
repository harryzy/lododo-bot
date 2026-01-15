#!/usr/bin/env python3
"""
ROS Bridge - Bridge other ROS topics to WebSocket
Provides real-time topic data (such as robot pose, map data, etc.)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from typing import Optional, Callable, Dict, Any
import json


class ROSBridgeNode(Node):
    """ROS topic bridge node"""
    
    def __init__(self, data_callback: Optional[Callable] = None):
        """
        Initialize node
        
        Args:
            data_callback: Data callback function (for pushing to WebSocket)
        """
        super().__init__('ros_bridge_node')
        
        self.data_callback = data_callback
        self._shutdown_flag = False
        
        # Subscribe to robot pose
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/rtabmap/localization_pose',
            self._pose_callback,
            10
        )
        
        # Subscribe to map
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )
        
        # Subscribe to laser scan (optional)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        
        # Cache latest data
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_map: Optional[OccupancyGrid] = None
        
        self.get_logger().info('ROS Bridge Node initialized')
    
    def _pose_callback(self, msg: PoseStamped):
        """Robot pose callback"""
        self.latest_pose = msg
        
        # Extract key information
        pose_data = {
            "type": "robot_pose",
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "orientation": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w
            },
            "frame_id": msg.header.frame_id,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
        
        # Push to WebSocket
        if self.data_callback:
            self.data_callback(pose_data)
    
    def _map_callback(self, msg: OccupancyGrid):
        """Map data callback"""
        self.latest_map = msg
        
        # 地图数据较大，只发送元数据，实际栅格数据通过 REST API 获取
        map_info = {
            "type": "map_info",
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "z": msg.info.origin.position.z
            },
            "frame_id": msg.header.frame_id,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
        
        if self.data_callback:
            self.data_callback(map_info)
    
    def _scan_callback(self, msg: LaserScan):
        """Laser scan callback (optional, large data volume)"""
        # Can selectively push laser scan data
        # Not pushing here to avoid WebSocket data overload
        pass
    
    def get_latest_pose(self) -> Optional[Dict[str, Any]]:
        """Get latest robot pose"""
        if self.latest_pose is None:
            return None
        
        msg = self.latest_pose
        return {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "orientation": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w
            },
            "frame_id": msg.header.frame_id,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
    
    def get_latest_map_info(self) -> Optional[Dict[str, Any]]:
        """Get latest map information"""
        if self.latest_map is None:
            return None
        
        msg = self.latest_map
        return {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y
            }
        }
    
    def spin(self):
        """Run node"""
        while rclpy.ok() and not self._shutdown_flag:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def shutdown(self):
        """关闭节点"""
        self._shutdown_flag = True
        self.get_logger().info('ROS Bridge Node 正在关闭...')
        self.destroy_node()
