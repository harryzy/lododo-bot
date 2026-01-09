#!/usr/bin/env python3
"""
ROS Bridge - 用于桥接其他 ROS 话题到 WebSocket
提供实时话题数据（如机器人位姿、地图数据等）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from typing import Optional, Callable, Dict, Any
import json


class ROSBridgeNode(Node):
    """ROS 话题桥接节点"""
    
    def __init__(self, data_callback: Optional[Callable] = None):
        """
        初始化节点
        
        Args:
            data_callback: 数据回调函数（用于推送到 WebSocket）
        """
        super().__init__('ros_bridge_node')
        
        self.data_callback = data_callback
        self._shutdown_flag = False
        
        # 订阅机器人位姿
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/rtabmap/localization_pose',
            self._pose_callback,
            10
        )
        
        # 订阅地图
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )
        
        # 订阅激光扫描（可选）
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        
        # 缓存最新数据
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_map: Optional[OccupancyGrid] = None
        
        self.get_logger().info('ROS Bridge Node 已初始化')
    
    def _pose_callback(self, msg: PoseStamped):
        """机器人位姿回调"""
        self.latest_pose = msg
        
        # 提取关键信息
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
        
        # 推送到 WebSocket
        if self.data_callback:
            self.data_callback(pose_data)
    
    def _map_callback(self, msg: OccupancyGrid):
        """地图数据回调"""
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
        """激光扫描回调（可选，数据量大）"""
        # 可以选择性地推送激光扫描数据
        # 这里暂时不推送，避免 WebSocket 数据过载
        pass
    
    def get_latest_pose(self) -> Optional[Dict[str, Any]]:
        """获取最新机器人位姿"""
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
        """获取最新地图信息"""
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
        """运行节点"""
        while rclpy.ok() and not self._shutdown_flag:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def shutdown(self):
        """关闭节点"""
        self._shutdown_flag = True
        self.get_logger().info('ROS Bridge Node 正在关闭...')
        self.destroy_node()
