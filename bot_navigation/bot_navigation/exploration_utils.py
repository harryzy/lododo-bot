#!/usr/bin/env python3
"""
Exploration Utils - 探索建图工具模块 / Exploration Mapping Utility Module

提供探索建图过程中使用的通用工具函数和辅助类
Provides general utility functions and helper classes for exploration mapping

Author: LeKiwi Bot Development Team
Date: 2025-12-23
"""

import math
import numpy as np
from typing import Tuple, Optional, List
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid


class CoordinateConverter:
    """坐标转换工具类 / Coordinate Conversion Utility Class"""
    
    @staticmethod
    def world_to_map(x: float, y: float, map_msg: OccupancyGrid) -> Optional[Tuple[int, int]]:
        """
        世界坐标转地图坐标 / Convert world coordinates to map coordinates
        
        Args:
            x: 世界坐标X / World coordinate X
            y: 世界坐标Y / World coordinate Y
            map_msg: 地图消息 / Map message
            
        Returns:
            地图坐标(mx, my)或None / Map coordinates (mx, my) or None
        """
        if map_msg is None:
            return None
            
        # 计算相对于地图原点的偏移 / Calculate offset relative to map origin
        dx = x - map_msg.info.origin.position.x
        dy = y - map_msg.info.origin.position.y
        
        # 转换为栅格坐标 / Convert to grid coordinates
        mx = int(dx / map_msg.info.resolution)
        my = int(dy / map_msg.info.resolution)
        
        # 边界检查 / Boundary check
        if 0 <= mx < map_msg.info.width and 0 <= my < map_msg.info.height:
            return (mx, my)
        else:
            return None
    
    @staticmethod
    def map_to_world(mx: int, my: int, map_msg: OccupancyGrid) -> Optional[Tuple[float, float]]:
        """
        地图坐标转世界坐标 / Convert map coordinates to world coordinates
        
        Args:
            mx: 地图坐标X / Map coordinate X
            my: 地图坐标Y / Map coordinate Y
            map_msg: 地图消息 / Map message
            
        Returns:
            世界坐标(x, y)或None / World coordinates (x, y) or None
        """
        if map_msg is None:
            return None
            
        # 边界检查 / Boundary check
        if 0 <= mx < map_msg.info.width and 0 <= my < map_msg.info.height:
            # 计算世界坐标 / Calculate world coordinates
            x = map_msg.info.origin.position.x + mx * map_msg.info.resolution
            y = map_msg.info.origin.position.y + my * map_msg.info.resolution
            return (x, y)
        else:
            return None


class MathUtils:
    """数学工具类 / Mathematical Utility Class"""
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        归一化角度到[-π, π] / Normalize angle to [-π, π]
        
        Args:
            angle: 输入角度 / Input angle
            
        Returns:
            归一化后的角度 / Normalized angle
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """
        计算两点间距离 / Calculate distance between two points
        
        Args:
            p1: 点1坐标 / Point 1 coordinates
            p2: 点2坐标 / Point 2 coordinates
            
        Returns:
            距离 / Distance
        """
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)
    
    @staticmethod
    def calculate_angle(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """
        计算从点1到点2的角度 / Calculate angle from point 1 to point 2
        
        Args:
            p1: 起点坐标 / Start point coordinates
            p2: 终点坐标 / End point coordinates
            
        Returns:
            角度 / Angle
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.atan2(dy, dx)
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> List[float]:
        """
        将yaw角转换为四元数 / Convert yaw angle to quaternion
        
        Args:
            yaw: Yaw角度 / Yaw angle
            
        Returns:
            四元数[x, y, z, w] / Quaternion [x, y, z, w]
        """
        return [
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        ]
    
    @staticmethod
    def quaternion_to_yaw(quaternion: Quaternion) -> float:
        """
        从四元数提取yaw角 / Extract yaw angle from quaternion
        
        Args:
            quaternion: 四元数 / Quaternion
            
        Returns:
            Yaw角度 / Yaw angle
        """
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        )


class MapUtils:
    """地图工具类 / Map Utility Class"""
    
    @staticmethod
    def calculate_map_completion(current_map: np.ndarray) -> float:
        """
        计算地图完成度 / Calculate map completion percentage
        
        Args:
            current_map: 当前地图数据 / Current map data
            
        Returns:
            完成度(0-1) / Completion percentage (0-1)
        """
        if current_map is None or current_map.size == 0:
            return 0.0
        
        # 已知区域 = 自由空间(0-50) + 已知障碍物(>50)
        # Known area = free space (0-50) + known obstacles (>50)
        free_cells = np.sum((current_map >= 0) & (current_map <= 50))
        obstacle_cells = np.sum(current_map > 50)
        known_cells = free_cells + obstacle_cells
        
        # 估算可探索的未知区域 / Estimate explorable unknown area
        unknown_cells = np.sum(current_map == -1)
        # 简单策略：未知区域不超过已知自由空间的30%
        # Simple strategy: unknown area not exceeding 30% of known free space
        explorable_unknown = min(unknown_cells, int(free_cells * 0.3))
        
        # 总区域 = 自由空间 + 障碍物 + 可探索未知
        # Total area = free space + obstacles + explorable unknown
        total_cells = free_cells + obstacle_cells + explorable_unknown
        
        if total_cells == 0:
            return 0.0
        
        return known_cells / total_cells
    
    @staticmethod
    def is_valid_map_position(mx: int, my: int, map_data: np.ndarray) -> bool:
        """
        检查地图位置是否有效 / Check if map position is valid
        
        Args:
            mx: 地图坐标X / Map coordinate X
            my: 地图坐标Y / Map coordinate Y
            map_data: 地图数据 / Map data
            
        Returns:
            是否有效 / Whether position is valid
        """
        if map_data is None:
            return False
        
        height, width = map_data.shape
        return 0 <= mx < width and 0 <= my < height
    
    @staticmethod
    def get_map_value(mx: int, my: int, map_data: np.ndarray) -> Optional[int]:
        """
        获取地图指定位置的值 / Get map value at specified position
        
        Args:
            mx: 地图坐标X / Map coordinate X
            my: 地图坐标Y / Map coordinate Y
            map_data: 地图数据 / Map data
            
        Returns:
            地图值或None / Map value or None
        """
        if not MapUtils.is_valid_map_position(mx, my, map_data):
            return None
        return int(map_data[my, mx])


class PoseUtils:
    """位姿工具类 / Pose Utility Class"""
    
    @staticmethod
    def create_pose_stamped(x: float, y: float, yaw: float, frame_id: str = 'map') -> PoseStamped:
        """
        创建位姿消息 / Create pose message
        
        Args:
            x: X坐标 / X coordinate
            y: Y坐标 / Y coordinate
            yaw: Yaw角度 / Yaw angle
            frame_id: 参考坐标系 / Reference frame
            
        Returns:
            位姿消息 / Pose message
        """
        from rclpy.time import Time
        from rclpy.clock import Clock
        
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = Clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 设置朝向 / Set orientation
        quat = MathUtils.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose


# 全局工具实例 / Global utility instances
coordinate_converter = CoordinateConverter()
math_utils = MathUtils()
map_utils = MapUtils()
pose_utils = PoseUtils()