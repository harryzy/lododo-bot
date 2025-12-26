#!/usr/bin/env python3
"""
WaypointRecorder - 路点记录器

功能 / Features:
  - 实时记录机器人路径
  - 路点编辑（添加、删除、修改）
  - 路点可视化
  - 路点持久化

Author: LeKiwi Bot Development Team
Date: 2025-12-22
"""

from typing import List, Optional, Dict
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import yaml
import math
import os
from datetime import datetime


class WaypointRecorder:
    """
    路点记录器
    
    用于记录、编辑和保存路点
    """
    
    def __init__(self, persistence_dir: str = None):
        """
        初始化路点记录器
        
        Args:
            persistence_dir: 路点持久化目录
        """
        self._waypoints: List[PoseStamped] = []
        self._is_recording: bool = False
        self._recording_interval: float = 1.0  # 记录间隔(秒)
        self._last_record_time: Optional[float] = None
        self._min_distance: float = 0.5  # 最小记录距离(m)
        
        # 持久化目录
        if persistence_dir is None:
            home = os.path.expanduser('~')
            persistence_dir = os.path.join(home, '.ros', 'lekiwi_bot', 'navigation', 'waypoints')
        
        self._persistence_dir = persistence_dir
        os.makedirs(self._persistence_dir, exist_ok=True)
    
    def start_recording(self, interval: float = 1.0, min_distance: float = 0.5):
        """
        开始记录路点
        
        Args:
            interval: 记录时间间隔(秒)
            min_distance: 最小记录距离(m)
        """
        self._is_recording = True
        self._recording_interval = interval
        self._min_distance = min_distance
        self._last_record_time = None
        self._waypoints.clear()
    
    def stop_recording(self) -> int:
        """
        停止记录路点
        
        Returns:
            int: 记录的路点数量
        """
        self._is_recording = False
        return len(self._waypoints)
    
    def is_recording(self) -> bool:
        """检查是否正在记录"""
        return self._is_recording
    
    def add_waypoint(self, pose: PoseStamped, current_time: float) -> bool:
        """
        添加路点（需要检查时间间隔和距离）
        
        Args:
            pose: 当前位姿
            current_time: 当前时间(秒)
            
        Returns:
            bool: 成功添加返回True
        """
        if not self._is_recording:
            return False
        
        # 检查时间间隔
        if self._last_record_time is not None:
            if current_time - self._last_record_time < self._recording_interval:
                return False
        
        # 检查距离（与最后一个路点的距离）
        if len(self._waypoints) > 0:
            last_wp = self._waypoints[-1]
            distance = self._calculate_distance(pose, last_wp)
            if distance < self._min_distance:
                return False
        
        # 添加路点
        self._waypoints.append(pose)
        self._last_record_time = current_time
        
        return True
    
    def add_waypoint_manual(self, pose: PoseStamped):
        """手动添加路点（不检查间隔和距离）"""
        self._waypoints.append(pose)
    
    def insert_waypoint(self, index: int, pose: PoseStamped) -> bool:
        """
        在指定位置插入路点
        
        Args:
            index: 插入位置
            pose: 路点位姿
            
        Returns:
            bool: 成功返回True
        """
        if index < 0 or index > len(self._waypoints):
            return False
        
        self._waypoints.insert(index, pose)
        return True
    
    def remove_waypoint(self, index: int) -> bool:
        """
        删除路点
        
        Args:
            index: 路点索引
            
        Returns:
            bool: 成功返回True
        """
        if index < 0 or index >= len(self._waypoints):
            return False
        
        self._waypoints.pop(index)
        return True
    
    def update_waypoint(self, index: int, pose: PoseStamped) -> bool:
        """
        更新路点
        
        Args:
            index: 路点索引
            pose: 新位姿
            
        Returns:
            bool: 成功返回True
        """
        if index < 0 or index >= len(self._waypoints):
            return False
        
        self._waypoints[index] = pose
        return True
    
    def get_waypoints(self) -> List[PoseStamped]:
        """获取所有路点"""
        return self._waypoints.copy()
    
    def get_waypoint(self, index: int) -> Optional[PoseStamped]:
        """获取指定路点"""
        if index < 0 or index >= len(self._waypoints):
            return None
        return self._waypoints[index]
    
    def clear_waypoints(self):
        """清空所有路点"""
        self._waypoints.clear()
    
    def get_waypoint_count(self) -> int:
        """获取路点数量"""
        return len(self._waypoints)
    
    def save_waypoints(self, filename: str) -> bool:
        """
        保存路点到 YAML 文件
        
        Args:
            filename: 文件名（不含路径）
            
        Returns:
            bool: 成功返回True
        """
        if len(self._waypoints) == 0:
            return False
        
        # 确保 .yaml 扩展名
        if not filename.endswith('.yaml'):
            filename = filename + '.yaml'
        
        filepath = os.path.join(self._persistence_dir, filename)
        
        # 转换为 YAML 格式（PatrolNode 兼容）
        waypoints_list = []
        for idx, wp in enumerate(self._waypoints, 1):
            yaw = self._quaternion_to_yaw(
                wp.pose.orientation.x,
                wp.pose.orientation.y,
                wp.pose.orientation.z,
                wp.pose.orientation.w
            )
            waypoints_list.append({
                'name': f'点{idx}',
                'x': float(wp.pose.position.x),
                'y': float(wp.pose.position.y),
                'yaw': float(yaw),
                'dwell_time': 2.0
            })
        
        data = {'waypoints': waypoints_list}
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)
            return True
        except Exception as e:
            print(f"Failed to save waypoints: {e}")
            return False
    
    def load_waypoints(self, filename: str) -> bool:
        """
        从 YAML 文件加载路点
        
        Args:
            filename: 文件名（不含路径）
            
        Returns:
            bool: 成功返回True
        """
        # 确保 .yaml 扩展名
        if not filename.endswith('.yaml'):
            filename = filename + '.yaml'
        
        filepath = os.path.join(self._persistence_dir, filename)
        
        if not os.path.exists(filepath):
            return False
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            # 从 YAML 格式转换为 PoseStamped
            self._waypoints = []
            for wp_data in data.get('waypoints', []):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(wp_data['x'])
                pose.pose.position.y = float(wp_data['y'])
                pose.pose.position.z = 0.0
                
                # yaw 转四元数
                yaw = float(wp_data['yaw'])
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                self._waypoints.append(pose)
            
            return True
        except Exception as e:
            print(f"Failed to load waypoints: {e}")
            return False
    
    def list_saved_waypoints(self) -> List[str]:
        """列出所有保存的路点文件"""
        if not os.path.exists(self._persistence_dir):
            return []
        
        return [f for f in os.listdir(self._persistence_dir) if f.endswith('.yaml')]
    
    def create_visualization_markers(self, namespace: str = 'waypoints') -> MarkerArray:
        """
        创建路点可视化标记
        
        Args:
            namespace: 标记命名空间
            
        Returns:
            MarkerArray: 可视化标记数组
        """
        marker_array = MarkerArray()
        
        # 创建路点球体标记
        for i, waypoint in enumerate(self._waypoints):
            marker = Marker()
            marker.header = waypoint.header
            marker.ns = namespace
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = waypoint.pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            marker_array.markers.append(marker)
        
        # 创建路径线条标记
        if len(self._waypoints) > 1:
            line_marker = Marker()
            line_marker.header = self._waypoints[0].header
            line_marker.ns = namespace + '_path'
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            line_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            
            for waypoint in self._waypoints:
                line_marker.points.append(waypoint.pose.position)
            
            marker_array.markers.append(line_marker)
        
        return marker_array
    
    @staticmethod
    def _calculate_distance(pose1: PoseStamped, pose2: PoseStamped) -> float:
        """计算两个位姿之间的距离"""
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        dz = pose2.pose.position.z - pose1.pose.position.z
        return (dx**2 + dy**2 + dz**2) ** 0.5
    
    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """
        将四元数转换为 yaw 角（绕 z 轴旋转）
        
        Args:
            x, y, z, w: 四元数分量
            
        Returns:
            float: yaw 角度（弧度）
        """
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
