#!/usr/bin/env python3
"""
PatrolManager - 巡航管理器

功能 / Features:
  - 巡航路线管理
  - 巡航执行逻辑
  - 循环模式支持
  - 巡航路线持久化

Author: LeKiwi Bot Development Team
Date: 2025-12-22
"""

from typing import List, Optional, Dict
from dataclasses import dataclass, asdict
from geometry_msgs.msg import PoseStamped
import json
import os
from datetime import datetime


@dataclass
class PatrolRoute:
    """巡航路线数据结构"""
    route_id: str
    name: str
    waypoints: List[Dict]  # PoseStamped 的字典表示
    loop: bool = True  # 是否循环巡航
    created_at: str = None
    
    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now().isoformat()
    
    def to_dict(self) -> Dict:
        """转换为字典"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'PatrolRoute':
        """从字典创建"""
        return cls(**data)


class PatrolManager:
    """
    巡航管理器
    
    负责巡航路线的创建、管理和执行
    """
    
    def __init__(self, persistence_dir: str = None):
        """
        初始化巡航管理器
        
        Args:
            persistence_dir: 巡航数据持久化目录
        """
        self._routes: Dict[str, PatrolRoute] = {}
        self._current_route_id: Optional[str] = None
        self._current_waypoint_index: int = 0
        self._is_patrolling: bool = False
        
        # 持久化目录
        if persistence_dir is None:
            home = os.path.expanduser('~')
            persistence_dir = os.path.join(home, '.ros', 'lekiwi_bot', 'navigation', 'patrol_routes')
        
        self._persistence_dir = persistence_dir
        os.makedirs(self._persistence_dir, exist_ok=True)
    
    def create_route(self, 
                    name: str,
                    waypoints: List[PoseStamped],
                    loop: bool = True,
                    route_id: Optional[str] = None) -> PatrolRoute:
        """
        创建巡航路线
        
        Args:
            name: 路线名称
            waypoints: 路点列表
            loop: 是否循环
            route_id: 路线ID（可选）
            
        Returns:
            PatrolRoute: 创建的路线
        """
        if route_id is None:
            route_id = f"route_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # 转换 PoseStamped 为字典
        waypoint_dicts = [self._pose_to_dict(wp) for wp in waypoints]
        
        route = PatrolRoute(
            route_id=route_id,
            name=name,
            waypoints=waypoint_dicts,
            loop=loop
        )
        
        self._routes[route_id] = route
        self._save_route(route)
        
        return route
    
    def get_route(self, route_id: str) -> Optional[PatrolRoute]:
        """获取路线"""
        return self._routes.get(route_id)
    
    def get_all_routes(self) -> List[PatrolRoute]:
        """获取所有路线"""
        return list(self._routes.values())
    
    def delete_route(self, route_id: str) -> bool:
        """删除路线"""
        if route_id not in self._routes:
            return False
        
        del self._routes[route_id]
        
        # 删除持久化文件
        route_file = os.path.join(self._persistence_dir, f'{route_id}.json')
        if os.path.exists(route_file):
            os.remove(route_file)
        
        return True
    
    def start_patrol(self, route_id: str) -> bool:
        """
        开始巡航
        
        Args:
            route_id: 路线ID
            
        Returns:
            bool: 成功返回True
        """
        if route_id not in self._routes:
            return False
        
        self._current_route_id = route_id
        self._current_waypoint_index = 0
        self._is_patrolling = True
        
        return True
    
    def stop_patrol(self):
        """停止巡航"""
        self._is_patrolling = False
        self._current_route_id = None
        self._current_waypoint_index = 0
    
    def get_next_waypoint(self) -> Optional[Dict]:
        """
        获取下一个巡航点
        
        Returns:
            Dict: 路点字典，如果没有返回None
        """
        if not self._is_patrolling or self._current_route_id is None:
            return None
        
        route = self._routes.get(self._current_route_id)
        if route is None or len(route.waypoints) == 0:
            return None
        
        # 获取当前路点
        waypoint = route.waypoints[self._current_waypoint_index]
        
        # 更新索引
        self._current_waypoint_index += 1
        
        # 检查是否到达终点
        if self._current_waypoint_index >= len(route.waypoints):
            if route.loop:
                self._current_waypoint_index = 0  # 循环
            else:
                self._is_patrolling = False  # 结束
        
        return waypoint
    
    def get_current_progress(self) -> Dict:
        """
        获取当前巡航进度
        
        Returns:
            Dict: 进度信息
        """
        if not self._is_patrolling or self._current_route_id is None:
            return {
                'is_patrolling': False,
                'route_id': None,
                'progress': 0.0
            }
        
        route = self._routes.get(self._current_route_id)
        if route is None:
            return {'is_patrolling': False}
        
        progress = self._current_waypoint_index / len(route.waypoints) if len(route.waypoints) > 0 else 0.0
        
        return {
            'is_patrolling': True,
            'route_id': self._current_route_id,
            'route_name': route.name,
            'current_waypoint': self._current_waypoint_index,
            'total_waypoints': len(route.waypoints),
            'progress': progress,
            'loop': route.loop
        }
    
    def load_all_routes(self):
        """从持久化目录加载所有路线"""
        if not os.path.exists(self._persistence_dir):
            return
        
        for filename in os.listdir(self._persistence_dir):
            if filename.endswith('.json'):
                filepath = os.path.join(self._persistence_dir, filename)
                try:
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                    route = PatrolRoute.from_dict(data)
                    self._routes[route.route_id] = route
                except Exception as e:
                    print(f"Failed to load route from {filename}: {e}")
    
    def _save_route(self, route: PatrolRoute):
        """保存路线到文件"""
        filepath = os.path.join(self._persistence_dir, f'{route.route_id}.json')
        with open(filepath, 'w') as f:
            json.dump(route.to_dict(), f, indent=2)
    
    @staticmethod
    def _pose_to_dict(pose: PoseStamped) -> Dict:
        """将 PoseStamped 转换为字典"""
        return {
            'header': {
                'frame_id': pose.header.frame_id,
                'stamp': {
                    'sec': pose.header.stamp.sec,
                    'nanosec': pose.header.stamp.nanosec
                }
            },
            'pose': {
                'position': {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y,
                    'z': pose.pose.position.z
                },
                'orientation': {
                    'x': pose.pose.orientation.x,
                    'y': pose.pose.orientation.y,
                    'z': pose.pose.orientation.z,
                    'w': pose.pose.orientation.w
                }
            }
        }
    
    @staticmethod
    def dict_to_pose(data: Dict) -> PoseStamped:
        """将字典转换为 PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = data['header']['frame_id']
        pose.header.stamp.sec = data['header']['stamp']['sec']
        pose.header.stamp.nanosec = data['header']['stamp']['nanosec']
        pose.pose.position.x = data['pose']['position']['x']
        pose.pose.position.y = data['pose']['position']['y']
        pose.pose.position.z = data['pose']['position']['z']
        pose.pose.orientation.x = data['pose']['orientation']['x']
        pose.pose.orientation.y = data['pose']['orientation']['y']
        pose.pose.orientation.z = data['pose']['orientation']['z']
        pose.pose.orientation.w = data['pose']['orientation']['w']
        return pose
