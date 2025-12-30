#!/usr/bin/env python3
"""
PatrolManager - 巡航管理器

功能 / Features:
  - 巡航路线管理
  - 巡航执行逻辑（使用NavigationExecutor）
  - 循环模式支持
  - 巡航路线持久化
  - 停留时间（dwell_time）支持

Author: LeKiwi Bot Development Team
Date: 2025-12-22
"""

from typing import List, Optional, Dict, Callable
from dataclasses import dataclass, asdict
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from enum import Enum
import yaml
import os
import math
import time
from datetime import datetime


class PatrolState(Enum):
    """巡航状态"""
    IDLE = "idle"                    # 空闲
    NAVIGATING = "navigating"        # 正在导航到路点
    DWELLING = "dwelling"            # 在路点停留
    COMPLETED = "completed"          # 巡航完成（非循环模式）
    PAUSED = "paused"               # 暂停
    FAILED = "failed"               # 失败


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
    集成NavigationExecutor实现实际导航
    """
    
    def __init__(self, node: Node, navigation_executor, persistence_dir: str = None):
        """
        初始化巡航管理器
        
        Args:
            node: ROS2 Node实例
            navigation_executor: NavigationExecutor实例（用于执行导航）
            persistence_dir: 巡航数据持久化目录
        """
        self._node = node
        self._nav_executor = navigation_executor
        self._routes: Dict[str, PatrolRoute] = {}
        self._current_route_id: Optional[str] = None
        self._current_waypoint_index: int = 0
        self._patrol_state: PatrolState = PatrolState.IDLE
        
        # 停留控制
        self._dwell_start_time: Optional[float] = None
        self._current_dwell_time: float = 0.0
        
        # 完成回调
        self._on_complete_callback: Optional[Callable] = None
        self._on_failed_callback: Optional[Callable] = None
        
        # 持久化目录
        if persistence_dir is None:
            persistence_dir = os.path.expanduser('~/lododo_bot/mission/patrol_routes')
        
        self._persistence_dir = persistence_dir
        os.makedirs(self._persistence_dir, exist_ok=True)
        
        self._node.get_logger().info(f'PatrolManager initialized with persistence_dir: {self._persistence_dir}')
    
    def set_complete_callback(self, callback: Callable):
        """设置巡航完成回调"""
        self._on_complete_callback = callback
    
    def set_failed_callback(self, callback: Callable):
        """设置巡航失败回调"""
        self._on_failed_callback = callback
    
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
        route_file = os.path.join(self._persistence_dir, f'{route_id}.yaml')
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
            self._node.get_logger().error(f"Route {route_id} not found")
            return False
        
        route = self._routes[route_id]
        if not route.waypoints or len(route.waypoints) == 0:
            self._node.get_logger().error(f"Route {route_id} has no waypoints")
            return False
        
        self._current_route_id = route_id
        self._current_waypoint_index = 0
        self._patrol_state = PatrolState.IDLE
        self._dwell_start_time = None
        
        self._node.get_logger().info(
            f"Starting patrol: route={route.name}, waypoints={len(route.waypoints)}, loop={route.loop}"
        )
        
        return True
    
    def stop_patrol(self):
        """停止巡航"""
        if self._patrol_state == PatrolState.NAVIGATING:
            # 取消当前导航
            self._nav_executor.cancel_navigation()
        
        self._patrol_state = PatrolState.IDLE
        self._current_route_id = None
        self._current_waypoint_index = 0
        self._dwell_start_time = None
        
        self._node.get_logger().info("Patrol stopped")
    
    def pause_patrol(self):
        """暂停巡航（保留当前路线和进度）"""
        if self._patrol_state == PatrolState.NAVIGATING:
            # 取消当前导航
            self._nav_executor.cancel_navigation()
        
        self._patrol_state = PatrolState.PAUSED
        self._dwell_start_time = None
        
        self._node.get_logger().info(
            f"Patrol paused at waypoint {self._current_waypoint_index}"
        )
    
    def resume_patrol(self) -> bool:
        """恢复巡航（从当前路线和进度继续）"""
        if self._current_route_id is None:
            self._node.get_logger().warn("No patrol route to resume")
            return False
        
        if self._patrol_state != PatrolState.PAUSED:
            self._node.get_logger().warn(f"Cannot resume from state: {self._patrol_state.value}")
            return False
        
        self._patrol_state = PatrolState.IDLE
        self._node.get_logger().info(
            f"Patrol resumed at waypoint {self._current_waypoint_index}"
        )
        return True
    
    def is_patrolling(self) -> bool:
        """
        检查是否正在巡航
        
        Returns:
            bool: 正在巡航返回True（包括IDLE/NAVIGATING/DWELLING状态，但不包括暂停）
        """
        # 有活动路线且未完成/失败即为"正在巡航"
        # IDLE状态表示等待发送下一个导航目标，仍然算作"正在巡航"
        if self._current_route_id is None:
            return False
        return self._patrol_state not in (PatrolState.COMPLETED, PatrolState.FAILED, PatrolState.PAUSED)
    
    def execute_patrol(self):
        """
        执行巡航逻辑（由MissionPlanner定期调用）
        
        这是核心执行方法，处理：
        - 发送导航目标
        - 检查导航完成
        - 停留时间管理
        - 路点切换
        - 循环控制
        """
        # 导入NavigationState
        from ..mission.navigation_executor import NavigationState
        
        # 如果没有活动巡航，直接返回
        if self._current_route_id is None:
            return
        
        # 如果暂停或完成，不执行
        if self._patrol_state in (PatrolState.PAUSED, PatrolState.COMPLETED, PatrolState.FAILED):
            return
        
        # 获取当前路线
        route = self._routes.get(self._current_route_id)
        if route is None:
            self._node.get_logger().error(f"Route {self._current_route_id} not found")
            self._patrol_state = PatrolState.FAILED
            if self._on_failed_callback:
                self._on_failed_callback("Route not found")
            return
        
        # 检查是否到达终点
        if self._current_waypoint_index >= len(route.waypoints):
            if route.loop:
                # 循环模式：重新开始
                self._current_waypoint_index = 0
                self._node.get_logger().info("Patrol loop completed, restarting...")
            else:
                # 非循环模式：完成
                self._patrol_state = PatrolState.COMPLETED
                self._node.get_logger().info("Patrol completed (non-loop mode)")
                if self._on_complete_callback:
                    self._on_complete_callback()
                return
        
        # 获取导航状态
        nav_state = self._nav_executor.get_state()
        
        # 状态机处理
        if self._patrol_state == PatrolState.IDLE:
            # 空闲状态：发送下一个导航目标
            
            # 检查NavigationExecutor状态，如果不是IDLE则重置
            if nav_state != NavigationState.IDLE:
                self._node.get_logger().warn(
                    f"NavigationExecutor in state {nav_state.name} before starting patrol, resetting..."
                )
                if nav_state == NavigationState.EXECUTING or nav_state == NavigationState.CANCELING:
                    # 如果有导航正在执行，先取消
                    self._nav_executor.cancel_navigation()
                    # 等待取消完成（在下个周期重试）
                    return
                else:
                    # FAILED/CANCELED/SUCCESS等终止状态，直接重置
                    self._nav_executor.reset_state()
                    self._node.get_logger().info("NavigationExecutor state reset")
            
            waypoint = route.waypoints[self._current_waypoint_index]
            
            # 转换waypoint为PoseStamped
            goal_pose = self._waypoint_to_pose(waypoint)
            
            # 发送导航目标
            success = self._nav_executor.navigate_to_pose(goal_pose)
            
            if success:
                self._patrol_state = PatrolState.NAVIGATING
                wp_name = waypoint.get('name', f"waypoint_{self._current_waypoint_index}")
                self._node.get_logger().info(
                    f"Navigating to waypoint {self._current_waypoint_index + 1}/{len(route.waypoints)}: "
                    f"{wp_name} ({waypoint['x']:.2f}, {waypoint['y']:.2f})"
                )
            else:
                self._node.get_logger().error("Failed to send navigation goal")
                self._patrol_state = PatrolState.FAILED
                if self._on_failed_callback:
                    self._on_failed_callback("Failed to send navigation goal")
        
        elif self._patrol_state == PatrolState.NAVIGATING:
            # 导航中：检查导航状态
            if nav_state == NavigationState.SUCCESS:
                # 导航成功：进入停留状态
                waypoint = route.waypoints[self._current_waypoint_index]
                self._current_dwell_time = waypoint.get('dwell_time', 0.0)
                
                if self._current_dwell_time > 0:
                    self._patrol_state = PatrolState.DWELLING
                    self._dwell_start_time = time.time()
                    self._node.get_logger().info(
                        f"Arrived at waypoint {self._current_waypoint_index + 1}, "
                        f"dwelling for {self._current_dwell_time}s"
                    )
                else:
                    # 无需停留，直接进入下一个路点
                    self._current_waypoint_index += 1
                    self._patrol_state = PatrolState.IDLE
                    
            elif nav_state == NavigationState.FAILED:
                # 导航失败
                error_msg = self._nav_executor.get_last_error() or "Navigation failed"
                self._node.get_logger().error(f"Navigation to waypoint {self._current_waypoint_index} failed: {error_msg}")
                self._patrol_state = PatrolState.FAILED
                if self._on_failed_callback:
                    self._on_failed_callback(error_msg)
        
        elif self._patrol_state == PatrolState.DWELLING:
            # 停留中：检查停留时间
            if self._dwell_start_time is None:
                # 异常：没有开始时间，跳过停留
                self._current_waypoint_index += 1
                self._patrol_state = PatrolState.IDLE
            else:
                elapsed = time.time() - self._dwell_start_time
                if elapsed >= self._current_dwell_time:
                    # 停留时间结束，进入下一个路点
                    self._node.get_logger().info(
                        f"Dwell completed at waypoint {self._current_waypoint_index + 1}"
                    )
                    self._current_waypoint_index += 1
                    self._patrol_state = PatrolState.IDLE
                    self._dwell_start_time = None
    
    def _waypoint_to_pose(self, waypoint: Dict) -> PoseStamped:
        """
        将waypoint字典转换为PoseStamped
        
        Args:
            waypoint: 包含x, y, yaw的字典
            
        Returns:
            PoseStamped: ROS消息
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._node.get_clock().now().to_msg()
        
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        pose.pose.position.z = 0.0
        
        # 将yaw转换为四元数
        yaw = float(waypoint['yaw'])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    
    def get_current_progress(self) -> Dict:
        """
        获取当前巡航进度
        
        Returns:
            Dict: 包含当前路线信息和进度
        """
        if self._current_route_id is None:
            return {
                'status': 'inactive',
                'route_id': None,
                'route_name': None,
                'current_waypoint': 0,
                'total_waypoints': 0,
                'patrol_state': PatrolState.IDLE.value
            }
        
        route = self._routes.get(self._current_route_id)
        if route is None:
            return {
                'status': 'error',
                'route_id': self._current_route_id,
                'patrol_state': PatrolState.FAILED.value
            }
        
        progress = self._current_waypoint_index / len(route.waypoints) if len(route.waypoints) > 0 else 0.0
        
        return {
            'status': 'active',
            'route_id': route.route_id,
            'route_name': route.name,
            'current_waypoint': self._current_waypoint_index,
            'total_waypoints': len(route.waypoints),
            'progress': progress,
            'loop': route.loop,
            'patrol_state': self._patrol_state.value
        }

    
    def load_all_routes(self):
        """从持久化目录加载所有路线"""
        if not os.path.exists(self._persistence_dir):
            return
        
        for filename in os.listdir(self._persistence_dir):
            if filename.endswith('.yaml'):
                filepath = os.path.join(self._persistence_dir, filename)
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                    route = PatrolRoute.from_dict(data)
                    self._routes[route.route_id] = route
                except Exception as e:
                    print(f"Failed to load route from {filename}: {e}")
    
    def load_route_from_waypoints_file(self, filepath: str, route_name: str = None) -> Optional[str]:
        """
        从简单的waypoints YAML文件加载路线（patrol_node格式）
        
        Args:
            filepath: waypoints YAML文件路径
            route_name: 路线名称（可选，默认使用文件名）
            
        Returns:
            str: 创建的路线ID，失败返回None
            
        YAML格式示例:
            waypoints:
              - name: "point1"
                x: 1.0
                y: 0.5
                yaw: 0.0
                dwell_time: 2.0
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            waypoints_data = data.get('waypoints', [])
            if not waypoints_data:
                print(f"No waypoints found in {filepath}")
                return None
            
            # 生成路线ID和名称
            route_id = f"route_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:24]}"
            if route_name is None:
                route_name = os.path.splitext(os.path.basename(filepath))[0]
            
            # 转换为PatrolRoute格式的waypoints
            # 注意：这里的waypoints应该是PoseStamped的字典表示，但为了兼容简单格式，
            # 我们直接存储简化的waypoint数据
            route_waypoints = []
            for wp_data in waypoints_data:
                # 保持简单格式，后续可以转换为PoseStamped
                route_waypoints.append({
                    'name': wp_data.get('name', ''),
                    'x': float(wp_data['x']),
                    'y': float(wp_data['y']),
                    'yaw': float(wp_data['yaw']),
                    'dwell_time': float(wp_data.get('dwell_time', 2.0))
                })
            
            # 创建PatrolRoute
            route = PatrolRoute(
                route_id=route_id,
                name=route_name,
                waypoints=route_waypoints,
                loop=True
            )
            
            self._routes[route_id] = route
            return route_id
            
        except Exception as e:
            print(f"Failed to load route from waypoints file {filepath}: {e}")
            return None
    
    def _save_route(self, route: PatrolRoute):
        """保存路线到文件"""
        filepath = os.path.join(self._persistence_dir, f'{route.route_id}.yaml')
        with open(filepath, 'w', encoding='utf-8') as f:
            yaml.safe_dump(route.to_dict(), f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    
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
