#!/usr/bin/env python3
"""
PatrolNode - 巡航执行节点 (Refactored with NavigationExecutor)

功能 / Features:
  - 继承NavigationExecutor基类，复用导航逻辑
  - 多种巡航模式: loop, ping_pong, once, random
  - 路点停留时间控制
  - YAML配置文件支持
  - 实时状态发布
  
Author: LeKiwi Bot Development Team  
Date: 2025-12-24 (Refactored)
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

import yaml
import math
import time
from typing import List, Optional, Tuple
from dataclasses import dataclass

from .navigation_executor import NavigationExecutor, NavigationState


@dataclass
class Waypoint:
    """路点数据结构 / Waypoint data structure"""
    x: float                    # X坐标 (米)
    y: float                    # Y坐标 (米)  
    yaw: float                  # 朝向 (弧度)
    name: str = ""              # 名称
    dwell_time: float = 2.0     # 停留时间 (秒)
    
    @staticmethod
    def from_dict(data: dict) -> 'Waypoint':
        """从字典创建 / Create from dictionary"""
        return Waypoint(
            x=data['x'],
            y=data['y'],
            yaw=data['yaw'],
            name=data.get('name', ''),
            dwell_time=data.get('dwell_time', 2.0)
        )


class WaypointManager:
    """路点管理器 / Waypoint manager"""
    
    def __init__(self):
        self.waypoints: List[Waypoint] = []
        self.current_index: int = 0
        self.direction: int = 1  # 1=正向, -1=反向 (for ping_pong)
        
    def add_waypoint(self, waypoint: Waypoint):
        """添加路点 / Add waypoint"""
        self.waypoints.append(waypoint)
        
    def clear_waypoints(self):
        """清空所有路点 / Clear all waypoints"""
        self.waypoints.clear()
        self.current_index = 0
        self.direction = 1
        
    def get_waypoint(self, index: int) -> Optional[Waypoint]:
        """获取指定路点 / Get waypoint by index"""
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None
        
    def get_next_waypoint(self, mode: str = 'loop') -> Optional[Waypoint]:
        """
        获取下一个路点 / Get next waypoint
        
        Args:
            mode: 遍历模式
                - 'loop': 循环模式 (0→1→2→0→...)
                - 'ping_pong': 往返模式 (0→1→2→1→0→...)
                - 'once': 单次模式 (0→1→2→结束)
                - 'random': 随机模式
                
        Returns:
            下一个路点，如果遍历结束则返回None
        """
        if not self.waypoints:
            return None
            
        if mode == 'loop':
            waypoint = self.waypoints[self.current_index]
            self.current_index = (self.current_index + 1) % len(self.waypoints)
            return waypoint
            
        elif mode == 'ping_pong':
            waypoint = self.waypoints[self.current_index]
            
            # 更新索引和方向
            self.current_index += self.direction
            
            # 检查边界并反转方向
            if self.current_index >= len(self.waypoints):
                self.current_index = len(self.waypoints) - 2
                self.direction = -1
            elif self.current_index < 0:
                self.current_index = 1
                self.direction = 1
                
            return waypoint
            
        elif mode == 'once':
            if self.current_index >= len(self.waypoints):
                return None  # 遍历结束
            waypoint = self.waypoints[self.current_index]
            self.current_index += 1
            return waypoint
            
        elif mode == 'random':
            import random
            return random.choice(self.waypoints)
            
        return None
        
    def reset_traversal(self):
        """重置遍历状态 / Reset traversal state"""
        self.current_index = 0
        self.direction = 1
        
    def load_from_yaml(self, filepath: str) -> bool:
        """
        从YAML文件加载路点 / Load waypoints from YAML file
        
        Args:
            filepath: 文件路径
            
        Returns:
            bool: 是否成功加载
        """
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                
            self.clear_waypoints()
            
            for wp_data in data.get('waypoints', []):
                waypoint = Waypoint.from_dict(wp_data)
                self.add_waypoint(waypoint)
                
            return True
        except Exception as e:
            print(f"Failed to load waypoints from {filepath}: {e}")
            return False


class PatrolNode(Node):
    """
    巡航执行节点 / Patrol execution node
    
    使用NavigationExecutor进行导航执行
    """
    
    def __init__(self):
        super().__init__('patrol_node')
        
        self.get_logger().info('Initializing PatrolNode...')
        
        # ===== 导航执行器 =====
        self._nav_executor = NavigationExecutor(self)
        
        # ===== 参数声明和加载 =====
        self._declare_parameters()
        self._load_parameters()
        
        # ===== 路点管理器 =====
        self._waypoint_mgr = WaypointManager()
        
        # ===== 巡航状态 =====
        self._is_patrolling = False
        self._current_waypoint: Optional[Waypoint] = None
        self._patrol_loop_count = 0
        self._max_patrol_loops = self._max_loops
        
        # ===== 停留控制 =====
        self._is_dwelling = False
        self._dwell_start_time = None
        self._dwell_timer = None
        
        # ===== 加载路点配置 =====
        if self._waypoint_file:
            if self._waypoint_mgr.load_from_yaml(self._waypoint_file):
                self.get_logger().info(f'Loaded {len(self._waypoint_mgr.waypoints)} waypoints from {self._waypoint_file}')
            else:
                self.get_logger().error(f'Failed to load waypoints from {self._waypoint_file}')
        
        # ===== 服务接口 =====
        self._start_service = self.create_service(
            Trigger,
            'patrol/start',
            self._start_patrol_callback
        )
        
        self._stop_service = self.create_service(
            Trigger,
            'patrol/stop',
            self._stop_patrol_callback
        )
        
        self._pause_service = self.create_service(
            Trigger,
            'patrol/pause',
            self._pause_patrol_callback
        )
        
        self._resume_service = self.create_service(
            Trigger,
            'patrol/resume',
            self._resume_patrol_callback
        )
        
        # ===== 发布器 =====
        self._status_pub = self.create_publisher(
            String,
            '/patrol/status',
            10
        )
        
        self._complete_pub = self.create_publisher(
            Bool,
            '/patrol/complete',
            10
        )
        
        # ===== 定时器 =====
        self._patrol_timer = self.create_timer(1.0, self._patrol_loop)
        self._status_timer = self.create_timer(2.0, self._publish_status)
        
        self.get_logger().info(f'PatrolNode initialized successfully')
        self.get_logger().info(f'Patrol mode: {self._patrol_mode}')
        self.get_logger().info(f'Max loops: {self._max_patrol_loops}')
        self.get_logger().info(f'Waypoints loaded: {len(self._waypoint_mgr.waypoints)}')
    
    def _declare_parameters(self):
        """声明参数 / Declare parameters"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_file', ''),
                ('patrol_mode', 'loop'),
                ('default_dwell_time', 2.0),
                ('max_loops', -1),
                ('arrival_tolerance', 0.3),
                ('auto_start', False),
            ]
        )
    
    def _load_parameters(self):
        """加载参数 / Load parameters"""
        self._waypoint_file = self.get_parameter('waypoint_file').value
        self._patrol_mode = self.get_parameter('patrol_mode').value
        self._default_dwell_time = self.get_parameter('default_dwell_time').value
        self._max_loops = self.get_parameter('max_loops').value
        self._arrival_tolerance = self.get_parameter('arrival_tolerance').value
        self._auto_start = self.get_parameter('auto_start').value
        
        # 如果启用自动开始，延迟3秒后启动
        if self._auto_start:
            self.create_timer(3.0, self._auto_start_patrol, one_shot=True)
    
    def _auto_start_patrol(self):
        """自动启动巡航 / Auto start patrol"""
        self.get_logger().info('Auto-starting patrol...')
        self._start_patrol()
    
    # ===== 服务回调 =====
    
    def _start_patrol_callback(self, request, response):
        """启动巡航服务 / Start patrol service"""
        if self._start_patrol():
            response.success = True
            response.message = 'Patrol started'
        else:
            response.success = False
            response.message = 'Failed to start patrol (no waypoints or already running)'
        return response
    
    def _stop_patrol_callback(self, request, response):
        """停止巡航服务 / Stop patrol service"""
        self._stop_patrol()
        response.success = True
        response.message = 'Patrol stopped'
        return response
    
    def _pause_patrol_callback(self, request, response):
        """暂停巡航服务 / Pause patrol service"""
        if self._pause_patrol():
            response.success = True
            response.message = 'Patrol paused'
        else:
            response.success = False
            response.message = 'Cannot pause (not patrolling)'
        return response
    
    def _resume_patrol_callback(self, request, response):
        """恢复巡航服务 / Resume patrol service"""
        if self._resume_patrol():
            response.success = True
            response.message = 'Patrol resumed'
        else:
            response.success = False
            response.message = 'Cannot resume (not paused)'
        return response
    
    # ===== 核心巡航逻辑 =====
    
    def _start_patrol(self) -> bool:
        """
        启动巡航 / Start patrol
        
        Returns:
            bool: 是否成功启动
        """
        if self._is_patrolling:
            self.get_logger().warn('Patrol already running')
            return False
            
        if not self._waypoint_mgr.waypoints:
            self.get_logger().error('No waypoints loaded')
            return False
        
        self._is_patrolling = True
        self._patrol_loop_count = 0
        self._waypoint_mgr.reset_traversal()
        
        self.get_logger().info(f'🚀 Patrol started: {len(self._waypoint_mgr.waypoints)} waypoints, mode={self._patrol_mode}')
        
        return True
    
    def _stop_patrol(self):
        """停止巡航 / Stop patrol"""
        self._is_patrolling = False
        self._current_waypoint = None
        self._is_dwelling = False
        
        # 取消当前导航目标
        if self.get_state() == NavigationState.EXECUTING:
            self.cancel_navigation()
        
        self.get_logger().info('🛑 Patrol stopped')
    
    def _pause_patrol(self) -> bool:
        """暂停巡航 / Pause patrol"""
        if not self._is_patrolling:
            return False
        
        self._is_patrolling = False
        self.cancel_navigation()
        
        self.get_logger().info('⏸️  Patrol paused')
        return True
    
    def _resume_patrol(self) -> bool:
        """恢复巡航 / Resume patrol"""
        # 暂停状态判断：有路点但未在巡航
        if self._waypoint_mgr.waypoints and not self._is_patrolling:
            self._is_patrolling = True
            self.get_logger().info('▶️  Patrol resumed')
            return True
        return False
    
    def _patrol_loop(self):
        """
        巡航主循环 / Main patrol loop
        
        状态机逻辑:
        1. 检查是否正在巡航
        2. 处理停留时间
        3. 等待当前导航目标完成
        4. 选择下一个路点
        5. 发送导航目标
        """
        if not self._is_patrolling:
            return
        
        # 1. 处理停留时间 / Handle dwell time
        if self._is_dwelling:
            if self._dwell_start_time is not None:
                elapsed = time.time() - self._dwell_start_time
                if elapsed >= self._current_waypoint.dwell_time:
                    self.get_logger().info(f'Dwell time complete ({elapsed:.1f}s)')
                    self._is_dwelling = False
                    self._dwell_start_time = None
            return
        
        # 2. 等待当前导航完成 / Wait for current navigation
        nav_state = self._nav_executor.get_state()
        
        if nav_state == NavigationState.EXECUTING:
            return  # 还在导航中 / Still navigating
        
        # 3. 处理导航结果 / Handle navigation result
        if nav_state == NavigationState.SUCCESS:
            self._handle_waypoint_arrival()
            self._nav_executor.reset_state()  # 重置状态以准备下一个目标
            return
        
        elif nav_state == NavigationState.FAILED:
            self.get_logger().warn('Navigation failed, continuing to next waypoint')
            self._nav_executor.reset_state()
            # 继续下一个路点 / Continue to next waypoint
        
        # 4. 选择并导航到下一个路点 / Select and navigate to next waypoint
        next_waypoint = self._select_next_waypoint()
        
        if next_waypoint is None:
            # 巡航完成 / Patrol complete
            self._handle_patrol_complete()
            return
        
        # 5. 发送导航目标 / Send navigation goal
        self._send_waypoint_goal(next_waypoint)
    
    def _select_next_waypoint(self) -> Optional[Waypoint]:
        """
        选择下一个路点 / Select next waypoint
        
        Returns:
            下一个路点，如果巡航结束则返回None
        """
        # 检查是否达到最大循环次数
        if self._max_patrol_loops > 0 and self._patrol_loop_count >= self._max_patrol_loops:
            self.get_logger().info(f'Reached max loops ({self._max_patrol_loops})')
            return None
        
        waypoint = self._waypoint_mgr.get_next_waypoint(mode=self._patrol_mode)
        
        # 检测是否完成一轮循环
        if self._patrol_mode == 'loop' and self._waypoint_mgr.current_index == 0:
            self._patrol_loop_count += 1
            self.get_logger().info(f'✅ Completed patrol loop {self._patrol_loop_count}')
        
        return waypoint
    
    def _send_waypoint_goal(self, waypoint: Waypoint):
        """
        发送路点导航目标 / Send waypoint navigation goal
        
        Args:
            waypoint: 目标路点
        """
        self._current_waypoint = waypoint
        
        # 创建目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoint.x
        goal_pose.pose.position.y = waypoint.y
        goal_pose.pose.position.z = 0.0
        
        # 转换yaw为四元数
        goal_pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        
        # 发送导航目标
        if self._nav_executor.navigate_to_pose(goal_pose):
            waypoint_name = waypoint.name if waypoint.name else f'waypoint_{self._waypoint_mgr.current_index}'
            self.get_logger().info(
                f'🎯 Navigating to {waypoint_name}: '
                f'({waypoint.x:.2f}, {waypoint.y:.2f}), yaw={math.degrees(waypoint.yaw):.1f}°'
            )
        else:
            self.get_logger().error('Failed to send navigation goal')
    
    def _handle_waypoint_arrival(self):
        """处理到达路点 / Handle waypoint arrival"""
        if self._current_waypoint is None:
            return
        
        waypoint_name = self._current_waypoint.name if self._current_waypoint.name else 'waypoint'
        self.get_logger().info(f'✅ Arrived at {waypoint_name}')
        
        # 开始停留 / Start dwelling
        if self._current_waypoint.dwell_time > 0:
            self._is_dwelling = True
            self._dwell_start_time = time.time()
            self.get_logger().info(f'⏱️  Dwelling for {self._current_waypoint.dwell_time:.1f}s')
    
    def _handle_patrol_complete(self):
        """处理巡航完成 / Handle patrol complete"""
        self._is_patrolling = False
        self._current_waypoint = None
        
        self.get_logger().info('🏁 Patrol complete!')
        
        # 发布完成消息
        self._complete_pub.publish(Bool(data=True))
    
    def _publish_status(self):
        """发布状态信息 / Publish status"""
        if not self._is_patrolling:
            return
        
        status = f'Patrolling: loop={self._patrol_loop_count}, ' \
                f'waypoint={self._waypoint_mgr.current_index}/{len(self._waypoint_mgr.waypoints)}, ' \
                f'dwelling={self._is_dwelling}'
        
        self._status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    
    node = PatrolNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received interrupt signal, shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
