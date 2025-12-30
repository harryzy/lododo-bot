#!/usr/bin/env python3
"""
ExplorationHandler - 探索任务处理器

功能 / Features:
  - 处理 FRONTIER_EXPLORATION 类型的探索任务
  - 集成边界检测和探索策略
  - 支持暂停、恢复、取消功能
  - 与 MissionPlanner 统一管理

Author: LeKiwi Bot Development Team
Date: 2025-12-30
"""

import math
import time
from enum import Enum
from typing import Optional
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from ..task_execution_handler import TaskExecutionHandler
from ...task_manager import Task, TaskState
from ...navigation_executor import NavigationState
from ....exploration.frontier_detector import FrontierDetector
from ....exploration.exploration_strategy import ExplorationStrategy


class ExplorationHandlerState(Enum):
    """探索处理器状态"""
    IDLE = "idle"
    DETECTING_FRONTIERS = "detecting"
    NAVIGATING_TO_FRONTIER = "navigating"
    ROTATING_AT_GOAL = "rotating"
    PAUSED = "paused"
    COMPLETED = "completed"


class ExplorationHandler(TaskExecutionHandler):
    """
    探索任务处理器（全新实现）
    
    基于 TaskExecutionHandler 接口的探索实现，
    集成到 MissionPlanner 统一管理
    """
    
    def __init__(self, node, task_manager, navigation_executor):
        super().__init__(node, task_manager, navigation_executor)
        
        # 探索状态
        self._exploration_state = ExplorationHandlerState.IDLE
        self._current_frontier = None
        self._frontiers = []
        self._explored_positions = []  # 已探索的位置
        
        # 地图数据
        self._current_map = None
        self._map_resolution = 0.05
        self._map_origin = None
        
        # 探索参数（可配置）
        self._exploration_radius = 5.0
        self._min_frontier_size = 5
        self._goal_tolerance = 0.3
        self._map_completion_threshold = 0.90
        self._visit_radius = 0.5  # 认为已访问的半径
        
        # 探索组件
        self._frontier_detector = FrontierDetector(self._min_frontier_size)
        self._exploration_strategy = ExplorationStrategy()
        
        # 配置探索策略
        self._configure_strategy()
        
        # 订阅地图
        self._setup_map_subscription()
        
        self._node.get_logger().info("[ExplorationHandler] Initialized")
    
    def _configure_strategy(self):
        """配置探索策略"""
        strategy_config = {
            'exploration_radius': self._exploration_radius,
            'min_goal_distance': 0.2,
            'goal_tolerance': self._goal_tolerance,
            'map_completion_threshold': self._map_completion_threshold,
            'min_completion_threshold': 0.75,
            'max_failures': 5,
            'max_no_goal_count': 3,
            'enable_smart_exploration': True,
            'visit_radius': self._visit_radius,
            'rotation_angle': 45.0,
        }
        self._exploration_strategy.configure(strategy_config)
    
    def _setup_map_subscription(self):
        """设置地图订阅"""
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._map_sub = self._node.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos
        )
    
    def _map_callback(self, msg: OccupancyGrid):
        """地图回调"""
        self._current_map = msg
        self._map_resolution = msg.info.resolution
        self._map_origin = msg.info.origin
    
    def execute(self, task: Task) -> None:
        """
        执行探索任务
        
        状态流转：
        WAITING_EXECUTION → RUNNING → IDLE → DETECTING_FRONTIERS → 
        NAVIGATING_TO_FRONTIER → ROTATING_AT_GOAL → (回到 DETECTING_FRONTIERS 或 COMPLETED)
        """
        # 1. WAITING_EXECUTION 状态 - 尝试获取 NavigationExecutor
        if task.state == TaskState.WAITING_EXECUTION:
            nav_state = self._nav_executor.get_state()
            
            if nav_state == NavigationState.IDLE and not self.is_executor_owner(task.task_id):
                if self.acquire_executor(task.task_id):
                    self._task_manager.update_task_state(task.task_id, TaskState.RUNNING)
                    self._exploration_state = ExplorationHandlerState.IDLE
                    self._node.get_logger().info(
                        f"[ExplorationHandler] Task {task.task_id} acquired executor, "
                        f"state: WAITING_EXECUTION -> RUNNING"
                    )
                else:
                    return
            else:
                self._node.get_logger().debug(
                    f"[ExplorationHandler] Task {task.task_id} waiting for executor "
                    f"(nav_state={nav_state.name})"
                )
                return
        
        # 2. PAUSED 状态 - 暂停处理
        if task.state == TaskState.PAUSED:
            if self._exploration_state != ExplorationHandlerState.PAUSED:
                self._handle_pause(task)
            return
        
        # 3. CANCELED 状态 - 取消处理
        if task.state == TaskState.CANCELED:
            self._handle_cancel(task)
            return
        
        # 4. RUNNING 状态 - 执行探索
        if task.state == TaskState.RUNNING:
            self._execute_exploration(task)
    
    def _execute_exploration(self, task: Task):
        """执行探索逻辑"""
        # 检查地图是否可用
        if self._current_map is None:
            self._node.get_logger().warn("[ExplorationHandler] Waiting for map...")
            return
        
        # 探索状态机
        if self._exploration_state == ExplorationHandlerState.IDLE:
            # 开始检测边界
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
            self._node.get_logger().info("[ExplorationHandler] Starting frontier detection")
        
        elif self._exploration_state == ExplorationHandlerState.DETECTING_FRONTIERS:
            self._process_frontier_detection(task)
        
        elif self._exploration_state == ExplorationHandlerState.NAVIGATING_TO_FRONTIER:
            self._monitor_navigation(task)
        
        elif self._exploration_state == ExplorationHandlerState.ROTATING_AT_GOAL:
            # 简化版：跳过旋转，直接继续探索
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        elif self._exploration_state == ExplorationHandlerState.COMPLETED:
            # 探索完成
            self._complete_exploration(task)
    
    def _process_frontier_detection(self, task: Task):
        """处理边界检测"""
        # 使用 FrontierDetector 检测边界
        try:
            # 转换地图数据为 numpy 数组
            import numpy as np
            map_array = np.array(self._current_map.data).reshape(
                (self._current_map.info.height, self._current_map.info.width)
            )
            
            # 检测边界
            frontiers = self._frontier_detector.detect_frontiers(
                map_array,
                self._map_resolution,
                self._map_origin,
                self._current_map.info.width,
                self._current_map.info.height
            )
            
            if not frontiers or len(frontiers) == 0:
                # 没有边界 → 检查是否完成
                self._check_exploration_completion(task)
                return
            
            # 过滤已访问的边界
            frontiers = self._filter_visited_frontiers(frontiers)
            
            if not frontiers:
                # 所有边界都已访问
                self._check_exploration_completion(task)
                return
            
            # 选择最优边界
            best_frontier = self._select_best_frontier(frontiers)
            
            if best_frontier is None:
                # 无法选择边界
                self._check_exploration_completion(task)
                return
            
            # 导航到边界
            self._navigate_to_frontier(best_frontier, task)
            
        except Exception as e:
            self._node.get_logger().error(
                f"[ExplorationHandler] Frontier detection error: {str(e)}"
            )
            # 失败重试
            time.sleep(1.0)
    
    def _filter_visited_frontiers(self, frontiers):
        """过滤已访问的边界"""
        filtered = []
        for frontier in frontiers:
            # 检查是否已访问
            is_visited = False
            for visited_pos in self._explored_positions:
                dist = math.sqrt(
                    (frontier.centroid[0] - visited_pos[0])**2 +
                    (frontier.centroid[1] - visited_pos[1])**2
                )
                if dist < self._visit_radius:
                    is_visited = True
                    break
            
            if not is_visited:
                filtered.append(frontier)
        
        return filtered
    
    def _select_best_frontier(self, frontiers):
        """选择最优边界（简化版：选择最近的）"""
        if not frontiers:
            return None
        
        # 获取机器人当前位置（从 NavigationExecutor 或话题）
        # 简化版：使用第一个边界
        return frontiers[0]
    
    def _navigate_to_frontier(self, frontier, task: Task):
        """导航到边界"""
        # 构造目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(frontier.centroid[0])
        goal_pose.pose.position.y = float(frontier.centroid[1])
        goal_pose.pose.position.z = 0.0
        
        # 方向：朝向边界中心（简化版：使用固定方向）
        goal_pose.pose.orientation.w = 1.0
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Navigating to frontier at "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
        )
        
        # 使用 NavigationExecutor 导航
        success = self._nav_executor.navigate_to_pose(goal_pose)
        
        if success:
            self._current_frontier = frontier
            self._exploration_state = ExplorationHandlerState.NAVIGATING_TO_FRONTIER
            # 更新任务进度（示例）
            self._task_manager.update_progress(task.task_id, 0.5)
        else:
            # 导航启动失败，重新检测
            self._node.get_logger().warn(
                "[ExplorationHandler] Failed to start navigation, retrying..."
            )
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
    
    def _monitor_navigation(self, task: Task):
        """监控导航进度"""
        nav_state = self._nav_executor.get_state()
        
        if nav_state == NavigationState.SUCCESS:
            # 导航成功 → 记录已探索位置
            if self._current_frontier:
                self._explored_positions.append(
                    (self._current_frontier.centroid[0], 
                     self._current_frontier.centroid[1])
                )
            
            self._node.get_logger().info(
                "[ExplorationHandler] Reached frontier, continuing exploration"
            )
            
            # 继续探索
            self._current_frontier = None
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        elif nav_state == NavigationState.FAILED:
            # 导航失败 → 重新检测边界
            self._node.get_logger().warn(
                "[ExplorationHandler] Navigation failed, re-detecting frontiers"
            )
            self._current_frontier = None
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        elif nav_state == NavigationState.CANCELED:
            # 被取消
            self._handle_cancel(task)
    
    def _check_exploration_completion(self, task: Task):
        """检查探索是否完成"""
        # 简化版：如果没有边界，认为完成
        self._node.get_logger().info(
            "[ExplorationHandler] No more frontiers - exploration may be complete"
        )
        
        # 计算地图完成度（简化版）
        completion = self._calculate_map_completion()
        
        if completion >= self._map_completion_threshold:
            self._exploration_state = ExplorationHandlerState.COMPLETED
            self._node.get_logger().info(
                f"[ExplorationHandler] Exploration completed! "
                f"Map coverage: {completion*100:.1f}%"
            )
        else:
            # 完成度不够，再尝试检测
            self._node.get_logger().info(
                f"[ExplorationHandler] Map coverage: {completion*100:.1f}%, "
                f"threshold: {self._map_completion_threshold*100:.1f}%, continuing..."
            )
            time.sleep(2.0)  # 等待地图更新
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
    
    def _calculate_map_completion(self) -> float:
        """计算地图完成度"""
        if self._current_map is None:
            return 0.0
        
        import numpy as np
        map_array = np.array(self._current_map.data)
        
        # 已知区域（0-100）vs 未知区域（-1）
        total_cells = len(map_array)
        known_cells = np.sum((map_array >= 0) & (map_array <= 100))
        
        if total_cells > 0:
            return known_cells / total_cells
        return 0.0
    
    def _complete_exploration(self, task: Task):
        """完成探索"""
        self._task_manager.complete_task(task.task_id)
        self.release_executor(task.task_id)
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Task {task.task_id} completed successfully"
        )
        
        # 保存地图（如果配置要求）
        if task.parameters.get('save_map', False):
            map_name = task.parameters.get('map_name', 'exploration_map')
            self._save_map(map_name)
    
    def _save_map(self, map_name: str):
        """保存地图（简化版：调用外部脚本）"""
        try:
            import subprocess
            import os
            
            # 使用 ros2 service call 保存地图
            map_dir = os.path.expanduser(f'~/lododo_bot/maps/{map_name}')
            os.makedirs(map_dir, exist_ok=True)
            
            # 调用 map_saver_cli
            cmd = f"ros2 run nav2_map_server map_saver_cli -f {map_dir}/map"
            subprocess.Popen(cmd, shell=True)
            
            self._node.get_logger().info(
                f"[ExplorationHandler] Saving map to {map_dir}"
            )
        except Exception as e:
            self._node.get_logger().error(
                f"[ExplorationHandler] Failed to save map: {str(e)}"
            )
    
    def _handle_pause(self, task: Task):
        """处理暂停"""
        nav_state = self._nav_executor.get_state()
        
        if nav_state in [NavigationState.EXECUTING, NavigationState.CANCELING]:
            self._nav_executor.cancel_navigation()
        
        self._exploration_state = ExplorationHandlerState.PAUSED
        self._node.get_logger().info(
            f"[ExplorationHandler] Exploration paused for task {task.task_id}"
        )
    
    def _handle_cancel(self, task: Task):
        """处理取消"""
        nav_state = self._nav_executor.get_state()
        
        if nav_state not in [NavigationState.IDLE, NavigationState.CANCELED]:
            self._nav_executor.cancel_navigation()
        
        self.release_executor(task.task_id)
        self._exploration_state = ExplorationHandlerState.IDLE
        self._current_frontier = None
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Exploration canceled for task {task.task_id}"
        )
    
    def pause(self, task: Task) -> bool:
        """暂停探索任务"""
        if self.is_executor_owner(task.task_id):
            self._handle_pause(task)
            return True
        return False
    
    def resume(self, task: Task) -> bool:
        """恢复探索任务"""
        if self.is_executor_owner(task.task_id):
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
            self._node.get_logger().info(
                f"[ExplorationHandler] Resuming exploration for task {task.task_id}"
            )
            return True
        return False
    
    def cancel(self, task: Task) -> bool:
        """取消探索任务"""
        if self.is_executor_owner(task.task_id):
            self._handle_cancel(task)
            return True
        return False
