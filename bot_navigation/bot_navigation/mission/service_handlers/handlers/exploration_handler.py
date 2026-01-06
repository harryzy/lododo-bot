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
from datetime import datetime
from enum import Enum
from typing import Optional, Tuple, Dict
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from ..task_execution_handler import TaskExecutionHandler
from ...task_manager import Task, TaskState
from ...navigation_executor import NavigationState
from ....exploration.frontier_detector import FrontierDetector
from ....exploration.exploration_strategy import ExplorationStrategy
from ....exploration.exploration_utils import CoordinateConverter
from ....map.map_library_manager import MapLibraryManager

# 🎯 重构：引入新的抽象组件（位于 exploration/ 目录）
from ....exploration.goal_boundary_validator import GoalBoundaryValidator
from ....exploration.task_termination_manager import TaskTerminationManager
from ....exploration.completion_strategy import CompletionStrategy


class ExplorationHandlerState(Enum):
    """探索处理器状态"""
    IDLE = "idle"
    NAVIGATING_TO_INITIAL_SCAN_POINT = "navigating_to_initial_scan_point"  # 导航到初始扫描点
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
        self._frontier_detector = FrontierDetector(self._min_frontier_size, logger=self._node.get_logger())
        self._exploration_strategy = ExplorationStrategy()
        
        # 配置探索策略
        self._configure_strategy()
        
        # 订阅地图
        self._setup_map_subscription()
        
        # 🎯 阶段1: 初始化核心依赖类
        self._initialize_rotation_controller()
        self._initialize_safety_manager()
        self._initialize_frontier_evaluator()
        
        # 🎯 阶段1: 添加关键状态变量
        self._visited_frontiers = []  # 已访问的边界中心点列表
        self._last_goal_direction = None  # 上次目标方向
        self._consecutive_failures = 0  # 连续失败计数
        self._max_consecutive_failures = 3  # 最大连续失败次数
        self._stagnant_count = 0  # 停滞计数
        self._pending_goal = None  # 待处理的目标（旋转避障后使用）
        self._initial_scan_done = False  # 初始扫描完成标志
        self._initial_scan_point_reached = False  # 初始扫描安全点是否到达
        self._initial_scan_point = None  # 初始扫描安全点坐标
        
        # 🎯 阶段3: 最终验证和脱困相关变量
        self._completion_scan_in_progress = False  # 完成度验证进行中标志
        self._completion_before_scan = 0.0  # 验证开始时的完成度
        self._completion_verification_attempts = 0  # 完成验证尝试次数
        self._max_verification_attempts = 3  # 最大验证次数
        self._is_in_verification = False  # 是否在验证模式
        self._verification_rotation_pending = False  # 验证后是否需要旋转180°
        self._verification_nav_completed = False  # 当前验证的导航是否已完成
        self._verification_rotation_pending = False  # 验证后是否需要旋转180°
        self._verification_nav_completed = False  # 当前验证的导航是否已完成
        
        # 🎯 方案C: 主动完成度检测相关变量
        self._completion_history = []  # [(timestamp, completion), ...]
        self._frontier_score_history = []  # [score1, score2, ...]
        self._last_detected_frontiers = []  # 最近检测到的frontiers列表
        self._stagnation_threshold_seconds = 180.0  # 3分钟停滞阈值
        self._min_progress_rate = 0.02  # 最小进展率 2%
        self._escape_attempt = 0  # 脱困尝试次数
        self._max_escape_attempts = 3  # 最大脱困次数
        
        # 🎯 P2: 导航超时相关变量
        self._goal_start_time = None  # 导航开始时间
        self._max_goal_time = 120.0  # 导航超时（2分钟）
        
        # 🎯 防止重复完成标志
        self._exploration_completed = False  # 探索是否已完成（防止重复调用_complete_exploration）
        
        # 🎯 方案2: Frontier失败黑名单机制
        # Frontier failure blacklist: {position_key: (fail_count, last_timestamp)}
        self._frontier_failure_history = {}  # 失败frontier记录
        self._max_frontier_failures = 2  # 最多失败2次才加入黑名单
        self._failed_frontier_radius = 0.5  # 0.5m范围内视为同一frontier
        self._failed_frontier_timeout = 180.0  # 3分钟后清除失败记录
        
        # 🔧 启动延迟：等待TF树建立
        self._exploration_start_time = None  # 探索启动时间
        self._tf_wait_duration = 3.0  # 等待TF树建立的时间（秒）
        
        # 订阅 local_costmap
        self._setup_local_costmap_subscription()
        
        # 🎯 地图库管理器（用于版本管理和保存）
        try:
            if self._node.has_parameter('maps_directory'):
                maps_dir = self._node.get_parameter('maps_directory').get_parameter_value().string_value
            else:
                maps_dir = '~/lododo_bot/maps'
        except Exception:
            maps_dir = '~/lododo_bot/maps'
        
        self._map_library_manager = MapLibraryManager(self._node, maps_dir)
        
        # 🎯 重构：初始化新的抽象组件
        self._boundary_validator = GoalBoundaryValidator(
            boundary_margin=5,
            free_space_threshold=50
        )
        self._termination_manager = TaskTerminationManager(
            node=self._node,
            task_manager=self._task_manager,
            nav_executor=self._nav_executor,
            rotation_controller=self._rotation_controller
        )
        self._completion_strategy = CompletionStrategy(
            target_threshold=self._map_completion_threshold,
            min_threshold=0.75,
            stagnation_time=180.0,
            min_progress_rate=0.02,
            logger=self._node.get_logger()
        )
        
        self._node.get_logger().info("[ExplorationHandler] Initialized with refactored components (2026-01-05)")
    
    def _get_robot_yaw_odom(self) -> Optional[float]:
        """
        获取机器人在odom坐标系中的yaw角度
        
        Returns:
            float: yaw角度（弧度），失败返回None
        """
        try:
            # 通过NavigationExecutor的TF buffer获取odom→base_link变换
            from tf2_ros import TransformException
            from rclpy.duration import Duration
            import rclpy.time
            
            transform = self._nav_executor._tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            
            # 从四元数提取yaw角
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return yaw
            
        except Exception as e:
            self._node.get_logger().warn(f"Cannot get robot yaw in odom frame: {e}")
            return None
    
    def _get_robot_position(self) -> Optional[tuple]:
        """
        获取机器人在map坐标系中的位置
        
        Returns:
            tuple: (x, y) 位置，失败返回None
        """
        # 🔧 Critical Fix: 使用NavigationExecutor的get_robot_pose方法获取机器人位置
        # NavigationExecutor没有get_robot_position，只有get_robot_pose
        pose = self._nav_executor.get_robot_pose('map')
        if pose is None:
            return None
        return (pose.pose.position.x, pose.pose.position.y)
    
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
        
        # 3. FAILED 状态 - 失败状态，清理资源
        if task.state == TaskState.FAILED:
            # 🎯 重构：使用 TaskTerminationManager 统一处理
            if self._termination_manager.is_any_motion_active():
                self._termination_manager.stop_motion_only()
                self._node.get_logger().info("[ExplorationHandler] Stopped motion on task failure")
            return
        
        # 4. CANCELED 状态 - 取消处理
        if task.state == TaskState.CANCELED:
            self._handle_cancel(task)
            return
        
        # 5. RUNNING 状态 - 执行探索
        if task.state == TaskState.RUNNING:
            self._execute_exploration(task)
    
    def _execute_exploration(self, task: Task):
        """执行探索逻辑"""
        # 🔧 启动延迟：等待TF树建立
        if self._exploration_start_time is None:
            self._exploration_start_time = time.time()
            self._node.get_logger().info("[ExplorationHandler] Exploration started, waiting for TF tree...")
            return
        
        # 等待TF树建立
        elapsed = time.time() - self._exploration_start_time
        if elapsed < self._tf_wait_duration:
            if int(elapsed * 2) % 2 == 0:  # 每0.5秒打印一次
                self._node.get_logger().info(
                    f"[ExplorationHandler] Waiting for TF tree... ({elapsed:.1f}s / {self._tf_wait_duration}s)",
                    throttle_duration_sec=0.5
                )
            return
        
        # 检查地图是否可用
        if self._current_map is None:
            self._node.get_logger().warn("[ExplorationHandler] Waiting for map...")
            return
        
        # 🎯 P0: 【最高优先级】旋转状态更新
        # ⚠️ 关键：update_rotation()必须在pending_goal之前调用
        # 作用：1)监控旋转进度 2)发布cmd_vel 3)障碍检测 4)完成判断
        if self._rotation_controller.update_rotation(self._get_robot_yaw_odom):
            self._node.get_logger().debug(
                "[ExplorationHandler] Rotation in progress, waiting for completion...",
                throttle_duration_sec=2.0
            )
            return  # 旋转未完成，阻塞一切后续逻辑
        
        # 🎯 Phase 1: 优先级2 - 处理pending_goal（次高优先级）
        if self._pending_goal is not None:
            # 检查是否因为障碍物而停止旋转
            if self._rotation_controller.skip_rotation_on_obstacle:
                # 检查导航状态
                nav_state = self._nav_executor.get_state()
                
                if nav_state in [NavigationState.EXECUTING, NavigationState.CANCELING]:
                    self._node.get_logger().debug(
                        "[ExplorationHandler] Waiting for navigation before handling rotation obstacle",
                        throttle_duration_sec=2.0
                    )
                    return  # 等待导航完成
                
                # 处理旋转障碍
                self._handle_rotation_obstacle(task)
            else:
                # 正常完成旋转，发送目标
                self._send_pending_goal(task)
            return  # ⚠️ 阻塞后续流程
        
        # 🔧 关键修复：处理初始扫描（必须在边界检测前完成）
        # 🔧 Critical Fix: Handle initial scan (must complete before frontier detection)
        # 🎯 新策略：先导航到前方已知区域的安全点，再进行360°扫描
        if not self._rotation_controller.initial_scan_done and self._exploration_state not in [
            ExplorationHandlerState.ROTATING_AT_GOAL,
            ExplorationHandlerState.NAVIGATING_TO_INITIAL_SCAN_POINT
        ]:
            # 阶段1：选择并导航到初始扫描安全点
            if not self._initial_scan_point_reached:
                # 选择前方已知区域的安全点
                safe_point = self._find_safe_initial_scan_point()
                
                if safe_point is not None:
                    self._initial_scan_point = safe_point
                    self._node.get_logger().info(
                        f"[ExplorationHandler] Found safe initial scan point at ({safe_point[0]:.2f}, {safe_point[1]:.2f}), "
                        f"navigating there before scanning..."
                    )
                    
                    # 发送导航目标
                    from geometry_msgs.msg import PoseStamped
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self._node.get_clock().now().to_msg()
                    goal_pose.pose.position.x = safe_point[0]
                    goal_pose.pose.position.y = safe_point[1]
                    goal_pose.pose.position.z = 0.0
                    goal_pose.pose.orientation.w = 1.0  # 朝向不重要，到达后会360°扫描
                    
                    if self._nav_executor.navigate_to_pose(goal_pose):
                        self._exploration_state = ExplorationHandlerState.NAVIGATING_TO_INITIAL_SCAN_POINT
                        self._node.get_logger().info(
                            "[ExplorationHandler] Navigating to safe initial scan point..."
                        )
                    else:
                        self._node.get_logger().warn(
                            "[ExplorationHandler] Failed to send navigation to initial scan point, "
                            "performing scan at current position"
                        )
                        self._initial_scan_point_reached = True  # 跳过导航，直接扫描
                else:
                    # 找不到安全点，在当前位置扫描
                    self._node.get_logger().warn(
                        "[ExplorationHandler] Cannot find safe initial scan point in forward known area, "
                        "scanning at current position"
                    )
                    self._initial_scan_point_reached = True
                return
            
            # 阶段2：到达安全点后，执行360°扫描
            # 检查是否正在旋转 / Check if already rotating
            if self._rotation_controller.is_rotating_state:
                return  # 正在旋转，等待完成 / Rotating, wait for completion
            
            # 🔧 使用 RotationController 的 start_initial_scan() 方法
            # 🔧 Use RotationController's start_initial_scan() method (handles multi-rotation logic)
            get_yaw_func = self._get_robot_yaw_odom
            success = self._rotation_controller.start_initial_scan(get_yaw_func)
            
            if success:
                # 成功启动旋转，进入ROTATING_AT_GOAL状态
                self._exploration_state = ExplorationHandlerState.ROTATING_AT_GOAL
                return  # 等待旋转完成 / Wait for rotation completion
            else:
                # 初始扫描已完成（或达到最大次数），继续探索
                # Initial scan completed (or max attempts reached), continue exploration
                # 注意：如果是障碍导致的失败，_monitor_rotation() 已经设置了完成标志
                if self._rotation_controller.initial_scan_done:
                    self._node.get_logger().info("[ExplorationHandler] Initial scan completed, starting exploration")
                    self._initial_scan_done = True
                else:
                    # 理论上不应该到这里，因为 start_initial_scan() 返回 False
                    # 意味着要么已完成，要么达到最大次数
                    self._node.get_logger().warn(
                        "[ExplorationHandler] Initial scan cannot start (unexpected state), proceeding with exploration"
                    )
                    self._rotation_controller.initial_scan_done = True
                    self._initial_scan_done = True
        
        # 探索状态机
        if self._exploration_state == ExplorationHandlerState.IDLE:
            # 开始探索（初始扫描已在上面处理）
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
            self._node.get_logger().info("[ExplorationHandler] Starting frontier detection")
        
        elif self._exploration_state == ExplorationHandlerState.NAVIGATING_TO_INITIAL_SCAN_POINT:
            # 监控导航到初始扫描点的进度
            nav_state = self._nav_executor.get_state()
            
            if nav_state == NavigationState.EXECUTING:
                return  # 导航中，等待完成
            
            elif nav_state == NavigationState.SUCCESS:
                self._node.get_logger().info(
                    "[ExplorationHandler] Reached safe initial scan point, ready for 360° scan"
                )
                self._initial_scan_point_reached = True
                # 🔧 关键修复：改变状态为IDLE，下次循环会重新进入初始扫描流程
                self._exploration_state = ExplorationHandlerState.IDLE
                return  # 返回，下次循环执行扫描
            
            elif nav_state == NavigationState.FAILED:
                self._node.get_logger().warn(
                    "[ExplorationHandler] Failed to reach initial scan point, "
                    "performing scan at current position"
                )
                self._initial_scan_point_reached = True
                # 🔧 关键修复：改变状态为IDLE，下次循环会重新进入初始扫描流程
                self._exploration_state = ExplorationHandlerState.IDLE
                return  # 返回，下次循环执行扫描
        
        elif self._exploration_state == ExplorationHandlerState.DETECTING_FRONTIERS:
            self._process_frontier_detection(task)
        
        elif self._exploration_state == ExplorationHandlerState.NAVIGATING_TO_FRONTIER:
            # 🔧 方案2修复：在导航状态时，只监控导航进度，不重复检测frontier
            # 🐛 DEBUG: 节流日志验证状态检测
            current_time = time.time()
            if not hasattr(self, '_last_state_log_time'):
                self._last_state_log_time = 0.0
            if current_time - self._last_state_log_time > 5.0:  # 每5秒记录一次
                self._node.get_logger().debug("[DEBUG] NAVIGATING_TO_FRONTIER state detected")
                self._last_state_log_time = current_time
            
            # 检查导航器状态
            nav_state = self._nav_executor.get_state()
            
            if nav_state == NavigationState.EXECUTING:
                # 🎯 关键修复：导航执行中也要检查超时，不能直接return
                # 导航正在执行，调用_monitor_navigation检查超时
                self._monitor_navigation(task)
                return
            elif nav_state == NavigationState.SUCCESS:
                # 导航成功，回到检测状态
                self._node.get_logger().info("[ExplorationHandler] Navigation succeeded, returning to frontier detection")
                self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
                
                # 🎯 关键修复：导航成功后重置所有失败计数器，确保连续失败才累积
                self._exploration_strategy.reset_failure_count()
                self._exploration_strategy.reset_no_goal_count()
                self._rotation_controller.escape_attempt = 0
                self._rotation_controller.consecutive_rotation_failures = 0
                self._consecutive_failures = 0
                self._node.get_logger().info(
                    "📦 [ExplorationHandler] All failure counters reset after navigation success"
                )
                return
            elif nav_state == NavigationState.FAILED:
                # 导航失败，记录并处理
                self._node.get_logger().warn("[ExplorationHandler] Navigation failed, handling failure...")
                if self._exploration_strategy.increment_failure_count():
                    self._check_exploration_completion(task)
                else:
                    self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
                return
            else:
                # 其他状态（IDLE等），调用原有逻辑
                self._monitor_navigation(task)
        
        elif self._exploration_state == ExplorationHandlerState.ROTATING_AT_GOAL:
            # 监控旋转进度 / Monitor rotation progress
            self._monitor_rotation(task)
        
        elif self._exploration_state == ExplorationHandlerState.COMPLETED:
            # 🎯 关键修复：探索完成时，直接退出，不重复处理
            # 任务已经在 _complete_exploration() 中被标记为完成并保存地图
            # 这里只需确保不再执行任何逻辑，等待任务管理器清理
            return
    
    def _process_frontier_detection(self, task: Task):
        """处理边界检测"""
        # 使用 FrontierDetector 检测边界
        try:
            # 更新地图数据 / Update map data
            self._frontier_detector.update_map(self._current_map)
            # 🔧 关键修复：同时更新safety manager的map数据
            self._safety_manager.update_map(self._current_map)
            
            # 检测边界 / Detect frontiers (returns raw grid points)
            raw_frontiers = self._frontier_detector.find_frontiers()
            
            self._node.get_logger().debug(f"[DEBUG] Step 1: find_frontiers() returned {len(raw_frontiers) if raw_frontiers else 0} clusters")
            
            if not raw_frontiers or len(raw_frontiers) == 0:
                # 没有边界 → 检查是否完成
                self._check_exploration_completion(task)
                return
            
            # 转换为包含完整信息的边界对象 / Convert to frontier info objects
            frontiers = []
            for idx, raw_frontier in enumerate(raw_frontiers):
                frontier_info = self._frontier_detector.calculate_frontier_info(raw_frontier)
                if frontier_info:  # 确保有效 / Ensure valid
                    frontiers.append(frontier_info)
                else:
                    self._node.get_logger().debug(f"[DEBUG] Cluster {idx} filtered by calculate_frontier_info (invalid)")
            
            self._node.get_logger().debug(f"[DEBUG] Step 2: calculate_frontier_info() kept {len(frontiers)}/{len(raw_frontiers)} clusters")
            
            if not frontiers:
                # 无有效边界
                self._check_exploration_completion(task)
                return
            
            # 过滤已访问的边界
            frontiers_before_filter = len(frontiers)
            frontiers = self._filter_visited_frontiers(frontiers)
            
            self._node.get_logger().debug(f"[DEBUG] Step 3: _filter_visited_frontiers() kept {len(frontiers)}/{frontiers_before_filter} clusters")
            
            if not frontiers:
                # 所有边界都已访问
                self._check_exploration_completion(task)
                return
            
            # 选择最优边界
            best_frontier = self._select_best_frontier(frontiers)
            
            self._node.get_logger().debug(f"[DEBUG] Step 4: _select_best_frontier() result: {'FOUND' if best_frontier else 'NONE'}")
            
            if best_frontier is None:
                # 无法选择边界
                self._check_exploration_completion(task)
                return
            
            # 🎯 Phase 1 & 3: 使用两步导航策略并添加安全检查
            frontier_center = best_frontier['center_world']
            robot_pos = self._get_robot_position()
            
            if robot_pos is None:
                self._node.get_logger().warn("[ExplorationHandler] Cannot get robot position")
                return
            
            # 最终安全检查（传递completion启用动态安全距离）
            completion = self._calculate_map_completion()
            is_safe, safety_reason = self._safety_manager.is_goal_safe(
                frontier_center[0], frontier_center[1], robot_pos, completion
            )
            
            if not is_safe:
                self._node.get_logger().warn(f"[ExplorationHandler] Final safety check failed: {safety_reason}")
                
                # 标记frontier为已访问
                self._exploration_strategy.update_visited_frontier(frontier_center)
                
                # 增加失败计数
                if self._exploration_strategy.increment_failure_count():
                    
                    # 🎯 Phase 3: 检查连续旋转失败，触发后退脱困
                    if self._rotation_controller.consecutive_rotation_failures >= \
                       self._rotation_controller.max_consecutive_rotation_failures:
                        
                        self._node.get_logger().warn("[ExplorationHandler] Multiple rotation failures, performing backward escape")
                        self._perform_backward_escape()
                        self._rotation_controller.consecutive_rotation_failures = 0
                    else:
                        self._node.get_logger().warn("[ExplorationHandler] Multiple safety check failures, triggering escape rotation")
                        self._rotation_controller.start_escape_rotation(self._get_robot_yaw_odom, self._pending_goal)
                    
                    self._exploration_strategy.reset_failure_count()
                
                return
            
            # 构造frontier评估结果（用于两步导航）
            frontier_eval = {
                'frontier_info': best_frontier,
                'distance': math.sqrt(
                    (frontier_center[0] - robot_pos[0])**2 +
                    (frontier_center[1] - robot_pos[1])**2
                ),
                'direction': math.atan2(
                    frontier_center[1] - robot_pos[1],
                    frontier_center[0] - robot_pos[0]
                )
            }
            
            # 🎯 方案C: 在发送导航目标前，检查是否应该主动完成探索
            # Proactively check if exploration should complete before sending nav goal
            should_complete, reason = self._should_complete_exploration_proactive(
                best_frontier, frontier_eval
            )
            
            if should_complete:
                self._node.get_logger().info(
                    f"✅ [Proactive Completion] {reason}"
                )
                # 🎯 关键修复：触发完成验证流程，而不是直接退出
                # 这会执行最后的扫描验证，确保没有遗漏区域
                if self._check_exploration_completion(task):
                    return  # 验证流程已启动或完成，退出当前循环
                # 如果验证返回False，说明还需继续探索，继续当前流程
                self._node.get_logger().info(
                    "[Proactive Completion] Verification did not trigger completion, continuing exploration"
                )
            
            # 使用两步导航策略
            self._send_navigation_goal(frontier_center, frontier_eval, task)
            
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
            # 检查是否已访问 (使用 center_world 键)
            is_visited = False
            frontier_center = frontier['center_world']  # (x, y) in world coordinates
            
            for visited_pos in self._explored_positions:
                dist = math.sqrt(
                    (frontier_center[0] - visited_pos[0])**2 +
                    (frontier_center[1] - visited_pos[1])**2
                )
                if dist < self._visit_radius:
                    is_visited = True
                    break
            
            if not is_visited:
                filtered.append(frontier)
        
        return filtered
    
    def _select_best_frontier(self, frontiers):
        """选择最优边界（完整评分算法）/ Select optimal frontier with full scoring"""
        if not frontiers:
            return None
        
        # 🎯 阶段1: 获取机器人位置和朝向
        robot_pose = self._nav_executor.get_robot_pose()
        robot_yaw = self._nav_executor.get_robot_yaw()
        
        if robot_pose is None or robot_yaw is None:
            self._node.get_logger().warn("[ExplorationHandler] Robot pose unavailable, using first frontier")
            return frontiers[0]
        
        robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y)
        
        # 🎯 阶段1: 计算地图完成度
        completion = self._calculate_map_completion()
        
        # 🎯 阶段1: 更新评估器状态
        self._frontier_evaluator.update_state(
            visited_frontiers=self._visited_frontiers,
            last_goal_direction=self._last_goal_direction,
            stagnant_count=self._stagnant_count
        )
        
        # 🎯 阶段1: 评估所有边界并选择最优
        best_frontier = None
        best_eval = None
        
        filtered_by_eval = 0
        filtered_by_safety = 0
        
        for idx, frontier in enumerate(frontiers):
            frontier_center = frontier['center_world']
            
            # 🎯 方案2: 检查是否在失败黑名单中
            if self._should_skip_frontier(frontier_center):
                filtered_by_eval += 1
                self._node.get_logger().debug(
                    f"[DEBUG] Frontier {idx} at ({frontier_center[0]:.2f}, {frontier_center[1]:.2f}) "
                    f"REJECTED by blacklist (failed >= {self._max_frontier_failures} times)"
                )
                continue
            
            # 使用 FrontierEvaluator 评估
            frontier_eval = self._frontier_evaluator.evaluate_frontier(
                frontier_info=frontier,
                robot_pos=robot_pos,
                robot_yaw=robot_yaw,
                completion=completion,
                min_completion_threshold=0.75,
                max_angle_deg=180,  # 🎯 使用 180° 让软角度评分生效，不硬过滤
                min_distance=0.2,
                max_distance=5.0
            )
            
            if frontier_eval is None:
                filtered_by_eval += 1
                self._node.get_logger().debug(
                    f"[DEBUG] Frontier {idx} at ({frontier_center[0]:.2f}, {frontier_center[1]:.2f}) "
                    f"REJECTED by evaluator (None)"
                )
                continue
            
            # 🎯 阶段1: 安全检查（传递completion启用动态安全距离）
            is_safe, reason = self._safety_manager.is_goal_safe(
                frontier_center[0], frontier_center[1], robot_pos, completion
            )
            
            if not is_safe:
                filtered_by_safety += 1
                self._node.get_logger().debug(
                    f"[DEBUG] Frontier {idx} at ({frontier_center[0]:.2f}, {frontier_center[1]:.2f}) "
                    f"REJECTED by safety: {reason}"
                )
                continue
            
            # 保存评分结果
            frontier_eval['frontier'] = frontier
            
            # 选择最高分
            if best_eval is None or frontier_eval['score'] > best_eval['score']:
                best_frontier = frontier
                best_eval = frontier_eval
        
        # 汇总过滤统计
        self._node.get_logger().debug(
            f"[DEBUG] Frontier selection summary: "
            f"Total={len(frontiers)}, "
            f"Rejected by evaluator={filtered_by_eval}, "
            f"Rejected by safety={filtered_by_safety}, "
            f"Final selected={'YES' if best_frontier else 'NO'}"
        )
        
        if best_frontier is not None:
            self._node.get_logger().info(
                f"[ExplorationHandler] Selected frontier with score {best_eval['score']:.3f} "
                f"at ({best_frontier['center_world'][0]:.2f}, {best_frontier['center_world'][1]:.2f}), "
                f"angle={math.degrees(best_eval.get('relative_angle', 0)):.1f}°"
            )
        
        return best_frontier
    
    def _send_navigation_goal(self, target_pos, frontier_eval, task: Task):
        """
        发送导航目标（两步策略：先转向，再移动）
        Send navigation goal (two-step: rotate first, then move)
        """
        
        # 1. 计算目标yaw
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            return
        
        target_yaw = math.atan2(target_pos[1] - robot_pos[1], target_pos[0] - robot_pos[0])
        
        # 2. 记录探索状态
        self._exploration_strategy.update_visited_frontier(target_pos)
        if 'direction' in frontier_eval:
            self._exploration_strategy.update_last_goal_direction(frontier_eval['direction'])
        self._exploration_strategy.reset_no_goal_count()
        
        # 3. 获取机器人当前yaw
        robot_yaw = self._get_robot_yaw_odom()
        if robot_yaw is None:
            # 降级：直接导航
            self._send_direct_navigation(target_pos, target_yaw, task)
            return
        
        # 4. 计算角度差
        angle_diff = self._normalize_angle(target_yaw - robot_yaw)
        
        # 5. ⚠️ 关键判断：是否需要先旋转（10°阈值）
        if abs(angle_diff) > math.radians(10):
            
            # 第一步：调整朝向
            self._node.get_logger().info(
                f"[ExplorationHandler] First step: adjusting orientation to target "
                f"(need to rotate {math.degrees(angle_diff):.1f}°)"
            )
            
            # 保存pending_goal
            self._pending_goal = {
                'position': target_pos,
                'yaw': target_yaw,
                'frontier_info': {
                    'size': frontier_eval.get('frontier_info', {}).get('size', 0),
                    'distance': frontier_eval.get('distance', 0),
                    'direction': frontier_eval.get('direction', 0)
                }
            }
            
            # 开始旋转
            get_yaw_func = self._get_robot_yaw_odom
            success = self._rotation_controller.start_rotation(
                math.degrees(angle_diff),
                get_yaw_func,
                self._pending_goal
            )
            
            if success:
                self._exploration_state = ExplorationHandlerState.ROTATING_AT_GOAL
            else:
                # 旋转失败，直接导航
                self._send_direct_navigation(target_pos, target_yaw, task)
            
            return  # ⚠️ 不发送导航，等待旋转完成
        
        # 6. 第二步：直接移动（角度差<10°）
        self._send_direct_navigation(target_pos, target_yaw, task)
    
    def _send_direct_navigation(self, target_pos, target_yaw, task):
        """发送直接导航命令 / Send direct navigation command"""
        self._node.get_logger().info(
            f"[ExplorationHandler] Second step: moving to target "
            f"position=({target_pos[0]:.2f}, {target_pos[1]:.2f})"
        )
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_pos[0]
        goal_pose.pose.position.y = target_pos[1]
        
        # 设置orientation
        quat = self._yaw_to_quaternion(target_yaw)
        goal_pose.pose.orientation = quat
        
        if self._nav_executor.navigate_to_pose(goal_pose):
            self._exploration_state = ExplorationHandlerState.NAVIGATING_TO_FRONTIER
            self._exploration_strategy.reset_failure_count()
            
            # 🎯 记录导航开始时间和目标距离（用于动态超时检测）
            self._goal_start_time = time.time()
            robot_pos = self._get_robot_position()
            if robot_pos:
                goal_distance = math.sqrt(
                    (target_pos[0] - robot_pos[0])**2 + (target_pos[1] - robot_pos[1])**2
                )
                self._goal_distance = goal_distance
            else:
                self._goal_distance = 5.0  # 默认5米
            self._node.get_logger().info(
                f"[ExplorationHandler] Navigation goal sent successfully, "
                f"state: {self._exploration_state.name}"
            )
        else:
            # 🔧 方案1修复：区分真正的发送失败 vs 导航器忙碌的重复发送
            nav_state = self._nav_executor.get_state()
            
            if nav_state == NavigationState.EXECUTING:
                # 导航器正在执行，这不是失败，只是重复发送
                self._node.get_logger().warn(
                    "[ExplorationHandler] Cannot send goal: navigation already in progress. "
                    "Waiting for current navigation to complete..."
                )
                # 不增加失败计数，保持当前状态
                return
            else:
                # 真正的失败（导航器空闲但拒绝请求）
                self._node.get_logger().error(
                    f"[ExplorationHandler] Failed to send navigation goal. "
                    f"Navigator state: {nav_state.name if nav_state else 'Unknown'}"
                )
                if self._exploration_strategy.increment_failure_count():
                    self._node.get_logger().error(
                        f"[ExplorationHandler] Max failures reached "
                        f"({self._exploration_strategy.consecutive_failures}/"
                        f"{self._exploration_strategy.max_failures}), checking completion"
                    )
                    self._check_exploration_completion(task)
                else:
                    # 失败但未达到上限，等待后重试
                    self._node.get_logger().warn(
                        f"[ExplorationHandler] Navigation failure count: "
                        f"{self._exploration_strategy.consecutive_failures}/"
                        f"{self._exploration_strategy.max_failures}"
                    )
                    time.sleep(1.0)  # 等待1秒后重试
    
    def _yaw_to_quaternion(self, yaw):
        """将yaw角转换为四元数 / Convert yaw to quaternion"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def _normalize_angle(self, angle):
        """归一化角度到[-pi, pi] / Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _navigate_to_frontier(self, frontier, task: Task):
        """
        导航到边界（使用两段式导航+边界验证）
        Navigate to frontier (using two-step navigation with boundary validation)
        
        🎯 重构：统一使用 _send_navigation_goal 确保两段式导航和边界检查
        """
        frontier_center = frontier['center_world']  # (x, y) tuple
        
        # 🎯 关键修复：使用边界验证器检查目标
        is_valid, reason, goal_coords = self._boundary_validator.validate_goal(
            goal_world=frontier_center,
            current_map=self._current_map,
            logger=self._node.get_logger()
        )
        
        if not is_valid:
            self._node.get_logger().warn(
                f"⚠️ [ExplorationHandler] Frontier at ({frontier_center[0]:.2f}, {frontier_center[1]:.2f}) "
                f"rejected by boundary validator: {reason}"
            )
            # 标记为已访问，避免重复尝试
            self._exploration_strategy.update_visited_frontier(frontier_center)
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
            return
        
        # 构造 frontier_eval（用于两段式导航）
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self._node.get_logger().warn("[ExplorationHandler] Cannot get robot position")
            return
        
        frontier_eval = {
            'frontier_info': frontier,
            'distance': math.sqrt(
                (frontier_center[0] - robot_pos[0])**2 +
                (frontier_center[1] - robot_pos[1])**2
            ),
            'direction': math.atan2(
                frontier_center[1] - robot_pos[1],
                frontier_center[0] - robot_pos[0]
            )
        }
        
        # 🎯 使用统一的两段式导航
        self._send_navigation_goal(frontier_center, frontier_eval, task)
    
    def _monitor_navigation(self, task: Task):
        """监控导航进度（新增：基于距离的动态超时检测）"""
        
        # 🐛 DEBUG: 验证_monitor_navigation被调用
        self._node.get_logger().debug("[DEBUG] _monitor_navigation() called")
        
        # 🎯 动态超时检测：根据目标距离计算超时时间
        # 公式：timeout = (distance / avg_speed) + buffer
        # avg_speed = 0.12 m/s (更宽松的估计，考虑障碍物绕行), buffer = 40s
        if hasattr(self, '_goal_start_time') and self._goal_start_time is not None:
            elapsed = time.time() - self._goal_start_time
            
            # 计算动态超时时间
            goal_distance = getattr(self, '_goal_distance', 5.0)
            avg_speed = 0.12  # 更宽松的平均速度 m/s（考虑绕行和避障，从0.15改为0.12）
            buffer_time = 40.0  # 额外缓冲时间（从30s增加到40s）
            dynamic_timeout = (goal_distance / avg_speed) + buffer_time
            
            # 最小40秒，最大180秒（从120s增加到180s，支持更长距离）
            dynamic_timeout = max(40.0, min(dynamic_timeout, 180.0))
            
            if elapsed > dynamic_timeout:
                self._node.get_logger().warn(
                    f"⏱️ [Navigation Timeout] {elapsed:.1f}s > {dynamic_timeout:.1f}s "
                    f"(distance={goal_distance:.2f}m, speed={avg_speed}m/s). "
                    f"Canceling and reselecting frontier..."
                )
                
                # 取消导航并当作失败处理
                self._nav_executor.cancel_navigation()
                self._handle_navigation_failure(task)
                return
            
            # 🎯 新增：距离停滞检测
            # 如果导航超过30秒，检查是否接近目标（避免慢速移动但不到达）
            if elapsed > 30.0 and goal_distance > 0:
                robot_pose = self._nav_executor.get_robot_pose()
                if robot_pose and self._current_frontier:
                    current_distance = math.sqrt(
                        (robot_pose.pose.position.x - self._current_frontier['center_world'][0])**2 +
                        (robot_pose.pose.position.y - self._current_frontier['center_world'][1])**2
                    )
                    
                    # 如果30秒后距离目标仍然很远（>80%初始距离），认为停滞
                    if current_distance > goal_distance * 0.8:
                        self._node.get_logger().warn(
                            f"⏱️ [Navigation Stagnation] After {elapsed:.1f}s, still {current_distance:.2f}m "
                            f"from goal ({current_distance/goal_distance*100:.1f}% of initial). "
                            f"Making insufficient progress. Canceling..."
                        )
                        
                        # 取消导航并当作失败处理
                        self._nav_executor.cancel_navigation()
                        self._handle_navigation_failure(task)
                        return
        
        nav_state = self._nav_executor.get_state()
        
        if nav_state == NavigationState.SUCCESS:
            # 🎯 Phase 1: 特殊检查 - 是否是预移动点导航
            if self._rotation_controller.skip_rotation_on_obstacle and \
               self._rotation_controller.pre_rotation_point is not None:
                # 预移动点导航成功，不在这里处理
                self._node.get_logger().debug("[ExplorationHandler] Pre-movement point navigation succeeded")
                # 保持SUCCESS状态，交给_handle_rotation_obstacle处理
                return  # ⚠️ 不重置状态，不进入DETECTING_FRONTIERS
            
            # 正常导航成功
            self._node.get_logger().info("[ExplorationHandler] Reached frontier, continuing exploration")
            
            # 清空skip标志（防止卡死）
            if self._rotation_controller.skip_rotation_on_obstacle:
                self._node.get_logger().info("[ExplorationHandler] Clearing skip_rotation_on_obstacle flag")
                self._rotation_controller.skip_rotation_on_obstacle = False
            
            # 清空pending_goal
            self._pending_goal = None
            
            # 🎯 P1: 重置脱困计数（导航成功后重置）
            if hasattr(self._rotation_controller, 'reset_escape_attempts'):
                self._rotation_controller.reset_escape_attempts()
            self._rotation_controller.consecutive_rotation_failures = 0
            
            # 🎯 P3: 检查探索进展（导航成功后）
            if hasattr(self, '_current_map') and self._current_map is not None:
                try:
                    import numpy as np
                    # 计算当前已知区域数量 / Calculate current known cells count
                    map_data = np.array(self._current_map.data).reshape(
                        (self._current_map.info.height, self._current_map.info.width)
                    )
                    known_cells = np.count_nonzero((map_data >= 0) & (map_data < 100))
                    
                    # 检查探索进展 / Check exploration progress
                    progress_status = self._exploration_strategy.check_exploration_progress(known_cells)
                    
                    # 🎯 P3: 判断是否需要触发旋转（停滞检测）
                    if self._exploration_strategy.should_trigger_rotation(progress_status):
                        self._node.get_logger().info(
                            f"[ExplorationHandler] Exploration stagnation detected "
                            f"(new cells: {progress_status['new_known_cells']}), triggering escape rotation"
                        )
                        # 触发脱困旋转 / Trigger escape rotation
                        self._rotation_controller.start_escape_rotation(
                            self._get_robot_yaw_odom, 
                            self._pending_goal
                        )
                    
                    # 🎯 P3: 判断是否需要触发360度扫描（有效探索）
                    elif self._exploration_strategy.should_trigger_360_scan(progress_status):
                        self._node.get_logger().info(
                            f"[ExplorationHandler] Effective exploration detected "
                            f"(new cells: {progress_status['new_known_cells']}), triggering 360° scan"
                        )
                        # 触发360度扫描 / Trigger 360-degree scan
                        # 使用固定角度360度 / Use fixed 360-degree angle
                        self._rotation_controller.start_rotation(
                            360,  # 360度扫描 / 360-degree scan
                            self._get_robot_yaw_odom,
                            None  # 扫描不关联pending_goal / Scan not associated with pending_goal
                        )
                    else:
                        # 正常情况，记录进展信息 / Normal case, log progress info
                        self._node.get_logger().debug(
                            f"[ExplorationHandler] Exploration progress: new cells = {progress_status['new_known_cells']}",
                            throttle_duration_sec=5.0
                        )
                except Exception as e:
                    self._node.get_logger().warn(f"[ExplorationHandler] Failed to check exploration progress: {e}")
            
            # 🎯 导航成功 → 重置失败计数
            self._consecutive_failures = 0
            
            # 导航成功 → 记录已探索位置
            if self._current_frontier:
                frontier_center = self._current_frontier['center_world']  # (x, y) tuple
                self._explored_positions.append(
                    (frontier_center[0], frontier_center[1])
                )
            
            # 更新任务进度
            coverage = self._calculate_map_completion()
            self._task_manager.update_progress(task.task_id, coverage)
            
            # 🎯 P1: 清空导航计时器
            self._goal_start_time = None
            
            # 继续探索
            self._current_frontier = None
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        elif nav_state == NavigationState.FAILED:
            # 🎯 导航失败 → 记录失败的frontier
            if self._current_frontier is not None:
                frontier_pos = self._current_frontier['center_world']
                self._record_failed_frontier(frontier_pos)
            
            # 增加失败计数
            self._consecutive_failures += 1
            self._node.get_logger().warn(
                f"[ExplorationHandler] Navigation failed (consecutive: {self._consecutive_failures}/{self._max_consecutive_failures})"
            )
            
            # 🎯 优化：检查是否因完成度高导致的规划失败
            # 如果完成度 >= 85% 且连续失败，可能是剩余区域不可达，视为完成
            completion = self._calculate_map_completion()
            
            if completion >= 0.85 and self._consecutive_failures >= 3:
                self._node.get_logger().info(
                    f"[ExplorationHandler] High completion ({completion*100:.1f}%) with repeated navigation failures. "
                    f"Remaining areas may be unreachable. Completing exploration."
                )
                self._complete_exploration(task)
                return
            
            # 🔴 检查是否达到连续失败上限
            if self._consecutive_failures >= self._max_consecutive_failures:
                # 🎯 重构：使用 TaskTerminationManager 终止任务
                self._termination_manager.terminate_task(
                    task=task,
                    final_state=TaskState.FAILED,
                    reason=f"Exceeded max consecutive failures ({self._max_consecutive_failures})",
                    release_callback=lambda task_id: (
                        setattr(self, '_exploration_state', ExplorationHandlerState.COMPLETED),
                        self.release_executor(task_id)
                    )
                )
                return
            
            # 🔴 检查 SafetyManager 失败限制
            if hasattr(self, '_safety_manager'):
                if self._safety_manager.consecutive_failures >= self._safety_manager.max_failures:
                    # 🎯 重构：使用 TaskTerminationManager 终止任务
                    self._termination_manager.terminate_task(
                        task=task,
                        final_state=TaskState.FAILED,
                        reason=f"SafetyManager max failures reached ({self._safety_manager.max_failures})",
                        release_callback=lambda task_id: (
                            setattr(self, '_exploration_state', ExplorationHandlerState.COMPLETED),
                            self.release_executor(task_id)
                        )
                    )
                    return
            
            # 否则继续重试 → 重新检测边界
            self._current_frontier = None
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        elif nav_state == NavigationState.CANCELED:
            # 🎯 关键：区分取消原因
            # 如果在NAVIGATING状态被取消 → Nav2失败（BT重试用尽），当作导航失败处理
            # 如果在其他状态被取消 → 用户主动取消
            if self._exploration_state == ExplorationHandlerState.NAVIGATING_TO_FRONTIER:
                self._node.get_logger().warn(
                    "[ExplorationHandler] Navigation was canceled by Nav2 (BT retries exhausted), "
                    "treating as navigation failure instead of task cancellation"
                )
                self._handle_navigation_failure(task)
            else:
                # 真正的用户取消
                self._node.get_logger().info(
                    "[ExplorationHandler] User-initiated cancellation detected"
                )
                self._handle_cancel(task)
    
    def _handle_navigation_failure(self, task: Task):
        """
        处理导航失败（不取消任务，继续探索）
        Handle navigation failure (continue exploration instead of canceling task)
        
        流程 / Process:
        1. 记录失败的frontier到黑名单 / Record failed frontier to blacklist
        2. 增加连续失败计数 / Increment consecutive failure count
        3. 检查是否应该触发完成验证 / Check if completion verification should be triggered
        4. 否则重新选择frontier继续探索 / Otherwise reselect frontier and continue
        """
        
        # 1. 记录失败的frontier到黑名单
        if self._current_frontier:
            frontier_pos = self._current_frontier['center_world']
            self._record_failed_frontier(frontier_pos)
            self._node.get_logger().warn(
                f"❌ Navigation failed to frontier at ({frontier_pos[0]:.2f}, {frontier_pos[1]:.2f}), "
                f"added to failure history"
            )
        
        # 2. 增加连续失败计数
        self._consecutive_failures += 1
        self._node.get_logger().warn(
            f"[ExplorationHandler] Navigation failed "
            f"(consecutive: {self._consecutive_failures}/{self._max_consecutive_failures})"
        )
        
        # 3. 检查是否应该触发完成验证
        completion = self._calculate_map_completion()
        
        # 🎯 重构：使用 CompletionStrategy 判断是否完成
        should_complete, reason = self._completion_strategy.should_complete(
            current_completion=completion,
            consecutive_failures=self._consecutive_failures,
            no_frontiers=False,
            context="navigation_failure"
        )
        
        if should_complete:
            self._node.get_logger().info(
                f"{reason}. Triggering completion verification."
            )
            # 触发完成验证（不是直接退出）
            if self._check_exploration_completion(task):
                return
        
        # 3b. 连续失败过多 → 触发完成验证
        if self._consecutive_failures >= self._max_consecutive_failures:
            self._node.get_logger().error(
                f"⚠️ Exceeded max consecutive failures ({self._max_consecutive_failures}). "
                f"Attempting final completion verification before giving up."
            )
            # 尝试最后验证
            if self._check_exploration_completion(task):
                return
            else:
                # 验证失败，任务真的失败了
                # 🎯 重构：使用 TaskTerminationManager
                self._termination_manager.terminate_task(
                    task=task,
                    final_state=TaskState.FAILED,
                    reason="Max consecutive failures exceeded after completion verification",
                    release_callback=lambda task_id: (
                        setattr(self, '_exploration_state', ExplorationHandlerState.COMPLETED),
                        self.release_executor(task_id)
                    )
                )
                return
        
        # 4. 否则继续探索 → 重新选择frontier
        self._current_frontier = None
        self._goal_start_time = None
        self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        self._node.get_logger().info(
            f"🔄 Reselecting frontier to continue exploration "
            f"(consecutive failures: {self._consecutive_failures}/{self._max_consecutive_failures})"
        )

        
        self._node.get_logger().info(
            f"🔄 Reselecting frontier to continue exploration "
            f"(consecutive failures: {self._consecutive_failures}/{self._max_consecutive_failures})"
        )
    
    def _check_exploration_completion(self, task: Task) -> bool:
        """
        检查探索是否完成（含验证循环）
        Check exploration completion (with verification loop)
        """
        
        # 1. 计算完成度
        completion = self._calculate_map_completion()
        
        # 2. 达到阈值且未进入验证模式
        if completion >= self._map_completion_threshold and not self._completion_scan_in_progress:
            self._node.get_logger().info(
                f"[ExplorationHandler] Completion threshold reached ({completion*100:.2f}%), "
                f"attempting final verification"
            )
            
            # 🎯 关键优化：达到阈值时，先重置脱困计数器，避免误判
            self._rotation_controller.escape_attempt = 0
            self._rotation_controller.consecutive_rotation_failures = 0
            self._exploration_strategy.reset_no_goal_count()
            
            # 进入验证模式
            self._completion_scan_in_progress = True
            self._completion_before_scan = completion
            self._completion_verification_attempts = 0
            
            # 🐛 方案B: 清空已尝试frontier集合，允许重新验证
            self._verified_frontiers = set()
            
            # 🎯 重置验证旋转状态
            self._verification_rotation_pending = False
            self._verification_nav_completed = False
            
            return self._attempt_final_verification(task)
        
        # 3. 正在验证中
        if self._completion_scan_in_progress:
            
            # 检查完成度是否增长
            if completion > self._completion_before_scan + 0.01:  # 增长>1%
                self._node.get_logger().info(
                    f"[ExplorationHandler] Map expanded during verification "
                    f"({self._completion_before_scan*100:.2f}% -> {completion*100:.2f}%), "
                    f"continuing exploration"
                )
                # 取消验证，继续探索
                self._completion_scan_in_progress = False
                self._verification_rotation_pending = False
                self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
                return False
            
            # 🎯 新增：检查是否需要执行验证后的180°旋转
            if self._verification_rotation_pending:
                # 检查旋转是否在进行
                if self._rotation_controller.is_rotating_state:
                    return True  # 旋转中，等待完成
                
                # 旋转已完成（或未开始），继续下一次验证
                self._verification_rotation_pending = False
                self._node.get_logger().info(
                    f"[ExplorationHandler] Verification rotation completed. "
                    f"Proceeding to next verification attempt ({self._completion_verification_attempts}/{self._max_verification_attempts})"
                )
                
                # 检查验证次数
                if self._completion_verification_attempts >= self._max_verification_attempts:
                    self._node.get_logger().info(
                        f"[ExplorationHandler] Final verification completed after "
                        f"{self._completion_verification_attempts} attempts. "
                        f"Final completion: {completion*100:.2f}%. Completing exploration..."
                    )
                    # 🔧 关键修复：使用标志防止重复完成
                    if not self._exploration_completed:
                        self._exploration_completed = True
                        self._complete_exploration(task)
                        self._exploration_state = ExplorationHandlerState.COMPLETED
                    return True
                
                # 继续下一次验证
                return self._attempt_final_verification(task)
            
            # 等待导航完成
            nav_state = self._nav_executor.get_state()
            if nav_state == NavigationState.EXECUTING:
                return True  # 阻塞，等待导航
            
            # 🎯 导航完成（成功或失败），触发180°旋转
            if nav_state in [NavigationState.SUCCESS, NavigationState.FAILED]:
                if not self._verification_nav_completed:
                    self._verification_nav_completed = True
                    
                    result_str = "succeeded" if nav_state == NavigationState.SUCCESS else "failed"
                    self._node.get_logger().info(
                        f"[ExplorationHandler] Verification navigation {result_str}. "
                        f"Starting 180° rotation for comprehensive scanning..."
                    )
                    
                    # 启动180°旋转
                    rotation_started = False
                    robot_yaw = self._get_robot_yaw_odom()
                    
                    if robot_yaw is not None:
                        try:
                            success = self._rotation_controller.start_rotation(
                                angle_degrees=180.0,  # 🔧 修复：使用正确的参数名 angle_degrees
                                current_yaw_func=self._get_robot_yaw_odom
                            )
                            
                            if success:
                                rotation_started = True
                                self._verification_rotation_pending = True
                                self._exploration_state = ExplorationHandlerState.ROTATING_AT_GOAL
                                self._node.get_logger().info(
                                    "[ExplorationHandler] Verification rotation started successfully"
                                )
                            else:
                                # 旋转启动失败
                                self._node.get_logger().warn(
                                    "[ExplorationHandler] Failed to start verification rotation"
                                )
                        except Exception as e:
                            self._node.get_logger().error(
                                f"[ExplorationHandler] Error starting verification rotation: {e}"
                            )
                    else:
                        self._node.get_logger().warn(
                            "[ExplorationHandler] Cannot get robot yaw for verification rotation"
                        )
                    
                    # 🔧 关键修复：如果旋转启动成功，返回True继续旋转
                    if rotation_started:
                        return True
                    
                    # 🔧 关键修复：旋转启动失败，直接进入下一次验证
                    # 重置标志
                    self._verification_rotation_pending = False
                    self._verification_nav_completed = False  # 🔧 重要：重置这个标志，允许下次导航完成后再试
                    
                    self._node.get_logger().info(
                        f"[ExplorationHandler] Skipped verification rotation, "
                        f"proceeding to next verification attempt ({self._completion_verification_attempts}/{self._max_verification_attempts})"
                    )
                    
                    # 检查验证次数
                    if self._completion_verification_attempts >= self._max_verification_attempts:
                        self._node.get_logger().info(
                            f"[ExplorationHandler] Final verification completed after "
                            f"{self._completion_verification_attempts} attempts. "
                            f"Final completion: {completion*100:.2f}%. Completing exploration..."
                        )
                        if not self._exploration_completed:
                            self._exploration_completed = True
                            self._complete_exploration(task)
                            self._exploration_state = ExplorationHandlerState.COMPLETED
                        return True
                    
                    # 🔧 继续下一次验证
                    return self._attempt_final_verification(task)
                
                return True
        
        # 4. 未达到阈值，继续探索
        self._node.get_logger().info(
            f"[ExplorationHandler] No frontiers found. "
            f"Map coverage: {completion*100:.1f}%, "
            f"threshold: {self._map_completion_threshold*100:.1f}%"
        )
        
        # 🎯 方案4优化：更智能的退出逻辑
        self._exploration_strategy.no_goal_count += 1
        
        # 🎯 优化1：完成度未达标时，逐步放宽安全距离重试
        if completion < 0.80 and self._exploration_strategy.no_goal_count <= 5:
            # 逐步放宽SafetyManager的安全距离
            if self._exploration_strategy.no_goal_count == 2:
                self._safety_manager.safe_distance = 0.30
                self._node.get_logger().warn(
                    f"[ExplorationHandler] Attempt {self._exploration_strategy.no_goal_count}: "
                    f"Reducing safety distance to 0.30m to reach unexplored areas"
                )
            elif self._exploration_strategy.no_goal_count == 3:
                self._safety_manager.safe_distance = 0.25
                self._node.get_logger().warn(
                    f"[ExplorationHandler] Attempt {self._exploration_strategy.no_goal_count}: "
                    f"Reducing safety distance to 0.25m (aggressive mode)"
                )
            elif self._exploration_strategy.no_goal_count == 4:
                # 清空visited frontiers，允许重访
                self._exploration_strategy.evaluator.visited_frontiers.clear()
                self._node.get_logger().warn(
                    f"[ExplorationHandler] Attempt {self._exploration_strategy.no_goal_count}: "
                    f"Cleared visited frontiers to retry previous areas"
                )
        
        # 🎯 优化2：只有在完成度>=80% 且 连续3次无frontier 时才退出
        if completion >= 0.80 and self._exploration_strategy.no_goal_count >= 3:
            self._node.get_logger().info(
                f"[ExplorationHandler] No frontiers found after 3 attempts with {completion*100:.1f}% completion. "
                f"Exploration complete!"
            )
            self._exploration_state = ExplorationHandlerState.COMPLETED
            return True
        
        # 🎯 优化3：完成度过低(<50%)时，直接触发aggressive recovery
        if completion < 0.50 and self._exploration_strategy.no_goal_count >= 3:
            self._node.get_logger().warn(
                f"[ExplorationHandler] Completion too low ({completion*100:.1f}%), "
                f"performing aggressive recovery: clearing visited frontiers and reducing safety"
            )
            # 重置状态
            self._exploration_strategy.evaluator.visited_frontiers.clear()
            self._exploration_strategy.no_goal_count = 0
            self._safety_manager.safe_distance = 0.25  # 最激进的安全距离
        
        # 尝试脱困旋转获取更多视野
        self._node.get_logger().warn(
            f"[ExplorationHandler] No frontiers found (attempt {self._exploration_strategy.no_goal_count}), "
            f"triggering escape rotation"
        )
        
        # 🔧 关键优化：在尝试旋转前先检查障碍物距离
        nearest_obstacle = self._rotation_controller.get_nearest_obstacle_distance()
        self._node.get_logger().debug(f"[DEBUG] Nearest obstacle distance: {nearest_obstacle:.2f}m")
        
        # 🔧 改进的脱困策略：
        # 如果障碍物<0.3m：先执行backward escape腾出空间，然后继续escape rotation
        # 如果障碍物>=0.3m：直接执行escape rotation
        
        if nearest_obstacle < 0.3:
            self._node.get_logger().warn(
                f"[ExplorationHandler] Obstacle too close ({nearest_obstacle:.2f}m < 0.3m), "
                f"performing backward escape before rotation"
            )
            self._perform_backward_escape()
            # 不return，继续执行下面的escape rotation
        
        # 🔧 关键修复：如果连续旋转失败多次（正反向都试过），执行后退脱困
        # 由于escape rotation现在是正反交替(+45,-45,+90,-90...)，4次失败意味着±45°和±90°都试过了
        if self._rotation_controller.consecutive_rotation_failures >= 4:
            self._node.get_logger().warn(
                f"[ExplorationHandler] Multiple rotation failures ({self._rotation_controller.consecutive_rotation_failures}), "
                f"both directions blocked, performing backward escape"
            )
            self._perform_backward_escape()
            self._rotation_controller.consecutive_rotation_failures = 0
            self._rotation_controller.escape_attempt = 0  # 重置escape尝试计数
            # 不return，继续执行下面的escape rotation
        
        # 🎯 关键优化：如果escape rotation尝试次数达到上限，触发完成检测
        # 可能是所有方向都被阻挡，或者确实探索完成了
        if self._rotation_controller.escape_attempt >= self._rotation_controller.max_escape_attempts:
            self._node.get_logger().warn(
                f"[ExplorationHandler] Escape rotation attempts exhausted "
                f"({self._rotation_controller.escape_attempt}/{self._rotation_controller.max_escape_attempts}), "
                f"triggering completion verification"
            )
            # 重置计数器
            self._rotation_controller.escape_attempt = 0
            self._rotation_controller.consecutive_rotation_failures = 0
            # 触发完成检测（会尝试最后验证）
            if self._check_exploration_completion(task):
                return True  # 探索完成
            else:
                # 完成检测返回False，继续探索
                return False
        
        success = self._rotation_controller.start_escape_rotation(
            self._get_robot_yaw_odom,
            None  # 无pending_goal
        )
        
        if success:
            # 开始脱困旋转，进入旋转状态
            self._exploration_state = ExplorationHandlerState.ROTATING_AT_GOAL
            return False
        else:
            # 旋转失败，等待下次尝试
            self._task_manager.update_progress(task.task_id, completion)
            time.sleep(2.0)  # 等待地图更新
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
            return False
    
    def _attempt_final_verification(self, task: Task) -> bool:
        """
        尝试最后验证：寻找远距离未探索区域
        Attempt final verification: search for distant unexplored frontiers
        
        🐛 方案B: 每次选择不同的最远 frontier，避免重复尝试
        """
        
        self._completion_verification_attempts += 1
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Final verification attempt "
            f"[{self._completion_verification_attempts}/{self._max_verification_attempts}]: "
            f"searching for distant unexplored frontiers"
        )
        
        # 寻找所有边界
        self._frontier_detector.update_map(self._current_map)
        raw_frontiers = self._frontier_detector.find_frontiers()
        
        if not raw_frontiers or len(raw_frontiers) == 0:
            self._node.get_logger().info(
                f"No frontiers found in verification attempt {self._completion_verification_attempts}"
            )
            
            if self._completion_verification_attempts >= self._max_verification_attempts:
                self._node.get_logger().info("No explorable frontiers found after 3 attempts. Completed!")
                self._exploration_state = ExplorationHandlerState.COMPLETED
                return True
            
            return True  # 继续验证
        
        # 转换边界信息并计算距离
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            return True
        
        frontier_infos = []
        for raw_frontier in raw_frontiers:
            frontier_info = self._frontier_detector.calculate_frontier_info(raw_frontier)
            if not frontier_info or 'center_world' not in frontier_info:
                continue
            
            center_world = frontier_info['center_world']
            
            # 🎯 重构：使用 GoalBoundaryValidator 统一边界检查
            is_valid, reason, goal_coords = self._boundary_validator.validate_goal(
                goal_world=center_world,
                current_map=self._current_map,
                logger=self._node.get_logger()
            )
            
            if not is_valid:
                self._node.get_logger().debug(
                    f"[Verification] Skipping frontier at ({center_world[0]:.2f}, {center_world[1]:.2f}) - {reason}"
                )
                continue
            
            distance = math.sqrt(
                (center_world[0] - robot_pos[0])**2 +
                (center_world[1] - robot_pos[1])**2
            )
            
            # 🐛 方案B: 计算frontier的grid key用于判重
            grid_key = self._get_frontier_key(center_world)
            
            frontier_infos.append({
                'position': center_world,
                'size': frontier_info.get('size', len(raw_frontier)),
                'distance': distance,
                'frontier': frontier_info,
                'grid_key': grid_key
            })
        
        if not frontier_infos:
            self._node.get_logger().warn(
                f"[ExplorationHandler] No valid frontiers found after boundary filtering. "
                f"All detected frontiers are outside map or too close to boundary."
            )
            if self._completion_verification_attempts >= self._max_verification_attempts:
                self._exploration_state = ExplorationHandlerState.COMPLETED
                return True
            return True
        
        # 🐛 方案B: 初始化已尝试frontier集合（如果不存在）
        if not hasattr(self, '_verified_frontiers'):
            self._verified_frontiers = set()
        
        # 🐛 方案B: 过滤掉已经尝试过的frontier
        available_frontiers = [
            f for f in frontier_infos 
            if f['grid_key'] not in self._verified_frontiers
        ]
        
        if not available_frontiers:
            self._node.get_logger().warn(
                f"[ExplorationHandler] All {len(frontier_infos)} frontiers have been tried. "
                f"No new frontiers to verify."
            )
            if self._completion_verification_attempts >= self._max_verification_attempts:
                self._exploration_state = ExplorationHandlerState.COMPLETED
                return True
            return True
        
        # 🎯 优化：计算每个frontier周围的未知区域密度，选择未知区域最多的点
        # 而不是单纯选择最远的点
        # 🔧 渐进式阈值：第1次35%，第2次25%，第3次15%
        thresholds = [0.35, 0.25, 0.15]
        min_unknown_ratio = thresholds[min(self._completion_verification_attempts - 1, len(thresholds) - 1)]
        
        high_unknown_frontiers = []
        for frontier in available_frontiers:
            unknown_ratio = self._calculate_unknown_area_ratio(frontier['position'])
            frontier['unknown_ratio'] = unknown_ratio
            
            self._node.get_logger().debug(
                f"[Verification] Frontier at ({frontier['position'][0]:.2f}, {frontier['position'][1]:.2f}): "
                f"distance={frontier['distance']:.2f}m, unknown_ratio={unknown_ratio:.2%}"
            )
            
            # 🎯 关键过滤：只保留未知区域 >= 阈值 的点
            if unknown_ratio >= min_unknown_ratio:
                high_unknown_frontiers.append(frontier)
            else:
                self._node.get_logger().debug(
                    f"[Verification] Skipping low-unknown frontier ({unknown_ratio:.1%} < {min_unknown_ratio:.1%})"
                )
        
        # 检查是否有高未知区域的 frontier
        if not high_unknown_frontiers:
            self._node.get_logger().warn(
                f"[ExplorationHandler] No frontiers with >{min_unknown_ratio:.0%} unknown area found. "
                f"All {len(available_frontiers)} candidates have low unknown ratios (likely map edges)."
            )
            
            # 🔧 关键修复：当没有合格frontier时，检查是否应该完成探索
            if self._completion_verification_attempts >= self._max_verification_attempts:
                # 达到最大尝试次数，触发完成
                self._node.get_logger().info(
                    f"[ExplorationHandler] Final verification exhausted all attempts with no valid targets. "
                    f"Completion: {self._calculate_map_completion()*100:.1f}%. Completing exploration..."
                )
                if not self._exploration_completed:
                    self._exploration_completed = True
                    self._complete_exploration(task)
                    self._exploration_state = ExplorationHandlerState.COMPLETED
                return True
            
            # 未达到最大次数，继续尝试（会使用更低的阈值）
            return True
        
        # 🎯 综合评分：未知区域密度（70%权重） + 距离（30%权重）
        # 鼓励选择靠近大片未知区域的点，而非已知区域的远点
        max_distance = max(f['distance'] for f in high_unknown_frontiers)
        for frontier in high_unknown_frontiers:
            unknown_score = frontier['unknown_ratio'] * 0.7
            distance_score = (frontier['distance'] / max_distance) * 0.3
            frontier['final_score'] = unknown_score + distance_score
        
        # 按综合评分降序排序
        high_unknown_frontiers.sort(key=lambda x: x['final_score'], reverse=True)
        target_frontier = high_unknown_frontiers[0]
        
        # 🐛 方案B: 记录这个frontier已经被尝试
        self._verified_frontiers.add(target_frontier['grid_key'])
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Verification target: frontier with {target_frontier['unknown_ratio']:.1%} unknown area "
            f"(score={target_frontier['final_score']:.3f}, distance={target_frontier['distance']:.2f}m, size={target_frontier['size']} cells) "
            f"(tried {len(self._verified_frontiers)}/{len(frontier_infos)} frontiers)"
        )
        
        # 🎯 新增：计算安全导航点（避免边缘点导致worldToMap错误）
        # 导航到该边界（使用两步导航）
        # 🎯 重构：使用统一的两段式导航（先旋转再移动，旋转失败则直达）
        original_pos = target_frontier['position']
        self._current_frontier = target_frontier['frontier']
        
        # 🔧 关键优化：找到安全的导航点，避免地图边缘导致坐标越界
        safe_target_pos = self._find_safe_navigation_point(original_pos, robot_pos)
        if safe_target_pos is None:
            self._node.get_logger().warn(
                f"[ExplorationHandler] Cannot find safe navigation point for frontier at "
                f"({original_pos[0]:.2f}, {original_pos[1]:.2f}), skipping this verification..."
            )
            # 继续下一次验证
            return True
        
        target_pos = safe_target_pos
        
        # 重新计算距离（使用安全点）
        actual_distance = math.sqrt(
            (target_pos[0] - robot_pos[0])**2 +
            (target_pos[1] - robot_pos[1])**2
        )
        
        # 构造 frontier_eval（用于两段式导航）
        frontier_eval = {
            'frontier_info': target_frontier['frontier'],
            'distance': actual_distance,
            'direction': math.atan2(
                target_pos[1] - robot_pos[1],
                target_pos[0] - robot_pos[0]
            )
        }
        
        self._node.get_logger().info(
            f"[Verification] Using safe navigation point at "
            f"({target_pos[0]:.2f}, {target_pos[1]:.2f}), distance={actual_distance:.2f}m"
        )
        
        # 🎯 重置导航完成标志，准备开始新的验证导航
        self._verification_nav_completed = False
        
        # 🎯 使用 _send_navigation_goal 统一处理两段式导航
        # 这会自动处理：1) 角度差>10°先旋转 2) 旋转失败直达 3) 边界检查
        self._send_navigation_goal(target_pos, frontier_eval, task)
        
        return True  # 继续验证流程
    
    def _should_complete_exploration_proactive(self, best_frontier: dict, 
                                                frontier_eval: dict) -> tuple:
        """
        主动判断是否应该完成探索（多维度综合判断 - 方案C）
        Proactively determine if exploration should complete (multi-dimensional - Plan C)
        
        在选定frontier但未发送导航目标前调用，综合多个维度判断：
        1. 完成度必须 >= 80%（硬性要求）
        2. 至少满足以下2个软性条件：
           - 完成度增长停滞（3分钟 < 2%）
           - Frontier质量衰减
           - 当前frontier数量很少（< 3个）
           - 连续遇到小frontier
        
        Args:
            best_frontier: 选定的最佳frontier
            frontier_eval: frontier评估结果
        
        Returns:
            (should_complete: bool, reason: str)
        """
        completion = self._calculate_map_completion()
        
        # 🔒 硬性要求：完成度必须 >= 80%
        if completion < 0.80:
            return False, f"Completion too low ({completion*100:.1f}% < 80%)"
        
        # 记录frontier评分历史
        frontier_score = frontier_eval.get('score', 0) if frontier_eval else 0
        self._frontier_score_history.append(frontier_score)
        if len(self._frontier_score_history) > 10:
            self._frontier_score_history.pop(0)  # 只保留最近10个
        
        # 📊 维度1: 完成度增长停滞检测
        stagnation = self._check_completion_stagnation(completion)
        
        # 📊 维度2: Frontier质量衰减
        quality_decline = self._check_frontier_quality_decline()
        
        # 📊 维度3: 当前frontier数量很少
        current_frontiers_count = len(self._last_detected_frontiers)
        few_frontiers = current_frontiers_count < 3
        
        # 📊 维度4: 连续遇到小frontier（得分 < 15）
        recent_scores = self._frontier_score_history[-3:] if len(self._frontier_score_history) >= 3 else []
        small_frontiers_count = sum(1 for score in recent_scores if score < 15)
        many_small = small_frontiers_count >= 2
        
        # 🎯 综合判断：至少满足2个软性条件
        conditions = {
            'stagnation': stagnation,
            'quality_decline': quality_decline,
            'few_frontiers': few_frontiers,
            'many_small': many_small
        }
        
        conditions_met = sum(conditions.values())
        
        if conditions_met >= 2:
            # 构造原因说明
            met_conditions = [name for name, met in conditions.items() if met]
            reason = (
                f"Completion {completion*100:.1f}%, "
                f"met {conditions_met}/4 criteria: {', '.join(met_conditions)} "
                f"(frontiers: {current_frontiers_count}, score: {frontier_score:.1f})"
            )
            return True, reason
        
        # 记录未满足条件的情况（调试用）
        self._node.get_logger().debug(
            f"[Proactive Check] Completion {completion*100:.1f}%, "
            f"criteria met: {conditions_met}/4 (need 2+) - "
            f"stagnation={stagnation}, quality={quality_decline}, "
            f"few={few_frontiers}, small={many_small}"
        )
        
        return False, "Criteria not met"
    
    def _check_completion_stagnation(self, current_completion: float) -> bool:
        """
        检查完成度是否停滞（3分钟增长 < 2%）
        Check if completion has stagnated (3-min growth < 2%)
        """
        current_time = time.time()
        
        # 记录历史
        self._completion_history.append((current_time, current_completion))
        
        # 只保留最近10分钟的历史
        cutoff_time = current_time - 600
        self._completion_history = [
            (t, c) for t, c in self._completion_history if t >= cutoff_time
        ]
        
        # 需要至少3分钟的数据才能判断
        if not self._completion_history:
            return False
        
        oldest_time = self._completion_history[0][0]
        if current_time - oldest_time < self._stagnation_threshold_seconds:
            return False
        
        # 计算最近3分钟的完成度增长
        three_min_ago = current_time - self._stagnation_threshold_seconds
        recent_entries = [(t, c) for t, c in self._completion_history if t >= three_min_ago]
        
        if len(recent_entries) < 2:
            return False
        
        completion_3min_ago = recent_entries[0][1]
        completion_now = recent_entries[-1][1]
        progress = completion_now - completion_3min_ago
        
        # 判断：增长 < 2%
        is_stagnant = progress < self._min_progress_rate
        
        if is_stagnant:
            self._node.get_logger().debug(
                f"[Stagnation Detected] 3-min progress: {progress*100:.2f}% "
                f"({completion_3min_ago*100:.1f}% → {completion_now*100:.1f}%)"
            )
        
        return is_stagnant
    
    def _check_frontier_quality_decline(self) -> bool:
        """
        检查frontier质量是否持续衰减
        Check if frontier quality is declining consistently
        """
        # 需要至少5个历史记录
        if len(self._frontier_score_history) < 5:
            return False
        
        recent_scores = self._frontier_score_history[-5:]
        avg_recent_score = sum(recent_scores) / len(recent_scores)
        
        # 判断条件：平均得分 < 10（很低）且呈下降趋势
        if avg_recent_score >= 10:
            return False
        
        # 检查是否下降趋势（后半段比前半段低20%+）
        first_half_avg = sum(recent_scores[:2]) / 2
        second_half_avg = sum(recent_scores[-2:]) / 2
        
        is_declining = second_half_avg < first_half_avg * 0.8
        
        if is_declining:
            self._node.get_logger().debug(
                f"[Quality Decline Detected] Avg score: {avg_recent_score:.1f}, "
                f"trend: {first_half_avg:.1f} → {second_half_avg:.1f}"
            )
        
        return is_declining
    
    def _get_frontier_key(self, pos: Tuple[float, float]) -> Tuple[int, int]:
        """
        生成frontier的唯一标识（网格化坐标）
        Generate unique key for frontier (gridded coordinates)
        
        将坐标量化到0.5m网格，相近位置视为同一frontier
        Quantize coordinates to 0.5m grid, nearby positions treated as same frontier
        
        Args:
            pos: (x, y) 世界坐标 / World coordinates
            
        Returns:
            (grid_x, grid_y) 网格坐标 / Grid coordinates
        """
        grid_x = round(pos[0] / self._failed_frontier_radius)
        grid_y = round(pos[1] / self._failed_frontier_radius)
        return (grid_x, grid_y)
    
    def _record_failed_frontier(self, frontier_pos: Tuple[float, float]):
        """
        记录导航失败的frontier（方案2：多次失败才加入黑名单）
        Record navigation failed frontier (Plan 2: blacklist after multiple failures)
        
        Args:
            frontier_pos: frontier中心世界坐标 / Frontier center world coordinates
        """
        pos_key = self._get_frontier_key(frontier_pos)
        
        if pos_key in self._frontier_failure_history:
            fail_count, _ = self._frontier_failure_history[pos_key]
            self._frontier_failure_history[pos_key] = (fail_count + 1, time.time())
            
            if fail_count + 1 >= self._max_frontier_failures:
                self._node.get_logger().warn(
                    f"🚫 [Frontier Blacklist] Frontier at {frontier_pos} failed {fail_count + 1} times, "
                    f"adding to blacklist"
                )
            else:
                self._node.get_logger().info(
                    f"⚠️ [Frontier Retry] Frontier at {frontier_pos} failed {fail_count + 1}/{self._max_frontier_failures} times, "
                    f"will retry {self._max_frontier_failures - fail_count - 1} more time(s)"
                )
        else:
            self._frontier_failure_history[pos_key] = (1, time.time())
            self._node.get_logger().info(
                f"⚠️ [Frontier First Failure] Frontier at {frontier_pos} failed for the first time, "
                f"will retry {self._max_frontier_failures - 1} more time(s)"
            )
        
        # 清理过期记录（超过3分钟的记录）
        current_time = time.time()
        expired_keys = [
            key for key, (_, timestamp) in self._frontier_failure_history.items()
            if current_time - timestamp > self._failed_frontier_timeout
        ]
        
        for key in expired_keys:
            del self._frontier_failure_history[key]
            self._node.get_logger().debug(
                f"[Frontier Blacklist] Removed expired failure record for grid {key}"
            )
    
    def _should_skip_frontier(self, frontier_pos: Tuple[float, float]) -> bool:
        """
        检查是否应该跳过该frontier（失败次数过多）
        Check if frontier should be skipped (too many failures)
        
        Args:
            frontier_pos: frontier中心世界坐标 / Frontier center world coordinates
            
        Returns:
            True if should skip (blacklisted), False otherwise
        """
        pos_key = self._get_frontier_key(frontier_pos)
        
        if pos_key in self._frontier_failure_history:
            fail_count, timestamp = self._frontier_failure_history[pos_key]
            
            # 检查是否过期
            if time.time() - timestamp > self._failed_frontier_timeout:
                # 过期，删除记录
                del self._frontier_failure_history[pos_key]
                return False
            
            # 达到失败上限，跳过
            if fail_count >= self._max_frontier_failures:
                return True
        
        return False
    
    def _find_safe_navigation_point(self, frontier_pos: Tuple[float, float], 
                                     robot_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        为边缘frontier找到安全的导航点
        Find a safe navigation point for edge frontiers
        
        策略：
        1. 如果frontier在地图边缘，向地图中心方向偏移
        2. 确保目标点周围有足够的自由空间（支持180度旋转）
        3. 检查目标点在地图边界内
        
        Args:
            frontier_pos: frontier中心点世界坐标
            robot_pos: 机器人当前位置世界坐标
            
        Returns:
            安全导航点世界坐标，如果找不到返回None
        """
        if self._current_map is None:
            return frontier_pos
        
        import numpy as np
        from ....exploration.exploration_utils import CoordinateConverter
        
        # 转换frontier到地图坐标
        frontier_coords = CoordinateConverter.world_to_map(
            frontier_pos[0], frontier_pos[1], self._current_map
        )
        
        if frontier_coords is None:
            # frontier不在地图内，计算向机器人方向的安全点
            self._node.get_logger().warn(
                f"[SafeNav] Frontier ({frontier_pos[0]:.2f}, {frontier_pos[1]:.2f}) outside map, "
                f"computing safe point towards robot"
            )
            # 向机器人方向偏移1.0米
            direction_x = robot_pos[0] - frontier_pos[0]
            direction_y = robot_pos[1] - frontier_pos[1]
            distance = math.sqrt(direction_x**2 + direction_y**2)
            if distance > 0:
                safe_x = frontier_pos[0] + (direction_x / distance) * 1.0
                safe_y = frontier_pos[1] + (direction_y / distance) * 1.0
                return (safe_x, safe_y)
            return frontier_pos
        
        fx, fy = frontier_coords
        width = self._current_map.info.width
        height = self._current_map.info.height
        
        # 检查是否靠近边界（离边界<10个格子）
        margin = 10
        is_near_edge = (fx < margin or fx >= width - margin or 
                       fy < margin or fy >= height - margin)
        
        if not is_near_edge:
            # 不在边缘，检查周围自由空间
            map_array = np.array(self._current_map.data).reshape((height, width))
            rotation_radius = int(0.8 / self._current_map.info.resolution)  # 0.8米旋转半径
            
            # 提取旋转区域
            min_x = max(0, fx - rotation_radius)
            max_x = min(width, fx + rotation_radius + 1)
            min_y = max(0, fy - rotation_radius)
            max_y = min(height, fy + rotation_radius + 1)
            
            region = map_array[min_y:max_y, min_x:max_x]
            free_ratio = np.sum((region >= 0) & (region < 50)) / region.size if region.size > 0 else 0
            
            if free_ratio > 0.7:  # 70%以上是自由空间
                self._node.get_logger().debug(
                    f"[SafeNav] Frontier has sufficient free space ({free_ratio:.1%}), using original position"
                )
                return frontier_pos
        
        # 需要找安全点：向地图中心方向偏移
        center_x = width // 2
        center_y = height // 2
        
        # 计算向中心的方向
        to_center_x = center_x - fx
        to_center_y = center_y - fy
        distance = math.sqrt(to_center_x**2 + to_center_y**2)
        
        if distance < 1:
            # 已经在中心附近
            return frontier_pos
        
        # 向中心偏移1.0米（约20个格子，假设分辨率0.05m）
        offset_cells = int(1.0 / self._current_map.info.resolution)
        safe_fx = fx + int((to_center_x / distance) * offset_cells)
        safe_fy = fy + int((to_center_y / distance) * offset_cells)
        
        # 确保在地图内
        safe_fx = max(margin, min(width - margin - 1, safe_fx))
        safe_fy = max(margin, min(height - margin - 1, safe_fy))
        
        # 转换回世界坐标
        safe_world = CoordinateConverter.map_to_world(safe_fx, safe_fy, self._current_map)
        
        if safe_world:
            self._node.get_logger().info(
                f"[SafeNav] Adjusted frontier position: "
                f"({frontier_pos[0]:.2f}, {frontier_pos[1]:.2f}) → ({safe_world[0]:.2f}, {safe_world[1]:.2f}), "
                f"offset={math.sqrt((safe_world[0]-frontier_pos[0])**2 + (safe_world[1]-frontier_pos[1])**2):.2f}m"
            )
            return safe_world
        
        return frontier_pos
    
    def _find_safe_initial_scan_point(self) -> Optional[Tuple[float, float]]:
        """
        在机器人前方已知区域中寻找安全的初始扫描点
        Find a safe initial scan point in the forward known area
        
        策略 / Strategy:
        1. 获取机器人当前位置和朝向
        2. 在前方1.5-2.5米范围内搜索
        3. 选择已知区域最大、最安全的点
        4. 该点周围0.8米内应有足够的自由空间（用于旋转）
        
        Returns:
            安全点的世界坐标 (x, y)，如果找不到则返回 None
        """
        if self._current_map is None:
            self._node.get_logger().warn("[InitialScan] No map available for finding safe scan point")
            return None
        
        # 获取机器人位置和朝向
        robot_pose = self._nav_executor.get_robot_pose()
        robot_yaw = self._nav_executor.get_robot_yaw()
        
        if robot_pose is None or robot_yaw is None:
            self._node.get_logger().warn("[InitialScan] Cannot get robot pose/yaw")
            return None
        
        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y
        
        import numpy as np
        from ....exploration.exploration_utils import CoordinateConverter
        
        width = self._current_map.info.width
        height = self._current_map.info.height
        resolution = self._current_map.info.resolution
        map_array = np.array(self._current_map.data).reshape((height, width))
        
        # 搜索参数
        min_distance = 1.5  # 最小距离（米）
        max_distance = 2.5  # 最大距离（米）
        angle_range = 60.0  # 前方视野角度范围（±30度）
        search_step = 0.3   # 搜索步长（米）
        rotation_radius = 0.8  # 旋转所需半径（米）
        
        best_point = None
        best_score = -1.0
        
        # 在前方扇形区域内搜索
        for distance in np.arange(min_distance, max_distance + search_step, search_step):
            for angle_offset in np.linspace(-angle_range/2, angle_range/2, 11):  # 检查11个方向
                angle_rad = math.radians(angle_offset)
                search_yaw = robot_yaw + angle_rad
                
                # 计算候选点世界坐标
                candidate_x = robot_x + distance * math.cos(search_yaw)
                candidate_y = robot_y + distance * math.sin(search_yaw)
                
                # 转换为地图坐标
                coords = CoordinateConverter.world_to_map(candidate_x, candidate_y, self._current_map)
                if coords is None:
                    continue
                
                cx, cy = coords
                
                # 边界检查
                if not (0 <= cx < width and 0 <= cy < height):
                    continue
                
                # 检查该点是否为自由空间
                if map_array[cy, cx] >= 50:  # 障碍物或未知区域
                    continue
                
                # 检查旋转半径内的空间
                rotation_cells = int(rotation_radius / resolution)
                min_x = max(0, cx - rotation_cells)
                max_x = min(width, cx + rotation_cells + 1)
                min_y = max(0, cy - rotation_cells)
                max_y = min(height, cy + rotation_cells + 1)
                
                region = map_array[min_y:max_y, min_x:max_x]
                
                if region.size == 0:
                    continue
                
                # 计算自由空间占比
                free_cells = np.sum((region >= 0) & (region < 50))
                free_ratio = free_cells / region.size
                
                # 计算已知区域占比（自由+障碍）
                known_cells = np.sum(region >= 0)
                known_ratio = known_cells / region.size
                
                # 评分：优先选择自由空间多且已知区域大的点
                # 同时考虑距离（稍微偏好较远的点，视野更好）
                if free_ratio < 0.7:  # 自由空间不足70%，跳过
                    continue
                
                if known_ratio < 0.8:  # 已知区域不足80%，跳过
                    continue
                
                # 综合评分
                distance_score = (distance - min_distance) / (max_distance - min_distance)  # 0-1
                score = free_ratio * 0.5 + known_ratio * 0.3 + distance_score * 0.2
                
                if score > best_score:
                    best_score = score
                    best_point = (candidate_x, candidate_y)
        
        if best_point:
            self._node.get_logger().info(
                f"[InitialScan] Found safe scan point at ({best_point[0]:.2f}, {best_point[1]:.2f}), "
                f"distance from robot: {math.sqrt((best_point[0]-robot_x)**2 + (best_point[1]-robot_y)**2):.2f}m, "
                f"score: {best_score:.3f}"
            )
        else:
            self._node.get_logger().warn(
                "[InitialScan] No suitable safe scan point found in forward area"
            )
        
        return best_point
    
    def _calculate_unknown_area_ratio(self, world_pos: Tuple[float, float], radius: float = 2.0) -> float:
        """
        计算指定世界坐标周围的未知区域占比
        Calculate the ratio of unknown area around a world position
        
        Args:
            world_pos: 世界坐标 (x, y) / World coordinates
            radius: 检测半径（米）/ Detection radius in meters
            
        Returns:
            未知区域占比 [0, 1] / Unknown area ratio
        """
        if self._current_map is None:
            return 0.0
        
        import numpy as np
        from ....exploration.exploration_utils import CoordinateConverter
        
        # 转换为地图坐标
        coords = CoordinateConverter.world_to_map(
            world_pos[0], world_pos[1],
            self._current_map
        )
        
        if coords is None:
            return 0.0
        
        map_x, map_y = coords
        
        width = self._current_map.info.width
        height = self._current_map.info.height
        
        # 边界检查
        if not (0 <= map_x < width and 0 <= map_y < height):
            return 0.0
        
        # 计算检测范围（地图坐标）
        radius_cells = int(radius / self._current_map.info.resolution)
        
        map_array = np.array(self._current_map.data).reshape((height, width))
        
        # 提取检测区域
        min_x = max(0, map_x - radius_cells)
        max_x = min(width, map_x + radius_cells + 1)
        min_y = max(0, map_y - radius_cells)
        max_y = min(height, map_y + radius_cells + 1)
        
        region = map_array[min_y:max_y, min_x:max_x]
        
        if region.size == 0:
            return 0.0
        
        # 计算未知区域占比（-1表示未知）
        unknown_cells = np.sum(region == -1)
        total_cells = region.size
        
        return unknown_cells / total_cells if total_cells > 0 else 0.0
    
    def _calculate_map_completion(self) -> float:
        """
        计算地图完成度（优化版：只计算有效探索区域）
        Calculate map completion (optimized: only count valid exploration area)
        """
        if self._current_map is None:
            return 0.0
        
        import numpy as np
        
        # 获取地图尺寸
        width = self._current_map.info.width
        height = self._current_map.info.height
        map_array = np.array(self._current_map.data).reshape((height, width))
        
        # 🎯 优化：去除10%边界作为无效区域（通常是地图外围的padding）
        # Optimization: Remove 10% border as invalid area (usually outer padding)
        border_x = max(1, int(width * 0.1))
        border_y = max(1, int(height * 0.1))
        
        # 提取有效探索区域（中心80%×80%）
        valid_area = map_array[border_y:-border_y, border_x:-border_x]
        
        # 计算有效区域内的已知格子 (0-100) vs 未知格子 (-1)
        total_cells = valid_area.size
        known_cells = np.sum((valid_area >= 0) & (valid_area <= 100))
        
        if total_cells > 0:
            completion = known_cells / total_cells
            return min(completion, 1.0)
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
        """保存地图（使用MapLibraryManager，支持版本管理）"""
        try:
            self._node.get_logger().info(
                f"[ExplorationHandler] Saving map '{map_name}' with version management..."
            )
            
            # 🎯 关键：等待 RTABMap 数据库写入完成
            # RTABMap 需要时间将数据写入磁盘，特别是在探索刚完成时
            import time
            self._node.get_logger().info("⏳ Waiting 3 seconds for RTABMap database sync...")
            time.sleep(3.0)
            
            # 使用 MapLibraryManager 保存地图
            # 自动处理版本号递增、元数据管理、可视化生成
            success, message = self._map_library_manager.save_map(
                map_name=map_name,
                map_topic='/map',
                description=f"Exploration completed at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
                tags=['exploration', 'auto_saved']
            )
            
            if success:
                self._node.get_logger().info(
                    f"✅ [ExplorationHandler] {message}"
                )
            else:
                self._node.get_logger().error(
                    f"❌ [ExplorationHandler] Map save failed: {message}"
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
        # 🔴 防止重复调用导致死循环
        if self._exploration_state == ExplorationHandlerState.COMPLETED:
            return  # 已经处理过，直接返回
        
        nav_state = self._nav_executor.get_state()
        
        if nav_state not in [NavigationState.IDLE, NavigationState.CANCELED]:
            self._nav_executor.cancel_navigation()
        
        # 🔴 更新任务状态为 CANCELED（修复拼写）
        self._task_manager.update_task_state(task.task_id, TaskState.CANCELED)
        self.release_executor(task.task_id)
        
        # 🎯 关键修复：设置停止标志，让主循环退出
        self._should_stop = True
        self._exploration_state = ExplorationHandlerState.COMPLETED
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
    
    # ========== 🎯 阶段1: 依赖类初始化方法 ==========
    
    def _initialize_rotation_controller(self):
        """初始化旋转控制器 / Initialize rotation controller"""
        from ....utils.rotation_controller import RotationController
        
        self._rotation_controller = RotationController(self._node, '/cmd_vel')
        
        # 配置旋转参数 / Configure rotation parameters
        rotation_config = {
            'rotation_speed': 0.5,
            'rotation_timeout': 30.0,
            'rotation_tolerance_deg': 5,
            'check_obstacle_during_rotation': True,
            'max_escape_attempts': 7,
            'initial_scan_angle': 100,
        }
        self._rotation_controller.configure(rotation_config)
        self._node.get_logger().info("[ExplorationHandler] RotationController initialized")
    
    def _initialize_safety_manager(self):
        """初始化安全管理器 / Initialize safety manager"""
        from ....utils.safety_manager import SafetyManager
        
        self._safety_manager = SafetyManager()
        
        # 配置安全参数 / Configure safety parameters
        safety_config = {
            'safe_distance': 0.4,
            'min_goal_distance': 0.2,
            'goal_tolerance': 0.3,
            'obstacle_threshold': 50,
            'max_failures': 8,
            'max_safety_violations': 3,
        }
        self._safety_manager.configure(safety_config)
        self._node.get_logger().info("[ExplorationHandler] SafetyManager initialized")
    
    def _initialize_frontier_evaluator(self):
        """初始化边界评估器 / Initialize frontier evaluator"""
        from ....exploration.exploration_strategy import FrontierEvaluator
        
        self._frontier_evaluator = FrontierEvaluator()
        self._node.get_logger().info("[ExplorationHandler] FrontierEvaluator initialized")
    
    def _perform_initial_scan(self):
        """执行初始360°扫描 / Perform initial 360° scan"""
        self._node.get_logger().info("[ExplorationHandler] Starting initial 360° scan...")

    # ========== 🎯 阶段2: 旋转避障和 pending_goal 机制 ==========
    
    def _monitor_rotation(self, task: Task):
        """监控旋转进度 / Monitor rotation progress"""
        # 检查旋转是否完成
        is_rotating = self._rotation_controller.is_rotating_state
        rotation_completed = self._rotation_controller.rotation_completed_successfully
        
        if not is_rotating:
            if rotation_completed:
                self._node.get_logger().info("[ExplorationHandler] Rotation completed successfully")
                
                # 如果是初始扫描，继续探索
                if not self._initial_scan_done:
                    self._initial_scan_done = True
                    self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
                else:
                    # 旋转到达目标后，如果有 pending_goal，发送它
                    if self._pending_goal is not None:
                        # 🎯 新增：发送pending goal前检查完成度
                        should_complete, reason = self._should_complete_exploration_proactive()
                        if should_complete:
                            self._node.get_logger().info(
                                f"✅ [Proactive Completion Before Pending Goal] {reason}"
                            )
                            self._pending_goal = None  # 清空pending goal
                            if self._check_exploration_completion(task):
                                return
                        
                        # 完成度未达标，发送pending goal
                        if self._pending_goal is not None:  # 再次检查（可能被清空）
                            self._send_pending_goal(task)
                    else:
                        self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
            else:
                # 旋转失败或超时
                self._node.get_logger().warn("[ExplorationHandler] Rotation failed or timed out")
                
                # 🎯 关键优化：两步导航的预转向失败时，不触发脱困逻辑
                # 直接跳过转向，让Nav2自己处理朝向问题（增加成功率）
                # 只有在escape rotation（无frontiers脱困）时才检查连续失败
                
                # 如果有 pending_goal，说明是预转向失败，直接跳过转向发送目标
                if self._pending_goal is not None:
                    self._node.get_logger().info(
                        "[ExplorationHandler] Pre-rotation failed due to obstacle, "
                        "skipping rotation and navigating directly (let Nav2 handle orientation)"
                    )
                    self._send_pending_goal(task)
                else:
                    # 无pending_goal，说明是escape rotation或初始扫描失败
                    # 检查连续旋转失败，考虑后退脱困
                    if self._rotation_controller.consecutive_rotation_failures >= \
                       self._rotation_controller.max_consecutive_rotation_failures:
                        self._node.get_logger().warn(
                            f"[ExplorationHandler] Multiple escape rotation failures "
                            f"({self._rotation_controller.consecutive_rotation_failures}/{self._rotation_controller.max_consecutive_rotation_failures}), "
                            f"performing backward escape"
                        )
                        self._perform_backward_escape()
                        self._rotation_controller.consecutive_rotation_failures = 0
                        self._rotation_controller.escape_attempt = 0
                    
                    # 进入边界检测寻找新目标
                    self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
    
    def _perform_backward_escape(self):
        """
        执行后退脱困 / Perform backward escape maneuver
        
        Mapper原逻辑：当连续旋转失败时，后退一小段距离并清空访问记录
        """
        from geometry_msgs.msg import Twist
        import time
        
        self._node.get_logger().info("🔙 [ExplorationHandler] Performing backward escape maneuver...")
        
        # 1. 创建后退速度命令
        cmd = Twist()
        cmd.linear.x = -0.15  # 后退速度 / Backward speed
        cmd.angular.z = 0.0
        
        # 2. 后退1.5秒（约0.225米）
        backward_duration = 1.5
        start_time = time.time()
        
        # 使用简单的循环发布命令
        while (time.time() - start_time) < backward_duration:
            self._rotation_controller.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)  # 20Hz
        
        # 3. 停止
        cmd.linear.x = 0.0
        self._rotation_controller.cmd_vel_pub.publish(cmd)
        
        self._node.get_logger().info("🔙 [ExplorationHandler] Backward escape completed (~0.225m)")
        
        # 4. ❗ 清空访问记录（允许重试之前的frontiers）
        if hasattr(self._exploration_strategy, 'evaluator'):
            self._exploration_strategy.evaluator.visited_frontiers.clear()
            self._node.get_logger().info("[ExplorationHandler] Cleared visited frontiers after backward escape")
        
        # 5. 重置无边界计数器，给机器人一个新的开始
        self._exploration_strategy.no_goal_count = 0
    
    def _send_pending_goal(self, task: Task):
        """发送待处理目标 / Send pending goal"""
        if self._pending_goal is None:
            self._node.get_logger().debug("[ExplorationHandler] No pending goal to send")
            return
        
        # 🛡️ 关键修复：在发送目标前必须重置导航状态
        nav_state = self._nav_executor.get_state()
        if nav_state != NavigationState.IDLE:
            self._node.get_logger().info(
                f"[ExplorationHandler] Resetting navigation state from {nav_state.value} to IDLE before sending goal"
            )
            self._nav_executor.reset_state()
        
        goal_info = self._pending_goal
        target_pos = goal_info['position']
        target_yaw = goal_info['yaw']
        
        # 🎯 重构：使用 GoalBoundaryValidator 统一边界检查
        is_valid, reason, goal_coords = self._boundary_validator.validate_goal(
            goal_world=target_pos,
            current_map=self._current_map if hasattr(self, '_current_map') else None,
            logger=self._node.get_logger()
        )
        
        if not is_valid:
            self._node.get_logger().warn(
                f"⚠️ [ExplorationHandler] Pending goal ({target_pos[0]:.2f}, {target_pos[1]:.2f}) rejected: {reason}"
            )
            self._pending_goal = None
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
            return
        
        # 边界检查已在前面完成，直接发送目标
        self._node.get_logger().info(
            f"[ExplorationHandler] Sending pending goal to ({target_pos[0]:.2f}, {target_pos[1]:.2f})"
        )
        
        # 创建目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(target_pos[0])
        goal_pose.pose.position.y = float(target_pos[1])
        goal_pose.pose.position.z = 0.0
        
        # 设置朝向
        import math
        goal_pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        
        # 发送导航目标
        success = self._nav_executor.navigate_to_pose(goal_pose)
        
        if success:
            self._exploration_state = ExplorationHandlerState.NAVIGATING_TO_FRONTIER
            
            # 🎯 关键修复：记录导航开始时间和目标距离（用于超时检测）
            self._goal_start_time = time.time()
            robot_pos = self._get_robot_position()
            if robot_pos:
                goal_distance = math.sqrt(
                    (target_pos[0] - robot_pos[0])**2 + (target_pos[1] - robot_pos[1])**2
                )
                self._goal_distance = goal_distance
                self._node.get_logger().info(
                    f"[ExplorationHandler] Pending goal sent successfully, "
                    f"distance={goal_distance:.2f}m, timeout={((goal_distance/0.2)+30):.1f}s"
                )
            else:
                self._goal_distance = 5.0
                self._node.get_logger().info("[ExplorationHandler] Pending goal sent successfully")
        else:
            self._node.get_logger().warn("[ExplorationHandler] Failed to send pending goal")
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        # 清理状态
        self._pending_goal = None
        self._rotation_controller.skip_rotation_on_obstacle = False
        self._rotation_controller.pre_rotation_point = None
        self._rotation_controller.pre_rotation_retry_count = 0
    
    def _handle_rotation_obstacle(self, task: Task):
        """
        处理旋转遇到障碍物的情况（5种场景）
        Handle rotation obstacle (5 scenarios)
        """
        
        if self._pending_goal is None:
            self._node.get_logger().debug("[ExplorationHandler] _handle_rotation_obstacle: pending_goal is None")
            return
        
        # 获取导航状态和标志
        nav_state = self._nav_executor.get_state()
        pre_rot_point = self._rotation_controller.pre_rotation_point
        skip_flag = self._rotation_controller.skip_rotation_on_obstacle
        
        self._node.get_logger().info(
            f"[ExplorationHandler] _handle_rotation_obstacle: nav_state={nav_state.value}, "
            f"pre_rotation_point={pre_rot_point is not None}, skip={skip_flag}",
            throttle_duration_sec=1.0
        )
        
        # 场景1：已经到达预移动点，现在要旋转
        if nav_state == NavigationState.SUCCESS and pre_rot_point is not None:
            self._node.get_logger().info("[ExplorationHandler] Reached safe rotation point, now rotating")
            
            self._rotation_controller.pre_rotation_point = None
            
            robot_yaw = self._get_robot_yaw_odom()
            if robot_yaw is not None and self._pending_goal is not None:
                target_yaw = self._pending_goal['yaw']
                angle_diff = self._normalize_angle(target_yaw - robot_yaw)
                
                self._rotation_controller.pre_rotation_retry_count += 1
                self._rotation_controller.after_pre_movement = True
                self._rotation_controller.skip_rotation_on_obstacle = False
                
                if self._rotation_controller.start_rotation(
                    math.degrees(angle_diff),
                    self._get_robot_yaw_odom,
                    self._pending_goal
                ):
                    self._node.get_logger().info(f"Started rotation to target yaw: {math.degrees(target_yaw):.1f}°")
                else:
                    self._node.get_logger().warn("Failed to start rotation, sending goal directly")
                    self._send_pending_goal(task)
            else:
                self._send_pending_goal(task)
            
            self._nav_executor.reset_state()
            return
        
        # 场景2：预移动点导航被取消
        if nav_state == NavigationState.CANCELED:
            self._node.get_logger().warn("[ExplorationHandler] Safe point navigation canceled, using direct navigation")
            self._rotation_controller.pre_rotation_point = None
            self._rotation_controller.skip_rotation_on_obstacle = False
            self._rotation_controller.pre_rotation_retry_count = 0
            self._rotation_controller.after_pre_movement = False
            self._send_pending_goal(task)
            return
        
        # 重置导航状态
        if nav_state != NavigationState.IDLE:
            self._nav_executor.reset_state()
        
        # 场景3：已尝试过预移动策略
        if self._rotation_controller.pre_rotation_retry_count >= 1:
            self._node.get_logger().warn("[ExplorationHandler] Pre-rotation already tried, using direct navigation")
            self._rotation_controller.skip_rotation_on_obstacle = False
            self._send_pending_goal(task)
            return
        
        # 场景4：首次遇到障碍，寻找安全点
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            self._send_pending_goal(task)
            return
        
        target_pos = self._pending_goal['position']
        
        # 寻找安全的预移动点
        safe_point = self._safety_manager.find_safe_rotation_point(
            robot_pos, target_pos, search_radius=1.5
        )
        
        if safe_point is not None:
            dist = math.sqrt((safe_point[0]-robot_pos[0])**2 + (safe_point[1]-robot_pos[1])**2)
            if dist < 0.5:
                self._node.get_logger().warn(f"Safe point too close ({dist:.2f}m), using direct navigation")
                self._send_pending_goal(task)
                return
            
            self._node.get_logger().info(
                f"[ExplorationHandler] Moving to safe point ({safe_point[0]:.2f}, {safe_point[1]:.2f}), distance={dist:.2f}m"
            )
            
            safe_point_yaw = math.atan2(safe_point[1] - robot_pos[1], safe_point[0] - robot_pos[0])
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self._node.get_clock().now().to_msg()
            goal_pose.pose.position.x = safe_point[0]
            goal_pose.pose.position.y = safe_point[1]
            goal_pose.pose.orientation = self._yaw_to_quaternion(safe_point_yaw)
            
            if self._nav_executor.navigate_to_pose(goal_pose):
                self._rotation_controller.pre_rotation_point = safe_point
                # 保持 skip_rotation_on_obstacle = True
            else:
                self._send_pending_goal(task)
        else:
            # 场景5：找不到安全点
            self._node.get_logger().warn("[ExplorationHandler] No safe rotation point found, using direct navigation")
            self._rotation_controller.skip_rotation_on_obstacle = False
            self._send_pending_goal(task)
    
    # ========== 🎯 阶段3: 最终验证和脱困机制 ==========
    # Note: _attempt_final_verification() is already defined above (line ~1141)
    
    def _perform_backward_escape(self):
        """执行后退脱困 / Perform backward escape"""
        self._node.get_logger().info("🔙 [ExplorationHandler] Performing backward escape maneuver...")
        
        from geometry_msgs.msg import Twist
        import time
        
        # 发送后退速度命令
        cmd = Twist()
        cmd.linear.x = -0.15  # 后退速度
        cmd.angular.z = 0.0
        
        # 🔧 Critical Fix: 在ROS2回调中不能使用time.sleep()，会被事件循环阻塞
        # 改为连续发布多次命令，让omni_controller保持执行
        backward_duration = 1.5
        num_publishes = 30  # 1.5秒 × 20Hz = 30次
        
        # 连续发布后退命令
        for i in range(num_publishes):
            self._rotation_controller.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)  # 虽然会被延迟，但至少保证发布
        
        # 额外等待确保执行
        time.sleep(0.5)
        
        # 停止移动
        cmd.linear.x = 0.0
        self._rotation_controller.cmd_vel_pub.publish(cmd)
        
        self._node.get_logger().info("🔙 [ExplorationHandler] Backward escape completed (~0.225m)")
        
        # 🎯 P3: 清空访问记录（允许重试之前的frontiers）/ Clear visited history
        if hasattr(self._exploration_strategy, 'evaluator'):
            self._exploration_strategy.evaluator.visited_frontiers.clear()
            self._node.get_logger().info("[ExplorationHandler] Cleared visited frontiers after backward escape")
        
        # 重置无边界计数器，给机器人一个新的开始
        self._exploration_strategy.no_goal_count = 0
        self._node.get_logger().info("[ExplorationHandler] Reset no_goal_count after backward escape")
    
    def _setup_local_costmap_subscription(self):
        """订阅 local_costmap / Subscribe to local_costmap"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        
        # 配置 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self._local_costmap_sub = self._node.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self._local_costmap_callback,
            qos
        )
        
        self._node.get_logger().info("[ExplorationHandler] Subscribed to /local_costmap/costmap")
    
    def _local_costmap_callback(self, msg: OccupancyGrid):
        """local_costmap 回调 / Local costmap callback"""
        # 同步到 RotationController 用于障碍检测
        if hasattr(self, '_rotation_controller'):
            self._rotation_controller.update_local_costmap(msg)
        
        # 同步到 SafetyManager
        if hasattr(self, '_safety_manager'):
            self._safety_manager.update_local_costmap(msg)
