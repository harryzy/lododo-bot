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
from typing import Optional
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from ..task_execution_handler import TaskExecutionHandler
from ...task_manager import Task, TaskState
from ...navigation_executor import NavigationState
from ....exploration.frontier_detector import FrontierDetector
from ....exploration.exploration_strategy import ExplorationStrategy
from ....map.map_library_manager import MapLibraryManager


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
        
        # 🎯 阶段3: 最终验证和脱困相关变量
        self._completion_scan_in_progress = False  # 完成度验证进行中标志
        self._completion_before_scan = 0.0  # 验证开始时的完成度
        self._completion_verification_attempts = 0  # 完成验证尝试次数
        self._max_verification_attempts = 3  # 最大验证次数
        self._is_in_verification = False  # 是否在验证模式
        self._escape_attempt = 0  # 脱困尝试次数
        self._max_escape_attempts = 3  # 最大脱困次数
        
        # 🎯 P2: 导航超时相关变量
        self._goal_start_time = None  # 导航开始时间
        self._max_goal_time = 120.0  # 导航超时（2分钟）
        
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
        
        self._node.get_logger().info("[ExplorationHandler] Initialized with full algorithm support (Phase 3)")
    
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
        
        # 3. CANCELED 状态 - 取消处理
        if task.state == TaskState.CANCELED:
            self._handle_cancel(task)
            return
        
        # 4. RUNNING 状态 - 执行探索
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
        if not self._rotation_controller.initial_scan_done and self._exploration_state != ExplorationHandlerState.ROTATING_AT_GOAL:
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
        
        elif self._exploration_state == ExplorationHandlerState.DETECTING_FRONTIERS:
            self._process_frontier_detection(task)
        
        elif self._exploration_state == ExplorationHandlerState.NAVIGATING_TO_FRONTIER:
            # 🔧 方案2修复：在导航状态时，只监控导航进度，不重复检测frontier
            # 检查导航器状态
            nav_state = self._nav_executor.get_state()
            
            if nav_state == NavigationState.EXECUTING:
                # 导航正在执行，继续等待
                self._node.get_logger().debug("[ExplorationHandler] Navigation in progress, waiting...")
                return  # 不进入 _monitor_navigation，避免状态混乱
            elif nav_state == NavigationState.SUCCESS:
                # 导航成功，回到检测状态
                self._node.get_logger().info("[ExplorationHandler] Navigation succeeded, returning to frontier detection")
                self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
                self._exploration_strategy.reset_failure_count()
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
            # 探索完成
            self._complete_exploration(task)
    
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
            
            self._node.get_logger().info(f"[DEBUG] Step 1: find_frontiers() returned {len(raw_frontiers) if raw_frontiers else 0} clusters")
            
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
            
            self._node.get_logger().info(f"[DEBUG] Step 2: calculate_frontier_info() kept {len(frontiers)}/{len(raw_frontiers)} clusters")
            
            if not frontiers:
                # 无有效边界
                self._check_exploration_completion(task)
                return
            
            # 过滤已访问的边界
            frontiers_before_filter = len(frontiers)
            frontiers = self._filter_visited_frontiers(frontiers)
            
            self._node.get_logger().info(f"[DEBUG] Step 3: _filter_visited_frontiers() kept {len(frontiers)}/{frontiers_before_filter} clusters")
            
            if not frontiers:
                # 所有边界都已访问
                self._check_exploration_completion(task)
                return
            
            # 选择最优边界
            best_frontier = self._select_best_frontier(frontiers)
            
            self._node.get_logger().info(f"[DEBUG] Step 4: _select_best_frontier() result: {'FOUND' if best_frontier else 'NONE'}")
            
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
            
            # 最终安全检查
            is_safe, safety_reason = self._safety_manager.is_goal_safe(
                frontier_center[0], frontier_center[1], robot_pos
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
                self._node.get_logger().info(
                    f"[DEBUG] Frontier {idx} at ({frontier_center[0]:.2f}, {frontier_center[1]:.2f}) "
                    f"REJECTED by evaluator (None)"
                )
                continue
            
            # 🎯 阶段1: 安全检查
            is_safe, reason = self._safety_manager.is_goal_safe(
                frontier_center[0], frontier_center[1], robot_pos
            )
            
            if not is_safe:
                filtered_by_safety += 1
                self._node.get_logger().info(
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
        self._node.get_logger().info(
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
            
            # 🎯 P2: 记录导航开始时间（用于超时检测）
            self._goal_start_time = time.time()
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
        """导航到边界"""
        # 构造目标位姿 (使用 center_world 键)
        frontier_center = frontier['center_world']  # (x, y) tuple
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(frontier_center[0])
        goal_pose.pose.position.y = float(frontier_center[1])
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
            # 更新任务进度（基于地图覆盖率）
            coverage = self._calculate_map_completion()
            self._task_manager.update_progress(task.task_id, coverage)
        else:
            # 导航启动失败，重新检测
            self._node.get_logger().warn(
                "[ExplorationHandler] Failed to start navigation, retrying..."
            )
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
    
    def _monitor_navigation(self, task: Task):
        """监控导航进度"""
        
        # 🎯 P1: 检查导航超时（2分钟）
        if hasattr(self, '_goal_start_time') and self._goal_start_time is not None:
            elapsed = time.time() - self._goal_start_time
            max_goal_time = 120.0  # 2分钟超时
            
            if elapsed > max_goal_time:
                self._node.get_logger().warn(
                    f"[ExplorationHandler] Navigation timeout ({elapsed:.1f}s > {max_goal_time}s), canceling..."
                )
                # 取消导航
                self._nav_executor.cancel_navigation()
                
                # 清空计时器
                self._goal_start_time = None
                
                # 增加失败计数并触发脱困
                self._consecutive_failures += 1
                if self._consecutive_failures >= self._max_consecutive_failures:
                    self._node.get_logger().error("[ExplorationHandler] Max consecutive failures, terminating")
                    self._task_manager.update_task_state(task.task_id, TaskState.FAILED)
                    self._exploration_state = ExplorationHandlerState.COMPLETED
                    self.release_executor(task.task_id)
                else:
                    # 触发脱困旋转
                    self._rotation_controller.start_escape_rotation(self._get_robot_yaw_odom, self._pending_goal)
                    self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
                
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
            # 🎯 导航失败 → 增加失败计数
            self._consecutive_failures += 1
            self._node.get_logger().warn(
                f"[ExplorationHandler] Navigation failed (consecutive: {self._consecutive_failures}/{self._max_consecutive_failures})"
            )
            
            # 🔴 检查是否达到连续失败上限
            if self._consecutive_failures >= self._max_consecutive_failures:
                self._node.get_logger().error(
                    f"[ExplorationHandler] Exceeded max consecutive failures ({self._max_consecutive_failures}), terminating exploration"
                )
                self._task_manager.update_task_state(task.task_id, TaskState.FAILED)
                self._exploration_state = ExplorationHandlerState.COMPLETED
                self.release_executor(task.task_id)
                return
            
            # 🔴 检查 SafetyManager 失败限制
            if hasattr(self, '_safety_manager'):
                if self._safety_manager.consecutive_failures >= self._safety_manager.max_failures:
                    self._node.get_logger().error(
                        f"[ExplorationHandler] SafetyManager max failures reached ({self._safety_manager.max_failures}), terminating exploration"
                    )
                    self._task_manager.update_task_state(task.task_id, TaskState.FAILED)
                    self._exploration_state = ExplorationHandlerState.COMPLETED
                    self.release_executor(task.task_id)
                    return
            
            # 否则继续重试 → 重新检测边界
            self._current_frontier = None
            self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
        
        elif nav_state == NavigationState.CANCELED:
            # 被取消
            self._handle_cancel(task)
    
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
                self._exploration_state = ExplorationHandlerState.DETECTING_FRONTIERS
                return False
            
            # 等待导航完成
            nav_state = self._nav_executor.get_state()
            if nav_state == NavigationState.EXECUTING:
                return True  # 阻塞，等待导航
            
            # 检查验证次数
            if self._completion_verification_attempts >= self._max_verification_attempts:
                self._node.get_logger().info(
                    f"[ExplorationHandler] Final verification completed after "
                    f"{self._completion_verification_attempts} attempts. "
                    f"Final completion: {completion*100:.2f}%"
                )
                self._exploration_state = ExplorationHandlerState.COMPLETED
                return True
            
            # 继续下一次验证
            return self._attempt_final_verification(task)
        
        # 4. 未达到阈值，继续探索
        self._node.get_logger().info(
            f"[ExplorationHandler] No frontiers found. "
            f"Map coverage: {completion*100:.1f}%, "
            f"threshold: {self._map_completion_threshold*100:.1f}%"
        )
        
        # 如果完成度接近阈值（75%以上），也可能完成
        if completion >= 0.75:
            self._node.get_logger().info(
                f"[ExplorationHandler] Coverage >= 75%, considering exploration complete"
            )
            self._exploration_state = ExplorationHandlerState.COMPLETED
            return True
        
        # 🔧 Mapper原逻辑：无边界时触发脱困旋转，而不是简单等待
        # 这是mapper中_handle_no_frontiers()的核心逻辑
        if self._exploration_strategy.increment_no_goal_count():
            # 连续无边界次数达到上限，认为探索完成
            self._node.get_logger().info(
                f"[ExplorationHandler] No frontiers found after {self._exploration_strategy.no_goal_count} attempts, "
                f"exploration may be complete (completion: {completion*100:.1f}%)"
            )
            self._exploration_state = ExplorationHandlerState.COMPLETED
            return True
        
        # 尝试脱困旋转获取更多视野
        self._node.get_logger().warn(
            f"[ExplorationHandler] No frontiers found (attempt {self._exploration_strategy.no_goal_count}), "
            f"triggering escape rotation"
        )
        
        # 🔧 关键优化：在尝试旋转前先检查障碍物距离
        nearest_obstacle = self._rotation_controller.get_nearest_obstacle_distance()
        self._node.get_logger().info(f"[DEBUG] Nearest obstacle distance: {nearest_obstacle:.2f}m")
        
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
            distance = math.sqrt(
                (center_world[0] - robot_pos[0])**2 +
                (center_world[1] - robot_pos[1])**2
            )
            
            frontier_infos.append({
                'position': center_world,
                'size': frontier_info.get('size', len(raw_frontier)),
                'distance': distance,
                'frontier': frontier_info
            })
        
        if not frontier_infos:
            return True
        
        # ⚠️ 关键：按距离降序排序，选择最远的
        frontier_infos.sort(key=lambda x: x['distance'], reverse=True)
        target_frontier = frontier_infos[0]
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Verification target: distant frontier at "
            f"distance {target_frontier['distance']:.2f}m, size {target_frontier['size']} cells"
        )
        
        # 导航到该边界（使用两步导航）
        target_pos = target_frontier['position']
        robot_yaw = self._get_robot_yaw_odom()
        if robot_yaw is None:
            # 无法获取朝向，直接导航
            self._node.get_logger().warn("[ExplorationHandler] Cannot get robot yaw, navigating directly")
            self._current_frontier = target_frontier['frontier']
            self._navigate_to_frontier(self._current_frontier, task)
        else:
            target_yaw = math.atan2(target_pos[1] - robot_pos[1], target_pos[0] - robot_pos[0])
            angle_diff = self._normalize_angle(target_yaw - robot_yaw)
            
            self._node.get_logger().info(
                f"[ExplorationHandler] First step: adjusting orientation "
                f"(rotate {math.degrees(angle_diff):.1f}°)"
            )
            
            self._pending_goal = {
                'position': target_pos,
                'yaw': target_yaw,
                'frontier_info': {
                    'size': target_frontier['size'],
                    'distance': target_frontier['distance'],
                    'direction': 0.0  # 不重要
                }
            }
            
            self._rotation_controller.start_rotation(
                math.degrees(angle_diff),
                self._get_robot_yaw_odom,
                self._pending_goal
            )
            
            self._exploration_state = ExplorationHandlerState.ROTATING_AT_GOAL
        
        return True  # 继续验证流程
    
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
        nav_state = self._nav_executor.get_state()
        
        if nav_state not in [NavigationState.IDLE, NavigationState.CANCELED]:
            self._nav_executor.cancel_navigation()
        
        # 🔴 更新任务状态为 CANCELLED
        self._task_manager.update_task_state(task.task_id, TaskState.CANCELLED)
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
    
    def _attempt_final_verification(self, task: Task) -> bool:
        """
        尝试最后验证：找远距离未探索区域 / Attempt final verification: find distant unexplored areas
        
        Returns:
            是否继续验证 / Whether to continue verification
        """
        self._completion_verification_attempts += 1
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Final verification attempt [{self._completion_verification_attempts}/{self._max_verification_attempts}]: "
            "searching for distant unexplored frontiers"
        )
        
        # 寻找边界，优先选择远距离的
        if self._current_map is None:
            self._node.get_logger().warn("[ExplorationHandler] No map available for verification")
            return True
        
        frontiers = self._frontier_detector.find_frontiers(self._current_map)
        
        if not frontiers:
            self._node.get_logger().info(
                f"[ExplorationHandler] No frontiers found in verification attempt {self._completion_verification_attempts}"
            )
            # 没找到边界，验证下一次或结束
            if self._completion_verification_attempts >= self._max_verification_attempts:
                self._node.get_logger().info(
                    "[ExplorationHandler] No explorable frontiers found after 3 attempts. Exploration completed!"
                )
                return False  # 结束验证
            return True  # 继续验证
        
        # 获取机器人位置
        robot_pose = self._nav_executor.get_robot_pose()
        if robot_pose is None:
            self._node.get_logger().warn("[ExplorationHandler] Cannot get robot position for verification")
            return True
        
        robot_pos = (robot_pose.pose.position.x, robot_pose.pose.position.y)
        
        # 转换边界信息并按距离排序（远的优先）
        frontier_infos = []
        for frontier in frontiers:
            if 'center_world' not in frontier:
                continue
            
            center_world = frontier['center_world']
            import math
            distance = math.sqrt(
                (center_world[0] - robot_pos[0])**2 + 
                (center_world[1] - robot_pos[1])**2
            )
            frontier_infos.append({
                'position': center_world,
                'size': frontier.get('size', len(frontier.get('cells', []))),
                'distance': distance,
                'frontier': frontier
            })
        
        if not frontier_infos:
            self._node.get_logger().info("[ExplorationHandler] No valid frontiers after conversion")
            return True
        
        # 按距离降序排序，选择最远的
        frontier_infos.sort(key=lambda x: x['distance'], reverse=True)
        
        # 选择最远的边界作为验证目标
        target_frontier = frontier_infos[0]
        
        self._node.get_logger().info(
            f"[ExplorationHandler] Verification target: distant frontier at distance {target_frontier['distance']:.2f}m, "
            f"size {target_frontier['size']} cells"
        )
        
        # 导航到该边界
        target_pos = target_frontier['position']
        robot_yaw = self._nav_executor.get_robot_yaw()
        
        if robot_yaw is not None:
            import math
            dx = target_pos[0] - robot_pos[0]
            dy = target_pos[1] - robot_pos[1]
            target_yaw = math.atan2(dy, dx)
            
            # 首先旋转朝向目标
            angle_diff = target_yaw - robot_yaw
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            self._node.get_logger().info(
                f"[ExplorationHandler] First step: adjusting orientation to target "
                f"(need to rotate {math.degrees(angle_diff):.1f}°)"
            )
            
            # 保存为 pending_goal
            self._pending_goal = {
                'position': target_pos,
                'yaw': target_yaw,
                'size': target_frontier['size'],
                'distance': target_frontier['distance']
            }
            
            # 启动旋转
            import math
            robot_yaw = self._nav_executor.get_robot_yaw()
            if robot_yaw is not None:
                angle_diff = target_yaw - robot_yaw
                # 规范化到[-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                angle_degrees = math.degrees(angle_diff)
                
                self._rotation_controller.start_rotation(
                    angle_degrees,
                    self._nav_executor.get_robot_yaw,
                    pending_goal=self._pending_goal
                )
            else:
                self._node.get_logger().warn("[ExplorationHandler] Cannot get robot yaw, skipping rotation")
            
            self._exploration_state = ExplorationHandlerState.ROTATING_AT_GOAL
        else:
            # 无法获取朝向，直接导航
            self._node.get_logger().warn("[ExplorationHandler] Cannot get robot yaw, navigating directly")
            self._current_frontier = target_frontier['frontier']
            self._navigate_to_frontier(self._current_frontier, task)
        
        return True  # 继续验证流程
    
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
