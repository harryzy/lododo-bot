#!/usr/bin/env python3
"""
ExplorationMapper - 自主探索建图节点 (重构版本) / Autonomous Exploration Mapping Node (Refactored Version)

功能 / Features:
  - 自动探索未知区域进行快速建图 / Automatic exploration of unknown areas for rapid mapping
  - 基于Frontier的探索策略 / Frontier-based exploration strategy
  - 模块化架构设计 / Modular architecture design
  - 中英文注释 / Chinese-English comments
  - 英文日志输出 / English log output
  
架构模块 / Architecture Modules:
  - exploration_utils.py: 工具函数和辅助类 / Utility functions and helper classes
  - frontier_detector.py: 边界检测和聚类 / Frontier detection and clustering
  - exploration_strategy.py: 探索策略和目标选择 / Exploration strategy and goal selection
  - rotation_controller.py: 旋转控制和避障 / Rotation control and obstacle avoidance
  - safety_manager.py: 安全检测和管理 / Safety detection and management

Author: LeKiwi Bot Development Team
Date: 2025-12-23 (Refactored)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from nav2_msgs.action import Spin

import numpy as np
import math
import time
from typing import Tuple, Optional

# 导入重构模块 / Import refactored modules
from .navigation_executor import NavigationExecutor, NavigationState
from .exploration_utils import map_utils, pose_utils, math_utils
from .frontier_detector import FrontierDetector
from .exploration_strategy import ExplorationStrategy
from .rotation_controller import RotationController
from .safety_manager import SafetyManager


class ExplorationMapper(Node):
    """自主探索建图节点 (重构版本) / Autonomous exploration mapping node (refactored version)"""
    
    def __init__(self):
        super().__init__('exploration_mapper')
        
        self.get_logger().info('Initializing ExplorationMapper (Refactored)...')
        
        # ===== 参数声明和加载 / Parameter declaration and loading =====
        self._declare_parameters()
        self._load_parameters()
        
        # ===== 核心模块初始化 / Core module initialization =====
        self._initialize_navigation_executor()
        self._initialize_frontier_detector()
        self._initialize_exploration_strategy()
        self._initialize_rotation_controller()
        self._initialize_safety_manager()
        
        # ===== 状态变量初始化 / State variable initialization =====
        self._initialize_state_variables()
        
        # ===== ROS接口初始化 / ROS interface initialization =====
        self._initialize_ros_interfaces()
        
        self.get_logger().info('ExplorationMapper (Refactored) initialized successfully')
        self.get_logger().info(f'Exploration radius: {self.exploration_radius}m')
        self.get_logger().info(f'Completion threshold: {self.map_completion_threshold*100}%')
        self.get_logger().info(f'Using modular architecture with {self._get_module_count()} modules')
    
    def _declare_parameters(self):
        """声明参数 / Declare parameters"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('local_costmap_topic', '/local_costmap/costmap'),
                ('exploration_radius', 5.0),
                ('min_frontier_size', 5),
                ('min_goal_distance', 0.2),
                ('goal_tolerance', 0.3),
                ('rotation_speed', 0.5),
                ('forward_speed', 0.20),
                ('map_completion_threshold', 0.90),
                ('min_completion_threshold', 0.75),
                ('safe_distance', 0.4),
                ('camera_fov', 60.0),
                ('goal_timeout', 60.0),
                ('max_failures', 8),
                ('enable_smart_exploration', True),
                ('visit_radius', 0.5),
                ('rotation_angle', 45.0),
                ('max_no_goal_count', 5),
                ('rotation_timeout', 30.0),
                ('max_escape_attempts', 7),
                ('initial_scan_angle', 100),
                ('obstacle_threshold', 50),
            ]
        )
    
    def _load_parameters(self):
        """加载参数 / Load parameters"""
        # 基础参数 / Basic parameters
        self.map_topic = self.get_parameter('map_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.map_completion_threshold = self.get_parameter('map_completion_threshold').value
        self.min_completion_threshold = self.get_parameter('min_completion_threshold').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.camera_fov_rad = math.radians(self.get_parameter('camera_fov').value)
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.max_failures = self.get_parameter('max_failures').value
        
        # 智能探索参数 / Smart exploration parameters
        self.enable_smart_exploration = self.get_parameter('enable_smart_exploration').value
        self.visit_radius = self.get_parameter('visit_radius').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.max_no_goal_count = self.get_parameter('max_no_goal_count').value
        
        # 旋转控制参数 / Rotation control parameters
        self.rotation_timeout = self.get_parameter('rotation_timeout').value
        self.max_escape_attempts = self.get_parameter('max_escape_attempts').value
        self.initial_scan_angle = self.get_parameter('initial_scan_angle').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
    
    def _initialize_navigation_executor(self):
        """初始化导航执行器 / Initialize navigation executor"""
        self._nav_executor = NavigationExecutor(self, 'navigate_to_pose')
        self._nav_executor.set_timeout(self.goal_timeout)
        self._nav_executor.set_safe_distance(self.safe_distance)
    
    def _initialize_frontier_detector(self):
        """初始化边界检测器 / Initialize frontier detector"""
        self._frontier_detector = FrontierDetector(self.min_frontier_size)
    
    def _initialize_exploration_strategy(self):
        """初始化探索策略 / Initialize exploration strategy"""
        self._exploration_strategy = ExplorationStrategy()
        
        # 配置策略参数 / Configure strategy parameters
        strategy_config = {
            'exploration_radius': self.exploration_radius,
            'min_goal_distance': self.min_goal_distance,
            'goal_tolerance': self.goal_tolerance,
            'map_completion_threshold': self.map_completion_threshold,
            'min_completion_threshold': self.min_completion_threshold,
            'max_failures': self.max_failures,
            'max_no_goal_count': self.max_no_goal_count,
            'enable_smart_exploration': self.enable_smart_exploration,
            'visit_radius': self.visit_radius,
            'rotation_angle': self.rotation_angle,
        }
        self._exploration_strategy.configure(strategy_config)
    
    def _initialize_rotation_controller(self):
        """初始化旋转控制器 / Initialize rotation controller"""
        self._rotation_controller = RotationController(self, self.cmd_vel_topic)
        
        # 配置旋转参数 / Configure rotation parameters
        rotation_config = {
            'rotation_speed': self.rotation_speed,
            'rotation_timeout': self.rotation_timeout,
            'rotation_tolerance_deg': 5,
            'check_obstacle_during_rotation': True,
            'max_escape_attempts': self.max_escape_attempts,
            'initial_scan_angle': self.initial_scan_angle,
        }
        self._rotation_controller.configure(rotation_config)
    
    def _initialize_safety_manager(self):
        """初始化安全管理器 / Initialize safety manager"""
        self._safety_manager = SafetyManager()
        
        # 配置安全参数 / Configure safety parameters
        safety_config = {
            'safe_distance': self.safe_distance,
            'min_goal_distance': self.min_goal_distance,
            'goal_tolerance': self.goal_tolerance,
            'obstacle_threshold': self.obstacle_threshold,
            'max_failures': self.max_failures,
            'max_safety_violations': 3,
        }
        self._safety_manager.configure(safety_config)
    
    def _initialize_state_variables(self):
        """初始化状态变量 / Initialize state variables"""
        # 地图数据 / Map data
        self.current_map = None
        self.current_map_msg = None
        self.map_resolution = 0.05
        self.map_origin = None
        self.total_cells = 0
        self.known_cells = 0
        
        # 机器人状态 / Robot state
        self.robot_pose = None
        self.robot_yaw = None
        
        # 探索状态 / Exploration state
        self.is_exploring = False
        self.exploration_complete = False
        self.current_goal = None
        self.goal_start_time = None
        
        # 目标管理 / Goal management
        self.pending_goal = None
        self.skip_rotation = False
        
        # 完成度验证 / Completion verification
        self.completion_scan_in_progress = False
        self.completion_before_scan = 0.0
        self.completion_verification_attempts = 0  # 验证尝试次数 / Verification attempt count
    
    def _initialize_ros_interfaces(self):
        """初始化ROS接口 / Initialize ROS interfaces"""
        # QoS配置 / QoS configuration
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅器 / Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )
        
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, self.local_costmap_topic, self.local_costmap_callback, map_qos
        )
        
        # 发布器 / Publishers
        self.completion_pub = self.create_publisher(Bool, '/exploration/complete', 10)
        
        # Action客户端 / Action clients
        self.spin_client = ActionClient(self, Spin, 'spin')
        
        # 定时器 / Timers
        self.exploration_timer = self.create_timer(2.0, self.exploration_loop)
        self.status_timer = self.create_timer(5.0, self.print_status)
    
    def _get_module_count(self) -> int:
        """获取模块数量 / Get module count"""
        return 5  # frontier_detector, exploration_strategy, rotation_controller, safety_manager, navigation_executor
    
    # ===== 回调函数 / Callback functions =====
    
    def map_callback(self, msg: OccupancyGrid):
        """地图回调 / Map callback"""
        self.current_map_msg = msg
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # 更新所有模块的地图数据 / Update map data for all modules
        self._frontier_detector.update_map(msg)
        self._safety_manager.update_map(msg)
        self._nav_executor.set_map(msg)
        
        # 计算地图完成度 / Calculate map completion
        self.total_cells, self.known_cells = self._calculate_map_statistics()
    
    def local_costmap_callback(self, msg: OccupancyGrid):
        """局部代价地图回调 / Local costmap callback"""
        self._rotation_controller.update_local_costmap(msg)
        self._safety_manager.update_local_costmap(msg)
    
    def _calculate_map_statistics(self) -> Tuple[int, int]:
        """计算地图统计信息 / Calculate map statistics"""
        if self.current_map is None:
            return 0, 0
        
        completion_data = map_utils.calculate_map_completion(self.current_map)
        # 这里需要重新计算，因为map_utils返回的是比例，我们需要具体数值
        # Need to recalculate as map_utils returns ratio, we need specific values
        
        free_cells = np.sum((self.current_map >= 0) & (self.current_map <= 50))
        obstacle_cells = np.sum(self.current_map > 50)
        known_cells = free_cells + obstacle_cells
        
        unknown_cells = np.sum(self.current_map == -1)
        explorable_unknown = min(unknown_cells, int(free_cells * 0.3))
        total_cells = free_cells + obstacle_cells + explorable_unknown
        
        return total_cells, known_cells
    
    # ===== 核心探索逻辑 / Core exploration logic =====
    
    def exploration_loop(self):
        """探索主循环 / Main exploration loop"""
        if self.exploration_complete:
            return
        
        # 优先处理旋转状态 / Priority handling of rotation state
        if self._rotation_controller.update_rotation(self.get_robot_yaw):
            return  # 还在旋转中，等待下次循环 / Still rotating, wait for next loop
        
        # 旋转刚完成，检查是否有待发送的目标 / Rotation just completed, check pending goal
        if self.pending_goal is not None:
            # 检查是否因为障碍物而停止旋转，需要寻找安全点 / Check if rotation stopped due to obstacle, need to find safe point
            if self._rotation_controller.skip_rotation_on_obstacle:
                # 检查导航状态 / Check navigation state
                nav_state = self._nav_executor.get_state()
                
                # 如果导航正在执行或取消中，等待 / If navigating or canceling, wait
                if nav_state in [NavigationState.EXECUTING, NavigationState.CANCELING]:
                    self.get_logger().debug(
                        'Waiting for current navigation to complete before handling rotation obstacle',
                        throttle_duration_sec=2.0
                    )
                    return  # 等待当前导航完成 / Wait for current navigation to complete
                
                # 导航已完成(SUCCESS或IDLE)，处理旋转障碍 / Navigation completed (SUCCESS or IDLE), handle rotation obstacle
                # 注意：_handle_rotation_obstacle会检查pre_rotation_point状态并决定下一步操作
                # Note: _handle_rotation_obstacle will check pre_rotation_point status and decide next action
                self._handle_rotation_obstacle()
            else:
                # 正常完成旋转，发送目标 / Rotation completed normally, send goal
                self._send_pending_goal()
            return
        
        # 等待地图数据 / Wait for map data
        if self.current_map is None:
            self.get_logger().info('Waiting for map data...', throttle_duration_sec=5.0)
            return
        
        # 检查导航状态 / Check navigation status
        if not self._handle_navigation_state():
            return
        
        # 检查完成条件 / Check completion criteria
        if self._check_completion():
            return
        
        # 处理各种探索模式 / Handle various exploration modes
        if not self._handle_exploration_modes():
            return
        
        # 寻找和选择目标 / Find and select goal
        self._find_and_select_goal()
    
    def _handle_navigation_state(self) -> bool:
        """处理导航状态 / Handle navigation state"""
        nav_state = self._nav_executor.get_state()
        
        # 如果正在导航，检查超时 / If navigating, check timeout
        if nav_state in [NavigationState.EXECUTING, NavigationState.CANCELING]:
            if self.goal_start_time is not None:
                elapsed = time.time() - self.goal_start_time
                if elapsed > self.goal_timeout and nav_state == NavigationState.EXECUTING:
                    self.get_logger().error(
                        f'Goal timeout ({elapsed:.1f}s > {self.goal_timeout}s), requesting cancellation'
                    )
                    self._nav_executor.cancel_navigation()
                    self._exploration_strategy.increment_failure_count()
                    self.goal_start_time = None
                    return False
            
            # 等待导航完成 / Wait for navigation completion
            self.get_logger().debug(
                f'Waiting for navigation completion... (state: {nav_state.value})',
                throttle_duration_sec=2.0
            )
            return False
        
        # 处理导航结果 / Handle navigation results
        # 如果是预移动点导航，跳过这里的处理，让_handle_rotation_obstacle处理
        # If it's pre-movement point navigation, skip handling here, let _handle_rotation_obstacle handle it
        if nav_state == NavigationState.SUCCESS:
            # 检查是否是预移动点导航 / Check if it's pre-movement point navigation
            if self._rotation_controller.skip_rotation_on_obstacle and self._rotation_controller.pre_rotation_point is not None:
                # 这是预移动点导航成功，不在这里处理，等待_handle_rotation_obstacle处理
                # This is pre-movement point navigation success, don't handle here, wait for _handle_rotation_obstacle
                self.get_logger().debug('Pre-movement point navigation succeeded, will be handled by _handle_rotation_obstacle')
                pass  # 不处理，保持SUCCESS状态 / Don't handle, keep SUCCESS state
            else:
                # 正常导航成功 / Normal navigation success
                self._handle_navigation_success()
        elif nav_state == NavigationState.FAILED:
            self._handle_navigation_failure()
        elif nav_state == NavigationState.CANCELED:
            self._handle_navigation_cancellation()
        
        # 🔧 关键修复：确保在IDLE状态才能处理新任务 / Ensure IDLE state before processing new tasks
        # 但如果是预移动点的SUCCESS状态，也返回False让_handle_rotation_obstacle处理
        # But if it's pre-movement point SUCCESS state, also return False to let _handle_rotation_obstacle handle
        if nav_state == NavigationState.SUCCESS and self._rotation_controller.skip_rotation_on_obstacle:
            return False  # 等待_handle_rotation_obstacle处理 / Wait for _handle_rotation_obstacle to handle
        
        if nav_state != NavigationState.IDLE:
            return False
        
        return True
    
    def _handle_navigation_success(self):
        """处理导航成功 / Handle navigation success"""
        # 注意：不在这里打印"Navigation succeeded"，避免与navigation_executor的回调重复
        # Note: Don't print "Navigation succeeded" here to avoid duplication with navigation_executor callback
        
        # 检查是否到达了预移动点 / Check if reached pre-rotation point
        if self._rotation_controller.pre_rotation_point is not None and self.pending_goal is not None:
            self.get_logger().info('Reached safe rotation point, now rotating to target orientation')
            
            # 清空预移动点标记和skip标志 / Clear pre-rotation point marker and skip flag
            self._rotation_controller.pre_rotation_point = None
            self._rotation_controller.skip_rotation_on_obstacle = False  # 清空标志，允许后续处理 / Clear flag to allow subsequent processing
            
            # 检查重试次数，防止循环 / Check retry count to prevent loop
            if self._rotation_controller.pre_rotation_retry_count >= self._rotation_controller.max_pre_rotation_retries:
                self.get_logger().warn(
                    f'Pre-rotation retry limit reached ({self._rotation_controller.max_pre_rotation_retries}), '
                    'abandoning pre-rotation strategy and using direct navigation'
                )
                self._rotation_controller.pre_rotation_retry_count = 0
                self._rotation_controller.skip_rotation_on_obstacle = False  # 重置标志 / Reset flag
                self._send_pending_goal()
                return
            
            # 重新尝试旋转到目标朝向 / Retry rotation to target orientation
            robot_pos = self.get_robot_position()
            robot_yaw = self.get_robot_yaw()
            
            if robot_pos is not None and robot_yaw is not None:
                target_pos = self.pending_goal['position']
                target_yaw = self.pending_goal['yaw']
                
                angle_diff = target_yaw - robot_yaw
                angle_diff = math_utils.normalize_angle(angle_diff)
                
                # 增加重试计数 / Increment retry count
                self._rotation_controller.pre_rotation_retry_count += 1
                
                # 开始旋转到目标朝向 / Start rotation to target orientation
                self._rotation_controller.skip_rotation_on_obstacle = False
                self._rotation_controller.start_rotation(
                    math.degrees(angle_diff), self.get_robot_yaw, self.pending_goal
                )
                
                # 重置导航状态等待旋转完成 / Reset navigation state and wait for rotation
                self.goal_start_time = None
                self._nav_executor.reset_state()
                return
            else:
                # 无法获取位置或朝向，直接发送目标 / Cannot get position or orientation, send goal directly
                self.get_logger().warn('Cannot get robot pose after reaching safe point, sending goal directly')
                self._send_pending_goal()
                return
        else:
            # 正常探索流程：导航成功但不是预移动点场景 / Normal exploration flow: navigation succeeded but not pre-rotation case
            # 这是正常情况，不需要警告 / This is normal, no warning needed
            self.get_logger().debug(
                f'Normal navigation completed (pre_rotation_point={self._rotation_controller.pre_rotation_point}, pending_goal={self.pending_goal})',
                throttle_duration_sec=2.0
            )
            
            # 清空skip标志，防止卡死 / Clear skip flag to prevent deadlock
            if self._rotation_controller.skip_rotation_on_obstacle:
                self.get_logger().info('Clearing skip_rotation_on_obstacle flag to prevent deadlock')
                self._rotation_controller.skip_rotation_on_obstacle = False
            
            # 清空待发送的目标点 / Clear pending goal
            self.pending_goal = None
        
        # 检查探索进展 / Check exploration progress
        progress_status = self._exploration_strategy.check_exploration_progress(self.known_cells)
        
        # 处理停滞状态 / Handle stagnant state
        if self._exploration_strategy.should_trigger_rotation(progress_status):
            self.get_logger().warn('Exploration stagnation detected, triggering rotation')
            self._rotation_controller.start_escape_rotation(self.get_robot_yaw, self.pending_goal)
        
        # 处理有效探索 / Handle effective exploration
        if self._exploration_strategy.should_trigger_360_scan(progress_status):
            self.get_logger().info('Effective exploration detected, triggering 360-degree scan')
            scan_angle = self._rotation_controller.find_nearest_unknown_direction(
                self.get_robot_position(), self.get_robot_yaw(), self.current_map, self.current_map_msg
            )
            self._rotation_controller.start_rotation(scan_angle, self.get_robot_yaw)
        
        # 重置状态 / Reset state
        self._exploration_strategy.reset_failure_count()
        self._rotation_controller.reset_escape_attempts()  # 导航成功，重置脱困计数 / Navigation succeeded, reset escape attempts
        self.goal_start_time = None
        self._nav_executor.reset_state()
    
    def _handle_navigation_failure(self):
        """处理导航失败 / Handle navigation failure"""
        self.get_logger().warn(f'Navigation failed')
        
        # 增加失败计数 / Increment failure count
        if self._exploration_strategy.increment_failure_count():
            self.get_logger().error(f'Maximum failures ({self.max_failures}) reached, exploration terminated')
            self._terminate_exploration(success=False)
            return
        
        # 触发旋转以获取新视野 / Trigger rotation to get new view
        self._rotation_controller.start_escape_rotation(self.get_robot_yaw, self.pending_goal)
        
        self.goal_start_time = None
        self._nav_executor.reset_state()
    
    def _handle_navigation_cancellation(self):
        """处理导航取消 / Handle navigation cancellation"""
        self.get_logger().warn('Navigation cancelled')
        
        # 清除预移动点相关标志，避免死循环 / Clear pre-rotation flags to avoid infinite loop
        if self._rotation_controller.pre_rotation_point is not None:
            # 注意：导航到安全点被取消不算作探索失败，因为这只是预移动策略的一部分
            # Note: Safe point navigation cancellation doesn't count as exploration failure, it's just pre-movement strategy
            self.get_logger().warn('Navigation to safe point cancelled, clearing pre-rotation state (not counting as failure)')
            self._rotation_controller.pre_rotation_point = None
            self._rotation_controller.skip_rotation_on_obstacle = False
            self._rotation_controller.pre_rotation_retry_count = 0
            self._rotation_controller.after_pre_movement = False
            # 清除pending_goal，避免重复尝试 / Clear pending_goal to avoid retry
            self.pending_goal = None
        else:
            # 只有真正的探索目标取消才计入失败 / Only count real exploration goal cancellation as failure
            self._exploration_strategy.increment_failure_count()
        
        self.goal_start_time = None
        self._nav_executor.reset_state()
    
    def _handle_rotation_obstacle(self):
        """处理旋转遇到障碍物的情况 / Handle rotation obstacle case"""
        if self.pending_goal is None:
            self.get_logger().debug('_handle_rotation_obstacle: pending_goal is None, skipping')
            return
        
        # 检查导航状态，如果成功到达pre_rotation_point，启动旋转 / Check navigation state, if reached pre_rotation_point, start rotation
        nav_state = self._nav_executor.get_state()
        pre_rot_point = self._rotation_controller.pre_rotation_point
        skip_flag = self._rotation_controller.skip_rotation_on_obstacle
        
        self.get_logger().info(
            f'_handle_rotation_obstacle called: nav_state={nav_state.value}, '
            f'pre_rotation_point={pre_rot_point}, skip_rotation_on_obstacle={skip_flag}, '
            f'retry_count={self._rotation_controller.pre_rotation_retry_count}',
            throttle_duration_sec=1.0
        )
        
        if nav_state == NavigationState.SUCCESS and pre_rot_point is not None:
            self.get_logger().info('Reached safe rotation point, now rotating to target orientation')
            
            # 清空预移动点标记 / Clear pre-rotation point marker
            self._rotation_controller.pre_rotation_point = None
            
            # 获取机器人位置和朝向 / Get robot position and orientation
            robot_pos = self.get_robot_position()
            robot_yaw = self.get_robot_yaw()
            
            if robot_pos is not None and robot_yaw is not None and self.pending_goal is not None:
                target_yaw = self.pending_goal['yaw']
                angle_diff = target_yaw - robot_yaw
                angle_diff = math_utils.normalize_angle(angle_diff)
                
                # 增加重试计数 / Increment retry count
                self._rotation_controller.pre_rotation_retry_count += 1
                
                # 设置after_pre_movement标志，在到达安全点后旋转时使用更宽松的障碍检测
                # Set after_pre_movement flag for more tolerant obstacle detection after reaching safe point
                self._rotation_controller.after_pre_movement = True
                
                # 开始旋转到目标朝向 / Start rotation to target orientation
                self._rotation_controller.skip_rotation_on_obstacle = False
                if self._rotation_controller.start_rotation(target_yaw, self.get_robot_yaw, self.pending_goal):
                    self.get_logger().info(f'Started rotation to target yaw: {math.degrees(target_yaw):.1f}°')
                else:
                    self.get_logger().warn('Failed to start rotation after reaching safe point, sending goal directly')
                    self._rotation_controller.skip_rotation_on_obstacle = False
                    self._rotation_controller.after_pre_movement = False  # 重置标志 / Reset flag
                    self._send_pending_goal()
            else:
                self.get_logger().warn('Cannot get robot pose after reaching safe point, sending goal directly')
                self._rotation_controller.skip_rotation_on_obstacle = False
                self._rotation_controller.after_pre_movement = False  # 重置标志 / Reset flag
                self._send_pending_goal()
            
            # 重置导航状态 / Reset navigation state
            self.goal_start_time = None
            self._nav_executor.reset_state()
            return
        
        # 检查导航状态 - 如果取消，说明无法到达安全点 / Check nav state - if canceled, cannot reach safe point
        if nav_state == NavigationState.CANCELED:
            self.get_logger().warn(
                'Navigation to safe point canceled, likely unreachable. Using direct navigation with Nav2.'
            )
            # 清除预移动点标志 / Clear pre-rotation flags
            self._rotation_controller.pre_rotation_point = None
            self._rotation_controller.skip_rotation_on_obstacle = False
            self._rotation_controller.pre_rotation_retry_count = 0
            self._rotation_controller.after_pre_movement = False
            # 直接导航到目标，让Nav2处理避障 / Navigate directly, let Nav2 handle obstacles
            self._send_pending_goal()
            return
        
        # 检查导航状态是否需要重置 / Check if navigation state needs reset
        # 只在非IDLE状态时重置，避免"Cannot reset state from idle"警告
        # Only reset when not in IDLE state to avoid "Cannot reset state from idle" warning
        if nav_state != NavigationState.IDLE:
            self._nav_executor.reset_state()
        
        # 检查是否已经尝试过预移动策略 / Check if pre-rotation strategy has been tried
        if self._rotation_controller.pre_rotation_retry_count >= 1:
            self.get_logger().warn(
                'Pre-rotation already tried once, using direct navigation to avoid getting stuck'
            )
            self._rotation_controller.skip_rotation_on_obstacle = False
            self._send_pending_goal()
            return
        
        # 尝试找一个安全的预移动点 / Try to find a safe pre-movement point
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            self.get_logger().warn('Cannot get robot position, using direct navigation')
            self._rotation_controller.skip_rotation_on_obstacle = False
            self._send_pending_goal()
            return
        
        target_pos = self.pending_goal['position']
        
        # 寻找安全的预移动点 / Find safe pre-movement point
        safe_point = self._safety_manager.find_safe_rotation_point(
            robot_pos, target_pos, search_radius=1.5
        )
        
        if safe_point is not None:
            # 检查距离是否足够远 / Check if distance is sufficient
            dist_to_safe_point = math_utils.calculate_distance(safe_point, robot_pos)
            if dist_to_safe_point < 0.5:
                self.get_logger().warn(
                    f'Safe point too close ({dist_to_safe_point:.2f}m), using direct navigation'
                )
                self._rotation_controller.skip_rotation_on_obstacle = False
                self._send_pending_goal()
                return
            
            # 找到安全点，先导航到该点 / Found safe point, navigate there first
            self.get_logger().info(
                f'Rotation blocked by obstacle. Moving to safe point '
                f'({safe_point[0]:.2f}, {safe_point[1]:.2f}), distance={dist_to_safe_point:.2f}m'
            )
            
            # 计算朝向安全点的yaw角 / Calculate yaw to safe point
            safe_point_yaw = math_utils.calculate_angle(robot_pos, safe_point)
            goal_pose = pose_utils.create_pose_stamped(
                safe_point[0], safe_point[1], safe_point_yaw
            )
            
            # 导航到安全点（不重置pending_goal，到达后会继续处理原目标）
            # Navigate to safe point (don't reset pending_goal, will continue to original target after arrival)
            if self._nav_executor.navigate_to_pose(goal_pose):
                self.current_goal = safe_point
                self.goal_start_time = time.time()
                self._rotation_controller.pre_rotation_point = safe_point
                # 不重置skip_rotation_on_obstacle，等到达安全点后再处理
                # Don't reset skip_rotation_on_obstacle until we reach the safe point
            else:
                self.get_logger().warn('Failed to navigate to safe rotation point, using direct navigation')
                self._send_pending_goal()
        else:
            # 没找到安全点，直接导航（让Nav2处理避障）
            # No safe point found, navigate directly (let Nav2 handle obstacle avoidance)
            self.get_logger().warn('No safe rotation point found, using direct navigation with Nav2 obstacle avoidance')
            self._rotation_controller.skip_rotation_on_obstacle = False  # 重置标志 / Reset flag
            self._send_pending_goal()
    
    def _check_completion(self) -> bool:
        """检查完成条件 / Check completion criteria"""
        # 检查地图完成度 / Check map completion
        completion = self.known_cells / self.total_cells if self.total_cells > 0 else 0.0
        completion_status = self._exploration_strategy.check_completion_criteria(completion)
        
        # 如果达到完成度阈值，尝试找远距离未探索区域进行最后验证
        # If completion threshold reached, try to find distant unexplored areas for final verification
        if completion_status['is_complete'] and not self.completion_scan_in_progress:
            self.get_logger().info(
                f'Completion threshold reached ({completion*100:.2f}%), '
                f'attempting to find distant unexplored areas for final verification'
            )
            self.completion_scan_in_progress = True
            self.completion_before_scan = completion
            self.completion_verification_attempts = 0
            
            # 尝试找远距离未探索目标 / Try to find distant unexplored target
            return self._attempt_final_verification()
        
        # 如果正在进行完成度验证 / If completion verification in progress
        if self.completion_scan_in_progress:
            # 检查完成度是否有增长 / Check if completion has increased
            if completion > self.completion_before_scan + 0.01:  # 增长超过1% / Increased more than 1%
                self.get_logger().info(
                    f'Map expanded during verification '
                    f'({self.completion_before_scan*100:.2f}% -> {completion*100:.2f}%), '
                    f'continuing exploration'
                )
                # 取消完成度验证，继续探索 / Cancel verification, continue exploration
                self.completion_scan_in_progress = False
                return False
            
            # 如果还在导航中，等待 / If still navigating, wait
            nav_state = self._nav_executor.get_state()
            if nav_state == NavigationState.EXECUTING:
                return True  # 等待导航完成 / Wait for navigation
            
            # 导航完成或失败，检查是否达到验证次数上限 / Navigation done or failed, check attempts limit
            if self.completion_verification_attempts >= 3:
                self.get_logger().info(
                    f'Final verification completed after {self.completion_verification_attempts} attempts. '
                    f'Final completion: {completion*100:.2f}%. Exploration completed!'
                )
                self._terminate_exploration(success=True)
                return True
            
            # 继续尝试下一次验证 / Continue next verification attempt
            return self._attempt_final_verification()
        
        return False
    
    def _handle_exploration_modes(self) -> bool:
        """处理探索模式 / Handle exploration modes"""
        # 🔧 修复：如果正在进行完成度验证，不处理其他模式
        # Fix: If completion verification in progress, don't handle other modes
        if self.completion_scan_in_progress:
            return False  # 等待验证完成 / Wait for verification to complete
        
        # 检查是否正在旋转 / Check if rotating
        if self._rotation_controller.is_rotating:
            return False  # 正在旋转，等待完成 / Rotating, wait for completion
        
        # 🔧 修复：初始扫描模式 - 只在未完成时执行
        if not self._rotation_controller.initial_scan_done:
            if self._rotation_controller.start_initial_scan(self.get_robot_yaw):
                return False  # 开始初始扫描 / Start initial scan
            # 扫描完成，继续执行 / Scan completed, continue
        
        return True
    
    def _find_and_select_goal(self):
        """寻找和选择目标 / Find and select goal"""
        # 寻找边界 / Find frontiers
        frontiers = self._frontier_detector.find_frontiers()
        
        if not frontiers:
            self._handle_no_frontiers()
            return
        
        # 转换边界信息 / Convert frontier information
        frontier_infos = []
        for frontier in frontiers:
            frontier_info = self._frontier_detector.calculate_frontier_info(frontier)
            if frontier_info:
                frontier_infos.append(frontier_info)
        
        # 选择最佳边界 / Select best frontier
        robot_pos = self.get_robot_position()
        robot_yaw = self.get_robot_yaw()
        
        if robot_pos is None or robot_yaw is None:
            self.get_logger().warn('Cannot get robot position or orientation')
            return
        
        best_frontier = self._exploration_strategy.select_best_frontier(
            frontier_infos, robot_pos, robot_yaw, self.known_cells, self.total_cells
        )
        
        if best_frontier is None:
            self._handle_no_suitable_frontier()
            return
        
        # 在边界附近找安全目标点 / Find safe goal point near frontier
        frontier_cells = best_frontier['frontier_info']['cells']
        safe_target = self._frontier_detector.find_safe_goal_near_frontier(
            frontier_cells, robot_pos, self.safe_distance, 1.5
        )
        
        if safe_target is None:
            # 尝试寻找替代目标 / Try to find alternative goal
            center_world = best_frontier['frontier_info']['center_world']
            safe_target = self._safety_manager.find_alternative_goal(
                center_world, robot_pos, search_radius=1.0
            )
            
            if safe_target is None:
                self.get_logger().warn('No safe goal found near frontier')
                if self._exploration_strategy.increment_failure_count():
                    self._terminate_exploration(success=False)
                return
        
        # 验证目标安全性 / Validate goal safety
        is_safe, safety_reason = self._safety_manager.is_goal_safe(
            safe_target[0], safe_target[1], robot_pos
        )
        
        if not is_safe:
            self.get_logger().warn(f'Goal safety check failed: {safety_reason}')
            # 标记该frontier为已访问，避免重复选择 / Mark frontier as visited to avoid reselection
            self._exploration_strategy.update_visited_frontier(safe_target)
            # 增加失败计数，达到阈值后触发脱困 / Increment failure count, trigger escape if threshold reached
            if self._exploration_strategy.increment_failure_count():
                # 检查是否连续旋转失败 / Check if consecutive rotation failures
                if self._rotation_controller.consecutive_rotation_failures >= self._rotation_controller.max_consecutive_rotation_failures:
                    self.get_logger().warn('Multiple rotation failures detected, performing backward escape')
                    self._perform_backward_escape()
                    self._rotation_controller.consecutive_rotation_failures = 0
                else:
                    self.get_logger().warn('Multiple safety check failures, triggering escape rotation')
                    self._rotation_controller.start_escape_rotation(self.get_robot_yaw, self.pending_goal)
                self._exploration_strategy.reset_failure_count()
            return
        
        # 发送导航目标 / Send navigation goal
        self._send_navigation_goal(safe_target, best_frontier)
    
    def _handle_no_frontiers(self):
        """处理无边界情况 / Handle no frontiers case"""
        # 🔧 修复：只调用一次increment
        if self._exploration_strategy.increment_no_goal_count():
            completion = self.known_cells / self.total_cells if self.total_cells > 0 else 0.0
            self.get_logger().info(f'Exploration completed! No more frontiers (completion: {completion*100:.1f}%)')
            self._terminate_exploration(success=True)
            return
        
        # 尝试旋转获取更多视野 / Try rotation to get more view
        self.get_logger().warn('No frontiers found, triggering escape rotation')
        self._rotation_controller.start_escape_rotation(self.get_robot_yaw, self.pending_goal)
    
    def _handle_no_suitable_frontier(self):
        """处理无合适边界情况 / Handle no suitable frontier case"""
        completion = self.known_cells / self.total_cells if self.total_cells > 0 else 0.0
        
        if completion >= self.min_completion_threshold:
            self.get_logger().info(f'Exploration near completion ({completion*100:.2f}%) with no suitable frontiers')
            self._terminate_exploration(success=True)
            return
        
        # 🔧 修复：只调用一次increment
        if self._exploration_strategy.increment_no_goal_count():
            self.get_logger().warn('Multiple failures to find suitable frontier, triggering escape mode')
            self._rotation_controller.start_escape_rotation(self.get_robot_yaw, self.pending_goal)
            self._exploration_strategy.reset_no_goal_count()
    
    def _send_navigation_goal(self, target_pos: Tuple[float, float], frontier_eval: dict):
        """发送导航目标 / Send navigation goal"""
        # 计算朝向目标的yaw角 / Calculate yaw angle to target
        target_yaw = math_utils.calculate_angle(self.get_robot_position(), target_pos)
        
        # 创建目标位姿 / Create goal pose
        goal_pose = pose_utils.create_pose_stamped(target_pos[0], target_pos[1], target_yaw)
        
        # 记录探索策略状态 / Record exploration strategy state
        self._exploration_strategy.update_visited_frontier(target_pos)
        self._exploration_strategy.update_last_goal_direction(frontier_eval['direction'])
        self._exploration_strategy.reset_no_goal_count()
        
        # 两步导航策略：先调整朝向 / Two-step navigation: first adjust orientation
        robot_yaw = self.get_robot_yaw()
        if robot_yaw is not None:
            angle_diff = target_yaw - robot_yaw
            angle_diff = math_utils.normalize_angle(angle_diff)
            
            if abs(angle_diff) > math.radians(10):
                self.get_logger().info(
                    f'First step: adjusting orientation to target '
                    f'(need to rotate {math.degrees(angle_diff):.1f}°)'
                )
                
                # 保存目标信息 / Save goal information
                self.pending_goal = {
                    'position': target_pos,
                    'yaw': target_yaw,
                    'frontier_info': (frontier_eval['frontier_info']['size'], 
                                    frontier_eval['distance'], 
                                    frontier_eval['direction'])
                }
                
                # 开始旋转 / Start rotation
                self._rotation_controller.start_rotation(
                    math.degrees(angle_diff), self.get_robot_yaw, self.pending_goal
                )
                return
        
        # 直接发送导航目标 / Send navigation goal directly
        self.get_logger().info(
            f'Second step: moving to target - size={frontier_eval["frontier_info"]["size"]}, '
            f'distance={frontier_eval["distance"]:.2f}m, '
            f'direction={math.degrees(frontier_eval["direction"]):.1f}°, '
            f'position=({target_pos[0]:.2f}, {target_pos[1]:.2f})'
        )
        
        # 使用NavigationExecutor导航 / Use NavigationExecutor for navigation
        if self._nav_executor.navigate_to_pose(goal_pose):
            self.current_goal = target_pos
            self.goal_start_time = time.time()
            self._exploration_strategy.reset_failure_count()
            self.get_logger().info('State: IDLE -> EXECUTING')
        else:
            self.get_logger().error('Failed to send navigation goal')
            if self._exploration_strategy.increment_failure_count():
                self._terminate_exploration(success=False)
    
    def _send_pending_goal(self):
        """发送待处理目标 / Send pending goal"""
        if self.pending_goal is None:
            return
        
        goal_info = self.pending_goal
        target_pos = goal_info['position']
        target_yaw = goal_info['yaw']
        
        # 兼容两种格式：frontier_info元组格式和单独字段格式 / Support both formats
        if 'frontier_info' in goal_info:
            frontier_size, distance, direction = goal_info['frontier_info']
            self.get_logger().info(
                f'Moving to target (skipping rotation) - size={frontier_size}, '
                f'distance={distance:.2f}m, direction={math.degrees(direction):.1f}°, '
                f'position=({target_pos[0]:.2f}, {target_pos[1]:.2f})'
            )
        else:
            # 完成验证模式下的格式 / Format in completion verification mode
            frontier_size = goal_info.get('size', 0)
            distance = goal_info.get('distance', 0.0)
            self.get_logger().info(
                f'Moving to verification target - size={frontier_size}, '
                f'distance={distance:.2f}m, position=({target_pos[0]:.2f}, {target_pos[1]:.2f})'
            )
        
        # 创建目标位姿 / Create goal pose
        goal_pose = pose_utils.create_pose_stamped(target_pos[0], target_pos[1], target_yaw)
        
        # 发送导航目标 / Send navigation goal
        if self._nav_executor.navigate_to_pose(goal_pose):
            self.current_goal = target_pos
            self.goal_start_time = time.time()
            self._exploration_strategy.reset_no_goal_count()
        
        # 清理状态 / Clean up state
        self.pending_goal = None
        self._rotation_controller.skip_rotation_on_obstacle = False
        self._rotation_controller.pre_rotation_point = None
        self._rotation_controller.pre_rotation_retry_count = 0
    
    # ===== 辅助函数 / Helper functions =====
    
    def _attempt_final_verification(self) -> bool:
        """
        尝试最后验证：找远距离未探索区域
        Attempt final verification: find distant unexplored areas
        
        Returns:
            是否继续验证 / Whether to continue verification
        """
        self.completion_verification_attempts += 1
        
        self.get_logger().info(
            f'Final verification attempt [{self.completion_verification_attempts}/3]: '
            f'searching for distant unexplored frontiers'
        )
        
        # 寻找边界，优先选择远距离的 / Find frontiers, prioritize distant ones
        frontiers = self._frontier_detector.find_frontiers()
        
        if not frontiers:
            self.get_logger().info(f'No frontiers found in verification attempt {self.completion_verification_attempts}')
            # 没找到边界，验证下一次或结束 / No frontiers, verify next or finish
            if self.completion_verification_attempts >= 3:
                self.get_logger().info('No explorable frontiers found after 3 attempts. Exploration completed!')
                self._terminate_exploration(success=True)
                return True
            return True  # 继续验证 / Continue verification
        
        # 转换边界信息并按距离排序（远的优先）
        # Convert frontier info and sort by distance (distant first)
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return True
        
        frontier_infos = []
        for frontier in frontiers:
            # 使用frontier_detector计算边界信息 / Use frontier_detector to calculate frontier info
            frontier_info = self._frontier_detector.calculate_frontier_info(frontier)
            if not frontier_info or 'center_world' not in frontier_info:
                continue
            
            center_world = frontier_info['center_world']
            distance = math_utils.calculate_distance(center_world, robot_pos)
            frontier_infos.append({
                'position': center_world,
                'size': len(frontier),
                'distance': distance
            })
        
        if not frontier_infos:
            self.get_logger().info('No valid frontiers after conversion')
            return True
        
        # 按距离降序排序，选择最远的 / Sort by distance descending, choose farthest
        frontier_infos.sort(key=lambda x: x['distance'], reverse=True)
        
        # 选择最远的边界作为验证目标 / Choose farthest frontier as verification target
        target_frontier = frontier_infos[0]
        
        self.get_logger().info(
            f'Verification target: distant frontier at distance {target_frontier["distance"]:.2f}m, '
            f'size {target_frontier["size"]} cells'
        )
        
        # 导航到该边界 / Navigate to that frontier
        target_pos = target_frontier['position']
        target_yaw = math_utils.calculate_angle(robot_pos, target_pos)
        
        # 首先旋转朝向目标 / First rotate towards target
        angle_diff = target_yaw - self.get_robot_yaw()
        angle_diff = math_utils.normalize_angle(angle_diff)
        
        self.get_logger().info(
            f'First step: adjusting orientation to target (need to rotate {math.degrees(angle_diff):.1f}°)'
        )
        
        self.pending_goal = {
            'position': target_pos,
            'yaw': target_yaw,
            'size': target_frontier['size'],
            'distance': target_frontier['distance']
        }
        
        self._rotation_controller.start_rotation(
            math.degrees(angle_diff), self.get_robot_yaw, self.pending_goal
        )
        
        return True  # 继续验证流程 / Continue verification process
    
    def get_robot_position(self) -> Optional[Tuple[float, float]]:
        """获取机器人当前位置 / Get current robot position"""
        try:
            pose_stamped = self._nav_executor.get_robot_pose()
            if pose_stamped:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                self.robot_pose = (x, y)
                return (x, y)
        except Exception as e:
            self.get_logger().warn(f'Cannot get robot position: {e}', throttle_duration_sec=5.0)
        
        # 降级：使用地图中心 / Fallback: use map center
        if self.current_map is not None and self.map_origin is not None:
            height, width = self.current_map.shape
            center_mx = width // 2
            center_my = height // 2
            return map_utils.map_to_world(center_mx, center_my, self.current_map_msg)
        
        return (0.0, 0.0)
    
    def get_robot_yaw(self) -> Optional[float]:
        """获取机器人当前朝向 / Get current robot yaw"""
        return self._nav_executor.get_robot_yaw()
    
    def print_status(self):
        """打印状态信息 / Print status information"""
        if self.current_map is None:
            return
        
        completion = self.known_cells / self.total_cells if self.total_cells > 0 else 0.0
        
        # 获取各模块状态 / Get module statuses
        strategy_status = self._exploration_strategy.get_strategy_status()
        rotation_status = self._rotation_controller.get_rotation_status()
        safety_status = self._safety_manager.get_safety_status()
        
        self.get_logger().info(
            f'Status: completion={completion*100:.2f}%, '
            f'known={self.known_cells}, total={self.total_cells}, '
            f'failures={strategy_status["consecutive_failures"]}/{strategy_status["max_failures"]}, '
            f'rotating={rotation_status["is_rotating"]}, '
            f'safe={safety_status["safety_violations"]}/{safety_status["max_safety_violations"]}'
        )
    
    def _perform_backward_escape(self):
        """执行后退脱困 / Perform backward escape"""
        self.get_logger().info('🔙 Performing backward escape maneuver...')
        
        # 发送后退速度命令 / Send backward velocity command
        cmd = Twist()
        cmd.linear.x = -0.15  # 后退速度 / Backward speed
        cmd.angular.z = 0.0
        
        # 后退1.5秒 / Move backward for 1.5 seconds
        backward_duration = 1.5
        start_time = time.time()
        rate = self.create_rate(20)  # 20Hz
        
        while (time.time() - start_time) < backward_duration:
            # 检查后方是否有障碍物 / Check if obstacle behind
            if self._safety_manager.check_backward_safety(self.local_costmap, self.get_robot_position()):
                self._rotation_controller.cmd_vel_pub.publish(cmd)
            else:
                self.get_logger().warn('Obstacle detected behind, stopping backward escape')
                break
            try:
                rate.sleep()
            except:
                break
        
        # 停止移动 / Stop movement
        cmd.linear.x = 0.0
        self._rotation_controller.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('Backward escape completed')
        
        # 清空访问记录，允许重新尝试之前的frontiers / Clear visit records to retry previous frontiers
        self._exploration_strategy.evaluator.visited_frontiers.clear()
    
    def _terminate_exploration(self, success: bool = True):
        """终止探索 / Terminate exploration"""
        self.exploration_complete = True
        self.get_logger().info(f'Exploration terminated. Success: {success}')
        
        # 发布完成状态 / Publish completion status
        self.completion_pub.publish(Bool(data=success))
        
        # 停止机器人 / Stop robot
        self._rotation_controller.stop_robot()
        
        # 停止定时器 / Stop timers
        self.exploration_timer.cancel()
        self.status_timer.cancel()
        self.get_logger().info('Exploration timers stopped')
        
        # 5秒后关闭节点 / Shutdown node after 5 seconds
        self.create_timer(5.0, self.shutdown_node)
    
    def shutdown_node(self):
        """关闭节点 / Shutdown node"""
        self.get_logger().info('Exploration mapping task completed, node shutting down')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = ExplorationMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received interrupt signal, shutting down...')
    finally:
        node._rotation_controller.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()