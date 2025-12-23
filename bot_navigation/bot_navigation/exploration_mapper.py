#!/usr/bin/env python3
"""
ExplorationMapper - 自主探索建图节点 (重构版本)

功能 / Features:
  - 自动探索未知区域进行快速建图
  - 基于Frontier的探索策略
  - 使用 NavigationExecutor 基类
  - 配置文件驱动
  
Author: LeKiwi Bot Development Team
Date: 2025-12-22 (Refactored)
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
from collections import deque
import math
import time

from .navigation_executor import NavigationExecutor, NavigationState


class ExplorationMapper(Node):
    """自主探索建图节点 (使用 NavigationExecutor)"""
    
    def __init__(self):
        super().__init__('exploration_mapper')
        
    def __init__(self):
        super().__init__('exploration_mapper')
        
        self.get_logger().info('Initializing ExplorationMapper (Refactored)...')
        
        # ===== 参数声明（从配置文件加载） =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('exploration_radius', 5.0),
                ('min_frontier_size', 5),
                ('min_goal_distance', 0.2),
                ('goal_tolerance', 0.3),
                ('rotation_speed', 0.5),
                ('forward_speed', 0.20),
                ('map_completion_threshold', 0.90),
                ('min_completion_threshold', 0.75),  # 最小完成度（提前完成）
                ('safe_distance', 0.4),
                ('camera_fov', 60.0),
                ('goal_timeout', 100.0),
                ('max_failures', 8),  # 增加到8次，避免过早终止
                ('enable_smart_exploration', True),
                ('visit_radius', 0.5),
                ('rotation_angle', 45.0),
                ('max_no_goal_count', 5),  # 增加到5次
            ]
        )
        
        # 获取参数
        self.map_topic = self.get_parameter('map_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
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
        
        # 智能探索参数
        self.enable_smart_exploration = self.get_parameter('enable_smart_exploration').value
        self.visit_radius = self.get_parameter('visit_radius').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.max_no_goal_count = self.get_parameter('max_no_goal_count').value
        
        # ===== NavigationExecutor 初始化 =====
        self._nav_executor = NavigationExecutor(self, 'navigate_to_pose')
        self._nav_executor.set_timeout(self.goal_timeout)
        self._nav_executor.set_safe_distance(self.safe_distance)
        
        # ===== 状态变量 =====
        self.current_map = None
        self.current_map_msg = None  # 保存原始地图消息用于坐标转换
        self.map_resolution = 0.05
        self.map_origin = None
        self.robot_pose = None
        self.is_exploring = False
        self.exploration_complete = False
        self.current_goal = None
        self.goal_start_time = None
        self.total_cells = 0
        self.known_cells = 0
        self.consecutive_failures = 0
        
        # 智能探索状态
        self.last_known_cells = 0
        self.last_goal_direction = None
        self.stagnant_count = 0
        self.max_stagnant = 2
        self.visited_frontiers = []
        self.need_rotation = False
        self.need_360_scan = False  # 标记需要360度扫描
        self.scan_segment_count = 0  # 分段扫描计数器（0-2对应3次100度扫描）
        self.initial_scan_done = False  # 标记是否完成初始扫描
        self.initial_scan_count = 0  # 初始扫描计数器（0-2）
        self.no_goal_count = 0
        
        # 旋转状态机
        self.is_rotating = False
        self.rotation_start_time = None
        self.rotation_duration = 0.0
        self.rotation_angular_vel = 0.0
        self.rotation_target_yaw = None  # 目标朝向（使用实际反馈）
        self.rotation_timeout = 30.0  # 旋转超时时间
        
        # 待处理的目标（旋转完成后发送）
        self.pending_goal = None
        
        # ===== QoS配置 =====
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
        
        # ===== 订阅器 =====
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )
        
        # ===== 发布器 =====
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            cmd_qos
        )
        
        self.completion_pub = self.create_publisher(
            Bool,
            '/exploration/complete',
            10
        )
        
        # ===== Action客户端(仅保留Spin) =====
        self.spin_client = ActionClient(
            self,
            Spin,
            'spin'
        )
        
        # ===== 定时器 =====
        self.exploration_timer = self.create_timer(2.0, self.exploration_loop)
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('🗺️  ExplorationMapper initialized')
        self.get_logger().info(f'   Exploration radius: {self.exploration_radius}m')
        self.get_logger().info(f'   Completion threshold: {self.map_completion_threshold*100}%')
        self.get_logger().info(f'   Using NavigationExecutor for navigation')
    
    def map_callback(self, msg: OccupancyGrid):
        """地图回调 - 更新当前地图"""
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.current_map_msg = msg  # 保存原始消息
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # 更新 NavigationExecutor 的地图
        self._nav_executor.set_map(msg)
        
        # 计算地图完成度（更准确的可探索区域估算）
        # 已知区域 = 自由空间(0-50) + 已知障碍物(>50)
        free_cells = np.sum((self.current_map >= 0) & (self.current_map <= 50))
        obstacle_cells = np.sum(self.current_map > 50)
        self.known_cells = free_cells + obstacle_cells
        
        # 计算真正可探索的未知区域：邻近自由空间的未知格子
        unknown_cells = np.sum(self.current_map == -1)
        
        # 估算可探索的未知区域（靠近自由空间的部分）
        # 简单策略：未知区域不超过已知自由空间的30%
        explorable_unknown = min(unknown_cells, int(free_cells * 0.3))
        
        # 总区域 = 自由空间 + 障碍物 + 可探索未知
        self.total_cells = free_cells + obstacle_cells + explorable_unknown
        
    def world_to_map(self, x, y):
        """世界坐标转地图坐标（使用基类方法）"""
        if self.current_map_msg is None:
            return None
        return self._nav_executor.world_to_map(x, y, self.current_map_msg)
    
    def map_to_world(self, mx, my):
        """地图坐标转世界坐标（使用基类方法）"""
        if self.current_map_msg is None:
            return None
        return self._nav_executor.map_to_world(mx, my, self.current_map_msg)
    
    def is_frontier_cell(self, mx, my):
        """判断是否为边界点 (已知区域与未知区域的交界)"""
        if self.current_map is None:
            return False
        
        height, width = self.current_map.shape
        
        # 边界检查
        if mx < 1 or mx >= width - 1 or my < 1 or my >= height - 1:
            return False
        
        # 当前点必须是未知(-1)
        if self.current_map[my, mx] != -1:
            return False
        
        # 检查8邻域是否有自由空间(0-50)
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor_val = self.current_map[my + dy, mx + dx]
                if 0 <= neighbor_val <= 50:  # 自由空间
                    return True
        
        return False
    
    def find_frontiers(self):
        """寻找所有边界点并聚类"""
        if self.current_map is None or self.map_origin is None:
            return []
        
        height, width = self.current_map.shape
        frontiers = []
        visited = np.zeros((height, width), dtype=bool)
        
        # 扫描所有可能的边界点
        for my in range(1, height - 1):
            for mx in range(1, width - 1):
                if visited[my, mx] or not self.is_frontier_cell(mx, my):
                    continue
                
                # BFS聚类
                cluster = []
                queue = deque([(mx, my)])
                visited[my, mx] = True
                
                while queue:
                    cx, cy = queue.popleft()
                    cluster.append((cx, cy))
                    
                    # 检查4邻域
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = cx + dx, cy + dy
                        if (0 <= nx < width and 0 <= ny < height and 
                            not visited[ny, nx] and self.is_frontier_cell(nx, ny)):
                            visited[ny, nx] = True
                            queue.append((nx, ny))
                
                # 保存足够大的边界
                if len(cluster) >= self.min_frontier_size:
                    frontiers.append(cluster)
        
        self.get_logger().debug(f'找到 {len(frontiers)} 个边界区域')
        return frontiers
    
    def get_robot_position(self):
        """获取机器人当前位置（从NavigationExecutor获取）"""
        try:
            pose_stamped = self._nav_executor.get_robot_pose()
            if pose_stamped:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                self.robot_pose = (x, y)
                return (x, y)
        except Exception as e:
            self.get_logger().warn(f'无法获取机器人位置: {e}', throttle_duration_sec=5.0)
        
        # 降级：使用地图中心
        if self.current_map is not None and self.map_origin is not None:
            height, width = self.current_map.shape
            center_mx = width // 2
            center_my = height // 2
            return self.map_to_world(center_mx, center_my)
        
        return (0.0, 0.0)
    
    def get_robot_yaw(self):
        """获取机器人当前朝向（yaw角，使用基类方法）"""
        return self._nav_executor.get_robot_yaw()
    
    def select_best_frontier(self, frontiers):
        """选择最佳边界目标（智能策略：方向多样性 + 覆盖率优先 + 避免身后目标）"""
        if not frontiers:
            return None
        
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return None
        
        # 获取机器人当前朝向
        robot_yaw = self.get_robot_yaw()
        if robot_yaw is None:
            self.get_logger().warn('无法获取机器人朝向，使用默认角度过滤')
            robot_yaw = 0.0
        
        # 🎯 方案B：根据完成度动态调整frontier选择条件
        completion = self.calculate_map_completion()
        if completion >= self.min_completion_threshold:
            # 接近完成时放宽限制
            max_angle_deg = 90  # 从±60°放宽到±90°
            min_distance = 0.5  # 从1.0m降低到0.5m
            self.get_logger().info(
                f'⚠️  接近完成({completion*100:.2f}%)，放宽选择条件: '
                f'角度范围±{max_angle_deg}°, 最小距离{min_distance}m'
            )
        else:
            max_angle_deg = 60
            min_distance = self.min_goal_distance
        
        max_angle_rad = math.radians(max_angle_deg)
        
        best_frontier = None
        best_score = -float('inf')
        
        self.get_logger().info(
            f'评估 {len(frontiers)} 个边界区域，机器人位置=({robot_pos[0]:.2f}, {robot_pos[1]:.2f}), '
            f'朝向={math.degrees(robot_yaw):.1f}°, 完成度={completion*100:.2f}%, 停滞计数={self.stagnant_count}/{self.max_stagnant}'
        )
        
        for i, frontier in enumerate(frontiers):
            # 计算边界中心
            center_mx = sum(p[0] for p in frontier) / len(frontier)
            center_my = sum(p[1] for p in frontier) / len(frontier)
            center_world = self.map_to_world(center_mx, center_my)
            
            if center_world is None:
                continue
            
            # 距离和方向
            dx = center_world[0] - robot_pos[0]
            dy = center_world[1] - robot_pos[1]
            distance = math.sqrt(dx**2 + dy**2)
            direction = math.atan2(dy, dx)  # 当前frontier的方向角（世界坐标系）
            
            # 计算frontier相对于机器人当前朝向的角度
            relative_angle = direction - robot_yaw
            # 归一化到[-π, π]
            while relative_angle > math.pi:
                relative_angle -= 2 * math.pi
            while relative_angle < -math.pi:
                relative_angle += 2 * math.pi
            
            # 跳过太近或太远的边界（使用动态min_distance）
            if distance < min_distance or distance > self.exploration_radius:
                self.get_logger().info(
                    f'跳过边界{i}: 大小={len(frontier)}, 距离={distance:.2f}m '
                    f'(要求{min_distance}m~{self.exploration_radius}m)'
                )
                continue
            
            # 🎯 关键优化：避免选择身后的frontier（使用动态max_angle）
            # 正常模式：[-60°, +60°]，接近完成：[-90°, +90°]
            if abs(relative_angle) > max_angle_rad:
                self.get_logger().info(
                    f'⚠️  跳过边界{i}: 超出{max_angle_deg*2}度范围 (相对角度={math.degrees(relative_angle):.1f}°), '
                    f'大小={len(frontier)}, 距离={distance:.2f}m'
                )
                continue
            
            # 检查是否已访问过（避免重复尝试同一个frontier）
            # 但如果没有其他选择，允许重新访问（is_last_resort）
            is_visited = False
            visit_distance = float('inf')
            for visited_pos in self.visited_frontiers:
                visit_dist = math.sqrt(
                    (center_world[0] - visited_pos[0])**2 + 
                    (center_world[1] - visited_pos[1])**2
                )
                if visit_dist < self.visit_radius:
                    is_visited = True
                    visit_distance = visit_dist
                    break
            
            if is_visited:
                self.get_logger().info(
                    f'边界{i}已访问过 (距离上次访问{visit_distance:.2f}m < {self.visit_radius}m)，先跳过'
                )
                # 不立即continue，而是标记为已访问，后续如果没有其他选择再考虑
                pass
            
            # 基础评分：大小/距离
            base_score = len(frontier) / max(distance, 0.5)
            
            # 如果已访问过，大幅降低优先级（但不完全排除）
            if is_visited:
                base_score *= 0.1  # 已访问过的frontier评分降低到1/10
            
            # 方向多样性加成
            direction_bonus = 1.0
            if self.last_goal_direction is not None:
                # 计算与上次方向的角度差
                angle_diff = abs(direction - self.last_goal_direction)
                # 归一化到[0, π]
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                
                # 如果停滞了，强烈倾向于相反方向（180度）
                if self.stagnant_count > 0:
                    # angle_diff接近π时加成最大（最多3倍）
                    direction_bonus = 1.0 + 2.0 * (angle_diff / math.pi)
                    self.get_logger().info(
                        f'  停滞模式：方向差={math.degrees(angle_diff):.1f}°, 方向加成={direction_bonus:.2f}x'
                    )
                else:
                    # 正常模式：稍微倾向于不同方向（最多1.5倍）
                    direction_bonus = 1.0 + 0.5 * (angle_diff / math.pi)
            
            # 最终评分
            score = base_score * direction_bonus
            
            self.get_logger().info(
                f'候选边界{i}: 大小={len(frontier)}, 距离={distance:.2f}m, '
                f'相对角度={math.degrees(relative_angle):.1f}°, '
                f'基础分={base_score:.2f}, 方向加成={direction_bonus:.2f}x, 总分={score:.2f}'
            )
            
            if score > best_score:
                best_score = score
                # 将frontier列表也保存，用于后续寻找安全目标点
                best_frontier = (center_world, len(frontier), distance, direction, frontier)
        
        return best_frontier
    
    def find_safe_goal_near_frontier(self, frontier_cells, robot_pos):
        """
        在frontier附近找到一个安全的目标点（已知为free的区域且周围无障碍物）
        
        Args:
            frontier_cells: frontier的栅格坐标列表 [(mx, my), ...]
            robot_pos: 机器人位置 (x, y)
            
        Returns:
            tuple: (safe_x, safe_y) 或 None
        """
        # 🎯 方案C：根据完成度动态调整搜索范围和安全标准
        completion = self.calculate_map_completion()
        if completion >= self.min_completion_threshold:
            search_radius = 2.0  # 扩大搜索范围
            safe_distance = 0.10  # 降低安全标准
            self.get_logger().debug(
                f'接近完成，扩大安全点搜索: 半径={search_radius}m, 安全距离={safe_distance}m'
            )
        else:
            search_radius = 1.5
            safe_distance = self.safe_distance
        
        if self.current_map is None:
            return None
        
        height, width = self.current_map.shape
        
        # 在frontier周围搜索safe点（使用动态search_radius）
        search_radius_cells = int(search_radius / self.map_resolution)  # 转换为栅格数
        safe_candidates = []
        
        # 计算safe_distance对应的栅格数（使用动态safe_distance）
        safe_cells = int(safe_distance / self.map_resolution)
        
        # 遍历frontier的每个点
        for mx, my in frontier_cells:
            # 在frontier点周围搜索free空间
            # 优先选择靠近机器人方向的点
            for offset in [(0, 2), (2, 0), (0, -2), (-2, 0),  # 四个正方向（距离2格）
                          (2, 2), (2, -2), (-2, 2), (-2, -2),  # 四个对角
                          (0, 1), (1, 0), (0, -1), (-1, 0)]:    # 四个正方向（距离1格）
                check_mx = mx + offset[0]
                check_my = my + offset[1]
                
                if check_mx < 0 or check_mx >= width or check_my < 0 or check_my >= height:
                    continue
                
                # 检查该点是否为free（0值）
                if self.current_map[check_my, check_mx] != 0:
                    continue
                
                # 🎯 关键检查：检查该点周围safe_distance范围内是否有障碍物（使用动态safe_distance）
                has_obstacle = False
                for dx in range(-safe_cells, safe_cells + 1):
                    for dy in range(-safe_cells, safe_cells + 1):
                        nx = check_mx + dx
                        ny = check_my + dy
                        
                        if nx < 0 or nx >= width or ny < 0 or ny >= height:
                            continue
                        
                        # 检查是否为障碍物（>50为占据）
                        if self.current_map[ny, nx] > 50:
                            has_obstacle = True
                            break
                    
                    if has_obstacle:
                        break
                
                # 只有周围无障碍物的点才是真正安全的
                if not has_obstacle:
                    # 转换为世界坐标
                    safe_world = self.map_to_world(check_mx, check_my)
                    if safe_world:
                        # 计算到机器人的距离
                        dist = math.sqrt(
                            (safe_world[0] - robot_pos[0])**2 + 
                            (safe_world[1] - robot_pos[1])**2
                        )
                        safe_candidates.append((safe_world, dist))
        
        # 如果找到了safe点，选择距离适中的（前1/3位置）
        if safe_candidates:
            safe_candidates.sort(key=lambda x: x[1])  # 按距离排序
            # 选择中间位置的点，避免过近或过远
            mid_idx = len(safe_candidates) // 3  # 选择前1/3位置的点
            return safe_candidates[mid_idx][0]
        
        # 如果没找到，返回None（使用原来的frontier中心）
        return None
    
    def calculate_yaw_to_target(self, target_x, target_y):
        """计算朝向目标所需的yaw角"""
        robot_pos = self.get_robot_position()
        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        return math.atan2(dy, dx)
    
    def send_navigation_goal(self, x, y, yaw):
        """发送导航目标到Nav2 (使用NavigationExecutor)"""
        # 创建目标pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # 设置朝向
        quat = self.yaw_to_quaternion(yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        
        self.get_logger().info(f'📍 发送目标: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°')
        
        # 使用NavigationExecutor导航
        success = self._nav_executor.navigate_to_pose(goal_pose)
        
        if success:
            self.current_goal = (x, y)
            self.goal_start_time = time.time()
            self.consecutive_failures = 0
            self.get_logger().info('🔄 状态: IDLE -> EXECUTING')
            return True
        else:
            self.get_logger().error('❌ 导航目标发送失败')
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_failures:
                self.get_logger().error(f'❌ 连续{self.consecutive_failures}次失败，探索终止')
                self.exploration_complete = True
                self.completion_pub.publish(Bool(data=False))
            return False
    
    def yaw_to_quaternion(self, yaw):
        """将yaw角转换为四元数"""
        return [
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        ]
    
    def calculate_map_completion(self):
        """计算地图完成度"""
        if self.total_cells == 0:
            return 0.0
        return self.known_cells / self.total_cells
    
    def exploration_loop(self):
        """探索主循环 - 使用NavigationExecutor管理导航状态"""
        if self.exploration_complete:
            return
        
        # 优先处理旋转状态
        if self.update_rotation():
            return  # 还在旋转中，等待下次循环
        
        # 等待地图数据
        if self.current_map is None:
            self.get_logger().info('等待地图数据...', throttle_duration_sec=5.0)
            return
        
        # 检查NavigationExecutor状态
        nav_state = self._nav_executor.get_state()
        
        # 如果正在导航，检查超时
        if nav_state in [NavigationState.EXECUTING, NavigationState.CANCELING]:
            if self.goal_start_time is not None:
                elapsed = time.time() - self.goal_start_time
                if elapsed > self.goal_timeout and nav_state == NavigationState.EXECUTING:
                    self.get_logger().error(
                        f'❌ 目标超时({elapsed:.1f}s > {self.goal_timeout}s)，请求取消'
                    )
                    self._nav_executor.cancel_navigation()
                    self.consecutive_failures += 1
                    self.goal_start_time = None
                    return
            
            # 等待NavigationExecutor完成当前导航
            self.get_logger().debug(
                f'等待导航完成... (状态: {nav_state.value})',
                throttle_duration_sec=2.0
            )
            return
        
        # 检查导航结果
        if nav_state == NavigationState.SUCCESS:
            self.get_logger().info('✅ 导航成功')
            
            # 🎯 优先检查完成度（在设置任何标志前）
            completion = self.calculate_map_completion()
            self.get_logger().info(
                f'🔍 检查完成度: {completion*100:.2f}% (阈值: {self.map_completion_threshold*100:.2f}%)'
            )
            
            if completion >= self.map_completion_threshold:
                self.exploration_complete = True
                self.get_logger().info(f'✅ 地图探索完成! 完成度: {completion*100:.2f}%')
                self.completion_pub.publish(Bool(data=True))
                
                # 停止机器人
                self.stop_robot()
                
                # 停止定时器
                self.exploration_timer.cancel()
                self.status_timer.cancel()
                self.get_logger().info('🛑 探索定时器已停止')
                
                # 5秒后关闭节点
                self.create_timer(5.0, self.shutdown_node)
                return
            
            # 检查探索效果
            new_known_cells = self.known_cells - self.last_known_cells
            self.last_known_cells = self.known_cells
            
            if new_known_cells < 10:
                self.stagnant_count += 1
                self.get_logger().warn(
                    f'⚠️  探索停滞 (新增{new_known_cells}格) '
                    f'[{self.stagnant_count}/{self.max_stagnant}]'
                )
                
                if self.stagnant_count >= self.max_stagnant:
                    self.get_logger().warn('🔄 停滞过多，标记需要旋转')
                    self.need_rotation = True
                    self.stagnant_count = 0
                    self.last_goal_direction = None
                    self.visited_frontiers.clear()
            else:
                self.get_logger().info(f'✅ 探索有效 (新增{new_known_cells}格)')
                self.stagnant_count = 0
                # 只在新增区域较多时才扫描，减少频繁扫描
                if new_known_cells > 50:  # 新增超过50格才扫描
                    self.need_360_scan = True
                    self.get_logger().info('📷 新增区域较多，标记需要扫描')
            
            self.consecutive_failures = 0
            self.goal_start_time = None
            self._nav_executor.reset_state()  # 重置NavigationExecutor状态
        elif nav_state == NavigationState.FAILED:
            self.get_logger().warn(f'⚠️  导航失败 [{self.consecutive_failures + 1}/{self.max_failures}]')
            self.consecutive_failures += 1
            self.stagnant_count += 1
            self.goal_start_time = None
            self._nav_executor.reset_state()  # 重置NavigationExecutor状态
            
            # 失败后尝试小幅旋转再继续
            if self.consecutive_failures < self.max_failures:
                self.need_rotation = True
            
            if self.consecutive_failures >= self.max_failures:
                self.get_logger().error(f'❌ 连续{self.consecutive_failures}次失败，探索终止')
                self.exploration_complete = True
                self.completion_pub.publish(Bool(data=False))
                
                # 停止定时器
                if hasattr(self, 'exploration_timer'):
                    self.exploration_timer.cancel()
                if hasattr(self, 'status_timer'):
                    self.status_timer.cancel()
                self.get_logger().info('🛑 探索定时器已停止')
                
                # 5秒后关闭节点
                self.create_timer(5.0, self.shutdown_node)
                
                return
        
        elif nav_state == NavigationState.CANCELED:
            self.get_logger().warn('⚠️  导航被取消')
            self.consecutive_failures += 1
            self.goal_start_time = None
            self._nav_executor.reset_state()  # 重置NavigationExecutor状态
        
        # 确保在IDLE状态才能处理新任务
        if self._nav_executor.get_state() != NavigationState.IDLE:
            return
        
        # 🎯 最高优先级：检查地图完成度（必须先检查，避免被其他操作阻塞）
        completion = self.calculate_map_completion()
        if completion >= self.map_completion_threshold:
            self.exploration_complete = True
            self.get_logger().info(f'✅ 地图探索完成! 完成度: {completion*100:.2f}%')
            self.completion_pub.publish(Bool(data=True))
            
            # 停止机器人
            self.stop_robot()
            
            # 停止定时器
            self.exploration_timer.cancel()
            self.status_timer.cancel()
            self.get_logger().info('🛑 探索定时器已停止')
            
            # 5秒后关闭节点
            self.create_timer(5.0, self.shutdown_node)
            return
        
        # 检查是否需要执行90度扫描（单次快速扫描）
        if self.need_360_scan:
            self.get_logger().info('📷 执行90度扫描以提高建图质量')
            self.need_360_scan = False
            time.sleep(0.3)
            self.rotate_in_place(angle_degrees=90)  # 单次90度快速扫描
            return
        
        # 检查是否需要执行旋转（脱困模式）
        if self.need_rotation:
            self.get_logger().info(f'🔄 执行旋转操作（脱困模式）')
            self.need_rotation = False
            time.sleep(0.5)
            self.rotate_in_place(angle_degrees=100)  # 100度旋转改变视角
            return
        
        # 程序启动时的初始扫描（3次100度扫描开拓视野）
        if not self.initial_scan_done:
            if self.initial_scan_count < 3:
                self.get_logger().info(
                    f'🔄 初始扫描 [{self.initial_scan_count + 1}/3]: 100度（开拓视野）'
                )
                time.sleep(0.3)
                self.rotate_in_place(angle_degrees=100)
                self.initial_scan_count += 1
                return  # 等待旋转完成
            else:
                self.initial_scan_done = True
                self.get_logger().info('✅ 初始扫描完成，开始自主导航')
                # 继续执行后续逻辑
        
        # 寻找边界
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.no_goal_count += 1
            self.get_logger().warn(
                f'⚠️  未找到可探索的边界区域 [{self.no_goal_count}/{self.max_no_goal_count}]',
                throttle_duration_sec=2.0
            )
            
            # 连续多次没有frontier，认为探索完成
            if self.no_goal_count >= self.max_no_goal_count:
                completion = self.calculate_map_completion()
                self.exploration_complete = True
                self.get_logger().info(f'✅ 探索完成! 无更多可探索区域 (完成度: {completion*100:.1f}%)')
                self.completion_pub.publish(Bool(data=True))
                self.stop_robot()
                
                # 停止定时器
                self.exploration_timer.cancel()
                self.status_timer.cancel()
                self.get_logger().info('🛑 探索定时器已停止')
                
                self.create_timer(5.0, self.shutdown_node)
                return
            
            # 尝试旋转获取更多视野
            if not self.need_rotation:
                self.need_rotation = True
                self.rotation_angle = 45
            return
        
        # 选择最佳目标
        best_frontier = self.select_best_frontier(frontiers)
        
        # 🎯 方案A：接近完成且无有效frontier时提前结束
        if best_frontier is None:
            completion = self.calculate_map_completion()
            if completion >= self.min_completion_threshold:
                self.exploration_complete = True
                self.get_logger().info(
                    f'🎉 探索接近完成({completion*100:.2f}%)且无有效frontier，提前结束'
                )
                self.completion_pub.publish(Bool(data=True))
                self.stop_robot()
                self.exploration_timer.cancel()
                self.status_timer.cancel()
                self.create_timer(5.0, self.shutdown_node)
                return
            
            # 如果没有找到目标，可能所有frontier都太近或已访问过
            self.no_goal_count += 1
            self.get_logger().warn(
                f'⚠️  未找到合适的探索目标 [{self.no_goal_count}/{self.max_no_goal_count}]',
                throttle_duration_sec=1.0
            )
            
            # 连续多次找不到目标，触发旋转
            if self.no_goal_count >= self.max_no_goal_count:
                self.get_logger().warn('🔄 连续多次无法找到目标，设置旋转标志')
                self.need_rotation = True
                self.rotation_angle = 45
                self.no_goal_count = 0
                self.visited_frontiers.clear()
            
            return
        
        # 解包best_frontier（现在包含frontier列表）
        center_pos, frontier_size, distance, direction, frontier_cells = best_frontier
        
        # 🎯 关键优化：在frontier附近找一个安全的free点作为目标
        robot_pos = self.get_robot_position()
        safe_target = self.find_safe_goal_near_frontier(frontier_cells, robot_pos)
        
        if safe_target:
            target_pos = safe_target
            self.get_logger().info(
                f'✅ 找到安全目标点: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) '
                f'(frontier中心: ({center_pos[0]:.2f}, {center_pos[1]:.2f}))'
            )
        else:
            # 如果没找到安全点，检查是否连续失败
            if self.consecutive_failures >= 3:
                # 触发脱困模式：旋转改变视角
                self.get_logger().warn(
                    f'⚠️  连续{self.consecutive_failures}次未找到安全点，'
                    f'触发脱困模式：旋转100度改变视角'
                )
                self.need_rotation = True
                self.consecutive_failures = 0
                return
            
            # 使用frontier中心（fallback）
            target_pos = center_pos
            self.get_logger().warn(
                f'⚠️  未找到安全点，使用frontier中心: ({target_pos[0]:.2f}, {target_pos[1]:.2f})'
            )
        
        # 记录这次的方向和位置
        self.last_goal_direction = direction
        self.visited_frontiers.append(target_pos)
        
        # 只保留最近10个访问记录，避免内存膨胀
        if len(self.visited_frontiers) > 10:
            self.visited_frontiers.pop(0)
        
        # 🎯 关键优化：两步导航策略
        # 第一步：调整朝向让camera frame朝向目标方向
        target_angle = self._nav_executor.calculate_angle_to_target(target_pos[0], target_pos[1])
        
        if target_angle is not None:
            robot_yaw = self.get_robot_yaw()
            if robot_yaw is not None:
                angle_diff = target_angle - robot_yaw
                # 归一化到[-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # 如果角度差超过10度，先旋转对准
                if abs(angle_diff) > math.radians(10):
                    self.get_logger().info(
                        f'🔄 第一步: 调整朝向对准目标 '
                        f'(需转动{math.degrees(angle_diff):.1f}°)'
                    )
                    
                    # 保存目标信息，旋转完成后发送
                    target_yaw = self.calculate_yaw_to_target(target_pos[0], target_pos[1])
                    self.pending_goal = {
                        'position': target_pos,
                        'yaw': target_yaw,
                        'frontier_info': (frontier_size, distance, direction)
                    }
                    
                    # 直接使用cmd_vel旋转
                    angle_to_rotate = math.degrees(angle_diff)
                    self.rotate_in_place(angle_degrees=angle_to_rotate)
                    return  # 等待旋转完成
        
        # 计算朝向目标的yaw角
        target_yaw = self.calculate_yaw_to_target(target_pos[0], target_pos[1])
        
        # 第二步：发送导航目标
        self.get_logger().info(
            f'🎯 第二步: 向目标移动 - 大小={frontier_size}, 距离={distance:.2f}m, '
            f'方向={math.degrees(direction):.1f}°, '
            f'位置=({target_pos[0]:.2f}, {target_pos[1]:.2f})'
        )
        
        # 重置无目标计数器
        self.no_goal_count = 0
        
        self.send_navigation_goal(target_pos[0], target_pos[1], target_yaw)
    
    def rotate_in_place(self, angle_degrees=90):
        """启动原地旋转（非阻塞，基于实际朝向反馈）"""
        if self.is_rotating:
            self.get_logger().warn('已经在旋转中，跳过')
            return
        
        # 获取当前朝向
        current_yaw = self.get_robot_yaw()
        if current_yaw is None:
            self.get_logger().error('无法获取当前朝向，旋转取消')
            return
        
        # 计算目标朝向
        angle_rad = math.radians(angle_degrees)
        self.rotation_target_yaw = current_yaw + angle_rad
        
        # 归一化到[-pi, pi]
        while self.rotation_target_yaw > math.pi:
            self.rotation_target_yaw -= 2 * math.pi
        while self.rotation_target_yaw < -math.pi:
            self.rotation_target_yaw += 2 * math.pi
        
        self.get_logger().info(
            f'🔄 开始旋转{angle_degrees}度: '
            f'当前朝向={math.degrees(current_yaw):.1f}°, '
            f'目标朝向={math.degrees(self.rotation_target_yaw):.1f}°'
        )
        
        # 确定旋转方向
        self.rotation_angular_vel = self.rotation_speed if angle_degrees > 0 else -self.rotation_speed
        
        # 设置旋转状态
        self.is_rotating = True
        self.rotation_start_time = time.time()
    
    def update_rotation(self):
        """更新旋转状态（基于实际朝向反馈）"""
        if not self.is_rotating:
            return False
        
        elapsed = time.time() - self.rotation_start_time
        
        # 检查超时
        if elapsed > self.rotation_timeout:
            self.get_logger().warn(f'⚠️  旋转超时（>{self.rotation_timeout:.1f}秒），强制停止')
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.is_rotating = False
            return False
        
        # 获取当前朝向
        current_yaw = self.get_robot_yaw()
        if current_yaw is None:
            # 无法获取朝向，继续旋转
            twist = Twist()
            twist.angular.z = self.rotation_angular_vel
            self.cmd_vel_pub.publish(twist)
            return True
        
        # 计算角度差
        angle_diff = self.rotation_target_yaw - current_yaw
        # 归一化到[-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 判断是否达到目标（5度容差）
        if abs(angle_diff) < math.radians(5):
            # 旋转完成
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            self.is_rotating = False
            self.rotation_start_time = None
            self.rotation_target_yaw = None
            
            self.get_logger().info(
                f'✅ 旋转完成（耗时{elapsed:.1f}秒，'
                f'当前朝向={math.degrees(current_yaw):.1f}°）'
            )
            
            # 旋转后清空访问记录
            self.visited_frontiers.clear()
            
            # 旋转完成后立即检查完成度
            completion = self.calculate_map_completion()
            if completion >= self.map_completion_threshold:
                self.exploration_complete = True
                self.get_logger().info(f'✅ 地图探索完成! 完成度: {completion*100:.2f}%')
                self.completion_pub.publish(Bool(data=True))
                
                # 停止机器人
                self.stop_robot()
                
                # 停止定时器
                self.exploration_timer.cancel()
                self.status_timer.cancel()
                self.get_logger().info('🛑 探索定时器已停止')
                
                # 5秒后关闭节点
                self.create_timer(5.0, self.shutdown_node)
                return False
            
            # 检查是否有待发送的目标（朝向对准后）
            if self.pending_goal is not None:
                self.get_logger().info('✅ 朝向对准完成，发送导航目标')
                
                goal_info = self.pending_goal
                target_pos = goal_info['position']
                target_yaw = goal_info['yaw']
                frontier_size, distance, direction = goal_info['frontier_info']
                
                self.get_logger().info(
                    f'🎯 第二步: 向目标移动 - 大小={frontier_size}, 距离={distance:.2f}m, '
                    f'方向={math.degrees(direction):.1f}°, '
                    f'位置=({target_pos[0]:.2f}, {target_pos[1]:.2f})'
                )
                
                self.no_goal_count = 0
                self.send_navigation_goal(target_pos[0], target_pos[1], target_yaw)
                
                self.pending_goal = None
            
            return False  # 旋转完成
        else:
            # 继续旋转
            twist = Twist()
            twist.angular.z = self.rotation_angular_vel
            self.cmd_vel_pub.publish(twist)
            
            # 每5秒输出一次进度
            if int(elapsed) % 5 == 0 and elapsed > 0:
                self.get_logger().info(
                    f'🔄 旋转中... 剩余{math.degrees(abs(angle_diff)):.1f}° '
                    f'(已耗时{elapsed:.1f}秒)',
                    throttle_duration_sec=4.9
                )
            
            return True  # 还在旋转中
    
    def _get_camera_yaw(self):
        """获取camera_optical_frame相对于map的yaw角度"""
        try:
            # 使用NavigationExecutor的TF能力
            camera_pose = self._nav_executor.transform_pose(
                PoseStamped(header={'frame_id': 'camera_optical_frame'}),
                'map'
            )
            
            if camera_pose:
                # 从四元数提取yaw角度
                quat = camera_pose.pose.orientation
                yaw = math.atan2(
                    2.0 * (quat.w * quat.z + quat.x * quat.y),
                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
                )
                return yaw
            
        except Exception as e:
            self.get_logger().warn(f'无法获取camera_optical_frame的TF: {e}', throttle_duration_sec=5.0)
        return None
    
    def _get_base_link_yaw(self):
        """获取base_link相对于map的yaw角度"""
        try:
            pose_stamped = self._nav_executor.get_robot_pose()
            if pose_stamped:
                quat = pose_stamped.pose.orientation
                yaw = math.atan2(
                    2.0 * (quat.w * quat.z + quat.x * quat.y),
                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
                )
                return yaw
        except Exception as e:
            self.get_logger().warn(f'无法获取base_link yaw: {e}', throttle_duration_sec=5.0)
        return None
    
    def _rotate_with_cmd_vel(self, angle_degrees):
        """使用cmd_vel直接旋转（降低速度以便RTABMap处理）"""
        # 使用较慢的旋转速度，让RTABMap有时间处理
        slow_rotation_speed = 0.3  # rad/s (原来是0.5)
        
        twist = Twist()
        twist.angular.z = slow_rotation_speed
        
        # 旋转指定角度
        rotation_time = math.radians(angle_degrees) / slow_rotation_speed
        
        self.get_logger().info(f'⏱️  预计旋转时间: {rotation_time:.1f}秒')
        
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        end_time = start_time + rotation_time
        
        # 持续发布旋转命令
        rate_count = 0
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            rate_count += 1
            
            # 每1秒打印一次进度
            if rate_count % 10 == 0:
                elapsed = self.get_clock().now().seconds_nanoseconds()[0] - start_time
                self.get_logger().info(f'⏳ 旋转中... {elapsed:.1f}s / {rotation_time:.1f}s')
        
        self.stop_robot()
        self.get_logger().info('🛑 旋转完成，机器人已停止')
    
    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def print_status(self):
        """打印状态信息"""
        if self.current_map is None:
            return
        
        completion = self.calculate_map_completion()
        
        self.get_logger().info(
            f'📊 状态: 完成度={completion*100:.2f}%, '
            f'已知区域={self.known_cells}, 总区域={self.total_cells}'
        )
    
    def shutdown_node(self):
        """关闭节点"""
        self.get_logger().info('🎉 探索建图任务完成，节点即将关闭')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = ExplorationMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在关闭...')
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
