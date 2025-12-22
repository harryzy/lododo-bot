#!/usr/bin/env python3
"""
自主探索建图节点 - Autonomous Exploration Mapping Node

功能 / Features:
  - 自动探索未知区域进行快速建图
  - 保证相机朝向始终对准运动方向（前方）
  - 基于Frontier的探索策略
  - 地图完成度检测自动结束
  
策略 / Strategy:
  - 相机朝向: camera_optical_frame的Z轴指向base_link的+X（前方）
  - 运动策略: 机器人旋转使+X方向对准目标，然后前进
  - 探索算法: Frontier-based exploration (寻找已知/未知边界)
  
Author: Auto-generated for LeKiwi Bot
Date: 2025-12-11
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose, Spin
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

from tf2_ros import TransformException, Buffer, TransformListener

import numpy as np
from collections import deque
import math
import time


class ExplorationMapper(Node):
    """自主探索建图节点"""
    
    def __init__(self):
        super().__init__('exploration_mapper')
        
        # ===== 参数配置 =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('exploration_radius', 5.0),        # 探索半径(m)
                ('min_frontier_size', 5),           # 最小边界点数
                ('min_goal_distance', 0.2),         # 最小目标距离(m)
                ('goal_tolerance', 0.3),            # 目标容差(m)
                ('rotation_speed', 0.5),            # 旋转速度(rad/s)
                ('forward_speed', 0.20),            # 前进速度(m/s)
                ('map_completion_threshold', 0.90), # 地图完成度阈值
                ('safe_distance', 0.4),             # 安全距离(m)
                ('camera_fov', 60.0),               # 相机FOV(度)
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
        self.safe_distance = self.get_parameter('safe_distance').value
        self.camera_fov_rad = math.radians(self.get_parameter('camera_fov').value)
        
        # ===== 状态变量 =====
        self.current_map = None
        self.map_resolution = 0.05
        self.map_origin = None
        self.robot_pose = None
        self.is_exploring = False
        self.exploration_complete = False
        self.current_goal = None
        self.current_goal_handle = None
        self.goal_result = None
        self.total_cells = 0
        self.known_cells = 0
        self.consecutive_failures = 0
        self.max_failures = 3
        
        # ===== 事务性状态管理 =====
        # 状态机：IDLE -> WAITING_ACCEPT -> EXECUTING -> (SUCCESS/ABORTED/CANCELED) -> IDLE
        self.nav_state = 'IDLE'  # IDLE, WAITING_ACCEPT, EXECUTING, CANCELING
        self.goal_start_time = None
        self.goal_timeout = 80.0  # 目标超时时间(秒)
        self.cancel_in_progress = False  # 正在取消目标
        
        # 智能探索状态
        self.last_known_cells = 0
        self.last_goal_direction = None  # 上次目标的方向角度
        self.stagnant_count = 0  # 停滞计数（到达目标但没有新增已知区域）
        self.max_stagnant = 2    # 最大停滞次数（降低到2次，更快反应）
        self.visited_frontiers = []  # 已访问过的frontier位置（避免重复）
        self.visit_radius = 0.5  # 访问判定半径（米）
        self.need_rotation = False  # 标志：需要在下次循环中执行旋转
        self.rotation_angle = 45  # 旋转角度
        self.no_goal_count = 0  # 连续无法找到目标的次数
        self.max_no_goal = 3  # 连续3次找不到目标就旋转
        
        # TF监听器
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
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
        
        # ===== Action客户端 =====
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.spin_client = ActionClient(
            self,
            Spin,
            'spin'
        )
        
        # ===== 定时器 =====
        self.exploration_timer = self.create_timer(2.0, self.exploration_loop)
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('🗺️  探索建图节点已启动')
        self.get_logger().info(f'   探索半径: {self.exploration_radius}m')
        self.get_logger().info(f'   相机FOV: {self.get_parameter("camera_fov").value}°')
        self.get_logger().info(f'   完成阈值: {self.map_completion_threshold*100}%')
        self.get_logger().info('   相机朝向策略: 始终保持朝向运动方向(+X)')
    
    def map_callback(self, msg: OccupancyGrid):
        """地图回调 - 更新当前地图"""
        self.current_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # 计算地图完成度
        self.total_cells = self.current_map.size
        self.known_cells = np.sum(self.current_map >= 0)  # 非未知区域
        
    def world_to_map(self, x, y):
        """世界坐标转地图坐标"""
        if self.map_origin is None:
            return None
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return (mx, my)
    
    def map_to_world(self, mx, my):
        """地图坐标转世界坐标"""
        if self.map_origin is None:
            return None
        x = mx * self.map_resolution + self.map_origin[0]
        y = my * self.map_resolution + self.map_origin[1]
        return (x, y)
    
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
        """获取机器人当前位置（从TF获取）"""
        # 从TF获取base_link在map中的位置
        try:
            # 获取最新的变换
            transform = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.5)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
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
    
    def select_best_frontier(self, frontiers):
        """选择最佳边界目标（智能策略：方向多样性 + 覆盖率优先）"""
        if not frontiers:
            return None
        
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return None
        
        best_frontier = None
        best_score = -float('inf')
        
        self.get_logger().info(
            f'评估 {len(frontiers)} 个边界区域，机器人位置=({robot_pos[0]:.2f}, {robot_pos[1]:.2f}), '
            f'停滞计数={self.stagnant_count}/{self.max_stagnant}'
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
            direction = math.atan2(dy, dx)  # 当前frontier的方向角
            
            # 跳过太近或太远的边界
            if distance < self.min_goal_distance or distance > self.exploration_radius:
                self.get_logger().info(
                    f'跳过边界{i}: 大小={len(frontier)}, 距离={distance:.2f}m '
                    f'(要求{self.min_goal_distance}m~{self.exploration_radius}m)'
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
                f'方向={math.degrees(direction):.1f}°, '
                f'基础分={base_score:.2f}, 方向加成={direction_bonus:.2f}x, 总分={score:.2f}'
            )
            
            if score > best_score:
                best_score = score
                best_frontier = (center_world, len(frontier), distance, direction)
        
        return best_frontier
    
    def calculate_yaw_to_target(self, target_x, target_y):
        """计算朝向目标所需的yaw角"""
        robot_pos = self.get_robot_position()
        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        return math.atan2(dy, dx)
    
    def send_navigation_goal(self, x, y, yaw):
        """
        发送导航目标 - 事务性：只能在IDLE状态下发送
        关键: yaw设置为朝向目标方向，保证相机(+X)朝前
        """
        # 事务性检查：必须在IDLE状态
        if self.nav_state != 'IDLE':
            self.get_logger().error(
                f'❌ 无法发送目标：当前状态={self.nav_state}，必须在IDLE状态'
            )
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # 关键: 设置yaw朝向目标，因为相机在+X方向
        # 这样机器人会先旋转到正确朝向，然后前进
        quat = self.yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        
        self.get_logger().info(f'📍 发送目标: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°')
        
        # 等待action服务器
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Nav2 action服务器未响应！请确保Nav2已启动')
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_failures:
                self.get_logger().error(f'❌ 连续{self.consecutive_failures}次失败，探索终止')
                self.exploration_complete = True
                self.completion_pub.publish(Bool(data=False))
            return False
        
        # 发送目标（异步）
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.goal_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # 状态转换: IDLE -> WAITING_ACCEPT
        self.current_goal = (x, y)
        self.nav_state = 'WAITING_ACCEPT'
        self.goal_start_time = time.time()
        self.get_logger().info('🔄 状态: IDLE -> WAITING_ACCEPT')
        return True
    
    def yaw_to_quaternion(self, yaw):
        """将yaw角转换为四元数"""
        return [
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        ]
    
    def goal_response_callback(self, future):
        """Goal响应回调 - 事务性：WAITING_ACCEPT -> EXECUTING 或 IDLE"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️  导航目标被拒绝')
            # 状态转换: WAITING_ACCEPT -> IDLE
            self.nav_state = 'IDLE'
            self.goal_start_time = None
            self.consecutive_failures += 1
            self.get_logger().info('🔄 状态: WAITING_ACCEPT -> IDLE (拒绝)')
            return
        
        self.get_logger().info('✅ 导航目标已接受，开始执行')
        self.current_goal_handle = goal_handle
        self.consecutive_failures = 0  # 重置失败计数
        
        # 状态转换: WAITING_ACCEPT -> EXECUTING
        self.nav_state = 'EXECUTING'
        self.get_logger().info('🔄 状态: WAITING_ACCEPT -> EXECUTING')
        
        # 获取结果
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)
    
    def goal_feedback_callback(self, feedback_msg):
        """Goal反馈回调（可选）"""
        # feedback = feedback_msg.feedback
        # 可以在这里处理导航进度反馈
        pass
    
    def goal_result_callback(self, future):
        """Goal结果回调 - 事务性：EXECUTING/CANCELING -> IDLE"""
        result = future.result().result
        status = future.result().status
        
        # 清理状态
        self.current_goal_handle = None
        self.goal_start_time = None
        
        # 状态码: 4=成功, 5=取消, 6=中止
        if status == 4:
            self.get_logger().info('✅ Nav2报告：目标成功')
            
            # 检查是否有新增已知区域
            new_known_cells = self.known_cells - self.last_known_cells
            self.last_known_cells = self.known_cells
            
            if new_known_cells < 10:  # 新增区域太少（小于10个格子）
                self.stagnant_count += 1
                self.get_logger().warn(
                    f'⚠️  导航完成但探索停滞 (新增{new_known_cells}格) '
                    f'[{self.stagnant_count}/{self.max_stagnant}]'
                )
                
                # 停滞太多次，设置旋转标志，在主循环中处理
                if self.stagnant_count >= self.max_stagnant:
                    self.get_logger().warn('🔄 停滞次数过多，标记需要旋转45度寻找新方向')
                    self.need_rotation = True
                    self.rotation_angle = 45
                    self.stagnant_count = 0
                    self.last_goal_direction = None  # 重置方向记忆
                    self.visited_frontiers.clear()  # 清空访问记录，允许重新尝试
            else:
                self.get_logger().info(f'✅ 探索有效 (新增{new_known_cells}格)')
                self.stagnant_count = 0  # 重置停滞计数
            
            self.consecutive_failures = 0
            
        elif status == 5:
            self.get_logger().warn('⚠️  Nav2报告：目标被取消')
            self.cancel_in_progress = False
            self.consecutive_failures += 1
            
        elif status == 6:
            self.get_logger().error('❌ Nav2报告：目标中止（可能遇到障碍）')
            self.consecutive_failures += 1
            self.stagnant_count += 1  # 中止也算停滞
            
        else:
            self.get_logger().warn(f'⚠️  Nav2报告：未知状态码 {status}')
            self.consecutive_failures += 1
        
        # 状态转换: EXECUTING/CANCELING -> IDLE
        old_state = self.nav_state
        self.nav_state = 'IDLE'
        self.get_logger().info(f'🔄 状态: {old_state} -> IDLE (status={status})')
        
        # 检查连续失败次数
        if self.consecutive_failures >= self.max_failures:
            self.get_logger().error(f'❌ 连续{self.consecutive_failures}次失败，探索终止')
            self.exploration_complete = True
            self.completion_pub.publish(Bool(data=False))
    
    def calculate_map_completion(self):
        """计算地图完成度"""
        if self.total_cells == 0:
            return 0.0
        return self.known_cells / self.total_cells
    
    def exploration_loop(self):
        """探索主循环 - 事务性：只在IDLE状态处理新任务"""
        if self.exploration_complete:
            return
        
        # 等待地图数据
        if self.current_map is None:
            self.get_logger().info('等待地图数据...', throttle_duration_sec=5.0)
            return
        
        # 事务性检查1：非IDLE状态，检查超时
        if self.nav_state != 'IDLE':
            if self.goal_start_time is not None:
                elapsed = time.time() - self.goal_start_time
                if elapsed > self.goal_timeout and not self.cancel_in_progress:
                    self.get_logger().error(
                        f'❌ 目标超时({elapsed:.1f}s > {self.goal_timeout}s)，请求取消'
                    )
                    
                    # 事务性取消：EXECUTING -> CANCELING
                    if self.current_goal_handle is not None:
                        self.cancel_in_progress = True
                        self.nav_state = 'CANCELING'
                        self.get_logger().info('🔄 状态: EXECUTING -> CANCELING')
                        
                        cancel_future = self.current_goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(
                            lambda f: self.get_logger().info('✅ 取消请求已发送，等待Nav2确认')
                        )
                    else:
                        # 没有handle但超时，强制重置
                        self.get_logger().warn('⚠️  超时但无goal_handle，强制重置到IDLE')
                        self.nav_state = 'IDLE'
                        self.goal_start_time = None
                    
                    self.consecutive_failures += 1
                    return
            
            # 等待Nav2完成/取消当前目标
            self.get_logger().debug(
                f'等待Nav2完成当前任务... (状态={self.nav_state})',
                throttle_duration_sec=2.0
            )
            return
        
        # 事务性检查2：确保在IDLE状态才能处理新任务
        assert self.nav_state == 'IDLE', f"逻辑错误：状态应为IDLE，实际为{self.nav_state}"
        
        # 检查是否需要执行旋转（在callback中设置的标志）
        if self.need_rotation:
            self.get_logger().info(f'🔄 执行延迟的旋转操作: {self.rotation_angle}度')
            self.need_rotation = False
            # 等待一下，确保Nav2完全释放
            time.sleep(0.5)
            self.rotate_in_place(angle_degrees=self.rotation_angle)
            return
        
        # 检查地图完成度
        completion = self.calculate_map_completion()
        if completion >= self.map_completion_threshold:
            self.exploration_complete = True
            self.get_logger().info(f'✅ 地图探索完成! 完成度: {completion*100:.1f}%')
            self.completion_pub.publish(Bool(data=True))
            
            # 停止机器人
            self.stop_robot()
            
            # 5秒后关闭节点
            self.create_timer(5.0, self.shutdown_node)
            return
        
        # 寻找边界
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().warn('⚠️  未找到可探索的边界区域', throttle_duration_sec=5.0)
            # 尝试旋转一圈以获取更多视野
            if not self.goal_in_progress:
                self.rotate_in_place()
            return
        
        # 选择最佳目标
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier is None:
            # 如果没有找到目标，可能所有frontier都太近或已访问过
            self.no_goal_count += 1
            self.get_logger().warn(
                f'⚠️  未找到合适的探索目标 [{self.no_goal_count}/{self.max_no_goal}]',
                throttle_duration_sec=1.0
            )
            
            # 连续多次找不到目标，触发旋转
            if self.no_goal_count >= self.max_no_goal:
                self.get_logger().warn('🔄 连续多次无法找到目标，设置旋转标志')
                self.need_rotation = True
                self.rotation_angle = 45
                self.no_goal_count = 0
                self.visited_frontiers.clear()
            
            return
        
        target_pos, frontier_size, distance, direction = best_frontier
        
        # 记录这次的方向和位置
        self.last_goal_direction = direction
        self.visited_frontiers.append(target_pos)
        
        # 只保留最近10个访问记录，避免内存膨胀
        if len(self.visited_frontiers) > 10:
            self.visited_frontiers.pop(0)
        
        # 计算朝向目标的yaw角
        target_yaw = self.calculate_yaw_to_target(target_pos[0], target_pos[1])
        
        # 发送导航目标
        self.get_logger().info(
            f'🎯 选定目标: 大小={frontier_size}, 距离={distance:.2f}m, '
            f'方向={math.degrees(direction):.1f}°, '
            f'位置=({target_pos[0]:.2f}, {target_pos[1]:.2f})'
        )
        
        # 重置无目标计数器
        self.no_goal_count = 0
        
        self.send_navigation_goal(target_pos[0], target_pos[1], target_yaw)
    
    def rotate_in_place(self, angle_degrees=90):
        """原地旋转指定角度以获取更多视野（通过发送小圆弧路径实现）"""
        self.get_logger().info(f'🔄 尝试旋转{angle_degrees}度扫描新区域...')
        
        # 获取当前位置和朝向
        robot_x, robot_y = self.get_robot_position()
        current_yaw = self._get_base_link_yaw()
        
        if current_yaw is None:
            self.get_logger().error('❌ 无法获取当前朝向，跳过旋转')
            return
        
        # 计算旋转后的目标yaw
        target_yaw = current_yaw + math.radians(angle_degrees)
        # 归一化到[-π, π]
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
        
        # 计算一个小圆弧上的目标点（半径0.3m），让Nav2规划一条弧形路径
        arc_radius = 0.3  # 0.3米半径的圆弧
        
        # 中间yaw（旋转angle_degrees/2）
        mid_yaw = current_yaw + math.radians(angle_degrees / 2.0)
        
        # 在中间yaw方向上，移动arc_radius距离
        target_x = robot_x + arc_radius * math.cos(mid_yaw)
        target_y = robot_y + arc_radius * math.sin(mid_yaw)
        
        self.get_logger().info(
            f'🎯 发送旋转目标: 位置({target_x:.2f}, {target_y:.2f}), '
            f'朝向从{math.degrees(current_yaw):.1f}°→{math.degrees(target_yaw):.1f}°'
        )
        
        # 发送旋转目标，异步等待完成（由主循环的超时机制管理）
        self.send_navigation_goal(target_x, target_y, target_yaw)
        
        # 旋转后清空访问记录，允许重新尝试之前跳过的frontier
        self.visited_frontiers.clear()
    
    def _get_camera_yaw(self):
        """获取camera_optical_frame相对于map的yaw角度"""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map',
                'camera_optical_frame',
                Time(),
                timeout=Duration(seconds=0.5)
            )
            
            # 从四元数提取yaw角度
            quat = transform.transform.rotation
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
            transform = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.5)
            )
            
            # 从四元数提取yaw角度
            quat = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            return yaw
            
        except Exception as e:
            self.get_logger().warn(f'无法获取base_link的TF: {e}', throttle_duration_sec=5.0)
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
            f'📊 状态: 完成度={completion*100:.1f}%, '
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
