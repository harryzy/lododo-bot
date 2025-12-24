#!/usr/bin/env python3
"""
Rotation Controller - 旋转控制模块 / Rotation Controller Module

负责机器人的旋转控制和避障检测
Responsible for robot rotation control and obstacle avoidance detection

Author: LeKiwi Bot Development Team
Date: 2025-12-23
"""

import math
import time
from typing import Optional, Tuple
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from .exploration_utils import math_utils


class RotationController:
    """旋转控制器类 / Rotation Controller Class"""
    
    def __init__(self, node, cmd_vel_topic: str = '/cmd_vel'):
        """
        初始化旋转控制器 / Initialize rotation controller
        
        Args:
            node: ROS节点 / ROS node
            cmd_vel_topic: 速度命令话题 / Velocity command topic
        """
        self.node = node
        self.cmd_vel_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        
        # 旋转参数 / Rotation parameters
        self.rotation_speed = 0.5
        self.rotation_timeout = 30.0
        self.rotation_tolerance = math.radians(5)  # 5度容差 / 5-degree tolerance
        
        # 旋转状态 / Rotation state
        self.is_rotating_state = False
        self.rotation_start_time = None
        self.rotation_target_yaw = None
        self.rotation_angular_vel = 0.0
        self.current_yaw = None
        self.rotation_completed_successfully = False  # 旋转是否成功完成（非超时/非障碍物中断）/ Whether rotation completed successfully
        
        # 避障检测 / Obstacle avoidance
        self.local_costmap = None
        self.check_obstacle_during_rotation = True
        
        # 待处理的目标 / Pending goal
        self.pending_goal = None
        self.skip_rotation_on_obstacle = False
        self.pre_rotation_point = None  # 预移动点：移动到此点后再旋转 / Pre-rotation point: move here before rotating
        self.pre_rotation_retry_count = 0  # 预移动点重试次数 / Pre-rotation retry count
        self.max_pre_rotation_retries = 2  # 最大重试次数 / Max retry count
        self.after_pre_movement = False  # 是否刚到达预移动点 / Whether just reached pre-movement point
        
        # 脱困模式 / Escape mode
        self.escape_attempt = 0
        self.max_escape_attempts = 7
        
        # 初始扫描 / Initial scan
        self.initial_scan_done = False
        self.initial_scan_count = 0
        self.max_initial_scans = 3
        self.initial_scan_angle = 100
        
        # 连续旋转失败计数器 / Consecutive rotation failure counter
        self.consecutive_rotation_failures = 0
        self.max_consecutive_rotation_failures = 3
    
    def configure(self, config: dict):
        """
        配置旋转参数 / Configure rotation parameters
        
        Args:
            config: 配置字典 / Configuration dictionary
        """
        self.rotation_speed = config.get('rotation_speed', 0.5)
        self.rotation_timeout = config.get('rotation_timeout', 30.0)
        self.rotation_tolerance = math.radians(config.get('rotation_tolerance_deg', 5))
        self.check_obstacle_during_rotation = config.get('check_obstacle_during_rotation', True)
        self.max_escape_attempts = config.get('max_escape_attempts', 7)
        self.initial_scan_angle = config.get('initial_scan_angle', 100)
    
    def update_local_costmap(self, costmap_msg: OccupancyGrid):
        """
        更新局部代价地图 / Update local costmap
        
        Args:
            costmap_msg: 代价地图消息 / Costmap message
        """
        if costmap_msg is not None:
            import numpy as np
            self.local_costmap = np.array(costmap_msg.data).reshape(
                (costmap_msg.info.height, costmap_msg.info.width)
            )
    
    def check_rotation_safety(self) -> bool:
        """
        检查旋转路径是否安全 / Check if rotation path is safe
        
        Returns:
            是否安全 / Whether it's safe
        """
        if not self.check_obstacle_during_rotation or self.local_costmap is None:
            return True
        
        try:
            # 检查机器人周围的costmap / Check costmap around robot
            height, width = self.local_costmap.shape
            center_x = width // 2
            center_y = height // 2
            
            # 如果刚到达预移动点，使用更宽松的检测参数 / Use more tolerant parameters after reaching pre-movement point
            if self.after_pre_movement:
                inner_radius = 6   # 0.3米内必须完全无障碍 / Must be completely clear within 0.3m
                outer_radius = 10  # 0.5米内不能有真障碍物 / No real obstacles within 0.5m
                inner_threshold = 95  # 内圈：只检测真障碍物 / Inner: only real obstacles
                outer_threshold = 100 # 外圈：必须是真障碍物才停止 / Outer: must be real obstacle to stop
            else:
                inner_radius = 6   # 0.3米内必须完全无障碍 / Must be completely clear within 0.3m  
                outer_radius = 10  # 0.5米内不能有真障碍物 / No real obstacles within 0.5m
                inner_threshold = 90  # 内圈：膨胀层也要避开 / Inner: avoid inflation too
                outer_threshold = 100 # 外圈：必须是真障碍物才停止 / Outer: must be real obstacle to stop
            
            # 分层检查：内圈严格，外圈宽松 / Layered check: strict inner, lenient outer
            # local_costmap resolution通常是0.05m / local_costmap resolution is usually 0.05m
            
            for dx in range(-outer_radius, outer_radius + 1):
                for dy in range(-outer_radius, outer_radius + 1):
                    dist_sq = dx*dx + dy*dy
                    if dist_sq > outer_radius*outer_radius:
                        continue
                    
                    cx = center_x + dx
                    cy = center_y + dy
                    
                    if 0 <= cx < width and 0 <= cy < height:
                        cell_value = self.local_costmap[cy, cx]
                        # 内圈使用严格阈值，外圈只检测真障碍物 / Inner circle uses strict threshold, outer only real obstacles
                        threshold = inner_threshold if dist_sq <= inner_radius*inner_radius else outer_threshold
                        
                        if cell_value > threshold:
                            distance_m = math.sqrt(dist_sq) * 0.05  # 转换为米 / Convert to meters
                            self.node.get_logger().warn(
                                f'Detected obstacle at ({dx}, {dy}), distance={distance_m:.2f}m, costmap value={cell_value}, threshold={threshold}',
                                throttle_duration_sec=2.0
                            )
                            return False
            
            return True
            
        except Exception as e:
            self.node.get_logger().error(f'Failed to check rotation safety: {e}')
            return True  # 异常时默认安全 / Default to safe on exception
    
    def start_rotation(self, angle_degrees: float, current_yaw_func, 
                      pending_goal: Optional[dict] = None) -> bool:
        """
        开始旋转 / Start rotation
        
        Args:
            angle_degrees: 旋转角度（度）/ Rotation angle (degrees)
            current_yaw_func: 获取当前yaw角的函数 / Function to get current yaw
            pending_goal: 待处理的目标 / Pending goal
            
        Returns:
            是否成功开始旋转 / Whether rotation started successfully
        """
        if self.is_rotating_state:
            self.node.get_logger().warn('Already rotating, skipping new rotation request')
            return False
        
        # 获取当前朝向 / Get current orientation
        current_yaw = current_yaw_func()
        if current_yaw is None:
            self.node.get_logger().error('Cannot get current yaw, rotation cancelled')
            return False
        
        # 计算目标朝向 / Calculate target orientation
        angle_rad = math.radians(angle_degrees)
        target_yaw = current_yaw + angle_rad
        target_yaw = math_utils.normalize_angle(target_yaw)
        
        self.node.get_logger().info(
            f'Starting rotation of {angle_degrees:.1f}°: '
            f'current yaw={math.degrees(current_yaw):.1f}°, '
            f'target yaw={math.degrees(target_yaw):.1f}°'
        )
        
        # 确定旋转方向 / Determine rotation direction
        self.rotation_angular_vel = self.rotation_speed if angle_degrees > 0 else -self.rotation_speed
        
        # 设置旋转状态 / Set rotation state
        self.is_rotating_state = True
        self.rotation_start_time = time.time()
        self.rotation_target_yaw = target_yaw
        self.current_yaw = current_yaw
        self.pending_goal = pending_goal
        self.rotation_completed_successfully = False  # 重置成功标志 / Reset success flag
        
        # 立即发布第一条旋转命令 / Immediately publish first rotation command
        self._publish_rotation_command()
        
        return True
    
    def update_rotation(self, current_yaw_func) -> bool:
        """
        更新旋转状态 / Update rotation state
        
        Args:
            current_yaw_func: 获取当前yaw角的函数 / Function to get current yaw
            
        Returns:
            是否还在旋转中 / Whether still rotating
        """
        if not self.is_rotating_state:
            return False
        
        elapsed = time.time() - self.rotation_start_time
        
        # 检查超时 / Check timeout
        if elapsed > self.rotation_timeout:
            self.node.get_logger().warn(f'Rotation timeout (> {self.rotation_timeout:.1f}s), forcing stop')
            self._stop_rotation()
            self.rotation_completed_successfully = False  # 超时不算成功完成 / Timeout is not successful completion
            
            # 清理pending_goal避免卡死 / Clear pending_goal to avoid deadlock
            if self.pending_goal is not None:
                self.node.get_logger().warn('Rotation timeout, clearing pending_goal')
                self.pending_goal = None
            
            return False
        
        # 🛡️ 检查旋转路径安全性 / Check rotation path safety
        if not self.check_rotation_safety():
            self.consecutive_rotation_failures += 1
            self.node.get_logger().warn(f'Detected obstacle during rotation, stopping rotation (failure #{self.consecutive_rotation_failures})')
            self._stop_rotation()
            self.rotation_completed_successfully = False  # 障碍物中断不算成功完成 / Obstacle interruption is not successful completion
            self.after_pre_movement = False  # 遇到障碍时重置标志 / Reset flag when obstacle detected
            
            # 🎯 设置障碍物标志，让主节点决定如何处理
            # Set obstacle flag, let main node decide how to handle
            # 注意：即使pending_goal为None（如完成度扫描），也需要设置标志
            # Note: Even if pending_goal is None (e.g., completion scan), still set flag
            self.skip_rotation_on_obstacle = True
            
            # 如果有pending_goal，提示会尝试找安全点 / If has pending_goal, hint will try to find safe point
            if self.pending_goal is not None:
                self.node.get_logger().info('Obstacle blocking rotation, will find a safe point to move before rotating')
            
            return False
        
        # 获取当前朝向 / Get current orientation
        current_yaw = current_yaw_func()
        if current_yaw is None:
            # 无法获取朝向，继续旋转 / Cannot get orientation, continue rotating
            self.node.get_logger().debug(
                f'Cannot get current yaw, continuing rotation (elapsed: {elapsed:.1f}s)',
                throttle_duration_sec=2.0
            )
            self._publish_rotation_command()
            return True
        
        # 计算角度差 / Calculate angle difference
        angle_diff = self.rotation_target_yaw - current_yaw
        angle_diff = math_utils.normalize_angle(angle_diff)
        
        # 判断是否达到目标（容差范围内）/ Check if target reached (within tolerance)
        if abs(angle_diff) < self.rotation_tolerance:
            # 旋转正常完成 / Rotation completed successfully
            self._stop_rotation()
            self.rotation_completed_successfully = True  # 标记为成功完成 / Mark as successfully completed
            self.after_pre_movement = False  # 旋转完成后重置标志 / Reset flag after rotation completes
            self.consecutive_rotation_failures = 0  # 重置失败计数 / Reset failure counter
            self.consecutive_rotation_failures = 0  # 重置失败计数 / Reset failure counter
            
            self.node.get_logger().info(
                f'Rotation completed (elapsed: {elapsed:.1f}s, '
                f'current yaw={math.degrees(current_yaw):.1f}°)'
            )
            
            # 旋转后清空访问记录 / Clear visit records after rotation
            self.evaluator.visited_frontiers.clear() if hasattr(self, 'evaluator') else None
            
            # 检查初始扫描是否全部完成 / Check if initial scan is fully completed
            if self.initial_scan_count >= self.max_initial_scans and not self.initial_scan_done:
                self.initial_scan_done = True
                self.node.get_logger().info('Initial scan completed, starting autonomous navigation')
            
            # 检查是否有待发送的目标 / Check if there's a pending goal
            if self.pending_goal is not None:
                self.node.get_logger().info('Orientation aligned, navigation goal will be sent by main node')
                # 主节点会在exploration_loop中处理pending_goal发送 / Main node handles goal sending in exploration_loop
                # 这里不清空pending_goal，由主节点的_send_pending_goal()负责清空
                # Don't clear pending_goal here, let main node's _send_pending_goal() handle it
            
            # 清空rotation_controller内部的pending_goal引用
            # Clear rotation_controller's internal pending_goal reference
            self.pending_goal = None
            
            return False  # 旋转完成 / Rotation complete
        else:
            # 继续旋转 / Continue rotating
            self._publish_rotation_command()
            
            # 每5秒输出一次进度 / Output progress every 5 seconds
            if int(elapsed) % 5 == 0 and elapsed > 0:
                self.node.get_logger().info(
                    f'Rotating... remaining {math.degrees(abs(angle_diff)):.1f}° '
                    f'(elapsed: {elapsed:.1f}s)',
                    throttle_duration_sec=4.9
                )
            
            return True  # 还在旋转中 / Still rotating
    
    def _publish_rotation_command(self):
        """发布旋转命令 / Publish rotation command"""
        twist = Twist()
        twist.angular.z = self.rotation_angular_vel
        self.cmd_vel_pub.publish(twist)
        
        self.node.get_logger().debug(f'Publishing rotation command: angular.z={self.rotation_angular_vel:.2f}')
    
    def _stop_rotation(self):
        """停止旋转 / Stop rotation"""
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.is_rotating_state = False
        self.rotation_start_time = None
        self.rotation_target_yaw = None
        self.skip_rotation_on_obstacle = False
    
    def stop_robot(self):
        """停止机器人 / Stop robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # 如果正在旋转，也停止旋转 / If rotating, also stop rotation
        if self.is_rotating_state:
            self._stop_rotation()
    
    def start_escape_rotation(self, current_yaw_func, pending_goal: Optional[dict] = None) -> bool:
        """
        开始脱困旋转 / Start escape rotation
        
        Args:
            current_yaw_func: 获取当前yaw角的函数 / Function to get current yaw
            pending_goal: 待处理的目标 / Pending goal
            
        Returns:
            是否开始旋转 / Whether rotation started
        """
        if self.escape_attempt >= self.max_escape_attempts:
            self.node.get_logger().error('Escape failed: all directions tried without finding valid frontier')
            self.escape_attempt = 0
            return False
        
        # 脱困模式：每45度尝试，从45度到315度 / Escape mode: try every 45 degrees, from 45° to 315°
        escape_angle = 45 + self.escape_attempt * 45  # 45, 90, 135, 180, 225, 270, 315
        self.escape_attempt += 1
        
        self.node.get_logger().info(
            f'Escape attempt [{self.escape_attempt}/{self.max_escape_attempts}]: '
            f'rotating {escape_angle}° for exploration'
        )
        
        return self.start_rotation(escape_angle, current_yaw_func, pending_goal)
    
    def start_initial_scan(self, current_yaw_func) -> bool:
        """
        开始初始扫描 / Start initial scan
        
        Args:
            current_yaw_func: 获取当前yaw角的函数 / Function to get current yaw
            
        Returns:
            是否开始扫描 / Whether scan started
        """
        if self.initial_scan_done or self.initial_scan_count >= self.max_initial_scans:
            return False
        
        self.node.get_logger().info(
            f'Initial scan [{self.initial_scan_count + 1}/{self.max_initial_scans}]: '
            f'{self.initial_scan_angle}° (expanding field of view)'
        )
        
        result = self.start_rotation(self.initial_scan_angle, current_yaw_func)
        if result:
            # 只增加计数，不在这里判断完成（旋转是异步的）
            # Only increment count, don't check completion here (rotation is async)
            self.initial_scan_count += 1
        
        return result
    
    def find_nearest_unknown_direction(self, robot_pos: Tuple[float, float], robot_yaw: float,
                                     current_map, map_msg) -> float:
        """
        寻找最近未知区域的方向 / Find direction to nearest unknown area
        
        Args:
            robot_pos: 机器人位置 / Robot position
            robot_yaw: 机器人朝向 / Robot yaw
            current_map: 当前地图数据 / Current map data
            map_msg: 地图消息 / Map message
            
        Returns:
            最佳扫描角度 / Best scan angle
        """
        if current_map is None or robot_pos is None or robot_yaw is None:
            return 90.0  # 默认90度 / Default 90 degrees
        
        height, width = current_map.shape
        
        # 在机器人周围360度范围内，每10度检测一次未知区域密度
        # In 360° range around robot, detect unknown area density every 10°
        best_angle = 90.0
        max_unknown_density = 0.0
        search_radius = 3.0  # 搜索半径3米 / Search radius 3 meters
        
        for angle_offset in range(-180, 180, 10):  # 每10度检测一次 / Detect every 10°
            angle_rad = robot_yaw + math.radians(angle_offset)
            unknown_count = 0
            total_count = 0
            
            # 沿着这个方向检测 / Detect along this direction
            for dist in [1.0, 1.5, 2.0, 2.5, 3.0]:  # 检测多个距离点 / Detect multiple distance points
                check_x = robot_pos[0] + dist * math.cos(angle_rad)
                check_y = robot_pos[1] + dist * math.sin(angle_rad)
                
                # 转换为栅格坐标 / Convert to grid coordinates
                from .exploration_utils import coordinate_converter
                check_coords = coordinate_converter.world_to_map(check_x, check_y, map_msg)
                if check_coords is None:
                    continue
                
                check_mx, check_my = check_coords
                
                # 检查9x9邻域 / Check 9x9 neighborhood
                for dx in range(-4, 5):
                    for dy in range(-4, 5):
                        nx = check_mx + dx
                        ny = check_my + dy
                        
                        if 0 <= nx < width and 0 <= ny < height:
                            total_count += 1
                            if current_map[ny, nx] == -1:  # 未知区域 / Unknown area
                                unknown_count += 1
            
            # 计算未知区域密度 / Calculate unknown area density
            if total_count > 0:
                density = unknown_count / total_count
                if density > max_unknown_density:
                    max_unknown_density = density
                    best_angle = float(angle_offset)
        
        self.node.get_logger().info(
            f'Found nearest unknown area direction: {best_angle}° '
            f'(unknown density: {max_unknown_density*100:.1f}%)'
        )
        
        return best_angle
    
    @property
    def is_rotating(self) -> bool:
        """是否正在旋转 / Whether currently rotating"""
        return getattr(self, 'is_rotating_state', False)
    
    def get_rotation_status(self) -> dict:
        """
        获取旋转状态 / Get rotation status
        
        Returns:
            旋转状态字典 / Rotation status dictionary
        """
        return {
            'is_rotating': self.is_rotating_state,
            'escape_attempt': self.escape_attempt,
            'max_escape_attempts': self.max_escape_attempts,
            'initial_scan_done': self.initial_scan_done,
            'initial_scan_count': self.initial_scan_count,
            'skip_rotation_on_obstacle': self.skip_rotation_on_obstacle,
            'has_pending_goal': self.pending_goal is not None
        }
    
    def reset_escape_attempts(self):
        """重置脱困尝试计数 / Reset escape attempt count"""
        self.escape_attempt = 0
    
    def set_evaluator(self, evaluator):
        """设置评估器 / Set evaluator"""
        self.evaluator = evaluator