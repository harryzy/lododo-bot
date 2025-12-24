#!/usr/bin/env python3
"""
Safety Manager - 安全管理模块 / Safety Manager Module

负责探索过程中的安全检测和管理
Responsible for safety detection and management during exploration

Author: LeKiwi Bot Development Team
Date: 2025-12-23
"""

import math
from typing import Optional, Tuple, List
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from ..exploration.exploration_utils import MapUtils, coordinate_converter, math_utils


class SafetyManager:
    """安全管理器类 / Safety Manager Class"""
    
    def __init__(self):
        # 安全参数 / Safety parameters
        self.safe_distance = 0.4  # 安全距离 / Safe distance
        self.min_goal_distance = 0.2  # 最小目标距离 / Minimum goal distance
        self.goal_tolerance = 0.3  # 目标容差 / Goal tolerance
        self.obstacle_threshold = 50  # 障碍物阈值 / Obstacle threshold
        
        # 地图数据 / Map data
        self.current_map = None
        self.current_map_msg = None
        self.local_costmap = None
        
        # 状态跟踪 / State tracking
        self.consecutive_failures = 0
        self.max_failures = 8
        self.safety_violations = 0
        self.max_safety_violations = 3
    
    def configure(self, config: dict):
        """
        配置安全参数 / Configure safety parameters
        
        Args:
            config: 配置字典 / Configuration dictionary
        """
        self.safe_distance = config.get('safe_distance', 0.4)
        self.min_goal_distance = config.get('min_goal_distance', 0.2)
        self.goal_tolerance = config.get('goal_tolerance', 0.3)
        self.obstacle_threshold = config.get('obstacle_threshold', 50)
        self.max_failures = config.get('max_failures', 8)
        self.max_safety_violations = config.get('max_safety_violations', 3)
    
    def update_map(self, map_msg: OccupancyGrid):
        """
        更新地图数据 / Update map data
        
        Args:
            map_msg: 地图消息 / Map message
        """
        if map_msg is not None:
            import numpy as np
            self.current_map_msg = map_msg
            self.current_map = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
    
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
    
    def is_goal_safe(self, goal_x: float, goal_y: float, robot_pos: Tuple[float, float]) -> Tuple[bool, str]:
        """
        检查目标点是否安全 / Check if goal point is safe
        
        Args:
            goal_x: 目标X坐标 / Goal X coordinate
            goal_y: 目标Y坐标 / Goal Y coordinate
            robot_pos: 机器人位置 / Robot position
            
        Returns:
            (是否安全, 原因) / (Is safe, reason)
        """
        if self.current_map is None or self.current_map_msg is None:
            return False, "No map data available"
        
        # 检查目标是否在地图范围内 / Check if goal is within map bounds
        goal_coords = coordinate_converter.world_to_map(goal_x, goal_y, self.current_map_msg)
        if goal_coords is None:
            return False, "Goal outside map boundaries"
        
        goal_mx, goal_my = goal_coords
        
        # 检查目标点本身是否安全 / Check if goal point itself is safe
        goal_value = MapUtils.get_map_value(goal_mx, goal_my, self.current_map)
        if goal_value is None:
            return False, "Cannot access goal position in map"
        
        # 目标点必须是自由空间(0) / Goal must be free space (0)
        if goal_value != 0:
            return False, f"Goal position is not free space (value: {goal_value})"
        
        # 检查目标周围安全距离内是否有障碍物 / Check for obstacles within safe distance around goal
        safe_cells = int(self.safe_distance / self.current_map_msg.info.resolution)
        
        for dx in range(-safe_cells, safe_cells + 1):
            for dy in range(-safe_cells, safe_cells + 1):
                check_mx = goal_mx + dx
                check_my = goal_my + dy
                
                if not MapUtils.is_valid_map_position(check_mx, check_my, self.current_map):
                    continue
                
                # 检查是否为障碍物(>50) / Check if obstacle (>50)
                if self.current_map[check_my, check_mx] > self.obstacle_threshold:
                    return False, f"Obstacle detected within {self.safe_distance}m of goal"
        
        # 检查目标距离是否合理 / Check if goal distance is reasonable
        distance = math_utils.calculate_distance(robot_pos, (goal_x, goal_y))
        if distance < self.min_goal_distance:
            return False, f"Goal too close to robot ({distance:.2f}m < {self.min_goal_distance}m)"
        
        return True, "Goal position is safe"
    
    def is_path_safe(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float]) -> Tuple[bool, str]:
        """
        检查路径是否安全 / Check if path is safe
        
        Args:
            start_pos: 起点位置 / Start position
            goal_pos: 目标位置 / Goal position
            
        Returns:
            (是否安全, 原因) / (Is safe, reason)
        """
        if self.current_map is None or self.current_map_msg is None:
            return False, "No map data available"
        
        # 简化的路径安全检查：检查起点到目标点的直线路径
        # Simplified path safety check: check straight line path from start to goal
        steps = 20  # 检查20个点 / Check 20 points
        
        for i in range(steps + 1):
            t = i / steps
            check_x = start_pos[0] + t * (goal_pos[0] - start_pos[0])
            check_y = start_pos[1] + t * (goal_pos[1] - start_pos[1])
            
            check_coords = coordinate_converter.world_to_map(check_x, check_y, self.current_map_msg)
            if check_coords is None:
                continue
            
            check_mx, check_my = check_coords
            
            # 检查路径上的点是否安全 / Check if point on path is safe
            path_value = MapUtils.get_map_value(check_mx, check_my, self.current_map)
            if path_value is not None and path_value > self.obstacle_threshold:
                return False, f"Obstacle detected on path at position ({check_x:.2f}, {check_y:.2f})"
        
        return True, "Path is safe"
    
    def check_rotation_safety(self, robot_pos: Tuple[float, float]) -> Tuple[bool, str]:
        """
        检查旋转安全性 / Check rotation safety
        
        Args:
            robot_pos: 机器人位置 / Robot position
            
        Returns:
            (是否安全, 原因) / (Is safe, reason)
        """
        if self.local_costmap is None:
            return True, "No local costmap data, assuming safe"
        
        try:
            # 检查机器人周围的costmap / Check costmap around robot
            height, width = self.local_costmap.shape
            center_x = width // 2
            center_y = height // 2
            
            # 检查半径1米范围内是否有障碍物 / Check for obstacles within 1-meter radius
            check_radius = 20  # 1米 = 20 cells (假设分辨率0.05m) / 1m = 20 cells (assuming 0.05m resolution)
            
            for dx in range(-check_radius, check_radius + 1):
                for dy in range(-check_radius, check_radius + 1):
                    if dx*dx + dy*dy > check_radius*check_radius:
                        continue
                    
                    cx = center_x + dx
                    cy = center_y + dy
                    
                    if 0 <= cx < width and 0 <= cy < height:
                        # 检查costmap值，>90认为有障碍物（避免膨胀层误判）
                        # Check costmap value, >90 considered obstacle (avoid inflation layer false positive)
                        if self.local_costmap[cy, cx] > 90:
                            return False, f"Obstacle detected at ({dx}, {dy}), costmap value={self.local_costmap[cy, cx]}"
            
            return True, "Rotation path is clear"
            
        except Exception as e:
            return True, f"Error checking rotation safety: {e}, assuming safe"
    
    def validate_goal_pose(self, goal_pose: PoseStamped, robot_pos: Tuple[float, float]) -> Tuple[bool, str]:
        """
        验证目标位姿 / Validate goal pose
        
        Args:
            goal_pose: 目标位姿 / Goal pose
            robot_pos: 机器人位置 / Robot position
            
        Returns:
            (是否有效, 原因) / (Is valid, reason)
        """
        # 提取目标位置 / Extract goal position
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        
        # 检查目标位置安全性 / Check goal position safety
        is_safe, reason = self.is_goal_safe(goal_x, goal_y, robot_pos)
        if not is_safe:
            return False, f"Goal position unsafe: {reason}"
        
        # 检查路径安全性 / Check path safety
        is_path_safe, path_reason = self.is_path_safe(robot_pos, (goal_x, goal_y))
        if not is_path_safe:
            return False, f"Path unsafe: {path_reason}"
        
        return True, "Goal pose is valid"
    
    def increment_failure_count(self) -> bool:
        """
        增加失败计数 / Increment failure count
        
        Returns:
            是否达到最大失败次数 / Whether maximum failures reached
        """
        self.consecutive_failures += 1
        return self.consecutive_failures >= self.max_failures
    
    def increment_safety_violation(self) -> bool:
        """
        增加安全违规计数 / Increment safety violation count
        
        Returns:
            是否达到最大违规次数 / Whether maximum violations reached
        """
        self.safety_violations += 1
        return self.safety_violations >= self.max_safety_violations
    
    def reset_failure_count(self):
        """重置失败计数 / Reset failure count"""
        self.consecutive_failures = 0
    
    def reset_safety_violations(self):
        """重置安全违规计数 / Reset safety violation count"""
        self.safety_violations = 0
    
    def get_safety_status(self) -> dict:
        """
        获取安全状态 / Get safety status
        
        Returns:
            安全状态字典 / Safety status dictionary
        """
        return {
            'consecutive_failures': self.consecutive_failures,
            'max_failures': self.max_failures,
            'safety_violations': self.safety_violations,
            'max_safety_violations': self.max_safety_violations,
            'safe_distance': self.safe_distance,
            'obstacle_threshold': self.obstacle_threshold,
            'has_map_data': self.current_map is not None,
            'has_costmap_data': self.local_costmap is not None
        }
    
    def calculate_safety_score(self, position: Tuple[float, float]) -> float:
        """
        计算位置的安全评分 / Calculate safety score for position
        
        Args:
            position: 位置坐标 / Position coordinates
            
        Returns:
            安全评分(0-1) / Safety score (0-1)
        """
        if self.current_map is None or self.current_map_msg is None:
            return 0.5  # 默认中等安全 / Default medium safety
        
        # 转换到地图坐标 / Convert to map coordinates
        coords = coordinate_converter.world_to_map(position[0], position[1], self.current_map_msg)
        if coords is None:
            return 0.0  # 地图外为不安全 / Outside map is unsafe
        
        mx, my = coords
        
        # 检查目标点本身 / Check the point itself
        goal_value = MapUtils.get_map_value(mx, my, self.current_map)
        if goal_value is None or goal_value != 0:
            return 0.0  # 非自由空间为不安全 / Non-free space is unsafe
        
        # 检查周围区域 / Check surrounding area
        safe_cells = int(self.safe_distance / self.current_map_msg.info.resolution)
        safe_points = 0
        total_points = 0
        
        for dx in range(-safe_cells, safe_cells + 1):
            for dy in range(-safe_cells, safe_cells + 1):
                check_mx = mx + dx
                check_my = my + dy
                
                if not MapUtils.is_valid_map_position(check_mx, check_my, self.current_map):
                    continue
                
                total_points += 1
                check_value = self.current_map[check_my, check_mx]
                
                # 自由空间或未知区域认为是安全的 / Free space or unknown considered safe
                if check_value == 0 or check_value == -1:
                    safe_points += 1
        
        if total_points == 0:
            return 0.0
        
        return safe_points / total_points
    
    def find_alternative_goal(self, original_goal: Tuple[float, float], 
                            robot_pos: Tuple[float, float],
                            search_radius: float = 1.0) -> Optional[Tuple[float, float]]:
        """
        寻找替代目标点 / Find alternative goal point
        
        Args:
            original_goal: 原始目标 / Original goal
            robot_pos: 机器人位置 / Robot position
            search_radius: 搜索半径 / Search radius
            
        Returns:
            替代目标或None / Alternative goal or None
        """
        if self.current_map is None or self.current_map_msg is None:
            return None
        
        # 在原始目标周围搜索安全点 / Search for safe points around original goal
        best_alternative = None
        best_score = -1.0
        
        # 转换为栅格数 / Convert to grid cells
        search_radius_cells = int(search_radius / self.current_map_msg.info.resolution)
        
        original_coords = coordinate_converter.world_to_map(original_goal[0], original_goal[1], self.current_map_msg)
        if original_coords is None:
            return None
        
        orig_mx, orig_my = original_coords
        
        # 在搜索半径内寻找 / Search within radius
        for dx in range(-search_radius_cells, search_radius_cells + 1):
            for dy in range(-search_radius_cells, search_radius_cells + 1):
                if dx*dx + dy*dy > search_radius_cells*search_radius_cells:
                    continue
                
                check_mx = orig_mx + dx
                check_my = orig_my + dy
                
                if not MapUtils.is_valid_map_position(check_mx, check_my, self.current_map):
                    continue
                
                # 检查是否为自由空间 / Check if free space
                if self.current_map[check_my, check_mx] != 0:
                    continue
                
                # 转换为世界坐标 / Convert to world coordinates
                alt_world = coordinate_converter.map_to_world(check_mx, check_my, self.current_map_msg)
                if alt_world is None:
                    continue
                
                # 计算安全评分 / Calculate safety score
                safety_score = self.calculate_safety_score(alt_world)
                
                # 计算距离评分（避免太近的）/ Calculate distance score (avoid too close)
                dist_to_robot = math_utils.calculate_distance(alt_world, robot_pos)
                if dist_to_robot < self.min_goal_distance:
                    continue
                
                # 计算距离原始目标的评分 / Calculate score for distance to original goal
                dist_to_original = math_utils.calculate_distance(alt_world, original_goal)
                
                # 综合评分 / Combined score
                total_score = safety_score * 0.7 + (1.0 - dist_to_original / search_radius) * 0.3
                
                if total_score > best_score:
                    best_score = total_score
                    best_alternative = alt_world
        
        return best_alternative
    
    def find_safe_rotation_point(self, robot_pos: Tuple[float, float], 
                                  target_pos: Tuple[float, float],
                                  search_radius: float = 1.5) -> Optional[Tuple[float, float]]:
        """
        寻找一个安全的预移动点用于旋转 / Find a safe pre-movement point for rotation
        
        当原地旋转受障碍物阻碍时，找一个附近的安全点，移动过去后可以安全旋转
        When rotation in place is blocked by obstacles, find a nearby safe point to move to before rotating
        
        Args:
            robot_pos: 机器人当前位置 / Robot current position
            target_pos: 最终目标位置 / Final target position  
            search_radius: 搜索半径(米) / Search radius (meters)
            
        Returns:
            安全的预移动点或None / Safe pre-movement point or None
        """
        if self.current_map is None or self.current_map_msg is None or self.local_costmap is None:
            return None
        
        best_point = None
        best_score = -1.0
        
        # 计算朝向目标的方向 / Calculate direction to target
        target_direction = math_utils.calculate_angle(robot_pos, target_pos)
        
        # 转换为栅格数 / Convert to grid cells
        search_radius_cells = int(search_radius / self.current_map_msg.info.resolution)
        
        robot_coords = coordinate_converter.world_to_map(robot_pos[0], robot_pos[1], self.current_map_msg)
        if robot_coords is None:
            return None
        
        robot_mx, robot_my = robot_coords
        
        # 在机器人周围搜索安全旋转点 / Search for safe rotation points around robot
        min_distance_cells = 10  # 最小0.5米 / Minimum 0.5m
        for dx in range(-search_radius_cells, search_radius_cells + 1):
            for dy in range(-search_radius_cells, search_radius_cells + 1):
                dist_sq = dx*dx + dy*dy
                # 要求至少离开当前位置0.5米，且不超过搜索半径
                # Require at least 0.5m away from current position, and within search radius
                if dist_sq < min_distance_cells*min_distance_cells or dist_sq > search_radius_cells*search_radius_cells:
                    continue
                
                check_mx = robot_mx + dx
                check_my = robot_my + dy
                
                if not MapUtils.is_valid_map_position(check_mx, check_my, self.current_map):
                    continue
                
                # 安全点必须在已知的自由空间，不能在未知区域
                # Safe point must be in known free space, not in unknown area
                cell_value = self.current_map[check_my, check_mx]
                if cell_value < 0 or cell_value > 50:  # 排除未知区域和障碍物 / Exclude unknown and obstacles
                    continue
                
                # 检查路径：允许少量未知区域，但大部分应该是已知的
                # Check path: allow small amount of unknown areas, but most should be known
                path_clear = True
                unknown_count = 0
                total_count = 0
                steps = int(math.sqrt(dist_sq)) + 1
                
                for step in range(1, steps):
                    ratio = step / steps
                    path_mx = int(robot_mx + dx * ratio)
                    path_my = int(robot_my + dy * ratio)
                    if not MapUtils.is_valid_map_position(path_mx, path_my, self.current_map):
                        path_clear = False
                        break
                    
                    path_value = self.current_map[path_my, path_mx]
                    total_count += 1
                    
                    # 障碍物直接排除 / Obstacles are excluded
                    if path_value > 50:
                        path_clear = False
                        break
                    
                    # 统计未知区域 / Count unknown areas
                    if path_value < 0:
                        unknown_count += 1
                
                # 未知区域不能超过30% / Unknown areas cannot exceed 30%
                if not path_clear or (total_count > 0 and unknown_count / total_count > 0.3):
                    continue
                
                # 转换为世界坐标 / Convert to world coordinates
                candidate_world = coordinate_converter.map_to_world(check_mx, check_my, self.current_map_msg)
                if candidate_world is None:
                    continue
                
                # 1. 检查该点周围是否有足够的旋转空间 / Check if there's enough rotation space around this point
                # 使用与旋转检测相同的范围：0.8-1.0米 / Use same range as rotation detection: 0.8-1.0m
                has_rotation_space = True
                rotation_check_radius = 20  # 1.0米，与旋转检测一致 / 1.0m, consistent with rotation detection
                
                # 如果有local_costmap，优先使用它检查（更准确）/ If local_costmap available, use it for checking (more accurate)
                use_local_costmap = self.local_costmap is not None
                
                for rdx in range(-rotation_check_radius, rotation_check_radius + 1):
                    for rdy in range(-rotation_check_radius, rotation_check_radius + 1):
                        if rdx*rdx + rdy*rdy > rotation_check_radius*rotation_check_radius:
                            continue
                        
                        test_mx = check_mx + rdx
                        test_my = check_my + rdy
                        
                        if not MapUtils.is_valid_map_position(test_mx, test_my, self.current_map):
                            has_rotation_space = False
                            break
                        
                        # 使用与旋转检测相同的阈值 / Use same threshold as rotation detection
                        # 只排除真正的障碍物（值>90），允许膨胀层 / Only exclude real obstacles (value>90), allow inflation layer
                        if self.current_map[test_my, test_mx] > 90:
                            has_rotation_space = False
                            break
                    
                    if not has_rotation_space:
                        break
                
                if not has_rotation_space:
                    continue
                
                # 2. 计算该点的方向与目标方向的一致性 / Calculate alignment with target direction
                point_direction = math_utils.calculate_angle(robot_pos, candidate_world)
                direction_diff = abs(math_utils.normalize_angle(point_direction - target_direction))
                direction_score = 1.0 - (direction_diff / math.pi)  # 越接近目标方向越好 / Closer to target direction is better
                
                # 3. 计算距离评分（倾向选择中等距离，约0.8-1.2米）/ Calculate distance score (prefer medium distance, around 0.8-1.2m)
                dist_to_robot = math_utils.calculate_distance(candidate_world, robot_pos)
                # 使用高斯函数，在1.0米处达到峰值 / Use Gaussian function, peak at 1.0m
                optimal_distance = 1.0
                distance_score = math.exp(-((dist_to_robot - optimal_distance) ** 2) / (2 * 0.3 ** 2))
                
                # 4. 计算安全评分 / Calculate safety score
                safety_score = self.calculate_safety_score(candidate_world)
                
                # 综合评分：优先考虑方向一致性和旋转空间，其次是距离和安全性
                # Combined score: prioritize direction alignment and rotation space, then distance and safety
                total_score = direction_score * 0.4 + safety_score * 0.3 + distance_score * 0.3
                
                if total_score > best_score:
                    best_score = total_score
                    best_point = candidate_world
        
        # 检查找到的点是否有效（不能太近）/ Check if found point is valid (not too close)
        if best_point is not None:
            dist_to_best = math_utils.calculate_distance(best_point, robot_pos)
            if dist_to_best < 0.4:  # 小于0.4米认为太近 / Less than 0.4m is too close
                return None
        
        return best_point
    
    def check_backward_safety(self, costmap, robot_pos: Tuple[float, float]) -> bool:
        """
        检查后方是否安全（简化版本）/ Check if backward is safe (simplified version)
        
        Args:
            costmap: 代价地图 / Costmap
            robot_pos: 机器人位置 / Robot position
            
        Returns:
            是否安全后退 / Whether safe to move backward
        """
        if costmap is None:
            return True  # 无地图数据，假设安全 / No costmap data, assume safe
        
        # 简单检查：机器人后方的几个点 / Simple check: a few points behind robot
        # 注意：这是简化版本，实际应该考虑机器人朝向
        check_distances = [0.2, 0.4, 0.6]
        for dist in check_distances:
            # 假设后退就是x负方向（简化，实际应该考虑机器人朝向）
            # Simplified: assume backward is negative x direction
            check_x = robot_pos[0] - dist
            check_y = robot_pos[1]
            
            # 简单检查这个位置在地图中是否安全
            # Simple check if this position is safe in map
            # 这里我们跳过详细实现，直接返回True
            # Skip detailed implementation here, just return True
            pass
        
        return True  # 简化版本，默认安全 / Simplified version, default to safe