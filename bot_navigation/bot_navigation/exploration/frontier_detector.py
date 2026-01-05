#!/usr/bin/env python3
"""
Frontier Detector - 边界检测模块 / Frontier Detection Module

负责检测和聚类地图中的边界区域
Responsible for detecting and clustering frontier areas in the map

Author: LeKiwi Bot Development Team
Date: 2025-12-23
"""

import math
import numpy as np
from collections import deque
from typing import List, Tuple, Optional
from nav_msgs.msg import OccupancyGrid

from .exploration_utils import MapUtils, coordinate_converter


class FrontierDetector:
    """边界检测器类 / Frontier Detector Class"""
    
    def __init__(self, min_frontier_size: int = 5, logger=None):
        """
        初始化边界检测器 / Initialize frontier detector
        
        Args:
            min_frontier_size: 最小边界区域大小 / Minimum frontier cluster size
            logger: ROS2 logger for debug output (optional)
        """
        self.min_frontier_size = min_frontier_size
        self.current_map = None
        self.current_map_msg = None
        self.logger = logger
        
    def update_map(self, map_msg: OccupancyGrid) -> None:
        """
        更新地图数据 / Update map data
        
        Args:
            map_msg: 地图消息 / Map message
        """
        self.current_map_msg = map_msg
        self.current_map = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
    
    def is_frontier_cell(self, mx: int, my: int) -> bool:
        """
        判断是否为边界点 / Determine if a cell is a frontier cell
        
        边界点定义：未知区域(-1)且邻近自由空间(0-50)
        Frontier definition: unknown area (-1) adjacent to free space (0-50)
        
        Args:
            mx: 地图坐标X / Map coordinate X
            my: 地图坐标Y / Map coordinate Y
            
        Returns:
            是否为边界点 / Whether it's a frontier cell
        """
        if self.current_map is None:
            return False
        
        height, width = self.current_map.shape
        
        # 边界检查 / Boundary check
        if mx < 1 or mx >= width - 1 or my < 1 or my >= height - 1:
            return False
        
        # 当前点必须是未知(-1) / Current cell must be unknown (-1)
        if self.current_map[my, mx] != -1:
            return False
        
        # 检查8邻域是否有自由空间(0-50) / Check 8-neighborhood for free space (0-50)
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor_val = self.current_map[my + dy, mx + dx]
                if 0 <= neighbor_val <= 50:  # 自由空间 / Free space
                    return True
        
        return False
    
    def find_frontiers(self) -> List[List[Tuple[int, int]]]:
        """
        寻找所有边界点并聚类 / Find all frontier cells and cluster them
        
        Returns:
            边界区域列表，每个区域包含一组栅格坐标 / List of frontier regions, each containing grid coordinates
        """
        if self.current_map is None or self.current_map_msg is None:
            return []
        
        height, width = self.current_map.shape
        frontiers = []
        visited = np.zeros((height, width), dtype=bool)
        
        # 🔧 Debug: 统计地图状态
        total_cells = height * width
        unknown_cells = np.sum(self.current_map == -1)
        free_cells = np.sum((self.current_map >= 0) & (self.current_map <= 50))
        obstacle_cells = np.sum(self.current_map > 50)
        
        # 扫描所有可能的边界点 / Scan all possible frontier cells
        frontier_candidates = 0
        for my in range(1, height - 1):
            for mx in range(1, width - 1):
                if visited[my, mx]:
                    continue
                
                # 检查是否是边界点
                if not self.is_frontier_cell(mx, my):
                    continue
                
                frontier_candidates += 1
                
                # BFS聚类 / BFS clustering
                cluster = []
                queue = deque([(mx, my)])
                visited[my, mx] = True
                
                while queue:
                    cx, cy = queue.popleft()
                    cluster.append((cx, cy))
                    
                    # 检查4邻域 / Check 4-neighborhood
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = cx + dx, cy + dy
                        if (0 <= nx < width and 0 <= ny < height and 
                            not visited[ny, nx] and self.is_frontier_cell(nx, ny)):
                            visited[ny, nx] = True
                            queue.append((nx, ny))
                
                # 保存足够大的边界 / Save sufficiently large frontier
                if len(cluster) >= self.min_frontier_size:
                    frontiers.append(cluster)
        
        # 🔧 Debug: 打印详细统计
        if hasattr(self, 'logger'):
            self.logger.info(
                f"[FrontierDetector] Map: {width}x{height}, "
                f"Unknown={unknown_cells}({unknown_cells/total_cells*100:.1f}%), "
                f"Free={free_cells}({free_cells/total_cells*100:.1f}%), "
                f"Obstacle={obstacle_cells}({obstacle_cells/total_cells*100:.1f}%), "
                f"Candidates={frontier_candidates}, "
                f"Clusters={len(frontiers)}, "
                f"MinSize={self.min_frontier_size}"
            )
        
        return frontiers
    
    def calculate_frontier_info(self, frontier: List[Tuple[int, int]]) -> dict:
        """
        计算边界区域信息 / Calculate frontier region information
        
        Args:
            frontier: 边界区域栅格坐标列表 / Frontier region grid coordinates
            
        Returns:
            边界信息字典 / Frontier information dictionary
        """
        if not frontier or self.current_map_msg is None:
            return {}
        
        # 计算边界中心（栅格坐标）/ Calculate frontier center (grid coordinates)
        center_mx = sum(p[0] for p in frontier) / len(frontier)
        center_my = sum(p[1] for p in frontier) / len(frontier)
        
        # 🎯 关键修复：在 frontier 附近找安全的自由空间作为目标
        # Find safe free space near frontier as navigation target
        safe_goal_world = self._find_navigable_point_near_frontier(frontier, center_mx, center_my)
        
        if safe_goal_world is None:
            # 如果找不到安全点，返回空字典（这个 frontier 不可用）
            return {}
        
        # 计算边界大小和密度 / Calculate frontier size and density
        info = {
            'cells': frontier,
            'size': len(frontier),
            'center_map': (center_mx, center_my),
            'center_world': safe_goal_world,  # 🎯 使用安全的自由空间点
            'density': self._calculate_frontier_density(frontier)
        }
        
        return info
    
    def _find_navigable_point_near_frontier(self, frontier: List[Tuple[int, int]], 
                                           center_mx: float, center_my: float) -> Optional[Tuple[float, float]]:
        """
        在 frontier 附近找到可导航的自由空间点 / Find navigable free space near frontier
        
        Args:
            frontier: frontier cells 列表
            center_mx, center_my: frontier 中心栅格坐标
            
        Returns:
            安全点的世界坐标或 None
        """
        if self.current_map is None:
            return None
        
        height, width = self.current_map.shape
        
        # 从 frontier 的每个点向内搜索自由空间
        # Search inward from each frontier point to find free space
        candidates = []
        
        for mx, my in frontier:
            # 检查 8 邻域中的自由空间点
            # Check 8-neighborhood for free space points
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    
                    check_mx = mx + dx
                    check_my = my + dy
                    
                    # 边界检查
                    if check_mx < 0 or check_mx >= width or check_my < 0 or check_my >= height:
                        continue
                    
                    # 检查是否为自由空间 (0-50)
                    cell_value = self.current_map[check_my, check_mx]
                    if 0 <= cell_value <= 50:
                        # 找到自由空间，转换为世界坐标
                        world_pos = coordinate_converter.map_to_world(check_mx, check_my, self.current_map_msg)
                        if world_pos:
                            candidates.append((world_pos, check_mx, check_my))
        
        if not candidates:
            # 没有找到自由空间点
            return None
        
        # 选择离 frontier 中心最近的自由空间点
        # Select the free space point closest to frontier center
        best_candidate = min(candidates, key=lambda c: (c[1] - center_mx)**2 + (c[2] - center_my)**2)
        
        return best_candidate[0]  # 返回世界坐标
    
    def _calculate_frontier_density(self, frontier: List[Tuple[int, int]]) -> float:
        """
        计算边界密度 / Calculate frontier density
        
        密度定义为边界点数量除以其包围盒面积
        Density is defined as frontier points divided by bounding box area
        
        Args:
            frontier: 边界区域栅格坐标列表 / Frontier region grid coordinates
            
        Returns:
            密度值(0-1) / Density value (0-1)
        """
        if not frontier:
            return 0.0
        
        # 计算包围盒 / Calculate bounding box
        min_x = min(p[0] for p in frontier)
        max_x = max(p[0] for p in frontier)
        min_y = min(p[1] for p in frontier)
        max_y = max(p[1] for p in frontier)
        
        # 计算面积 / Calculate area
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        area = width * height
        
        if area == 0:
            return 0.0
        
        return len(frontier) / area
    
    def find_safe_goal_near_frontier(self, frontier: List[Tuple[int, int]], 
                                   robot_pos: Tuple[float, float],
                                   safe_distance: float = 0.4,
                                   search_radius: float = 1.5) -> Optional[Tuple[float, float]]:
        """
        在边界附近找到安全的目标点 / Find safe goal point near frontier
        
        Args:
            frontier: 边界区域栅格坐标列表 / Frontier region grid coordinates
            robot_pos: 机器人位置 / Robot position
            safe_distance: 安全距离 / Safe distance
            search_radius: 搜索半径 / Search radius
            
        Returns:
            安全目标点或None / Safe goal point or None
        """
        if not frontier or self.current_map is None or self.current_map_msg is None:
            return None
        
        height, width = self.current_map.shape
        
        # 转换为栅格数 / Convert to grid cells
        search_radius_cells = int(search_radius / self.current_map_msg.info.resolution)
        safe_cells = int(safe_distance / self.current_map_msg.info.resolution)
        
        safe_candidates = []
        
        # 在边界周围搜索安全点 / Search for safe points around frontier
        for mx, my in frontier:
            # 在边界点周围搜索自由空间 / Search for free space around frontier point
            # 优先选择靠近机器人方向的点 / Prioritize points closer to robot direction
            for offset in [(0, 2), (2, 0), (0, -2), (-2, 0),  # 四个正方向 / Four cardinal directions
                          (2, 2), (2, -2), (-2, 2), (-2, -2),  # 四个对角 / Four diagonals
                          (0, 1), (1, 0), (0, -1), (-1, 0)]:   # 近距离 / Close range
                check_mx = mx + offset[0]
                check_my = my + offset[1]
                
                if not MapUtils.is_valid_map_position(check_mx, check_my, self.current_map):
                    continue
                
                # 检查该点是否为自由空间(0) / Check if point is free space (0)
                if self.current_map[check_my, check_mx] != 0:
                    continue
                
                # 检查周围是否有障碍物 / Check for obstacles in surrounding area
                has_obstacle = False
                for dx in range(-safe_cells, safe_cells + 1):
                    for dy in range(-safe_cells, safe_cells + 1):
                        nx = check_mx + dx
                        ny = check_my + dy
                        
                        if not MapUtils.is_valid_map_position(nx, ny, self.current_map):
                            continue
                        
                        # 检查是否为障碍物(>50) / Check if obstacle (>50)
                        if self.current_map[ny, nx] > 50:
                            has_obstacle = True
                            break
                    
                    if has_obstacle:
                        break
                
                # 只有周围无障碍物的点才是真正安全的 / Only points without obstacles are truly safe
                if not has_obstacle:
                    # 转换为世界坐标 / Convert to world coordinates
                    safe_world = coordinate_converter.map_to_world(check_mx, check_my, self.current_map_msg)
                    if safe_world:
                        # 计算到机器人的距离 / Calculate distance to robot
                        dist = math.sqrt(
                            (safe_world[0] - robot_pos[0])**2 + 
                            (safe_world[1] - robot_pos[1])**2
                        )
                        safe_candidates.append((safe_world, dist))
        
        # 如果找到了安全点，选择距离适中的 / If safe points found, choose moderately distant one
        if safe_candidates:
            safe_candidates.sort(key=lambda x: x[1])  # 按距离排序 / Sort by distance
            # 选择中间位置的点，避免过近或过远 / Choose middle position to avoid too close or too far
            mid_idx = len(safe_candidates) // 3  # 选择前1/3位置的点 / Choose first 1/3 position
            return safe_candidates[mid_idx][0]
        
        return None
    
    def get_frontier_statistics(self) -> dict:
        """
        获取边界统计信息 / Get frontier statistics
        
        Returns:
            统计信息字典 / Statistics dictionary
        """
        if self.current_map is None:
            return {}
        
        frontiers = self.find_frontiers()
        
        if not frontiers:
            return {
                'total_frontiers': 0,
                'total_cells': 0,
                'largest_frontier': 0,
                'average_size': 0.0
            }
        
        sizes = [len(frontier) for frontier in frontiers]
        
        return {
            'total_frontiers': len(frontiers),
            'total_cells': sum(sizes),
            'largest_frontier': max(sizes),
            'average_size': sum(sizes) / len(sizes)
        }