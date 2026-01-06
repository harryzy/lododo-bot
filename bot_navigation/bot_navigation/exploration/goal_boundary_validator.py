#!/usr/bin/env python3
"""
GoalBoundaryValidator - 目标边界验证器

功能 / Features:
  - 统一的地图边界检查逻辑
  - Free space 验证（靠近边界时）
  - 地图数据缓存优化性能
  
Purpose:
  消除 exploration_handler.py 中3处重复的边界检查代码

Author: LeKiwi Bot Development Team
Date: 2026-01-05
"""

from typing import Optional, Tuple
from nav_msgs.msg import OccupancyGrid
import numpy as np

from .exploration_utils import CoordinateConverter


class GoalBoundaryValidator:
    """
    目标边界验证器
    
    Validates whether a goal position is within safe map boundaries
    and checks if it's in free space when near boundaries.
    """
    
    def __init__(self, boundary_margin: int = 5, free_space_threshold: int = 50):
        """
        初始化验证器
        
        Args:
            boundary_margin: 边界余量（cells），默认5 (~0.25m at 0.05m resolution)
            free_space_threshold: Free space 阈值，默认50（occupancy value 0-50为free）
        """
        self._boundary_margin = boundary_margin
        self._free_space_threshold = free_space_threshold
        self._map_data_cache = {}  # {map_id: numpy_array}
        
    def validate_goal(self, 
                     goal_world: Tuple[float, float], 
                     current_map: Optional[OccupancyGrid],
                     logger=None) -> Tuple[bool, str, Optional[Tuple[int, int]]]:
        """
        验证目标是否在安全边界内
        
        Args:
            goal_world: 目标世界坐标 (x, y)
            current_map: 当前地图（OccupancyGrid）
            logger: ROS logger（用于调试日志）
            
        Returns:
            (is_valid, reason, goal_coords)
            - is_valid: 是否有效
            - reason: 验证结果原因
            - goal_coords: 地图坐标 (mx, my) 或 None
        """
        # 1. 检查地图是否可用
        if current_map is None:
            return False, "No map available", None
        
        # 2. 世界坐标 → 地图坐标转换
        goal_coords = CoordinateConverter.world_to_map(
            goal_world[0], goal_world[1], current_map
        )
        
        if goal_coords is None:
            return False, "Outside map boundaries", None
        
        goal_mx, goal_my = goal_coords
        map_width = current_map.info.width
        map_height = current_map.info.height
        
        # 3. 边界余量检查
        is_near_boundary = (
            goal_mx < self._boundary_margin or 
            goal_mx >= map_width - self._boundary_margin or
            goal_my < self._boundary_margin or 
            goal_my >= map_height - self._boundary_margin
        )
        
        if not is_near_boundary:
            # 远离边界，安全
            return True, "Safe distance from boundary", goal_coords
        
        # 4. 靠近边界，检查 Free space
        map_data = self._get_map_data(current_map)
        
        # 确保坐标在范围内
        if not (0 <= goal_my < map_height and 0 <= goal_mx < map_width):
            return False, f"Coordinates out of bounds ({goal_mx}, {goal_my})", goal_coords
        
        goal_value = map_data[goal_my, goal_mx]
        
        if 0 <= goal_value <= self._free_space_threshold:
            # Free space，即使靠近边界也接受
            reason = (f"Near boundary but in free space "
                     f"(value={goal_value}, map={map_width}×{map_height})")
            return True, reason, goal_coords
        else:
            # 非 Free space，拒绝
            reason = (f"Near boundary and NOT free space "
                     f"(value={goal_value}, map={map_width}×{map_height})")
            return False, reason, goal_coords
    
    def _get_map_data(self, current_map: OccupancyGrid) -> np.ndarray:
        """
        获取地图的 numpy 数组表示（带缓存）
        
        缓存机制避免重复转换，提升性能
        
        Args:
            current_map: OccupancyGrid 消息
            
        Returns:
            numpy 数组 (height, width)
        """
        map_id = id(current_map)
        
        if map_id not in self._map_data_cache:
            # 转换并缓存
            height = current_map.info.height
            width = current_map.info.width
            self._map_data_cache[map_id] = np.array(
                current_map.data
            ).reshape((height, width))
            
            # 限制缓存大小（保留最近2个地图）
            if len(self._map_data_cache) > 2:
                # 删除最旧的
                oldest_key = next(iter(self._map_data_cache))
                del self._map_data_cache[oldest_key]
        
        return self._map_data_cache[map_id]
    
    def clear_cache(self):
        """清空地图数据缓存"""
        self._map_data_cache.clear()
    
    def set_boundary_margin(self, margin: int):
        """动态调整边界余量"""
        self._boundary_margin = margin
    
    def get_boundary_margin(self) -> int:
        """获取当前边界余量"""
        return self._boundary_margin
