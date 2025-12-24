#!/usr/bin/env python3
"""
Exploration Strategy - 探索策略模块 / Exploration Strategy Module

负责选择最佳探索目标和制定探索策略
Responsible for selecting optimal exploration goals and formulating exploration strategies

Author: LeKiwi Bot Development Team
Date: 2025-12-23
"""

import math
from typing import List, Tuple, Optional, Dict
from nav_msgs.msg import OccupancyGrid

from .exploration_utils import math_utils, coordinate_converter


class FrontierEvaluator:
    """边界评估器类 / Frontier Evaluator Class"""
    
    def __init__(self):
        self.visited_frontiers = []
        self.last_goal_direction = None
        self.stagnant_count = 0
        self.max_stagnant = 2
        self.visit_radius = 0.5
    
    def update_state(self, visited_frontiers: List[Tuple[float, float]], 
                    last_goal_direction: Optional[float] = None,
                    stagnant_count: int = 0):
        """
        更新评估器状态 / Update evaluator state
        
        Args:
            visited_frontiers: 已访问的边界列表 / List of visited frontiers
            last_goal_direction: 上次目标方向 / Last goal direction
            stagnant_count: 停滞计数 / Stagnant count
        """
        self.visited_frontiers = visited_frontiers
        self.last_goal_direction = last_goal_direction
        self.stagnant_count = stagnant_count
    
    def evaluate_frontier(self, frontier_info: dict, robot_pos: Tuple[float, float], 
                         robot_yaw: float, completion: float,
                         min_completion_threshold: float = 0.75,
                         max_angle_deg: int = 60,
                         min_distance: float = 0.2,
                         max_distance: float = 5.0) -> Optional[dict]:
        """
        评估单个边界区域 / Evaluate a single frontier region
        
        Args:
            frontier_info: 边界信息 / Frontier information
            robot_pos: 机器人位置 / Robot position
            robot_yaw: 机器人朝向 / Robot yaw
            completion: 地图完成度 / Map completion
            min_completion_threshold: 最小完成度阈值 / Minimum completion threshold
            max_angle_deg: 最大角度范围 / Maximum angle range
            min_distance: 最小距离 / Minimum distance
            max_distance: 最大 distance
            
        Returns:
            评估结果或None / Evaluation result or None
        """
        if not frontier_info or robot_pos is None or robot_yaw is None:
            return None
        
        center_world = frontier_info.get('center_world')
        if center_world is None:
            return None
        
        frontier_size = frontier_info.get('size', 0)
        
        # 🎯 动态调整选择条件 / Dynamic adjustment of selection criteria
        if completion >= min_completion_threshold:
            # 接近完成时大幅放宽限制 / Significantly relax restrictions when near completion
            max_angle_deg = 150  # 从±60°放宽到±150° / From ±60° to ±150°
            min_distance = 0.3  # 从1.0m降低到0.3m / From 1.0m to 0.3m
        
        max_angle_rad = math.radians(max_angle_deg)
        
        # 距离和方向 / Distance and direction
        dx = center_world[0] - robot_pos[0]
        dy = center_world[1] - robot_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        direction = math.atan2(dy, dx)  # 当前frontier的方向角 / Direction angle of current frontier
        
        # 计算frontier相对于机器人当前朝向的角度 / Calculate angle relative to robot's current orientation
        relative_angle = direction - robot_yaw
        relative_angle = math_utils.normalize_angle(relative_angle)
        
        # 跳过太近或太远的边界 / Skip frontiers that are too close or too far
        if distance < min_distance or distance > max_distance:
            return None
        
        # 🎯 关键优化：避免选择身后的frontier / Key optimization: avoid selecting frontiers behind
        if abs(relative_angle) > max_angle_rad:
            return None
        
        # 检查是否已访问过 / Check if already visited
        is_visited = False
        visit_distance = float('inf')
        for visited_pos in self.visited_frontiers:
            visit_dist = math_utils.calculate_distance(center_world, visited_pos)
            if visit_dist < self.visit_radius:
                is_visited = True
                visit_distance = visit_dist
                break
        
        # 基础评分：大小/距离 / Base score: size/distance
        base_score = frontier_size / max(distance, 0.5)
        
        # 如果已访问过，大幅降低优先级 / If visited, significantly reduce priority
        if is_visited:
            base_score *= 0.1  # 已访问过的frontier评分降低到1/10 / Reduce to 1/10 for visited frontiers
        
        # 方向多样性加成 / Direction diversity bonus
        direction_bonus = 1.0
        if self.last_goal_direction is not None:
            # 计算与上次方向的角度差 / Calculate angle difference with last direction
            angle_diff = abs(direction - self.last_goal_direction)
            angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # 归一化到[0, π] / Normalize to [0, π]
            
            # 如果停滞了，强烈倾向于相反方向 / If stagnant, strongly prefer opposite direction
            if self.stagnant_count > 0:
                # angle_diff接近π时加成最大（最多3倍）/ Maximum bonus when angle_diff approaches π (up to 3x)
                direction_bonus = 1.0 + 2.0 * (angle_diff / math.pi)
            else:
                # 正常模式：稍微倾向于不同方向（最多1.5倍）/ Normal mode: slightly prefer different direction (up to 1.5x)
                direction_bonus = 1.0 + 0.5 * (angle_diff / math.pi)
        
        # 最终评分 / Final score
        score = base_score * direction_bonus
        
        return {
            'frontier_info': frontier_info,
            'score': score,
            'distance': distance,
            'direction': direction,
            'relative_angle': relative_angle,
            'is_visited': is_visited,
            'visit_distance': visit_distance if is_visited else None,
            'base_score': base_score,
            'direction_bonus': direction_bonus
        }


class ExplorationStrategy:
    """探索策略类 / Exploration Strategy Class"""
    
    def __init__(self):
        self.evaluator = FrontierEvaluator()
        self.exploration_radius = 5.0
        self.min_goal_distance = 0.2
        self.goal_tolerance = 0.3
        self.map_completion_threshold = 0.90
        self.min_completion_threshold = 0.75
        self.max_failures = 8
        self.max_no_goal_count = 5
        self.enable_smart_exploration = True
        self.visit_radius = 0.5
        self.rotation_angle = 45.0
        
        # 状态变量 / State variables
        self.consecutive_failures = 0
        self.no_goal_count = 0
        self.last_known_cells = 0
        
    def configure(self, config: dict):
        """
        配置策略参数 / Configure strategy parameters
        
        Args:
            config: 配置字典 / Configuration dictionary
        """
        self.exploration_radius = config.get('exploration_radius', 5.0)
        self.min_goal_distance = config.get('min_goal_distance', 0.2)
        self.goal_tolerance = config.get('goal_tolerance', 0.3)
        self.map_completion_threshold = config.get('map_completion_threshold', 0.90)
        self.min_completion_threshold = config.get('min_completion_threshold', 0.75)
        self.max_failures = config.get('max_failures', 8)
        self.max_no_goal_count = config.get('max_no_goal_count', 5)
        self.enable_smart_exploration = config.get('enable_smart_exploration', True)
        self.visit_radius = config.get('visit_radius', 0.5)
        self.rotation_angle = config.get('rotation_angle', 45.0)
    
    def select_best_frontier(self, frontiers: List[dict], robot_pos: Tuple[float, float], 
                           robot_yaw: float, known_cells: int, total_cells: int) -> Optional[dict]:
        """
        选择最佳边界目标 / Select the best frontier target
        
        Args:
            frontiers: 边界区域列表 / List of frontier regions
            robot_pos: 机器人位置 / Robot position
            robot_yaw: 机器人朝向 / Robot yaw
            known_cells: 已知区域数量 / Number of known cells
            total_cells: 总区域数量 / Total number of cells
            
        Returns:
            最佳边界评估结果或None / Best frontier evaluation result or None
        """
        if not frontiers or robot_pos is None or robot_yaw is None:
            return None
        
        # 计算地图完成度 / Calculate map completion
        completion = known_cells / total_cells if total_cells > 0 else 0.0
        
        # 更新评估器状态 / Update evaluator state
        self.evaluator.update_state(
            self.evaluator.visited_frontiers,
            self.evaluator.last_goal_direction,
            self.evaluator.stagnant_count
        )
        
        best_frontier = None
        best_score = -float('inf')
        
        # 评估所有边界 / Evaluate all frontiers
        for i, frontier_info in enumerate(frontiers):
            evaluation = self.evaluator.evaluate_frontier(
                frontier_info, robot_pos, robot_yaw, completion,
                self.min_completion_threshold, 60, self.min_goal_distance, self.exploration_radius
            )
            
            if evaluation is None:
                continue
            
            # 记录评估信息 / Record evaluation information
            self._log_frontier_evaluation(i, frontier_info, evaluation)
            
            if evaluation['score'] > best_score:
                best_score = evaluation['score']
                best_frontier = evaluation
        
        return best_frontier
    
    def _log_frontier_evaluation(self, index: int, frontier_info: dict, evaluation: dict):
        """
        记录边界评估信息 / Log frontier evaluation information
        
        Args:
            index: 边界索引 / Frontier index
            frontier_info: 边界信息 / Frontier information
            evaluation: 评估结果 / Evaluation result
        """
        # 这里可以添加日志记录 / Add logging here if needed
        pass
    
    def check_exploration_progress(self, current_known_cells: int) -> dict:
        """
        检查探索进展 / Check exploration progress
        
        Args:
            current_known_cells: 当前已知区域数量 / Current number of known cells
            
        Returns:
            进展状态字典 / Progress status dictionary
        """
        new_known_cells = current_known_cells - self.last_known_cells
        self.last_known_cells = current_known_cells
        
        is_stagnant = new_known_cells < 10
        is_effective = new_known_cells > 50
        
        return {
            'new_known_cells': new_known_cells,
            'is_stagnant': is_stagnant,
            'is_effective': is_effective,
            'stagnant_threshold': 10,
            'effective_threshold': 50
        }
    
    def should_trigger_rotation(self, progress_status: dict) -> bool:
        """
        判断是否应该触发旋转 / Determine if rotation should be triggered
        
        Args:
            progress_status: 进展状态 / Progress status
            
        Returns:
            是否应该旋转 / Whether rotation should be triggered
        """
        if progress_status.get('is_stagnant', False):
            self.evaluator.stagnant_count += 1
            if self.evaluator.stagnant_count >= self.evaluator.max_stagnant:
                self.evaluator.stagnant_count = 0
                self.evaluator.last_goal_direction = None
                self.evaluator.visited_frontiers.clear()
                return True
        else:
            self.evaluator.stagnant_count = 0
        
        return False
    
    def should_trigger_360_scan(self, progress_status: dict) -> bool:
        """
        判断是否应该触发360度扫描 / Determine if 360-degree scan should be triggered
        
        Args:
            progress_status: 进展状态 / Progress status
            
        Returns:
            是否应该扫描 / Whether scan should be triggered
        """
        return progress_status.get('is_effective', False)
    
    def check_completion_criteria(self, completion: float) -> dict:
        """
        检查完成条件 / Check completion criteria
        
        Args:
            completion: 地图完成度 / Map completion
            
        Returns:
            完成状态字典 / Completion status dictionary
        """
        is_complete = completion >= self.map_completion_threshold
        is_near_complete = completion >= self.min_completion_threshold
        
        return {
            'completion': completion,
            'is_complete': is_complete,
            'is_near_complete': is_near_complete,
            'map_completion_threshold': self.map_completion_threshold,
            'min_completion_threshold': self.min_completion_threshold
        }
    
    def update_visited_frontier(self, frontier_pos: Tuple[float, float]):
        """
        更新已访问的边界 / Update visited frontier
        
        Args:
            frontier_pos: 边界位置 / Frontier position
        """
        self.evaluator.visited_frontiers.append(frontier_pos)
        
        # 只保留最近10个访问记录，避免内存膨胀 / Keep only last 10 visits to avoid memory bloat
        if len(self.evaluator.visited_frontiers) > 10:
            self.evaluator.visited_frontiers.pop(0)
    
    def update_last_goal_direction(self, direction: float):
        """
        更新上次目标方向 / Update last goal direction
        
        Args:
            direction: 方向 / Direction
        """
        self.evaluator.last_goal_direction = direction
    
    def increment_failure_count(self):
        """增加失败计数 / Increment failure count"""
        self.consecutive_failures += 1
        return self.consecutive_failures >= self.max_failures
    
    def increment_no_goal_count(self):
        """增加无目标计数 / Increment no goal count"""
        self.no_goal_count += 1
        return self.no_goal_count >= self.max_no_goal_count
    
    def reset_failure_count(self):
        """重置失败计数 / Reset failure count"""
        self.consecutive_failures = 0
    
    def reset_no_goal_count(self):
        """重置无目标计数 / Reset no goal count"""
        self.no_goal_count = 0
    
    def get_strategy_status(self) -> dict:
        """
        获取策略状态 / Get strategy status
        
        Returns:
            策略状态字典 / Strategy status dictionary
        """
        return {
            'consecutive_failures': self.consecutive_failures,
            'no_goal_count': self.no_goal_count,
            'max_failures': self.max_failures,
            'max_no_goal_count': self.max_no_goal_count,
            'stagnant_count': self.evaluator.stagnant_count,
            'visited_frontiers_count': len(self.evaluator.visited_frontiers)
        }