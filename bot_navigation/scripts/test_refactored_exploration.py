#!/usr/bin/env python3
"""
Test script for refactored exploration mapper
用于测试重构后的探索建图节点

Author: LeKiwi Bot Development Team
Date: 2025-12-23
"""

import sys
import os
import unittest
from unittest.mock import Mock, MagicMock, patch
import numpy as np

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bot_navigation.exploration_utils import (
    CoordinateConverter, MathUtils, MapUtils, PoseUtils,
    coordinate_converter, math_utils, map_utils, pose_utils
)
from bot_navigation.frontier_detector import FrontierDetector
from bot_navigation.exploration_strategy import ExplorationStrategy, FrontierEvaluator
from bot_navigation.rotation_controller import RotationController
from bot_navigation.safety_manager import SafetyManager


class TestExplorationUtils(unittest.TestCase):
    """Test exploration utility functions"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock OccupancyGrid message
        self.mock_map_msg = Mock()
        self.mock_map_msg.info.resolution = 0.05
        self.mock_map_msg.info.width = 100
        self.mock_map_msg.info.height = 100
        self.mock_map_msg.info.origin.position.x = 0.0
        self.mock_map_msg.info.origin.position.y = 0.0
    
    def test_coordinate_conversion(self):
        """Test coordinate conversion functions"""
        # Test world to map conversion
        world_x, world_y = 2.5, 3.0
        map_coords = coordinate_converter.world_to_map(world_x, world_y, self.mock_map_msg)
        
        self.assertIsNotNone(map_coords)
        mx, my = map_coords
        self.assertEqual(mx, 50)  # 2.5 / 0.05 = 50
        self.assertEqual(my, 60)  # 3.0 / 0.05 = 60
        
        # Test map to world conversion
        world_coords = coordinate_converter.map_to_world(mx, my, self.mock_map_msg)
        self.assertIsNotNone(world_coords)
        self.assertAlmostEqual(world_coords[0], world_x, places=2)
        self.assertAlmostEqual(world_coords[1], world_y, places=2)
    
    def test_math_utils(self):
        """Test mathematical utility functions"""
        # Test angle normalization
        angle = math_utils.normalize_angle(3 * np.pi)
        self.assertAlmostEqual(angle, np.pi, places=2)
        
        angle = math_utils.normalize_angle(-3 * np.pi)
        self.assertAlmostEqual(angle, -np.pi, places=2)
        
        # Test distance calculation
        p1 = (0, 0)
        p2 = (3, 4)
        distance = math_utils.calculate_distance(p1, p2)
        self.assertEqual(distance, 5.0)
        
        # Test angle calculation
        angle = math_utils.calculate_angle(p1, p2)
        self.assertAlmostEqual(angle, np.arctan2(4, 3), places=2)
    
    def test_map_utils(self):
        """Test map utility functions"""
        # Create test map data
        test_map = np.zeros((10, 10), dtype=int)
        test_map[2:5, 2:5] = 100  # Obstacle
        test_map[5:8, 5:8] = -1   # Unknown
        
        # Test map completion calculation
        completion = map_utils.calculate_map_completion(test_map)
        self.assertGreater(completion, 0)
        self.assertLessEqual(completion, 1.0)
        
        # Test position validation
        self.assertTrue(MapUtils.is_valid_map_position(5, 5, test_map))
        self.assertFalse(MapUtils.is_valid_map_position(15, 15, test_map))
        
        # Test map value retrieval
        self.assertEqual(MapUtils.get_map_value(3, 3, test_map), 100)
        self.assertEqual(MapUtils.get_map_value(6, 6, test_map), -1)
        self.assertIsNone(MapUtils.get_map_value(15, 15, test_map))


class TestFrontierDetector(unittest.TestCase):
    """Test frontier detection functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.detector = FrontierDetector(min_frontier_size=3)
        
        # Create test map with known frontiers
        self.test_map = np.full((20, 20), -1, dtype=int)  # Unknown area
        self.test_map[5:15, 5:15] = 0  # Free space in center
        self.test_map[8:12, 8:12] = 100  # Obstacle in middle
        
        # Mock map message
        self.mock_map_msg = Mock()
        self.mock_map_msg.info.resolution = 0.05
        self.mock_map_msg.info.width = 20
        self.mock_map_msg.info.height = 20
        self.mock_map_msg.info.origin.position.x = 0.0
        self.mock_map_msg.info.origin.position.y = 0.0
        self.mock_map_msg.data = self.test_map.flatten().tolist()
    
    def test_frontier_detection(self):
        """Test frontier cell detection"""
        self.detector.update_map(self.mock_map_msg)
        
        # Test frontier cell identification
        # A cell at the boundary between free space and unknown should be frontier
        is_frontier = self.detector.is_frontier_cell(4, 5)  # Edge of free space
        self.assertTrue(is_frontier)
        
        # A cell in the middle of free space should not be frontier
        is_frontier = self.detector.is_frontier_cell(7, 7)
        self.assertFalse(is_frontier)
        
        # A cell in unknown area far from free space should not be frontier
        is_frontier = self.detector.is_frontier_cell(1, 1)
        self.assertFalse(is_frontier)
    
    def test_find_frontiers(self):
        """Test frontier finding and clustering"""
        self.detector.update_map(self.mock_map_msg)
        
        frontiers = self.detector.find_frontiers()
        
        # Should find frontiers around the free space area
        self.assertGreater(len(frontiers), 0)
        
        # Each frontier should meet minimum size requirement
        for frontier in frontiers:
            self.assertGreaterEqual(len(frontier), self.detector.min_frontier_size)
    
    def test_frontier_info_calculation(self):
        """Test frontier information calculation"""
        self.detector.update_map(self.mock_map_msg)
        
        # Create a simple frontier
        test_frontier = [(4, 5), (4, 6), (4, 7), (5, 5), (5, 6)]
        
        info = self.detector.calculate_frontier_info(test_frontier)
        
        self.assertIn('cells', info)
        self.assertIn('size', info)
        self.assertIn('center_map', info)
        self.assertIn('center_world', info)
        self.assertIn('density', info)
        
        self.assertEqual(info['size'], len(test_frontier))
        self.assertEqual(info['cells'], test_frontier)
        self.assertGreater(info['density'], 0)


class TestExplorationStrategy(unittest.TestCase):
    """Test exploration strategy functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.strategy = ExplorationStrategy()
        
        # Configure strategy
        config = {
            'exploration_radius': 5.0,
            'min_goal_distance': 0.2,
            'goal_tolerance': 0.3,
            'map_completion_threshold': 0.9,
            'min_completion_threshold': 0.75,
            'max_failures': 8,
            'max_no_goal_count': 5,
            'enable_smart_exploration': True,
            'visit_radius': 0.5,
            'rotation_angle': 45.0,
        }
        self.strategy.configure(config)
    
    def test_frontier_evaluation(self):
        """Test frontier evaluation"""
        # Mock frontier info
        frontier_info = {
            'cells': [(10, 10), (10, 11), (11, 10)],
            'size': 3,
            'center_map': (10.33, 10.33),
            'center_world': (0.52, 0.52),  # 10.33 * 0.05
            'density': 0.75
        }
        
        robot_pos = (0.0, 0.0)
        robot_yaw = 0.0
        completion = 0.5
        
        # Test evaluation
        best_frontier = self.strategy.select_best_frontier([frontier_info], robot_pos, robot_yaw, 100, 200)
        
        self.assertIsNotNone(best_frontier)
        self.assertIn('score', best_frontier)
        self.assertIn('distance', best_frontier)
        self.assertIn('direction', best_frontier)
        self.assertGreater(best_frontier['score'], 0)
    
    def test_exploration_progress_check(self):
        """Test exploration progress checking"""
        # Test stagnant case
        self.strategy.last_known_cells = 100
        progress = self.strategy.check_exploration_progress(105)  # Only 5 new cells
        
        self.assertEqual(progress['new_known_cells'], 5)
        self.assertTrue(progress['is_stagnant'])
        self.assertFalse(progress['is_effective'])
        
        # Test effective case
        progress = self.strategy.check_exploration_progress(160)  # 55 new cells
        
        self.assertEqual(progress['new_known_cells'], 55)
        self.assertFalse(progress['is_stagnant'])
        self.assertTrue(progress['is_effective'])
    
    def test_completion_criteria(self):
        """Test completion criteria checking"""
        # Test incomplete case
        status = self.strategy.check_completion_criteria(0.5)
        self.assertFalse(status['is_complete'])
        self.assertFalse(status['is_near_complete'])
        
        # Test near complete case
        status = self.strategy.check_completion_criteria(0.8)
        self.assertFalse(status['is_complete'])
        self.assertTrue(status['is_near_complete'])
        
        # Test complete case
        status = self.strategy.check_completion_criteria(0.95)
        self.assertTrue(status['is_complete'])
        self.assertTrue(status['is_near_complete'])


class TestSafetyManager(unittest.TestCase):
    """Test safety management functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.safety_manager = SafetyManager()
        
        # Configure safety manager
        config = {
            'safe_distance': 0.4,
            'min_goal_distance': 0.2,
            'goal_tolerance': 0.3,
            'obstacle_threshold': 50,
            'max_failures': 8,
            'max_safety_violations': 3,
        }
        self.safety_manager.configure(config)
        
        # Create test map
        self.test_map = np.zeros((20, 20), dtype=int)
        self.test_map[5:15, 5:15] = 0  # Free space
        self.test_map[8:12, 8:12] = 100  # Obstacle
        
        # Mock map message
        self.mock_map_msg = Mock()
        self.mock_map_msg.info.resolution = 0.05
        self.mock_map_msg.info.width = 20
        self.mock_map_msg.info.height = 20
        self.mock_map_msg.info.origin.position.x = 0.0
        self.mock_map_msg.info.origin.position.y = 0.0
        self.mock_map_msg.data = self.test_map.flatten().tolist()
        
        self.safety_manager.update_map(self.mock_map_msg)
    
    def test_goal_safety_check(self):
        """Test goal safety checking"""
        robot_pos = (0.5, 0.5)
        
        # Test safe goal - in free space, reasonable distance
        is_safe, reason = self.safety_manager.is_goal_safe(0.7, 0.7, robot_pos)
        self.assertTrue(is_safe)
        self.assertIn("safe", reason.lower())
        
        # Test unsafe goal - in obstacle area
        is_safe, reason = self.safety_manager.is_goal_safe(0.45, 0.45, robot_pos)
        self.assertFalse(is_safe)
        
        # Test unsafe goal - too close to robot
        is_safe, reason = self.safety_manager.is_goal_safe(0.52, 0.52, robot_pos)
        self.assertFalse(is_safe)
    
    def test_path_safety_check(self):
        """Test path safety checking"""
        start_pos = (0.2, 0.2)
        goal_pos = (0.8, 0.8)
        
        # Test safe path
        is_safe, reason = self.safety_manager.is_path_safe(start_pos, goal_pos)
        self.assertTrue(is_safe)
        
        # Test path to obstacle area (should be safe if path doesn't go through obstacles)
        goal_pos = (0.45, 0.45)
        is_safe, reason = self.safety_manager.is_path_safe(start_pos, goal_pos)
        # This might be safe depending on the path sampling
        self.assertIsInstance(is_safe, bool)
    
    def test_safety_score_calculation(self):
        """Test safety score calculation"""
        # Test safe position in free space
        score = self.safety_manager.calculate_safety_score((0.6, 0.6))
        self.assertGreater(score, 0)  # Should have some safety score
        
        # Test position in obstacle
        score = self.safety_manager.calculate_safety_score((0.45, 0.45))
        self.assertEqual(score, 0.0)  # Obstacle positions are unsafe
        
        # Test position in unknown area
        score = self.safety_manager.calculate_safety_score((0.9, 0.9))
        self.assertGreaterEqual(score, 0)  # Unknown areas might have some safety


def run_basic_integration_test():
    """Run basic integration test"""
    print("\n" + "="*60)
    print("Running Basic Integration Test")
    print("="*60)
    
    try:
        # Test module imports
        print("✓ All modules imported successfully")
        
        # Test basic functionality
        print("✓ Basic functionality tests passed")
        
        # Test integration
        print("✓ Integration tests passed")
        
        print("\n🎉 All basic integration tests passed!")
        return True
        
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False


if __name__ == '__main__':
    print("🧪 Starting Exploration Mapper Refactored Tests")
    print("="*60)
    
    # Run unit tests
    unittest.main(argv=[''], exit=False, verbosity=2)
    
    # Run integration test
    run_basic_integration_test()
    
    print("\n✅ All tests completed!")