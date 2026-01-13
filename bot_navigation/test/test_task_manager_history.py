#!/usr/bin/env python3
"""
TaskManager历史查询功能单元测试
Unit Tests for TaskManager History Query Feature

测试内容 / Test Coverage:
1. 历史任务查询 - 完成/失败/取消任务
2. 查询缓存性能 - LRU淘汰机制
3. 多天历史搜索 - 跨日期查询
4. 边界条件 - 不存在的任务、空历史文件

Author: LeKiwi Bot Development Team
Date: 2026-01-13
"""

import unittest
import tempfile
import shutil
import os
import json
from datetime import datetime, timedelta

# 导入被测试模块
from bot_navigation.mission.task_manager import TaskManager, TaskType, TaskState, Task


class TestTaskManagerHistory(unittest.TestCase):
    """TaskManager历史查询功能测试"""
    
    def setUp(self):
        """测试前准备 - 创建临时目录"""
        self.temp_dir = tempfile.mkdtemp(prefix='test_task_manager_')
        self.task_manager = TaskManager(persistence_dir=self.temp_dir)
    
    def tearDown(self):
        """测试后清理 - 删除临时文件"""
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)
    
    def test_get_active_task(self):
        """测试获取活动任务（快速路径）"""
        # 创建任务
        task = self.task_manager.create_task(
            task_type=TaskType.POINT_TO_POINT,
            parameters={'x': 1.0, 'y': 2.0},
            task_id='test_active_001'
        )
        
        # 查询活动任务（不查历史）
        found = self.task_manager.get_task('test_active_001', include_history=False)
        self.assertIsNotNone(found)
        self.assertEqual(found.task_id, 'test_active_001')
        self.assertEqual(found.state, TaskState.PENDING)
    
    def test_get_completed_task_from_history(self):
        """测试从历史记录查询已完成任务"""
        # 1. 创建并完成任务
        task = self.task_manager.create_task(
            task_type=TaskType.POINT_TO_POINT,
            parameters={'x': 1.0, 'y': 2.0},
            task_id='test_completed_001'
        )
        self.task_manager.complete_task('test_completed_001')
        
        # 2. 模拟任务被删除（完成后清理）
        self.task_manager.remove_task('test_completed_001')
        
        # 3. 验证活动任务中已不存在
        active = self.task_manager.get_task('test_completed_001', include_history=False)
        self.assertIsNone(active, "任务应该从活动任务中删除")
        
        # 4. 从历史记录中查询（慢速路径）
        from_history = self.task_manager.get_task('test_completed_001', include_history=True)
        self.assertIsNotNone(from_history, "应该能从历史记录中找到任务")
        self.assertEqual(from_history.task_id, 'test_completed_001')
        self.assertEqual(from_history.state, TaskState.COMPLETED)
        self.assertIsNotNone(from_history.completed_at)
    
    def test_get_failed_task_from_history(self):
        """测试从历史记录查询失败任务"""
        # 创建任务（设置max_retries=0避免重试）
        task = self.task_manager.create_task(
            task_type=TaskType.FRONTIER_EXPLORATION,
            parameters={},
            task_id='test_failed_001'
        )
        # 设置max_retries=0使任务在第一次失败后直接进入历史
        task.max_retries = 0
        
        # 失败任务（会保存到历史）
        self.task_manager.fail_task('test_failed_001', 'Navigation timeout')
        self.task_manager.remove_task('test_failed_001')
        
        # 从历史查询
        from_history = self.task_manager.get_task('test_failed_001', include_history=True)
        self.assertIsNotNone(from_history)
        self.assertEqual(from_history.state, TaskState.FAILED)
        self.assertIn('Navigation timeout', from_history.error_message)
    
    def test_get_canceled_task_from_history(self):
        """测试从历史记录查询取消任务"""
        # 创建并取消任务
        task = self.task_manager.create_task(
            task_type=TaskType.PATH_PATROL,
            parameters={},
            task_id='test_canceled_001'
        )
        self.task_manager.cancel_task('test_canceled_001')
        self.task_manager.remove_task('test_canceled_001')
        
        # 从历史查询
        from_history = self.task_manager.get_task('test_canceled_001', include_history=True)
        self.assertIsNotNone(from_history)
        self.assertEqual(from_history.state, TaskState.CANCELED)
    
    def test_history_cache_performance(self):
        """测试历史查询缓存性能"""
        # 1. 创建并完成任务
        task = self.task_manager.create_task(
            task_type=TaskType.POINT_TO_POINT,
            parameters={},
            task_id='test_cache_001'
        )
        self.task_manager.complete_task('test_cache_001')
        self.task_manager.remove_task('test_cache_001')
        
        # 2. 第一次查询（从磁盘读取）
        result1 = self.task_manager.get_task('test_cache_001', include_history=True)
        self.assertIsNotNone(result1)
        
        # 3. 验证已缓存
        self.assertIn('test_cache_001', self.task_manager._history_cache)
        
        # 4. 第二次查询（从缓存读取，应该更快）
        result2 = self.task_manager.get_task('test_cache_001', include_history=True)
        self.assertIsNotNone(result2)
        self.assertEqual(result1.task_id, result2.task_id)
    
    def test_history_cache_lru_eviction(self):
        """测试历史缓存LRU淘汰机制"""
        # 设置小的缓存大小方便测试
        original_size = self.task_manager._cache_max_size
        self.task_manager._cache_max_size = 3
        
        try:
            # 创建4个任务并完成
            for i in range(4):
                task_id = f'test_lru_{i:03d}'
                task = self.task_manager.create_task(
                    task_type=TaskType.POINT_TO_POINT,
                    parameters={},
                    task_id=task_id
                )
                self.task_manager.complete_task(task_id)
                self.task_manager.remove_task(task_id)
                
                # 从历史查询（触发缓存）
                self.task_manager.get_task(task_id, include_history=True)
            
            # 验证缓存大小被限制
            self.assertLessEqual(len(self.task_manager._history_cache), 3)
            
            # 验证最新的3个任务在缓存中
            self.assertIn('test_lru_001', self.task_manager._history_cache)
            self.assertIn('test_lru_002', self.task_manager._history_cache)
            self.assertIn('test_lru_003', self.task_manager._history_cache)
            
            # 最旧的任务应该被淘汰
            self.assertNotIn('test_lru_000', self.task_manager._history_cache)
        
        finally:
            # 恢复缓存大小
            self.task_manager._cache_max_size = original_size
    
    def test_nonexistent_task_query(self):
        """测试查询不存在的任务"""
        # 查询一个从未创建的任务
        result = self.task_manager.get_task('nonexistent_task', include_history=True)
        self.assertIsNone(result)
    
    def test_clear_history_cache(self):
        """测试清空历史缓存"""
        # 创建并查询任务（填充缓存）
        task = self.task_manager.create_task(
            task_type=TaskType.POINT_TO_POINT,
            parameters={},
            task_id='test_clear_001'
        )
        self.task_manager.complete_task('test_clear_001')
        self.task_manager.remove_task('test_clear_001')
        self.task_manager.get_task('test_clear_001', include_history=True)
        
        # 验证缓存不为空
        self.assertGreater(len(self.task_manager._history_cache), 0)
        
        # 清空缓存
        self.task_manager.clear_history_cache()
        
        # 验证缓存已清空
        self.assertEqual(len(self.task_manager._history_cache), 0)
        
        # 验证仍可从磁盘查询
        result = self.task_manager.get_task('test_clear_001', include_history=True)
        self.assertIsNotNone(result)
    
    def test_multiple_tasks_same_day(self):
        """测试同一天创建多个任务的历史查询"""
        task_ids = []
        
        # 创建多个任务
        for i in range(5):
            task_id = f'test_multi_{i:03d}'
            task = self.task_manager.create_task(
                task_type=TaskType.POINT_TO_POINT,
                parameters={'index': i},
                task_id=task_id
            )
            self.task_manager.complete_task(task_id)
            self.task_manager.remove_task(task_id)
            task_ids.append(task_id)
        
        # 验证所有任务都可以从历史查询
        for task_id in task_ids:
            result = self.task_manager.get_task(task_id, include_history=True)
            self.assertIsNotNone(result, f"任务 {task_id} 应该能从历史查询到")
            self.assertEqual(result.state, TaskState.COMPLETED)
    
    def test_history_file_corruption_resilience(self):
        """测试历史文件损坏时的容错性"""
        # 创建正常任务
        task = self.task_manager.create_task(
            task_type=TaskType.POINT_TO_POINT,
            parameters={},
            task_id='test_resilience_001'
        )
        self.task_manager.complete_task('test_resilience_001')
        self.task_manager.remove_task('test_resilience_001')
        
        # 损坏历史文件
        date_str = datetime.now().strftime('%Y%m%d')
        history_file = os.path.join(
            self.task_manager._history_dir, 
            f'tasks_{date_str}.json'
        )
        with open(history_file, 'w') as f:
            f.write('{ invalid json content }}')
        
        # 查询应该返回None而不是抛出异常
        result = self.task_manager.get_task('test_resilience_001', include_history=True)
        self.assertIsNone(result)
    
    def test_active_task_priority_over_history(self):
        """测试活动任务优先级高于历史记录"""
        # 1. 创建任务并完成（保存到历史）
        task = self.task_manager.create_task(
            task_type=TaskType.POINT_TO_POINT,
            parameters={'version': 1},
            task_id='test_priority_001'
        )
        self.task_manager.complete_task('test_priority_001')
        # 不删除任务，保持在活动任务中
        
        # 2. 手动修改历史文件（模拟旧版本）
        date_str = datetime.now().strftime('%Y%m%d')
        history_file = os.path.join(
            self.task_manager._history_dir,
            f'tasks_{date_str}.json'
        )
        
        # 3. 查询任务（应该返回活动任务，不是历史）
        result = self.task_manager.get_task('test_priority_001', include_history=True)
        self.assertIsNotNone(result)
        self.assertEqual(result.parameters.get('version'), 1)
        self.assertEqual(result.state, TaskState.COMPLETED)


class TestHistoryQueryEdgeCases(unittest.TestCase):
    """历史查询边界条件测试"""
    
    def setUp(self):
        """测试前准备"""
        self.temp_dir = tempfile.mkdtemp(prefix='test_edge_cases_')
        self.task_manager = TaskManager(persistence_dir=self.temp_dir)
    
    def tearDown(self):
        """测试后清理"""
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)
    
    def test_empty_history_directory(self):
        """测试空历史目录"""
        # 确保历史目录为空
        history_files = os.listdir(self.task_manager._history_dir)
        for f in history_files:
            os.remove(os.path.join(self.task_manager._history_dir, f))
        
        # 查询不存在的任务
        result = self.task_manager.get_task('nonexistent', include_history=True)
        self.assertIsNone(result)
    
    def test_future_date_history_file(self):
        """测试未来日期的历史文件（不应该被查询）"""
        # 创建未来日期的历史文件
        future_date = (datetime.now() + timedelta(days=1)).strftime('%Y%m%d')
        future_file = os.path.join(
            self.task_manager._history_dir,
            f'tasks_{future_date}.json'
        )
        
        fake_task = {
            'task_id': 'future_task',
            'task_type': 'POINT_TO_POINT',
            'state': 'COMPLETED',
            'parameters': {},
            'priority': 5,
            'progress': 1.0,
            'created_at': datetime.now().isoformat()
        }
        
        with open(future_file, 'w') as f:
            json.dump([fake_task], f)
        
        # 查询任务（只搜索今天+昨天，不应该找到）
        result = self.task_manager.get_task('future_task', include_history=True)
        self.assertIsNone(result, "不应该查询未来日期的历史文件")


def run_tests():
    """运行所有测试"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加测试类
    suite.addTests(loader.loadTestsFromTestCase(TestTaskManagerHistory))
    suite.addTests(loader.loadTestsFromTestCase(TestHistoryQueryEdgeCases))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 返回测试结果
    return result.wasSuccessful()


if __name__ == '__main__':
    import sys
    success = run_tests()
    sys.exit(0 if success else 1)
