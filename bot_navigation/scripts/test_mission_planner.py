#!/usr/bin/env python3
"""
test_mission_planner.py - MissionPlanner 测试脚本

测试 MissionPlanner 的各项功能
"""

import rclpy
from rclpy.node import Node
from bot_navigation_msgs.srv import (
    CreateTask, TaskControl, GetTaskStatus, ListTasks,
    StartExploration, StartPatrol, NavigateToPose, EmergencyStop
)
import time


class MissionPlannerTester(Node):
    def __init__(self):
        super().__init__('mission_planner_tester')
        
        # 创建服务客户端
        self.create_task_cli = self.create_client(CreateTask, '/mission/create_task')
        self.list_tasks_cli = self.create_client(ListTasks, '/mission/list_tasks')
        self.get_status_cli = self.create_client(GetTaskStatus, '/mission/get_task_status')
        self.start_task_cli = self.create_client(TaskControl, '/mission/start_task')
        self.cancel_task_cli = self.create_client(TaskControl, '/mission/cancel_task')
        self.navigate_cli = self.create_client(NavigateToPose, '/mission/navigate_to_pose')
        self.start_exploration_cli = self.create_client(StartExploration, '/mission/start_exploration')
        self.start_patrol_cli = self.create_client(StartPatrol, '/mission/start_patrol')
        self.emergency_stop_cli = self.create_client(EmergencyStop, '/mission/emergency_stop')
        
        # 等待服务可用
        self.get_logger().info('Waiting for MissionPlanner services...')
        self.create_task_cli.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('MissionPlanner services ready!')
    
    def test_create_task(self):
        """测试创建任务"""
        self.get_logger().info('\n=== Test 1: Create Task ===')
        
        request = CreateTask.Request()
        request.task_id = 'test_task_001'
        request.task_type = 'point_to_point'
        request.priority = 5
        request.parameters_keys = ['x', 'y', 'yaw']
        request.parameters_values = ['2.0', '3.0', '1.57']
        
        future = self.create_task_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            self.get_logger().info(
                f"✓ Task created: {response.task_id}, success={response.success}"
            )
            return response.success
        return False
    
    def test_list_tasks(self):
        """测试列出任务"""
        self.get_logger().info('\n=== Test 2: List Tasks ===')
        
        request = ListTasks.Request()
        request.filter = 'all'
        
        future = self.list_tasks_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            self.get_logger().info(f"✓ Found {len(response.task_ids)} tasks:")
            for i, task_id in enumerate(response.task_ids):
                self.get_logger().info(
                    f"  - {task_id}: type={response.task_types[i]}, "
                    f"state={response.states[i]}, priority={response.priorities[i]}"
                )
            return True
        return False
    
    def test_get_task_status(self, task_id='test_task_001'):
        """测试获取任务状态"""
        self.get_logger().info(f'\n=== Test 3: Get Task Status ({task_id}) ===')
        
        request = GetTaskStatus.Request()
        request.task_id = task_id
        
        future = self.get_status_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"✓ Task status: {response.state}, progress={response.progress:.2f}"
                )
            else:
                self.get_logger().info(f"✗ {response.message}")
            return response.success
        return False
    
    def test_navigate_to_pose(self):
        """测试导航到位姿"""
        self.get_logger().info('\n=== Test 4: Navigate to Pose ===')
        
        request = NavigateToPose.Request()
        request.x = 1.5
        request.y = 2.0
        request.yaw = 0.0
        request.frame_id = 'map'
        
        future = self.navigate_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            self.get_logger().info(
                f"✓ Navigation task created: {response.task_id}, success={response.success}"
            )
            return response.success, response.task_id
        return False, ""
    
    def test_start_exploration(self):
        """测试开始探索"""
        self.get_logger().info('\n=== Test 5: Start Exploration ===')
        
        request = StartExploration.Request()
        request.map_name = 'test_exploration_map'
        request.save_map = True
        request.max_duration = 300.0
        request.coverage_threshold = 0.9
        
        future = self.start_exploration_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            self.get_logger().info(
                f"✓ Exploration started: {response.task_id}, success={response.success}"
            )
            return response.success, response.task_id
        return False, ""
    
    def test_cancel_task(self, task_id):
        """测试取消任务"""
        self.get_logger().info(f'\n=== Test 6: Cancel Task ({task_id}) ===')
        
        request = TaskControl.Request()
        request.task_id = task_id
        
        future = self.cancel_task_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            self.get_logger().info(f"✓ Task cancelled: success={response.success}")
            return response.success
        return False
    
    def test_emergency_stop(self):
        """测试紧急停止"""
        self.get_logger().info('\n=== Test 7: Emergency Stop ===')
        
        request = EmergencyStop.Request()
        request.clear_tasks = False
        
        future = self.emergency_stop_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            response = future.result()
            self.get_logger().info(f"✓ Emergency stop: success={response.success}")
            return response.success
        return False
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Starting MissionPlanner Tests')
        self.get_logger().info('='*50)
        
        results = []
        
        # Test 1: Create task
        results.append(('Create Task', self.test_create_task()))
        time.sleep(0.5)
        
        # Test 2: List tasks
        results.append(('List Tasks', self.test_list_tasks()))
        time.sleep(0.5)
        
        # Test 3: Get task status
        results.append(('Get Task Status', self.test_get_task_status()))
        time.sleep(0.5)
        
        # Test 4: Navigate to pose
        success, nav_task_id = self.test_navigate_to_pose()
        results.append(('Navigate to Pose', success))
        time.sleep(0.5)
        
        # Test 5: Start exploration
        success, exp_task_id = self.test_start_exploration()
        results.append(('Start Exploration', success))
        time.sleep(0.5)
        
        # List all tasks
        self.test_list_tasks()
        time.sleep(0.5)
        
        # Test 6: Cancel task
        if nav_task_id:
            results.append(('Cancel Task', self.test_cancel_task(nav_task_id)))
            time.sleep(0.5)
        
        # Test 7: Emergency stop
        results.append(('Emergency Stop', self.test_emergency_stop()))
        
        # Print summary
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Test Summary')
        self.get_logger().info('='*50)
        
        passed = sum(1 for _, result in results if result)
        total = len(results)
        
        for test_name, result in results:
            status = '✓ PASS' if result else '✗ FAIL'
            self.get_logger().info(f"{status}: {test_name}")
        
        self.get_logger().info(f'\nTotal: {passed}/{total} tests passed')
        self.get_logger().info('='*50 + '\n')


def main(args=None):
    rclpy.init(args=args)
    
    tester = MissionPlannerTester()
    
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        tester.get_logger().error(f"Test failed: {str(e)}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
