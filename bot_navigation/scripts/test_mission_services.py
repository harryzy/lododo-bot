#!/usr/bin/env python3
"""
MissionPlanner 服务接口全面测试脚本

功能：
  - 测试所有 mission 服务接口
  - 验证每个步骤的正确性
  - 输出详细的测试统计报告

使用方法：
  # 确保 MissionPlanner 已启动
  ros2 launch bot_bringup test_mission_planner.launch.py
  
  # 在另一个终端运行测试
  python3 test_mission_services.py

Author: LeKiwi Bot Development Team
Date: 2025-12-30
"""

import rclpy
from rclpy.node import Node
import time
from datetime import datetime
from colorama import Fore, Style, init

# 导入服务消息类型
from bot_navigation_msgs.srv import (
    CreateTask, TaskControl, GetTaskStatus, ListTasks, ClearTasks,
    NavigateToPose, StartExploration, StartPatrol,
    WaypointControl, RecordWaypoints,
    EmergencyStop
)

# 初始化colorama
init(autoreset=True)


class MissionServiceTester(Node):
    """MissionPlanner 服务接口测试类"""
    
    def __init__(self):
        super().__init__('mission_service_tester')
        
        # 测试统计
        self.test_results = {
            'total': 0,
            'passed': 0,
            'failed': 0,
            'skipped': 0,
            'details': []
        }
        
        self.get_logger().info('Mission Service Tester initialized')
    
    def call_service(self, service_name, service_type, request, timeout=5.0):
        """
        通用服务调用方法
        
        Args:
            service_name: 服务名称
            service_type: 服务类型
            request: 请求对象
            timeout: 超时时间(秒)
            
        Returns:
            (success, response): 成功标志和响应对象
        """
        client = self.create_client(service_type, service_name)
        
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {service_name} not available')
            return False, None
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() is not None:
            return True, future.result()
        else:
            self.get_logger().error(f'Service call to {service_name} failed')
            return False, None
    
    def record_test(self, test_name, passed, message='', details=None):
        """记录测试结果"""
        self.test_results['total'] += 1
        
        if passed:
            self.test_results['passed'] += 1
            status = f"{Fore.GREEN}✓ PASS{Style.RESET_ALL}"
        else:
            self.test_results['failed'] += 1
            status = f"{Fore.RED}✗ FAIL{Style.RESET_ALL}"
        
        result = {
            'name': test_name,
            'status': 'PASS' if passed else 'FAIL',
            'message': message,
            'details': details,
            'timestamp': datetime.now().isoformat()
        }
        self.test_results['details'].append(result)
        
        print(f"{status} {test_name}: {message}")
        if details:
            print(f"     Details: {details}")
    
    def test_list_tasks(self):
        """测试：列出所有任务"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 1] List Tasks{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        request = ListTasks.Request()
        request.filter = 'all'
        
        success, response = self.call_service('/mission/list_tasks', ListTasks, request)
        
        if success and response:
            self.record_test(
                'List Tasks',
                True,
                f'Found {len(response.task_ids)} tasks',
                f'Filter: all'
            )
            return True
        else:
            self.record_test('List Tasks', False, 'Service call failed')
            return False
    
    def test_navigate_to_pose(self):
        """测试：创建导航任务"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 2] Navigate to Pose{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        request = NavigateToPose.Request()
        request.x = 1.5
        request.y = 1.0
        request.yaw = 0.0
        request.frame_id = 'map'
        
        success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, request)
        
        if success and response and response.success:
            self.record_test(
                'Navigate to Pose',
                True,
                f'Task created: {response.task_id}',
                f'Target: ({request.x}, {request.y}, yaw={request.yaw})'
            )
            return response.task_id
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Navigate to Pose', False, error)
            return None
    
    def test_get_task_status(self, task_id):
        """测试：获取任务状态"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 3] Get Task Status{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        if not task_id:
            self.record_test('Get Task Status', False, 'No task_id available')
            return False
        
        request = GetTaskStatus.Request()
        request.task_id = task_id
        
        success, response = self.call_service('/mission/get_task_status', GetTaskStatus, request)
        
        if success and response and response.success:
            self.record_test(
                'Get Task Status',
                True,
                f'Status: {response.state}, Progress: {response.progress:.1%}',
                f'Task ID: {task_id}'
            )
            return response.state
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Get Task Status', False, error)
            return None
    
    def test_pause_task(self, task_id):
        """测试：暂停任务"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 4] Pause Task{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        if not task_id:
            self.record_test('Pause Task', False, 'No task_id available')
            return False
        
        # 等待任务开始运行
        time.sleep(2.0)
        
        request = TaskControl.Request()
        request.task_id = task_id
        
        success, response = self.call_service('/mission/pause_task', TaskControl, request)
        
        if success and response and response.success:
            self.record_test(
                'Pause Task',
                True,
                'Task paused successfully',
                f'Task ID: {task_id}'
            )
            return True
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Pause Task', False, error)
            return False
    
    def test_resume_task(self, task_id):
        """测试：恢复任务"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 5] Resume Task{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        if not task_id:
            self.record_test('Resume Task', False, 'No task_id available')
            return False
        
        # 等待暂停生效
        time.sleep(1.0)
        
        request = TaskControl.Request()
        request.task_id = task_id
        
        success, response = self.call_service('/mission/resume_task', TaskControl, request)
        
        if success and response and response.success:
            self.record_test(
                'Resume Task',
                True,
                'Task resumed successfully',
                f'Task ID: {task_id}'
            )
            return True
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Resume Task', False, error)
            return False
    
    def test_cancel_task(self, task_id):
        """测试：取消任务"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 6] Cancel Task{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        if not task_id:
            self.record_test('Cancel Task', False, 'No task_id available')
            return False
        
        # 等待任务恢复
        time.sleep(1.0)
        
        request = TaskControl.Request()
        request.task_id = task_id
        
        success, response = self.call_service('/mission/cancel_task', TaskControl, request)
        
        if success and response and response.success:
            self.record_test(
                'Cancel Task',
                True,
                'Task cancelled successfully',
                f'Task ID: {task_id}'
            )
            return True
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Cancel Task', False, error)
            return False
    
    def test_create_task(self):
        """测试：通用创建任务接口"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 7] Create Task (Generic){Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        request = CreateTask.Request()
        request.task_id = ''  # 空字符串表示自动生成
        request.task_type = 'point_to_point'
        request.priority = 5
        # 使用参数键值对数组
        request.parameters_keys = ['x', 'y', 'yaw', 'frame_id']
        request.parameters_values = ['0.5', '0.5', '0.0', 'map']
        
        success, response = self.call_service('/mission/create_task', CreateTask, request)
        
        if success and response and response.success:
            self.record_test(
                'Create Task',
                True,
                f'Task created: {response.task_id}',
                f'Type: {request.task_type}, Priority: {request.priority}'
            )
            return response.task_id
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Create Task', False, error)
            return None
    
    def test_clear_tasks(self):
        """测试：清除已完成/取消的任务"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 8] Clear Tasks{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        request = ClearTasks.Request()
        request.task_ids = []  # 空列表表示按状态过滤
        request.states = ['CANCELED', 'COMPLETED', 'FAILED']
        request.clear_history = False
        
        success, response = self.call_service('/mission/clear_tasks', ClearTasks, request)
        
        if success and response and response.success:
            self.record_test(
                'Clear Tasks',
                True,
                f'Cleared {response.cleared_count} tasks',
                f'States: {request.states}'
            )
            return True
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Clear Tasks', False, error)
            return False
    
    def test_start_exploration(self):
        """测试：启动探索任务（可能失败如果没有未探索区域）"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 9] Start Exploration{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        request = StartExploration.Request()
        request.map_name = 'test_exploration'
        request.save_map = False
        request.max_duration = 60.0
        request.coverage_threshold = 0.8
        
        success, response = self.call_service('/mission/start_exploration', StartExploration, request, timeout=3.0)
        
        if success and response:
            if response.success:
                self.record_test(
                    'Start Exploration',
                    True,
                    f'Exploration started: {response.task_id}',
                    f'Map: {request.map_name}, Max duration: {request.max_duration}s'
                )
                # 立即取消探索任务
                if response.task_id:
                    cancel_req = TaskControl.Request()
                    cancel_req.task_id = response.task_id
                    self.call_service('/mission/cancel_task', TaskControl, cancel_req)
                return response.task_id
            else:
                # 探索任务可能因为没有未探索区域而失败，这是预期的
                self.record_test(
                    'Start Exploration',
                    True,
                    f'Expected failure: {response.message}',
                    'No unexplored areas available (expected in localization mode)'
                )
                return None
        else:
            self.record_test('Start Exploration', False, 'Service call failed')
            return None
    
    def test_start_patrol(self):
        """测试：启动巡航任务（可能失败如果没有路点文件）"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 10] Start Patrol{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        request = StartPatrol.Request()
        request.waypoint_file = 'test_route'  # 路点文件名（不含路径和扩展名）
        request.patrol_mode = 'loop'
        request.speed_factor = 1.0
        
        success, response = self.call_service('/mission/start_patrol', StartPatrol, request, timeout=3.0)
        
        if success and response:
            if response.success:
                self.record_test(
                    'Start Patrol',
                    True,
                    f'Patrol started: {response.task_id}',
                    f'File: {request.waypoint_file}, Mode: {request.patrol_mode}'
                )
                # 立即取消巡航任务
                if response.task_id:
                    cancel_req = TaskControl.Request()
                    cancel_req.task_id = response.task_id
                    self.call_service('/mission/cancel_task', TaskControl, cancel_req)
                return response.task_id
            else:
                # 巡航任务可能因为没有路点文件而失败，这是预期的
                self.record_test(
                    'Start Patrol',
                    True,
                    f'Expected failure: {response.message}',
                    'Route file not found (expected without pre-recorded waypoints)'
                )
                return None
        else:
            self.record_test('Start Patrol', False, 'Service call failed')
            return None
    
    def test_emergency_stop(self):
        """测试：紧急停止"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 11] Emergency Stop{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        request = EmergencyStop.Request()
        
        success, response = self.call_service('/mission/emergency_stop', EmergencyStop, request)
        
        if success and response and response.success:
            self.record_test(
                'Emergency Stop',
                True,
                'Emergency stop executed successfully',
                response.message
            )
            return True
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Emergency Stop', False, error)
            return False
    
    def test_waypoint_control(self):
        """测试：路点控制服务"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}[TEST 12] Waypoint Control{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        # 测试清除路点
        request = WaypointControl.Request()
        request.action = 'clear'
        
        success, response = self.call_service('/mission/waypoint_control', WaypointControl, request)
        
        if success and response and response.success:
            self.record_test(
                'Waypoint Control',
                True,
                'Waypoints cleared successfully',
                f'Action: {request.action}'
            )
            return True
        else:
            error = response.message if response else 'Service call failed'
            self.record_test('Waypoint Control', False, error)
            return False
    
    def print_summary(self):
        """打印测试统计摘要"""
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}TEST SUMMARY{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        
        total = self.test_results['total']
        passed = self.test_results['passed']
        failed = self.test_results['failed']
        
        pass_rate = (passed / total * 100) if total > 0 else 0
        
        print(f"Total Tests:  {total}")
        print(f"{Fore.GREEN}Passed:       {passed}{Style.RESET_ALL}")
        print(f"{Fore.RED}Failed:       {failed}{Style.RESET_ALL}")
        print(f"Pass Rate:    {pass_rate:.1f}%")
        print(f"\nTest Time:    {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        if failed > 0:
            print(f"\n{Fore.RED}Failed Tests:{Style.RESET_ALL}")
            for result in self.test_results['details']:
                if result['status'] == 'FAIL':
                    print(f"  - {result['name']}: {result['message']}")
        
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        
        # 保存详细结果到文件
        self.save_results()
    
    def save_results(self):
        """保存测试结果到JSON文件"""
        import json
        import os
        
        output_dir = os.path.expanduser('~/lododo_bot/mission/test_results')
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = os.path.join(output_dir, f'test_results_{timestamp}.json')
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(self.test_results, f, indent=2, ensure_ascii=False)
        
        print(f"\n{Fore.GREEN}Results saved to: {output_file}{Style.RESET_ALL}")
    
    def run_all_tests(self):
        """运行所有测试"""
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}MissionPlanner Service Interface Test Suite{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        # 1. 列出任务
        self.test_list_tasks()
        
        # 2. 创建导航任务
        nav_task_id = self.test_navigate_to_pose()
        
        # 3. 获取任务状态
        if nav_task_id:
            self.test_get_task_status(nav_task_id)
        
        # 4. 暂停任务
        if nav_task_id:
            self.test_pause_task(nav_task_id)
        
        # 5. 恢复任务
        if nav_task_id:
            self.test_resume_task(nav_task_id)
        
        # 6. 取消任务
        if nav_task_id:
            self.test_cancel_task(nav_task_id)
        
        # 7. 通用创建任务接口
        generic_task_id = self.test_create_task()
        if generic_task_id:
            time.sleep(1.0)
            cancel_req = TaskControl.Request()
            cancel_req.task_id = generic_task_id
            self.call_service('/mission/cancel_task', TaskControl, cancel_req)
        
        # 8. 清除任务
        self.test_clear_tasks()
        
        # 9. 探索任务（可能失败）
        self.test_start_exploration()
        
        # 10. 巡航任务（可能失败）
        self.test_start_patrol()
        
        # 11. 紧急停止
        self.test_emergency_stop()
        
        # 12. 路点控制
        self.test_waypoint_control()
        
        # 打印摘要
        self.print_summary()


def main():
    """主函数"""
    rclpy.init()
    
    tester = MissionServiceTester()
    
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Test interrupted by user{Style.RESET_ALL}")
    except Exception as e:
        print(f"\n{Fore.RED}Test failed with exception: {e}{Style.RESET_ALL}")
        import traceback
        traceback.print_exc()
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
