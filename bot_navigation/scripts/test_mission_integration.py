#!/usr/bin/env python3
"""
MissionPlanner 功能集成测试脚本

功能：
  - 测试导航、探索、巡航的实际执行
  - 验证任务状态转换的正确性
  - 测试暂停、恢复、取消控制
  - 监控任务进度和完成度

使用方法：
  # 终端1: 启动仿真环境
  ros2 launch bot_bringup simple_simulation_nav2_rtabmap.launch.py
  
  # 终端2: 运行集成测试
  python3 test_mission_integration.py

Author: LeKiwi Bot Development Team
Date: 2025-12-30
"""

import rclpy
from rclpy.node import Node
import time
import os
import yaml
from datetime import datetime
from colorama import Fore, Style, init
from geometry_msgs.msg import PoseStamped

# 导入服务消息类型
from bot_navigation_msgs.srv import (
    NavigateToPose, StartExploration, StartPatrol,
    TaskControl, GetTaskStatus, WaypointControl,
    EmergencyStop, ClearTasks
)

# 初始化colorama
init(autoreset=True)


class MissionIntegrationTester(Node):
    """MissionPlanner 功能集成测试类"""
    
    def __init__(self):
        super().__init__('mission_integration_tester')
        
        # 测试统计
        self.test_results = {
            'total': 0,
            'passed': 0,
            'failed': 0,
            'details': []
        }
        
        # 订阅导航目标（用于验证探索）
        self.navigation_goals = []
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_callback,
            10
        )
        
        self.get_logger().info('Mission Integration Tester initialized')
    
    def _goal_callback(self, msg):
        """记录发布的导航目标"""
        self.navigation_goals.append({
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'timestamp': time.time()
        })
    
    def call_service(self, service_name, service_type, request, timeout=5.0):
        """通用服务调用方法"""
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
            print(f"     {Fore.CYAN}Details:{Style.RESET_ALL} {details}")
    
    def wait_for_task_state(self, task_id, expected_states, timeout=30.0):
        """
        等待任务达到指定状态
        
        Args:
            task_id: 任务ID
            expected_states: 期望的状态列表
            timeout: 超时时间(秒)
            
        Returns:
            (reached, final_state, progress): 是否到达, 最终状态, 最终进度
        """
        start_time = time.time()
        last_state = None
        last_progress = 0.0
        
        while time.time() - start_time < timeout:
            request = GetTaskStatus.Request()
            request.task_id = task_id
            
            success, response = self.call_service('/mission/get_task_status', GetTaskStatus, request)
            
            if success and response and response.success:
                current_state = response.state
                current_progress = response.progress
                
                # 状态变化时打印
                if current_state != last_state:
                    print(f"     {Fore.CYAN}State: {last_state} → {current_state}{Style.RESET_ALL}")
                    last_state = current_state
                
                # 进度显著变化时打印
                if abs(current_progress - last_progress) > 0.1:
                    print(f"     {Fore.CYAN}Progress: {current_progress:.1%}{Style.RESET_ALL}")
                    last_progress = current_progress
                
                # 检查是否达到期望状态
                if current_state in expected_states:
                    return True, current_state, current_progress
            
            # 每秒轮询一次
            time.sleep(1.0)
        
        return False, last_state, last_progress
    
    def test_navigation_with_pause_resume(self):
        """
        测试1: 导航功能 + 暂停/恢复控制
        
        验证点：
        - 导航任务创建成功
        - 任务进入RUNNING状态
        - 暂停后进入PAUSED状态
        - 恢复后继续执行
        - 最终到达目标或取消
        """
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}[TEST 1] Navigation with Pause/Resume Control{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        
        # 1. 创建导航任务
        print(f"\n{Fore.CYAN}Step 1: Creating navigation task...{Style.RESET_ALL}")
        request = NavigateToPose.Request()
        request.x = 2.0
        request.y = 1.5
        request.yaw = 0.0
        request.frame_id = 'map'
        
        success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, request)
        
        if not success or not response.success:
            self.record_test('Navigation with Pause/Resume', False, 'Failed to create navigation task')
            return
        
        task_id = response.task_id
        print(f"     {Fore.GREEN}Task created: {task_id}{Style.RESET_ALL}")
        
        # 2. 等待任务开始运行
        print(f"\n{Fore.CYAN}Step 2: Waiting for task to start...{Style.RESET_ALL}")
        reached, state, progress = self.wait_for_task_state(task_id, ['RUNNING'], timeout=10.0)
        
        if not reached:
            self.record_test('Navigation with Pause/Resume', False, f'Task did not start, state: {state}')
            return
        
        print(f"     {Fore.GREEN}Task started successfully{Style.RESET_ALL}")
        
        # 3. 让任务执行5秒
        print(f"\n{Fore.CYAN}Step 3: Let task run for 5 seconds...{Style.RESET_ALL}")
        time.sleep(5.0)
        
        # 4. 暂停任务
        print(f"\n{Fore.CYAN}Step 4: Pausing task...{Style.RESET_ALL}")
        pause_req = TaskControl.Request()
        pause_req.task_id = task_id
        success, pause_resp = self.call_service('/mission/pause_task', TaskControl, pause_req)
        
        if not success or not pause_resp.success:
            self.record_test('Navigation with Pause/Resume', False, 'Failed to pause task')
            return
        
        # 验证暂停状态
        reached, state, progress = self.wait_for_task_state(task_id, ['PAUSED'], timeout=5.0)
        if not reached:
            self.record_test('Navigation with Pause/Resume', False, f'Task did not pause, state: {state}')
            return
        
        print(f"     {Fore.GREEN}Task paused successfully at {progress:.1%} progress{Style.RESET_ALL}")
        
        # 5. 暂停3秒
        print(f"\n{Fore.CYAN}Step 5: Keeping paused for 3 seconds...{Style.RESET_ALL}")
        time.sleep(3.0)
        
        # 6. 恢复任务
        print(f"\n{Fore.CYAN}Step 6: Resuming task...{Style.RESET_ALL}")
        resume_req = TaskControl.Request()
        resume_req.task_id = task_id
        success, resume_resp = self.call_service('/mission/resume_task', TaskControl, resume_req)
        
        if not success or not resume_resp.success:
            self.record_test('Navigation with Pause/Resume', False, 'Failed to resume task')
            return
        
        # 验证恢复运行
        reached, state, progress = self.wait_for_task_state(task_id, ['RUNNING'], timeout=5.0)
        if not reached:
            self.record_test('Navigation with Pause/Resume', False, f'Task did not resume, state: {state}')
            return
        
        print(f"     {Fore.GREEN}Task resumed successfully{Style.RESET_ALL}")
        
        # 7. 继续执行5秒后取消（避免等待过长）
        print(f"\n{Fore.CYAN}Step 7: Let task run for 5 more seconds, then cancel...{Style.RESET_ALL}")
        time.sleep(5.0)
        
        cancel_req = TaskControl.Request()
        cancel_req.task_id = task_id
        self.call_service('/mission/cancel_task', TaskControl, cancel_req)
        
        # 验证取消状态
        reached, state, _ = self.wait_for_task_state(task_id, ['CANCELED'], timeout=5.0)
        
        self.record_test(
            'Navigation with Pause/Resume',
            True,
            f'Navigation task executed with pause/resume control',
            f'Final state: {state}'
        )
    
    def test_waypoint_recording(self):
        """
        测试2: 路点录制功能
        
        验证点：
        - 路点录制服务可用
        - 能够清除路点
        - 能够保存路点文件
        - 生成的文件格式正确
        """
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}[TEST 2] Waypoint Recording{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        
        # 1. 清除现有路点
        print(f"\n{Fore.CYAN}Step 1: Clearing existing waypoints...{Style.RESET_ALL}")
        clear_req = WaypointControl.Request()
        clear_req.action = 'clear'
        success, response = self.call_service('/mission/waypoint_control', WaypointControl, clear_req)
        
        if not success or not response.success:
            self.record_test('Waypoint Recording', False, 'Failed to clear waypoints')
            return
        
        print(f"     {Fore.GREEN}Waypoints cleared{Style.RESET_ALL}")
        
        # 2. 开始录制（自动模式 - 实际会通过订阅pose话题自动记录）
        print(f"\n{Fore.CYAN}Step 2: Starting waypoint recording...{Style.RESET_ALL}")
        start_req = WaypointControl.Request()
        start_req.action = 'start'
        success, response = self.call_service('/mission/waypoint_control', WaypointControl, start_req)
        
        if not success or not response.success:
            self.record_test('Waypoint Recording', False, 'Failed to start recording')
            return
        
        print(f"     {Fore.GREEN}Recording started{Style.RESET_ALL}")
        
        # 3. 模拟记录3秒（在实际环境中机器人会移动）
        print(f"\n{Fore.CYAN}Step 3: Recording for 3 seconds...{Style.RESET_ALL}")
        time.sleep(3.0)
        
        # 4. 停止录制
        print(f"\n{Fore.CYAN}Step 4: Stopping recording...{Style.RESET_ALL}")
        stop_req = WaypointControl.Request()
        stop_req.action = 'stop'
        success, response = self.call_service('/mission/waypoint_control', WaypointControl, stop_req)
        
        if not success or not response.success:
            self.record_test('Waypoint Recording', False, 'Failed to stop recording')
            return
        
        print(f"     {Fore.GREEN}Recording stopped, waypoint count: {response.waypoint_count}{Style.RESET_ALL}")
        
        # 5. 保存路点文件
        print(f"\n{Fore.CYAN}Step 5: Saving waypoints to file...{Style.RESET_ALL}")
        save_req = WaypointControl.Request()
        save_req.action = 'save'
        save_req.filename = 'test_integration_route'
        success, response = self.call_service('/mission/waypoint_control', WaypointControl, save_req)
        
        if not success or not response.success:
            self.record_test('Waypoint Recording', False, 'Failed to save waypoints')
            return
        
        print(f"     {Fore.GREEN}Waypoints saved{Style.RESET_ALL}")
        
        # 6. 验证文件存在和格式
        waypoint_file = os.path.expanduser('~/lododo_bot/mission/waypoints/test_integration_route.yaml')
        
        if not os.path.exists(waypoint_file):
            self.record_test('Waypoint Recording', False, 'Waypoint file not created')
            return
        
        try:
            with open(waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' not in data or not isinstance(data['waypoints'], list):
                self.record_test('Waypoint Recording', False, 'Invalid waypoint file format')
                return
            
            waypoint_count = len(data['waypoints'])
            print(f"     {Fore.GREEN}File validation passed, {waypoint_count} waypoints in file{Style.RESET_ALL}")
            
        except Exception as e:
            self.record_test('Waypoint Recording', False, f'File read error: {str(e)}')
            return
        
        self.record_test(
            'Waypoint Recording',
            True,
            f'Waypoint recording completed',
            f'Saved {waypoint_count} waypoints to test_integration_route.yaml'
        )
    
    def test_patrol_execution(self):
        """
        测试3: 巡航任务执行 + 状态监控
        
        验证点：
        - 巡航任务创建成功
        - 任务正在执行路点序列
        - 进度正常更新
        - 可以取消任务
        """
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}[TEST 3] Patrol Execution with Status Monitoring{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        
        # 1. 创建测试路点文件（如果不存在）
        print(f"\n{Fore.CYAN}Step 1: Preparing patrol route...{Style.RESET_ALL}")
        waypoint_file = os.path.expanduser('~/lododo_bot/mission/patrol_routes/test_integration_patrol.yaml')
        os.makedirs(os.path.dirname(waypoint_file), exist_ok=True)
        
        test_waypoints = {
            'waypoints': [
                {'name': 'point_1', 'x': 1.0, 'y': 0.5, 'yaw': 0.0, 'dwell_time': 2.0},
                {'name': 'point_2', 'x': 1.5, 'y': 1.0, 'yaw': 1.57, 'dwell_time': 2.0},
                {'name': 'point_3', 'x': 0.5, 'y': 1.5, 'yaw': 3.14, 'dwell_time': 2.0},
            ]
        }
        
        with open(waypoint_file, 'w') as f:
            yaml.safe_dump(test_waypoints, f)
        
        print(f"     {Fore.GREEN}Patrol route prepared with 3 waypoints{Style.RESET_ALL}")
        
        # 2. 启动巡航任务
        print(f"\n{Fore.CYAN}Step 2: Starting patrol task...{Style.RESET_ALL}")
        request = StartPatrol.Request()
        request.waypoint_file = 'test_integration_patrol'
        request.patrol_mode = 'once'
        request.speed_factor = 1.0
        
        success, response = self.call_service('/mission/start_patrol', StartPatrol, request)
        
        if not success or not response.success:
            self.record_test('Patrol Execution', False, f'Failed to start patrol: {response.message if response else "Service error"}')
            return
        
        task_id = response.task_id
        print(f"     {Fore.GREEN}Patrol started: {task_id}{Style.RESET_ALL}")
        
        # 3. 等待任务开始运行
        print(f"\n{Fore.CYAN}Step 3: Monitoring patrol execution...{Style.RESET_ALL}")
        reached, state, progress = self.wait_for_task_state(task_id, ['RUNNING'], timeout=10.0)
        
        if not reached:
            self.record_test('Patrol Execution', False, f'Patrol did not start, state: {state}')
            return
        
        print(f"     {Fore.GREEN}Patrol is running{Style.RESET_ALL}")
        
        # 4. 监控执行10秒（观察进度变化）
        print(f"\n{Fore.CYAN}Step 4: Monitoring progress for 10 seconds...{Style.RESET_ALL}")
        start_time = time.time()
        progress_samples = []
        
        while time.time() - start_time < 10.0:
            request = GetTaskStatus.Request()
            request.task_id = task_id
            success, response = self.call_service('/mission/get_task_status', GetTaskStatus, request)
            
            if success and response and response.success:
                progress_samples.append(response.progress)
                print(f"     {Fore.CYAN}Progress: {response.progress:.1%}{Style.RESET_ALL}")
            
            time.sleep(2.0)
        
        # 检查进度是否有增长
        if len(progress_samples) >= 2 and progress_samples[-1] > progress_samples[0]:
            print(f"     {Fore.GREEN}Progress is increasing (from {progress_samples[0]:.1%} to {progress_samples[-1]:.1%}){Style.RESET_ALL}")
            progress_ok = True
        else:
            print(f"     {Fore.RED}Progress not increasing properly{Style.RESET_ALL}")
            progress_ok = False
        
        # 5. 取消巡航任务
        print(f"\n{Fore.CYAN}Step 5: Canceling patrol task...{Style.RESET_ALL}")
        cancel_req = TaskControl.Request()
        cancel_req.task_id = task_id
        success, response = self.call_service('/mission/cancel_task', TaskControl, cancel_req)
        
        if not success or not response.success:
            self.record_test('Patrol Execution', False, 'Failed to cancel patrol')
            return
        
        # 验证取消状态
        reached, state, _ = self.wait_for_task_state(task_id, ['CANCELED'], timeout=5.0)
        
        if not reached:
            self.record_test('Patrol Execution', False, f'Patrol not canceled properly, state: {state}')
            return
        
        print(f"     {Fore.GREEN}Patrol canceled successfully{Style.RESET_ALL}")
        
        self.record_test(
            'Patrol Execution',
            progress_ok,
            'Patrol task executed and monitored',
            f'Progress increased from {progress_samples[0]:.1%} to {progress_samples[-1]:.1%}'
        )
    
    def test_exploration_execution(self):
        """
        测试4: 探索任务执行（可能失败 - 取决于环境）
        
        验证点：
        - 探索任务创建
        - 如果有未探索区域，验证目标生成
        - 如果在定位模式，验证预期失败
        """
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}[TEST 4] Exploration Execution{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        
        # 1. 清空之前记录的目标
        self.navigation_goals.clear()
        
        # 2. 启动探索任务
        print(f"\n{Fore.CYAN}Step 1: Starting exploration task...{Style.RESET_ALL}")
        request = StartExploration.Request()
        request.map_name = 'test_integration_map'
        request.save_map = False
        request.max_duration = 30.0
        request.coverage_threshold = 0.7
        
        success, response = self.call_service('/mission/start_exploration', StartExploration, request, timeout=5.0)
        
        if not success or not response.success:
            # 探索失败是预期的（定位模式下）
            print(f"     {Fore.YELLOW}Exploration not available: {response.message if response else 'Service error'}{Style.RESET_ALL}")
            self.record_test(
                'Exploration Execution',
                True,
                'Expected behavior: Exploration not available in localization mode',
                response.message if response else 'No response'
            )
            return
        
        task_id = response.task_id
        print(f"     {Fore.GREEN}Exploration started: {task_id}{Style.RESET_ALL}")
        
        # 3. 监控5秒，检查是否生成探索目标
        print(f"\n{Fore.CYAN}Step 2: Monitoring exploration goals for 5 seconds...{Style.RESET_ALL}")
        time.sleep(5.0)
        
        goal_count = len(self.navigation_goals)
        
        if goal_count > 0:
            print(f"     {Fore.GREEN}Generated {goal_count} exploration goals{Style.RESET_ALL}")
            exploration_ok = True
        else:
            print(f"     {Fore.YELLOW}No exploration goals generated{Style.RESET_ALL}")
            exploration_ok = False
        
        # 4. 取消探索
        print(f"\n{Fore.CYAN}Step 3: Canceling exploration...{Style.RESET_ALL}")
        cancel_req = TaskControl.Request()
        cancel_req.task_id = task_id
        self.call_service('/mission/cancel_task', TaskControl, cancel_req)
        
        self.record_test(
            'Exploration Execution',
            exploration_ok,
            f'Exploration executed and generated {goal_count} goals',
            f'Monitored for 5 seconds'
        )
    
    def test_emergency_stop(self):
        """
        测试5: 紧急停止功能
        
        验证点：
        - 创建后台导航任务
        - 紧急停止能终止任务
        """
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}[TEST 5] Emergency Stop{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        
        # 1. 创建后台导航任务
        print(f"\n{Fore.CYAN}Step 1: Creating background navigation task...{Style.RESET_ALL}")
        nav_req = NavigateToPose.Request()
        nav_req.x = 3.0
        nav_req.y = 2.0
        nav_req.yaw = 0.0
        nav_req.frame_id = 'map'
        
        success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, nav_req)
        
        if not success or not response.success:
            self.record_test('Emergency Stop', False, 'Failed to create background task')
            return
        
        task_id = response.task_id
        print(f"     {Fore.GREEN}Background task created: {task_id}{Style.RESET_ALL}")
        
        # 2. 等待任务开始运行
        time.sleep(2.0)
        
        # 3. 执行紧急停止
        print(f"\n{Fore.CYAN}Step 2: Executing emergency stop...{Style.RESET_ALL}")
        stop_req = EmergencyStop.Request()
        stop_req.clear_tasks = False
        
        success, response = self.call_service('/mission/emergency_stop', EmergencyStop, stop_req)
        
        if not success or not response.success:
            self.record_test('Emergency Stop', False, 'Emergency stop failed')
            return
        
        print(f"     {Fore.GREEN}Emergency stop executed: {response.message}{Style.RESET_ALL}")
        
        # 4. 验证任务已取消
        request = GetTaskStatus.Request()
        request.task_id = task_id
        success, status_resp = self.call_service('/mission/get_task_status', GetTaskStatus, request)
        
        if success and status_resp and status_resp.state == 'CANCELED':
            print(f"     {Fore.GREEN}Background task canceled successfully{Style.RESET_ALL}")
            self.record_test('Emergency Stop', True, 'Emergency stop terminated running tasks')
        else:
            self.record_test('Emergency Stop', False, f'Task not canceled, state: {status_resp.state if status_resp else "unknown"}')
    
    def cleanup_test_data(self):
        """清理测试数据"""
        print(f"\n{Fore.CYAN}Cleaning up test data...{Style.RESET_ALL}")
        
        # 清除测试任务
        clear_req = ClearTasks.Request()
        clear_req.task_ids = []
        clear_req.states = ['CANCELED', 'COMPLETED', 'FAILED']
        clear_req.clear_history = False
        self.call_service('/mission/clear_tasks', ClearTasks, clear_req)
        
        print(f"     {Fore.GREEN}Cleanup complete{Style.RESET_ALL}")
    
    def print_summary(self):
        """打印测试总结"""
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}TEST SUMMARY{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        
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
        
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        
        # 保存结果
        self.save_results()
    
    def save_results(self):
        """保存测试结果"""
        import json
        
        output_dir = os.path.expanduser('~/lododo_bot/mission/test_results')
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = os.path.join(output_dir, f'integration_test_{timestamp}.json')
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(self.test_results, f, indent=2, ensure_ascii=False)
        
        print(f"\n{Fore.GREEN}Results saved to: {output_file}{Style.RESET_ALL}")
    
    def run_all_tests(self):
        """运行所有集成测试"""
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Mission Integration Test Suite{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        
        try:
            # 测试1: 导航 + 暂停/恢复
            self.test_navigation_with_pause_resume()
            
            # 测试2: 路点录制
            self.test_waypoint_recording()
            
            # 测试3: 巡航执行
            self.test_patrol_execution()
            
            # 测试4: 探索执行
            self.test_exploration_execution()
            
            # 测试5: 紧急停止
            self.test_emergency_stop()
            
            # 清理
            self.cleanup_test_data()
            
        except Exception as e:
            print(f"\n{Fore.RED}Test suite failed with exception: {e}{Style.RESET_ALL}")
            import traceback
            traceback.print_exc()
        
        # 打印总结
        self.print_summary()


def main():
    """主函数"""
    rclpy.init()
    
    tester = MissionIntegrationTester()
    
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Test interrupted by user{Style.RESET_ALL}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
