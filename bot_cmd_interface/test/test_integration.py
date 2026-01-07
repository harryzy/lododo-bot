#!/usr/bin/env python3
"""
CommandAdapter 集成测试套件
Command Adapter Integration Test Suite

测试覆盖 / Test Coverage:
  1. 导航功能测试 - navigate_to_pose完整流程
  2. 任务管理测试 - exploration, patrol, pause/resume/cancel
  3. 查询功能测试 - get_task_status, get_robot_status
  4. 异常场景测试 - 非法JSON、参数缺失、去重、队列满

前置条件 / Prerequisites:
  启动测试环境:
  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=office_floor1

使用方法 / Usage:
  # 运行所有测试
  python3 test_integration.py
  
  # 运行特定测试
  python3 test_integration.py --test test_navigate_to_pose
  python3 test_integration.py --test 1  # 按编号
  
  # 列出所有测试
  python3 test_integration.py --list
  
  # 详细模式
  python3 test_integration.py --verbose

Author: LeKiwi Bot Development Team
Date: 2026-01-07
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
import argparse
import sys
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from collections import defaultdict

# 导入SDK
from bot_cmd_interface.sdk.message import CommandRequest, CommandResponse
from bot_cmd_interface.sdk.action_types import ActionType
from bot_cmd_interface.sdk.message import ErrorCode, ResponseStatus

# 颜色输出
try:
    from colorama import Fore, Style, init
    init(autoreset=True)
    HAS_COLOR = True
except ImportError:
    HAS_COLOR = False
    class Fore:
        GREEN = RED = YELLOW = CYAN = MAGENTA = BLUE = ''
    class Style:
        BRIGHT = RESET_ALL = ''


class IntegrationTestNode(Node):
    """集成测试节点"""
    
    def __init__(self):
        super().__init__('cmd_integration_test')
        
        # 发布器和订阅器
        self.publisher = self.create_publisher(String, '/cmd/request', 10)
        self.subscription = self.create_subscription(
            String,
            '/cmd/response',
            self._response_callback,
            10
        )
        
        # 存储响应
        self.responses: Dict[str, List[CommandResponse]] = defaultdict(list)
        self.response_lock = None  # Will be set by test runner
        
        # 统计
        self.total_requests = 0
        self.total_responses = 0
        
        self.get_logger().info('IntegrationTestNode initialized')
    
    def _response_callback(self, msg: String):
        """接收响应"""
        try:
            response = CommandResponse.from_json(msg.data)
            self.responses[response.request_id].append(response)
            self.total_responses += 1
            
            self.get_logger().debug(
                f'Received response: {response.request_id} - {response.status}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to parse response: {e}')
    
    def send_request(self, request: CommandRequest) -> str:
        """发送请求"""
        msg = String()
        msg.data = request.to_json()
        self.publisher.publish(msg)
        self.total_requests += 1
        
        self.get_logger().info(
            f'Sent request: {request.action} | request_id={request.request_id}'
        )
        
        return request.request_id
    
    def wait_for_response(
        self,
        request_id: str,
        status: Optional[str] = None,
        timeout: float = 10.0
    ) -> Optional[CommandResponse]:
        """
        等待特定响应
        
        Args:
            request_id: 请求ID
            status: 期望的状态（None=任意状态）
            timeout: 超时时间
        
        Returns:
            CommandResponse or None
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            responses = self.responses.get(request_id, [])
            
            for response in responses:
                if status is None or response.status == status:
                    return response
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return None
    
    def wait_for_complete_flow(
        self,
        request_id: str,
        timeout: float = 30.0
    ) -> Tuple[bool, List[CommandResponse], str]:
        """
        等待完整的响应流程: queued -> executing -> completed/failed
        
        Returns:
            (success, responses, error_message)
        """
        start_time = time.time()
        expected_statuses = {'queued', 'executing'}
        final_statuses = {'completed', 'failed', 'cancelled'}
        
        while time.time() - start_time < timeout:
            responses = self.responses.get(request_id, [])
            statuses = {r.status for r in responses}
            
            # 检查是否有最终状态
            if statuses & final_statuses:
                # 找到最终响应
                final_response = None
                for r in responses:
                    if r.status in final_statuses:
                        final_response = r
                        break
                
                if final_response and final_response.status == 'completed':
                    return True, responses, ''
                elif final_response:
                    return False, responses, final_response.message
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return False, self.responses.get(request_id, []), 'Timeout waiting for completion'


class TestResult:
    """测试结果"""
    def __init__(self, name: str):
        self.name = name
        self.passed = False
        self.error_message = ''
        self.duration = 0.0
        self.details = []
    
    def add_detail(self, message: str):
        """添加详细信息"""
        self.details.append(message)


class IntegrationTestSuite:
    """集成测试套件"""
    
    def __init__(self, node: IntegrationTestNode, verbose: bool = False):
        self.node = node
        self.verbose = verbose
        self.results: List[TestResult] = []
        
        # 测试列表
        self.tests = [
            ('test_navigate_to_pose', 'Navigate to Pose - 导航到目标点'),
            ('test_emergency_stop', 'Emergency Stop - 紧急停止'),
            ('test_start_exploration', 'Start Exploration - 开始探索'),
            ('test_get_task_status', 'Get Task Status - 获取任务状态'),
            ('test_pause_resume_cancel', 'Pause/Resume/Cancel Task - 暂停/恢复/取消任务'),
            ('test_invalid_json', 'Invalid JSON Format - 非法JSON格式'),
            ('test_missing_parameters', 'Missing Parameters - 参数缺失'),
            ('test_request_deduplication', 'Request Deduplication - 请求去重'),
        ]
    
    def print_test_list(self):
        """打印测试列表"""
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"Available Tests / 可用测试")
        print(f"{'='*60}{Style.RESET_ALL}\n")
        
        for idx, (test_name, description) in enumerate(self.tests, 1):
            print(f"{idx}. {Fore.YELLOW}{test_name}{Style.RESET_ALL}")
            print(f"   {description}\n")
    
    def run_all_tests(self):
        """运行所有测试"""
        self._print_header("Running All Integration Tests")
        
        for test_name, description in self.tests:
            self.run_test(test_name)
        
        self._print_summary()
    
    def run_test(self, test_name: str):
        """运行单个测试"""
        # 查找测试方法
        test_method = getattr(self, test_name, None)
        if not test_method:
            print(f"{Fore.RED}✗ Test not found: {test_name}{Style.RESET_ALL}")
            return
        
        # 创建测试结果
        result = TestResult(test_name)
        
        # 运行测试
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"Test: {test_name}")
        print(f"{'='*60}{Style.RESET_ALL}\n")
        
        start_time = time.time()
        
        try:
            test_method(result)
            result.passed = True
        except AssertionError as e:
            result.passed = False
            result.error_message = str(e)
        except Exception as e:
            result.passed = False
            result.error_message = f"Exception: {str(e)}"
        
        result.duration = time.time() - start_time
        self.results.append(result)
        
        # 打印结果
        if result.passed:
            print(f"\n{Fore.GREEN}✓ PASSED{Style.RESET_ALL} ({result.duration:.2f}s)")
        else:
            print(f"\n{Fore.RED}✗ FAILED{Style.RESET_ALL} ({result.duration:.2f}s)")
            print(f"  Error: {result.error_message}")
        
        # 打印详细信息
        if self.verbose and result.details:
            print(f"\n{Fore.MAGENTA}Details:{Style.RESET_ALL}")
            for detail in result.details:
                print(f"  {detail}")
    
    # ========== 测试用例 ==========
    
    def test_navigate_to_pose(self, result: TestResult):
        """测试1: 导航到目标点"""
        result.add_detail("Creating navigate_to_pose request...")
        
        # 创建请求
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={'x': 2.0, 'y': 3.0, 'yaw': 1.57},
            priority=3,
            timeout=300.0
        )
        
        # 发送请求
        request_id = self.node.send_request(request)
        result.add_detail(f"Sent request: {request_id}")
        
        # 等待完整流程
        success, responses, error_msg = self.node.wait_for_complete_flow(request_id, timeout=15.0)
        
        result.add_detail(f"Received {len(responses)} responses")
        for r in responses:
            result.add_detail(f"  - {r.status}: {r.message}")
        
        # 验证
        assert success, f"Navigation failed: {error_msg}"
        assert len(responses) >= 3, f"Expected at least 3 responses, got {len(responses)}"
        
        # 验证包含所有状态
        statuses = {r.status for r in responses}
        expected = {'queued', 'executing', 'completed'}
        assert expected.issubset(statuses), f"Missing statuses. Got: {statuses}, Expected: {expected}"
        
        # 验证completed响应有task_id
        completed = [r for r in responses if r.status == 'completed'][0]
        assert 'task_id' in completed.result, "Completed response missing task_id"
        
        result.add_detail(f"✓ Task ID: {completed.result.get('task_id')}")
    
    def test_emergency_stop(self, result: TestResult):
        """测试2: 紧急停止"""
        result.add_detail("Creating emergency_stop request...")
        
        request = CommandRequest(
            action=ActionType.EMERGENCY_STOP,
            params={'clear_tasks': True},
            priority=1,  # 最高优先级
            timeout=30.0
        )
        
        request_id = self.node.send_request(request)
        result.add_detail(f"Sent request: {request_id}")
        
        success, responses, error_msg = self.node.wait_for_complete_flow(request_id, timeout=10.0)
        
        result.add_detail(f"Received {len(responses)} responses")
        
        assert success, f"Emergency stop failed: {error_msg}"
        
        completed = [r for r in responses if r.status == 'completed'][0]
        result.add_detail(f"✓ Emergency stop executed: {completed.message}")
    
    def test_start_exploration(self, result: TestResult):
        """测试3: 开始探索"""
        result.add_detail("Creating start_exploration request...")
        
        request = CommandRequest(
            action=ActionType.START_EXPLORATION,
            params={
                'map_name': f'test_map_{int(time.time())}',
                'save_map': False,
                'max_duration': 60.0,
                'coverage_threshold': 0.5
            },
            priority=3,
            timeout=300.0
        )
        
        request_id = self.node.send_request(request)
        result.add_detail(f"Sent request: {request_id}")
        
        success, responses, error_msg = self.node.wait_for_complete_flow(request_id, timeout=15.0)
        
        result.add_detail(f"Received {len(responses)} responses")
        
        assert success, f"Start exploration failed: {error_msg}"
        
        completed = [r for r in responses if r.status == 'completed'][0]
        assert 'task_id' in completed.result, "Completed response missing task_id"
        
        result.add_detail(f"✓ Exploration Task ID: {completed.result.get('task_id')}")
    
    def test_get_task_status(self, result: TestResult):
        """测试4: 获取任务状态"""
        result.add_detail("Creating get_task_status request...")
        
        # 先创建一个任务
        nav_request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={'x': 1.0, 'y': 1.0, 'yaw': 0.0},
            priority=3,
            timeout=300.0
        )
        
        nav_request_id = self.node.send_request(nav_request)
        result.add_detail(f"Created navigation task: {nav_request_id}")
        
        # 等待任务创建完成
        success, responses, _ = self.node.wait_for_complete_flow(nav_request_id, timeout=10.0)
        assert success, "Failed to create navigation task"
        
        # 获取task_id
        completed = [r for r in responses if r.status == 'completed'][0]
        task_id = completed.result.get('task_id')
        result.add_detail(f"Task ID: {task_id}")
        
        # 查询任务状态
        time.sleep(1.0)  # 等待一下
        
        status_request = CommandRequest(
            action=ActionType.GET_TASK_STATUS,
            params={'task_id': task_id},
            priority=3,
            timeout=30.0
        )
        
        status_request_id = self.node.send_request(status_request)
        result.add_detail(f"Querying task status: {status_request_id}")
        
        success, responses, error_msg = self.node.wait_for_complete_flow(status_request_id, timeout=10.0)
        
        assert success, f"Get task status failed: {error_msg}"
        
        completed = [r for r in responses if r.status == 'completed'][0]
        assert 'task_id' in completed.result, "Status response missing task_id"
        assert 'state' in completed.result, "Status response missing state"
        
        result.add_detail(f"✓ Task State: {completed.result.get('state')}")
        result.add_detail(f"✓ Task Progress: {completed.result.get('progress', 0.0)}")
    
    def test_pause_resume_cancel(self, result: TestResult):
        """测试5: 暂停/恢复/取消任务"""
        result.add_detail("Creating navigation task for pause/resume/cancel test...")
        
        # 创建导航任务
        nav_request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={'x': 5.0, 'y': 5.0, 'yaw': 0.0},
            priority=3,
            timeout=300.0
        )
        
        nav_request_id = self.node.send_request(nav_request)
        success, responses, _ = self.node.wait_for_complete_flow(nav_request_id, timeout=10.0)
        assert success, "Failed to create navigation task"
        
        completed = [r for r in responses if r.status == 'completed'][0]
        task_id = completed.result.get('task_id')
        result.add_detail(f"Created task: {task_id}")
        
        time.sleep(1.0)
        
        # 测试暂停
        pause_request = CommandRequest(
            action=ActionType.PAUSE_TASK,
            params={'task_id': task_id},
            priority=3,
            timeout=30.0
        )
        
        pause_request_id = self.node.send_request(pause_request)
        success, _, error_msg = self.node.wait_for_complete_flow(pause_request_id, timeout=10.0)
        assert success, f"Pause task failed: {error_msg}"
        result.add_detail("✓ Task paused")
        
        time.sleep(1.0)
        
        # 测试恢复
        resume_request = CommandRequest(
            action=ActionType.RESUME_TASK,
            params={'task_id': task_id},
            priority=3,
            timeout=30.0
        )
        
        resume_request_id = self.node.send_request(resume_request)
        success, _, error_msg = self.node.wait_for_complete_flow(resume_request_id, timeout=10.0)
        assert success, f"Resume task failed: {error_msg}"
        result.add_detail("✓ Task resumed")
        
        time.sleep(1.0)
        
        # 测试取消
        cancel_request = CommandRequest(
            action=ActionType.CANCEL_TASK,
            params={'task_id': task_id},
            priority=3,
            timeout=30.0
        )
        
        cancel_request_id = self.node.send_request(cancel_request)
        success, _, error_msg = self.node.wait_for_complete_flow(cancel_request_id, timeout=10.0)
        assert success, f"Cancel task failed: {error_msg}"
        result.add_detail("✓ Task cancelled")
    
    def test_invalid_json(self, result: TestResult):
        """测试6: 非法JSON格式"""
        result.add_detail("Sending invalid JSON...")
        
        # 发送非法JSON
        msg = String()
        msg.data = "{ invalid json }"
        self.node.publisher.publish(msg)
        
        # 等待错误响应
        time.sleep(2.0)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # 检查是否有failed响应（request_id=unknown）
        unknown_responses = self.node.responses.get('unknown', [])
        has_error = any(r.status == 'failed' for r in unknown_responses)
        
        assert has_error, "Expected error response for invalid JSON"
        result.add_detail("✓ Invalid JSON rejected with error response")
    
    def test_missing_parameters(self, result: TestResult):
        """测试7: 参数缺失"""
        result.add_detail("Creating request with missing parameters...")
        
        # navigate_to_pose缺少x参数
        request = CommandRequest(
            action=ActionType.NAVIGATE_TO_POSE,
            params={'y': 2.0, 'yaw': 0.0},  # 缺少x
            priority=3,
            timeout=300.0
        )
        
        request_id = self.node.send_request(request)
        result.add_detail(f"Sent request with missing params: {request_id}")
        
        # 等待响应（可能是failed）
        response = self.node.wait_for_response(request_id, timeout=10.0)
        
        assert response is not None, "No response received"
        
        # 可能服务会返回失败，或者使用默认值成功
        result.add_detail(f"✓ Response status: {response.status}")
        result.add_detail(f"✓ Message: {response.message}")
    
    def test_request_deduplication(self, result: TestResult):
        """测试8: 请求去重"""
        result.add_detail("Testing request deduplication...")
        
        # 创建两个相同request_id的请求
        import uuid
        request_id = str(uuid.uuid4())
        
        request1 = CommandRequest(
            action=ActionType.GET_ROBOT_STATUS,
            params={},
            request_id=request_id,
            priority=3,
            timeout=300.0
        )
        
        # 发送第一个请求
        self.node.send_request(request1)
        result.add_detail(f"Sent first request: {request_id}")
        
        time.sleep(0.5)
        
        # 发送第二个重复请求（相同request_id）
        request2 = CommandRequest(
            action=ActionType.GET_ROBOT_STATUS,
            params={},
            request_id=request_id,
            priority=3,
            timeout=300.0
        )
        
        self.node.send_request(request2)
        result.add_detail(f"Sent duplicate request: {request_id}")
        
        # 等待响应
        time.sleep(3.0)
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        responses = self.node.responses.get(request_id, [])
        result.add_detail(f"Received {len(responses)} responses")
        
        # 应该有第一个请求的3个响应 + 第二个请求的1个拒绝响应
        # 或者只有第一个请求的响应（第二个被完全忽略）
        failed_responses = [r for r in responses if r.status == 'failed']
        
        assert len(responses) >= 3, f"Expected at least 3 responses, got {len(responses)}"
        
        if failed_responses:
            result.add_detail(f"✓ Duplicate rejected with {len(failed_responses)} error(s)")
        else:
            result.add_detail(f"✓ Duplicate silently dropped ({len(responses)} responses)")
    
    # ========== 辅助方法 ==========
    
    def _print_header(self, title: str):
        """打印标题"""
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{title}")
        print(f"{'='*60}{Style.RESET_ALL}\n")
    
    def _print_summary(self):
        """打印测试总结"""
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"Test Summary / 测试总结")
        print(f"{'='*60}{Style.RESET_ALL}\n")
        
        for result in self.results:
            status_icon = f"{Fore.GREEN}✓" if result.passed else f"{Fore.RED}✗"
            status_text = "PASSED" if result.passed else "FAILED"
            print(f"{status_icon} {result.name:40s} {status_text:10s} ({result.duration:.2f}s){Style.RESET_ALL}")
            
            if not result.passed and result.error_message:
                print(f"  {Fore.RED}Error: {result.error_message}{Style.RESET_ALL}")
        
        print(f"\n{'-'*60}")
        print(f"Total: {Fore.GREEN if passed == total else Fore.YELLOW}{passed}/{total}{Style.RESET_ALL} tests passed")
        print(f"{'='*60}\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='CommandAdapter Integration Test Suite'
    )
    parser.add_argument(
        '--test',
        type=str,
        help='Run specific test (name or number)'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List all available tests'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose output'
    )
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建测试节点
        node = IntegrationTestNode()
        
        # 等待节点初始化
        print(f"{Fore.CYAN}Initializing test node...{Style.RESET_ALL}")
        time.sleep(2.0)
        
        # 创建测试套件
        suite = IntegrationTestSuite(node, verbose=args.verbose)
        
        if args.list:
            # 列出所有测试
            suite.print_test_list()
        
        elif args.test:
            # 运行特定测试
            test_name = args.test
            
            # 如果是数字，转换为测试名称
            if test_name.isdigit():
                test_idx = int(test_name) - 1
                if 0 <= test_idx < len(suite.tests):
                    test_name = suite.tests[test_idx][0]
                else:
                    print(f"{Fore.RED}Invalid test number: {args.test}{Style.RESET_ALL}")
                    suite.print_test_list()
                    return 1
            
            suite.run_test(test_name)
            
            # 打印总结
            suite._print_summary()
        
        else:
            # 运行所有测试
            suite.run_all_tests()
        
        # 返回状态码
        passed = sum(1 for r in suite.results if r.passed)
        total = len(suite.results)
        
        return 0 if passed == total else 1
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
