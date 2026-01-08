#!/usr/bin/env python3
"""
CommandAdapter 性能测试套件
Command Adapter Performance Test Suite

测试覆盖 / Test Coverage:
  1. 吞吐量测试 - Throughput testing (requests/second)
  2. 并发请求处理 - Concurrent request handling
  3. 响应时间分布 - Response time distribution
  4. 队列容量测试 - Queue capacity testing
  5. 内存使用测试 - Memory usage under load
  6. 长时间稳定性测试 - Long-running stability

前置条件 / Prerequisites:
  启动测试环境:
  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=exploration_test

使用方法 / Usage:
  # 运行所有性能测试
  python3 test_performance.py
  
  # 运行特定测试
  python3 test_performance.py --test throughput
  python3 test_performance.py --test concurrent
  
  # 列出所有测试
  python3 test_performance.py --list
  
  # 快速模式（减少测试时间）
  python3 test_performance.py --quick

Author: LeKiwi Bot Development Team
Date: 2026-01-08
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
import argparse
import sys
import threading
import statistics
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from collections import defaultdict, deque
import psutil
import os

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
        GREEN = RED = YELLOW = CYAN = MAGENTA = BLUE = WHITE = ''
    class Style:
        BRIGHT = RESET_ALL = DIM = ''


class PerformanceMonitor:
    """性能监控器"""
    
    def __init__(self, process_name: str = 'command_adapter'):
        self.process_name = process_name
        self.process = None
        self._find_process()
        
        # 监控数据
        self.cpu_samples = deque(maxlen=100)
        self.memory_samples = deque(maxlen=100)
        self.start_time = None
        self.monitoring = False
        self.monitor_thread = None
    
    def _find_process(self):
        """查找目标进程"""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = proc.info['cmdline']
                if cmdline and any(self.process_name in arg for arg in cmdline):
                    self.process = proc
                    return
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    
    def start_monitoring(self):
        """开始监控"""
        if not self.process:
            return
        
        self.monitoring = True
        self.start_time = time.time()
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """停止监控"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
    
    def _monitor_loop(self):
        """监控循环"""
        while self.monitoring:
            try:
                if self.process and self.process.is_running():
                    # CPU使用率
                    cpu_percent = self.process.cpu_percent(interval=0.1)
                    self.cpu_samples.append(cpu_percent)
                    
                    # 内存使用
                    mem_info = self.process.memory_info()
                    memory_mb = mem_info.rss / 1024 / 1024
                    self.memory_samples.append(memory_mb)
                
                time.sleep(0.5)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                break
    
    def get_stats(self) -> Dict:
        """获取统计数据"""
        if not self.cpu_samples or not self.memory_samples:
            return {
                'cpu_avg': 0.0,
                'cpu_max': 0.0,
                'memory_avg': 0.0,
                'memory_max': 0.0,
                'duration': 0.0
            }
        
        return {
            'cpu_avg': statistics.mean(self.cpu_samples),
            'cpu_max': max(self.cpu_samples),
            'cpu_min': min(self.cpu_samples),
            'memory_avg': statistics.mean(self.memory_samples),
            'memory_max': max(self.memory_samples),
            'memory_min': min(self.memory_samples),
            'duration': time.time() - self.start_time if self.start_time else 0.0
        }


class PerformanceTestNode(Node):
    """性能测试节点"""
    
    def __init__(self):
        super().__init__('cmd_performance_test')
        
        # 发布器和订阅器
        self.publisher = self.create_publisher(String, '/cmd/request', 10)
        self.subscription = self.create_subscription(
            String,
            '/cmd/response',
            self._response_callback,
            10
        )
        
        # 存储响应和统计
        self.responses: Dict[str, List[CommandResponse]] = defaultdict(list)
        self.response_times: Dict[str, float] = {}  # request_id -> completion_time
        self.send_times: Dict[str, float] = {}  # request_id -> send_time
        
        # 锁
        self.lock = threading.Lock()
        
        # 统计
        self.total_requests = 0
        self.total_responses = 0
        self.completed_count = 0
        self.failed_count = 0
        
        self.get_logger().info('PerformanceTestNode initialized')
    
    def _response_callback(self, msg: String):
        """接收响应"""
        try:
            response = CommandResponse.from_json(msg.data)
            
            with self.lock:
                self.responses[response.request_id].append(response)
                self.total_responses += 1
                
                # 记录完成时间
                if response.status in ['completed', 'failed']:
                    if response.request_id in self.send_times:
                        elapsed = time.time() - self.send_times[response.request_id]
                        self.response_times[response.request_id] = elapsed
                    
                    if response.status == 'completed':
                        self.completed_count += 1
                    else:
                        self.failed_count += 1
        
        except Exception as e:
            self.get_logger().error(f'Failed to parse response: {e}')
    
    def send_request(self, request: CommandRequest) -> str:
        """发送请求"""
        msg = String()
        msg.data = request.to_json()
        
        with self.lock:
            self.send_times[request.request_id] = time.time()
            self.total_requests += 1
        
        self.publisher.publish(msg)
        return request.request_id
    
    def wait_for_completions(self, count: int, timeout: float = 60.0) -> bool:
        """等待指定数量的请求完成"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self.lock:
                total_completed = self.completed_count + self.failed_count
                if total_completed >= count:
                    return True
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return False
    
    def get_response_time_stats(self) -> Dict:
        """获取响应时间统计"""
        with self.lock:
            if not self.response_times:
                return {
                    'count': 0,
                    'mean': 0.0,
                    'median': 0.0,
                    'min': 0.0,
                    'max': 0.0,
                    'p95': 0.0,
                    'p99': 0.0
                }
            
            times = sorted(self.response_times.values())
            count = len(times)
            
            return {
                'count': count,
                'mean': statistics.mean(times),
                'median': statistics.median(times),
                'min': min(times),
                'max': max(times),
                'p95': times[int(count * 0.95)] if count > 0 else 0.0,
                'p99': times[int(count * 0.99)] if count > 0 else 0.0,
                'stdev': statistics.stdev(times) if count > 1 else 0.0
            }
    
    def reset_stats(self):
        """重置统计数据"""
        with self.lock:
            self.responses.clear()
            self.response_times.clear()
            self.send_times.clear()
            self.total_requests = 0
            self.total_responses = 0
            self.completed_count = 0
            self.failed_count = 0


class TestResult:
    """测试结果"""
    def __init__(self, name: str):
        self.name = name
        self.passed = False
        self.error_message = ''
        self.duration = 0.0
        self.metrics = {}
        self.details = []
    
    def add_detail(self, message: str):
        """添加详细信息"""
        self.details.append(message)
    
    def add_metric(self, key: str, value: any):
        """添加性能指标"""
        self.metrics[key] = value


class PerformanceTestSuite:
    """性能测试套件"""
    
    def __init__(self, node: PerformanceTestNode, quick_mode: bool = False, verbose: bool = False):
        self.node = node
        self.quick_mode = quick_mode
        self.verbose = verbose
        self.results: List[TestResult] = []
        self.monitor = PerformanceMonitor('command_adapter')
        
        # 测试配置
        if quick_mode:
            self.throughput_requests = 50
            self.concurrent_count = 10
            self.stress_duration = 10
        else:
            self.throughput_requests = 200
            self.concurrent_count = 50
            self.stress_duration = 30
        
        # 测试列表
        self.tests = [
            ('test_throughput', 'Throughput Test - 吞吐量测试'),
            ('test_concurrent', 'Concurrent Requests - 并发请求测试'),
            ('test_response_time', 'Response Time Distribution - 响应时间分布'),
            ('test_queue_capacity', 'Queue Capacity - 队列容量测试'),
            ('test_sustained_load', 'Sustained Load - 持续负载测试'),
        ]
    
    def print_test_list(self):
        """打印测试列表"""
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"Available Performance Tests / 可用性能测试")
        print(f"{'='*60}{Style.RESET_ALL}\n")
        
        for idx, (test_name, description) in enumerate(self.tests, 1):
            print(f"{idx}. {Fore.YELLOW}{test_name}{Style.RESET_ALL}")
            print(f"   {description}\n")
    
    def run_all_tests(self):
        """运行所有测试"""
        self._print_header("Running All Performance Tests")
        
        if self.quick_mode:
            print(f"{Fore.YELLOW}⚡ Quick mode enabled - reduced test duration{Style.RESET_ALL}\n")
        
        for test_name, description in self.tests:
            self.run_test(test_name)
            time.sleep(2.0)  # 测试间隔
        
        self._print_summary()
    
    def run_test(self, test_name: str):
        """运行单个测试"""
        test_method = getattr(self, test_name, None)
        if not test_method:
            print(f"{Fore.RED}✗ Test not found: {test_name}{Style.RESET_ALL}")
            return
        
        # 重置统计
        self.node.reset_stats()
        
        # 创建测试结果
        result = TestResult(test_name)
        
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
        
        # 打印性能指标
        if result.metrics:
            print(f"\n{Fore.CYAN}Performance Metrics:{Style.RESET_ALL}")
            for key, value in result.metrics.items():
                if isinstance(value, float):
                    print(f"  {key}: {value:.3f}")
                else:
                    print(f"  {key}: {value}")
        
        # 打印详细信息
        if self.verbose and result.details:
            print(f"\n{Fore.MAGENTA}Details:{Style.RESET_ALL}")
            for detail in result.details:
                print(f"  {detail}")
    
    # ========== 测试用例 ==========
    
    def test_throughput(self, result: TestResult):
        """测试1: 吞吐量测试"""
        result.add_detail(f"Sending {self.throughput_requests} requests...")
        
        # 开始性能监控
        self.monitor.start_monitoring()
        
        # 批量发送请求 - 使用轻量级的GET_TASK_STATUS查询不存在的任务
        # 这样会快速返回failed响应，适合测试吞吐量
        start_time = time.time()
        request_ids = []
        
        for i in range(self.throughput_requests):
            request = CommandRequest(
                action=ActionType.GET_TASK_STATUS,
                params={'task_id': f'test_task_{i}'},  # 不存在的任务
                priority=3,
                timeout=30.0
            )
            request_id = self.node.send_request(request)
            request_ids.append(request_id)
            
            # 避免过快发送
            if i % 10 == 0:
                time.sleep(0.01)
        
        send_duration = time.time() - start_time
        
        # 等待所有请求完成
        completed = self.node.wait_for_completions(self.throughput_requests, timeout=60.0)
        
        # 停止监控
        self.monitor.stop_monitoring()
        system_stats = self.monitor.get_stats()
        
        total_duration = time.time() - start_time
        
        # 获取响应时间统计
        response_stats = self.node.get_response_time_stats()
        
        # 验证
        assert completed, f"Not all requests completed: {self.node.completed_count}/{self.throughput_requests}"
        
        # 计算吞吐量
        throughput = self.throughput_requests / total_duration
        
        # 记录指标
        result.add_metric('requests_sent', self.throughput_requests)
        result.add_metric('requests_completed', self.node.completed_count)
        result.add_metric('requests_failed', self.node.failed_count)
        result.add_metric('send_duration_sec', send_duration)
        result.add_metric('total_duration_sec', total_duration)
        result.add_metric('throughput_req_per_sec', throughput)
        result.add_metric('avg_response_time_sec', response_stats['mean'])
        result.add_metric('p95_response_time_sec', response_stats['p95'])
        result.add_metric('cpu_avg_percent', system_stats['cpu_avg'])
        result.add_metric('cpu_max_percent', system_stats['cpu_max'])
        result.add_metric('memory_avg_mb', system_stats['memory_avg'])
        result.add_metric('memory_max_mb', system_stats['memory_max'])
        
        result.add_detail(f"✓ Throughput: {throughput:.2f} requests/sec")
        result.add_detail(f"✓ Avg response time: {response_stats['mean']*1000:.2f}ms")
        result.add_detail(f"✓ P95 response time: {response_stats['p95']*1000:.2f}ms")
    
    def test_concurrent(self, result: TestResult):
        """测试2: 并发请求测试"""
        result.add_detail(f"Sending {self.concurrent_count} concurrent requests...")
        
        self.monitor.start_monitoring()
        
        # 并发发送请求
        start_time = time.time()
        threads = []
        
        def send_request_thread():
            request = CommandRequest(
                action=ActionType.NAVIGATE_TO_POSE,
                params={'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                priority=3,
                timeout=300.0
            )
            self.node.send_request(request)
        
        # 创建并启动线程
        for _ in range(self.concurrent_count):
            thread = threading.Thread(target=send_request_thread)
            thread.start()
            threads.append(thread)
        
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        
        send_duration = time.time() - start_time
        
        # 等待所有响应
        completed = self.node.wait_for_completions(self.concurrent_count, timeout=30.0)
        
        self.monitor.stop_monitoring()
        system_stats = self.monitor.get_stats()
        
        total_duration = time.time() - start_time
        response_stats = self.node.get_response_time_stats()
        
        # 验证
        assert completed, f"Not all requests completed: {self.node.completed_count}/{self.concurrent_count}"
        
        # 记录指标
        result.add_metric('concurrent_requests', self.concurrent_count)
        result.add_metric('all_completed', self.node.completed_count == self.concurrent_count)
        result.add_metric('send_duration_sec', send_duration)
        result.add_metric('total_duration_sec', total_duration)
        result.add_metric('avg_response_time_sec', response_stats['mean'])
        result.add_metric('max_response_time_sec', response_stats['max'])
        result.add_metric('cpu_max_percent', system_stats['cpu_max'])
        result.add_metric('memory_max_mb', system_stats['memory_max'])
        
        result.add_detail(f"✓ All {self.concurrent_count} concurrent requests handled")
        result.add_detail(f"✓ Max response time: {response_stats['max']*1000:.2f}ms")
    
    def test_response_time(self, result: TestResult):
        """测试3: 响应时间分布"""
        result.add_detail("Analyzing response time distribution...")
        
        # 发送请求 - 使用GET_TASK_STATUS查询
        test_count = 100 if not self.quick_mode else 50
        
        for i in range(test_count):
            request = CommandRequest(
                action=ActionType.GET_TASK_STATUS,
                params={'task_id': f'test_task_{i}'},
                priority=3,
                timeout=30.0
            )
            self.node.send_request(request)
            time.sleep(0.05)  # 50ms间隔
        
        # 等待完成
        completed = self.node.wait_for_completions(test_count, timeout=30.0)
        assert completed, f"Not all requests completed"
        
        # 获取统计
        stats = self.node.get_response_time_stats()
        
        # 记录指标
        result.add_metric('sample_count', stats['count'])
        result.add_metric('mean_ms', stats['mean'] * 1000)
        result.add_metric('median_ms', stats['median'] * 1000)
        result.add_metric('min_ms', stats['min'] * 1000)
        result.add_metric('max_ms', stats['max'] * 1000)
        result.add_metric('p95_ms', stats['p95'] * 1000)
        result.add_metric('p99_ms', stats['p99'] * 1000)
        result.add_metric('stdev_ms', stats['stdev'] * 1000)
        
        result.add_detail(f"✓ Mean: {stats['mean']*1000:.2f}ms")
        result.add_detail(f"✓ Median: {stats['median']*1000:.2f}ms")
        result.add_detail(f"✓ P95: {stats['p95']*1000:.2f}ms")
        result.add_detail(f"✓ P99: {stats['p99']*1000:.2f}ms")
        result.add_detail(f"✓ Std Dev: {stats['stdev']*1000:.2f}ms")
    
    def test_queue_capacity(self, result: TestResult):
        """测试4: 队列容量测试"""
        result.add_detail("Testing queue capacity with rapid requests...")
        
        # 快速发送大量请求（不等待）
        burst_count = 200 if not self.quick_mode else 100
        
        start_time = time.time()
        for i in range(burst_count):
            request = CommandRequest(
                action=ActionType.GET_TASK_STATUS,
                params={'task_id': f'burst_task_{i}'},
                priority=3,
                timeout=30.0
            )
            self.node.send_request(request)
        
        send_duration = time.time() - start_time
        
        # 等待处理
        time.sleep(5.0)
        
        # 检查结果
        with self.node.lock:
            queued = self.node.total_requests
            completed = self.node.completed_count
            failed = self.node.failed_count
        
        # 记录指标
        result.add_metric('burst_requests', burst_count)
        result.add_metric('send_duration_sec', send_duration)
        result.add_metric('requests_queued', queued)
        result.add_metric('requests_completed', completed)
        result.add_metric('requests_failed', failed)
        result.add_metric('success_rate', completed / burst_count if burst_count > 0 else 0.0)
        
        result.add_detail(f"✓ Sent {burst_count} requests in {send_duration:.2f}s")
        result.add_detail(f"✓ Completed: {completed}, Failed: {failed}")
        result.add_detail(f"✓ Success rate: {completed/burst_count*100:.1f}%")
    
    def test_sustained_load(self, result: TestResult):
        """测试5: 持续负载测试"""
        result.add_detail(f"Running sustained load for {self.stress_duration}s...")
        
        self.monitor.start_monitoring()
        
        start_time = time.time()
        request_count = 0
        
        # 持续发送请求
        while time.time() - start_time < self.stress_duration:
            request = CommandRequest(
                action=ActionType.GET_TASK_STATUS,
                params={'task_id': f'stress_task_{request_count}'},
                priority=3,
                timeout=30.0
            )
            self.node.send_request(request)
            request_count += 1
            time.sleep(0.1)  # 10 req/s
        
        duration = time.time() - start_time
        
        # 等待所有请求完成
        time.sleep(5.0)
        
        self.monitor.stop_monitoring()
        system_stats = self.monitor.get_stats()
        
        with self.node.lock:
            completed = self.node.completed_count
            failed = self.node.failed_count
        
        # 记录指标
        result.add_metric('test_duration_sec', duration)
        result.add_metric('total_requests', request_count)
        result.add_metric('requests_completed', completed)
        result.add_metric('requests_failed', failed)
        result.add_metric('avg_rate_req_per_sec', request_count / duration)
        result.add_metric('cpu_avg_percent', system_stats['cpu_avg'])
        result.add_metric('cpu_max_percent', system_stats['cpu_max'])
        result.add_metric('memory_avg_mb', system_stats['memory_avg'])
        result.add_metric('memory_max_mb', system_stats['memory_max'])
        
        result.add_detail(f"✓ Processed {request_count} requests in {duration:.1f}s")
        result.add_detail(f"✓ Avg rate: {request_count/duration:.2f} req/s")
        result.add_detail(f"✓ CPU avg: {system_stats['cpu_avg']:.1f}%")
        result.add_detail(f"✓ Memory avg: {system_stats['memory_avg']:.1f}MB")
    
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
        print(f"Performance Test Summary / 性能测试总结")
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
        description='CommandAdapter Performance Test Suite'
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
        '--quick',
        action='store_true',
        help='Quick mode (reduced test duration)'
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
        node = PerformanceTestNode()
        
        # 等待节点初始化
        print(f"{Fore.CYAN}Initializing performance test node...{Style.RESET_ALL}")
        time.sleep(2.0)
        
        # 检查CommandAdapter是否运行
        print(f"{Fore.CYAN}Checking if CommandAdapter is running...{Style.RESET_ALL}")
        node_names = node.get_node_names()
        if 'command_adapter' not in node_names:
            print(f"{Fore.RED}✗ ERROR: CommandAdapter node not found!{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}Please start the test environment first:{Style.RESET_ALL}")
            print(f"  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=exploration_test")
            return 1
        else:
            print(f"{Fore.GREEN}✓ CommandAdapter is running{Style.RESET_ALL}")
        
        # 创建测试套件
        suite = PerformanceTestSuite(node, quick_mode=args.quick, verbose=args.verbose)
        
        if args.list:
            suite.print_test_list()
        elif args.test:
            test_name = args.test
            if test_name.isdigit():
                test_idx = int(test_name) - 1
                if 0 <= test_idx < len(suite.tests):
                    test_name = suite.tests[test_idx][0]
                else:
                    print(f"{Fore.RED}Invalid test number: {args.test}{Style.RESET_ALL}")
                    suite.print_test_list()
                    return 1
            
            suite.run_test(test_name)
            suite._print_summary()
        else:
            suite.run_all_tests()
        
        # 返回状态码
        passed = sum(1 for r in suite.results if r.passed)
        total = len(suite.results)
        
        return 0 if passed == total else 1
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
