#!/usr/bin/env python3
"""
简化性能测试 - 测试CMD接口的基本性能指标
Simplified Performance Test - Tests basic performance metrics of CMD interface

使用方法:
  python3 test_performance_simple.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import statistics
from datetime import datetime

from bot_cmd_interface.sdk.message import CommandRequest, CommandResponse
from bot_cmd_interface.sdk.action_types import ActionType

try:
    from colorama import Fore, Style, init
    init(autoreset=True)
except ImportError:
    class Fore:
        GREEN = RED = YELLOW = CYAN = ''
    class Style:
        RESET_ALL = ''


class SimplePerformanceTest(Node):
    def __init__(self):
        super().__init__('simple_perf_test')
        
        self.publisher = self.create_publisher(String, '/cmd/request', 10)
        self.subscription = self.create_subscription(
            String,
            '/cmd/response',
            self._response_callback,
            10
        )
        
        self.responses = {}
        self.send_times = {}
        self.response_times = []
        
    def _response_callback(self, msg: String):
        try:
            response = CommandResponse.from_json(msg.data)
            
            if response.request_id not in self.responses:
                self.responses[response.request_id] = []
            self.responses[response.request_id].append(response)
            
            # 记录完成时间
            if response.status == 'completed' and response.request_id in self.send_times:
                elapsed = time.time() - self.send_times[response.request_id]
                self.response_times.append(elapsed)
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
    
    def send_request(self, action, params):
        request = CommandRequest(
            action=action,
            params=params,
            priority=3,
            timeout=30.0
        )
        
        msg = String()
        msg.data = request.to_json()
        
        self.send_times[request.request_id] = time.time()
        self.publisher.publish(msg)
        
        return request.request_id
    
    def wait_for_completions(self, count, timeout=30.0):
        start = time.time()
        
        while time.time() - start < timeout:
            completed = sum(
                1 for resps in self.responses.values()
                if any(r.status == 'completed' for r in resps)
            )
            
            if completed >= count:
                return True
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return False


def main():
    rclpy.init()
    
    try:
        node = SimplePerformanceTest()
        
        print(f"{Fore.CYAN}==================================")
        print("CMD Interface Performance Test")
        print(f"=================================={Style.RESET_ALL}\n")
        
        # 等待初始化
        print(f"{Fore.CYAN}Initializing...{Style.RESET_ALL}")
        time.sleep(2.0)
        
        # 检查节点
        if 'command_adapter' not in node.get_node_names():
            print(f"{Fore.RED}ERROR: CommandAdapter not running!{Style.RESET_ALL}")
            return 1
        
        print(f"{Fore.GREEN}✓ CommandAdapter is running{Style.RESET_ALL}\n")
        
        # ========== 测试1: 基本吞吐量 ==========
        print(f"{Fore.CYAN}Test 1: Throughput (20 requests){Style.RESET_ALL}")
        print("-" * 40)
        
        start_time = time.time()
        
        for i in range(20):
            node.send_request(
                ActionType.NAVIGATE_TO_POSE,
                {'x': 1.0 + i * 0.1, 'y': 1.0, 'yaw': 0.0}
            )
        
        send_duration = time.time() - start_time
        
        # 等待完成
        if node.wait_for_completions(20, timeout=15.0):
            total_duration = time.time() - start_time
            
            print(f"{Fore.GREEN}✓ All requests completed{Style.RESET_ALL}")
            print(f"  Send time: {send_duration:.3f}s")
            print(f"  Total time: {total_duration:.3f}s")
            print(f"  Throughput: {20/total_duration:.2f} req/s")
            
            if node.response_times:
                avg_time = statistics.mean(node.response_times)
                print(f"  Avg response: {avg_time*1000:.2f}ms")
        else:
            completed = sum(1 for r in node.responses.values() if any(rr.status == 'completed' for rr in r))
            print(f"{Fore.RED}✗ Timeout (completed {completed}/20){Style.RESET_ALL}")
        
        # 重置
        node.responses.clear()
        node.send_times.clear()
        node.response_times.clear()
        time.sleep(2.0)
        
        # ========== 测试2: 响应时间分布 ==========
        print(f"\n{Fore.CYAN}Test 2: Response Time Distribution (30 requests){Style.RESET_ALL}")
        print("-" * 40)
        
        for i in range(30):
            node.send_request(
                ActionType.GET_TASK_STATUS,
                {'task_id': f'perf_test_{i}_{int(time.time()*1000)}'}  # 唯一的task_id
            )
            time.sleep(0.05)  # 50ms间隔
        
        if node.wait_for_completions(30, timeout=15.0):
            if node.response_times:
                times_ms = [t * 1000 for t in node.response_times]
                
                print(f"{Fore.GREEN}✓ All requests completed{Style.RESET_ALL}")
                print(f"  Count: {len(times_ms)}")
                print(f"  Mean: {statistics.mean(times_ms):.2f}ms")
                print(f"  Median: {statistics.median(times_ms):.2f}ms")
                print(f"  Min: {min(times_ms):.2f}ms")
                print(f"  Max: {max(times_ms):.2f}ms")
                
                sorted_times = sorted(times_ms)
                p95 = sorted_times[int(len(sorted_times) * 0.95)]
                p99 = sorted_times[int(len(sorted_times) * 0.99)] if len(sorted_times) > 1 else sorted_times[-1]
                print(f"  P95: {p95:.2f}ms")
                print(f"  P99: {p99:.2f}ms")
        else:
            completed = sum(1 for r in node.responses.values() if any(rr.status == 'completed' for rr in r))
            print(f"{Fore.RED}✗ Timeout (completed {completed}/30){Style.RESET_ALL}")
        
        # 重置
        node.responses.clear()
        node.send_times.clear()
        node.response_times.clear()
        time.sleep(2.0)
        
        # ========== 测试3: 并发处理 ==========
        print(f"\n{Fore.CYAN}Test 3: Concurrent Handling (10 simultaneous requests){Style.RESET_ALL}")
        print("-" * 40)
        
        import threading
        
        def send_concurrent():
            node.send_request(
                ActionType.NAVIGATE_TO_POSE,
                {'x': 1.5, 'y': 1.5, 'yaw': 0.0}
            )
        
        start_time = time.time()
        threads = []
        for _ in range(10):
            t = threading.Thread(target=send_concurrent)
            t.start()
            threads.append(t)
        
        for t in threads:
            t.join()
        
        send_duration = time.time() - start_time
        
        if node.wait_for_completions(10, timeout=10.0):
            total_duration = time.time() - start_time
            
            print(f"{Fore.GREEN}✓ All concurrent requests handled{Style.RESET_ALL}")
            print(f"  Send time: {send_duration:.3f}s")
            print(f"  Total time: {total_duration:.3f}s")
            
            if node.response_times:
                times_ms = [t * 1000 for t in node.response_times]
                print(f"  Max response: {max(times_ms):.2f}ms")
                print(f"  Avg response: {statistics.mean(times_ms):.2f}ms")
        else:
            completed = sum(1 for r in node.responses.values() if any(rr.status == 'completed' for rr in r))
            print(f"{Fore.RED}✗ Timeout (completed {completed}/10){Style.RESET_ALL}")
        
        print(f"\n{Fore.CYAN}{'='*40}")
        print("Performance Test Complete")
        print(f"{'='*40}{Style.RESET_ALL}\n")
        
        return 0
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    sys.exit(main())
