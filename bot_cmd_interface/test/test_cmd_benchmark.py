#!/usr/bin/env python3
"""
CMD接口基准测试 - 只测试接口本身的性能，不依赖后端任务执行
CMD Interface Benchmark - Tests interface performance only, independent of backend task execution

测试策略：
1. 发送请求
2. 只等待queued/executing/completed状态
3. 不关心任务实际是否执行成功（那是MissionPlanner的责任）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import statistics

from bot_cmd_interface.sdk.message import CommandRequest, CommandResponse
from bot_cmd_interface.sdk.action_types import ActionType

try:
    from colorama import Fore, Style, init
    init(autoreset=True)
except ImportError:
    class Fore:
        GREEN = RED = YELLOW = CYAN = MAGENTA = ''
    class Style:
        RESET_ALL = ''


class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('cmd_benchmark')
        
        self.publisher = self.create_publisher(String, '/cmd/request', 10)
        self.subscription = self.create_subscription(
            String,
            '/cmd/response',
            self._response_callback,
            10
        )
        
        self.request_data = {}  # {request_id: {'send_time': float, 'responses': []}}
        
    def _response_callback(self, msg: String):
        try:
            response = CommandResponse.from_json(msg.data)
            
            if response.request_id not in self.request_data:
                self.request_data[response.request_id] = {
                    'send_time': None,
                    'responses': []
                }
            
            self.request_data[response.request_id]['responses'].append({
                'status': response.status,
                'time': time.time(),
                'message': response.message
            })
            
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
    
    def send_request(self, action, params):
        request = CommandRequest(
            action=action,
            params=params,
            priority=3,
            timeout=30.0
        )
        
        self.request_data[request.request_id] = {
            'send_time': time.time(),
            'responses': []
        }
        
        msg = String()
        msg.data = request.to_json()
        self.publisher.publish(msg)
        
        return request.request_id
    
    def wait_for_response(self, request_id, status, timeout=5.0):
        """等待特定状态的响应"""
        start = time.time()
        
        while time.time() - start < timeout:
            if request_id in self.request_data:
                responses = self.request_data[request_id]['responses']
                for resp in responses:
                    if resp['status'] == status:
                        return True, resp
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return False, None
    
    def get_response_time(self, request_id, status='completed'):
        """获取从发送到特定状态的时间"""
        if request_id not in self.request_data:
            return None
        
        data = self.request_data[request_id]
        send_time = data['send_time']
        
        for resp in data['responses']:
            if resp['status'] == status:
                return resp['time'] - send_time
        
        return None


def print_stats(title, times):
    """打印统计信息"""
    if not times:
        print(f"{Fore.RED}  No data{Style.RESET_ALL}")
        return
    
    times_ms = [t * 1000 for t in times]
    sorted_times = sorted(times_ms)
    
    print(f"  Count: {len(times_ms)}")
    print(f"  Mean: {statistics.mean(times_ms):.2f}ms")
    print(f"  Median: {statistics.median(times_ms):.2f}ms")
    print(f"  Min: {min(times_ms):.2f}ms")
    print(f"  Max: {max(times_ms):.2f}ms")
    
    if len(sorted_times) > 1:
        p95_idx = int(len(sorted_times) * 0.95)
        p99_idx = int(len(sorted_times) * 0.99)
        print(f"  P95: {sorted_times[p95_idx]:.2f}ms")
        print(f"  P99: {sorted_times[p99_idx]:.2f}ms")


def main():
    rclpy.init()
    
    try:
        node = BenchmarkNode()
        
        print(f"\n{Fore.CYAN}{'='*50}")
        print("CMD Interface Benchmark")
        print("测试CMD接口的响应性能（独立于后端执行）")
        print(f"{'='*50}{Style.RESET_ALL}\n")
        
        time.sleep(2.0)
        
        if 'command_adapter' not in node.get_node_names():
            print(f"{Fore.RED}ERROR: CommandAdapter not running{Style.RESET_ALL}")
            return 1
        
        print(f"{Fore.GREEN}✓ CommandAdapter running{Style.RESET_ALL}\n")
        
        # ========== Test 1: Queued响应时间 ==========
        print(f"{Fore.CYAN}Test 1: Queued Response Time (入队响应){Style.RESET_ALL}")
        print("-" * 50)
        
        queued_times = []
        test_count = 30
        
        for i in range(test_count):
            request_id = node.send_request(
                ActionType.NAVIGATE_TO_POSE,
                {'x': 1.0 + i * 0.01, 'y': 1.0, 'yaw': 0.0}
            )
            
            # 等待queued响应
            success, resp = node.wait_for_response(request_id, 'queued', timeout=2.0)
            
            if success:
                elapsed = node.get_response_time(request_id, 'queued')
                if elapsed:
                    queued_times.append(elapsed)
            
            time.sleep(0.05)  # 50ms间隔
        
        print(f"{Fore.GREEN}✓ Queued响应统计 ({len(queued_times)}/{test_count}):{Style.RESET_ALL}")
        print_stats("Queued", queued_times)
        
        node.request_data.clear()
        time.sleep(2.0)
        
        # ========== Test 2: Completed响应时间 ==========
        print(f"\n{Fore.CYAN}Test 2: Completed Response Time (完成响应){Style.RESET_ALL}")
        print("-" * 50)
        
        completed_times = []
        test_count = 20
        
        for i in range(test_count):
            request_id = node.send_request(
                ActionType.GET_TASK_STATUS,
                {'task_id': f'bench_{i}_{int(time.time()*1000000)}'}
            )
            
            # 等待completed响应（失败也算completed）
            success, resp = node.wait_for_response(request_id, 'completed', timeout=3.0)
            if not success:
                # 可能是failed
                success, resp = node.wait_for_response(request_id, 'failed', timeout=0.5)
                if success:
                    elapsed = node.get_response_time(request_id, 'failed')
                    if elapsed:
                        completed_times.append(elapsed)
            else:
                elapsed = node.get_response_time(request_id, 'completed')
                if elapsed:
                    completed_times.append(elapsed)
            
            time.sleep(0.05)
        
        print(f"{Fore.GREEN}✓ Completed响应统计 ({len(completed_times)}/{test_count}):{Style.RESET_ALL}")
        print_stats("Completed", completed_times)
        
        node.request_data.clear()
        time.sleep(2.0)
        
        # ========== Test 3: 吞吐量测试 ==========
        print(f"\n{Fore.CYAN}Test 3: Throughput (吞吐量){Style.RESET_ALL}")
        print("-" * 50)
        
        batch_size = 50
        start_time = time.time()
        
        request_ids = []
        for i in range(batch_size):
            request_id = node.send_request(
                ActionType.GET_TASK_STATUS,
                {'task_id': f'throughput_{i}_{int(time.time()*1000000)}'}
            )
            request_ids.append(request_id)
            
            if i % 10 == 0:
                time.sleep(0.01)
        
        send_duration = time.time() - start_time
        
        # 等待所有queued响应
        time.sleep(3.0)
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # 统计收到的响应
        queued_count = 0
        completed_count = 0
        
        for req_id in request_ids:
            if req_id in node.request_data:
                responses = node.request_data[req_id]['responses']
                if any(r['status'] == 'queued' for r in responses):
                    queued_count += 1
                if any(r['status'] in ['completed', 'failed'] for r in responses):
                    completed_count += 1
        
        print(f"{Fore.GREEN}✓ 吞吐量测试结果:{Style.RESET_ALL}")
        print(f"  发送: {batch_size} 请求 in {send_duration:.3f}s")
        print(f"  发送速率: {batch_size/send_duration:.1f} req/s")
        print(f"  Queued响应: {queued_count}/{batch_size} ({queued_count/batch_size*100:.1f}%)")
        print(f"  Completed响应: {completed_count}/{batch_size} ({completed_count/batch_size*100:.1f}%)")
        
        node.request_data.clear()
        time.sleep(2.0)
        
        # ========== Test 4: 并发请求 ==========
        print(f"\n{Fore.CYAN}Test 4: Concurrent Requests (并发){Style.RESET_ALL}")
        print("-" * 50)
        
        import threading
        
        concurrent_ids = []
        lock = threading.Lock()
        
        def send_concurrent(idx):
            req_id = node.send_request(
                ActionType.GET_TASK_STATUS,
                {'task_id': f'concurrent_{idx}_{int(time.time()*1000000)}'}
            )
            with lock:
                concurrent_ids.append(req_id)
        
        start_time = time.time()
        threads = []
        for i in range(15):
            t = threading.Thread(target=send_concurrent, args=(i,))
            t.start()
            threads.append(t)
        
        for t in threads:
            t.join()
        
        send_duration = time.time() - start_time
        
        # 等待响应
        time.sleep(2.0)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # 统计
        concurrent_completed = 0
        concurrent_times = []
        
        for req_id in concurrent_ids:
            if req_id in node.request_data:
                responses = node.request_data[req_id]['responses']
                if any(r['status'] in ['completed', 'failed'] for r in responses):
                    concurrent_completed += 1
                    elapsed = node.get_response_time(req_id, 'completed')
                    if not elapsed:
                        elapsed = node.get_response_time(req_id, 'failed')
                    if elapsed:
                        concurrent_times.append(elapsed)
        
        print(f"{Fore.GREEN}✓ 并发测试结果:{Style.RESET_ALL}")
        print(f"  发送: 15个并发请求 in {send_duration:.3f}s")
        print(f"  完成: {concurrent_completed}/15")
        if concurrent_times:
            print(f"  平均响应: {statistics.mean(concurrent_times)*1000:.2f}ms")
            print(f"  最大响应: {max(concurrent_times)*1000:.2f}ms")
        
        # ========== 总结 ==========
        print(f"\n{Fore.CYAN}{'='*50}")
        print("Benchmark Complete")
        print(f"{'='*50}{Style.RESET_ALL}\n")
        
        print(f"{Fore.GREEN}关键性能指标 (Key Metrics):{Style.RESET_ALL}")
        if queued_times:
            print(f"  • Queued响应: ~{statistics.median(queued_times)*1000:.1f}ms (中位数)")
        if completed_times:
            print(f"  • Completed响应: ~{statistics.median(completed_times)*1000:.1f}ms (中位数)")
        print(f"  • 吞吐量: {batch_size/send_duration:.1f} req/s (发送)")
        print(f"  • 并发处理: {concurrent_completed}/15 请求")
        
        return 0
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    sys.exit(main())
