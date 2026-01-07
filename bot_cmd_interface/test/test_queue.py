"""
RequestQueue单元测试 / RequestQueue unit tests

测试请求队列的基本操作、去重机制、优先级和线程安全
Tests basic operations, deduplication, priority, and thread safety of request queue
"""

import pytest
import time
import threading
from bot_cmd_interface.components.request_queue import RequestQueue
from bot_cmd_interface.sdk import (
    CommandRequest,
    ActionType,
    create_navigate_request,
    create_emergency_stop_request,
    create_patrol_request
)


class TestRequestQueueBasicOperations:
    """测试基本队列操作 / Test basic queue operations"""
    
    def setup_method(self):
        """每个测试前初始化队列 / Initialize queue before each test"""
        self.queue = RequestQueue(max_size=10, dedup_window=5.0)
    
    def test_enqueue_and_dequeue(self):
        """测试入队和出队 / Test enqueue and dequeue"""
        request = create_navigate_request(1.0, 2.0)
        
        # 入队 / Enqueue
        success, message = self.queue.enqueue(request)
        assert success is True
        assert "successfully" in message
        assert self.queue.size() == 1
        
        # 出队 / Dequeue
        dequeued = self.queue.dequeue(timeout=1.0)
        assert dequeued is not None
        assert dequeued.request_id == request.request_id
        assert self.queue.size() == 0
    
    def test_dequeue_empty_queue_timeout(self):
        """测试空队列超时出队 / Test dequeue timeout on empty queue"""
        start_time = time.time()
        result = self.queue.dequeue(timeout=0.5)
        elapsed = time.time() - start_time
        
        assert result is None
        assert 0.4 < elapsed < 0.7  # 允许一定误差
    
    def test_queue_size_limit(self):
        """测试队列大小限制 / Test queue size limit"""
        # 填满队列 / Fill queue
        for i in range(10):
            request = create_navigate_request(float(i), float(i))
            success, _ = self.queue.enqueue(request)
            assert success is True
        
        assert self.queue.size() == 10
        
        # 尝试再次入队（应该失败）/ Try to enqueue again (should fail)
        request = create_navigate_request(99.0, 99.0)
        success, message = self.queue.enqueue(request)
        assert success is False
        assert "full" in message.lower()
    
    def test_clear_queue(self):
        """测试清空队列 / Test clear queue"""
        # 添加多个请求 / Add multiple requests
        for i in range(5):
            request = create_navigate_request(float(i), float(i))
            self.queue.enqueue(request)
        
        assert self.queue.size() == 5
        
        # 清空队列 / Clear queue
        self.queue.clear()
        assert self.queue.size() == 0
    
    def test_get_statistics(self):
        """测试统计信息 / Test statistics"""
        # 入队3个请求 / Enqueue 3 requests
        for i in range(3):
            request = create_navigate_request(float(i), float(i))
            self.queue.enqueue(request)
        
        # 出队1个请求 / Dequeue 1 request
        self.queue.dequeue()
        
        stats = self.queue.get_statistics()
        assert stats['total_enqueued'] == 3
        assert stats['total_dequeued'] == 1
        assert stats['current_size'] == 2


class TestRequestQueueDeduplication:
    """测试去重机制 / Test deduplication mechanism"""
    
    def setup_method(self):
        """每个测试前初始化队列 / Initialize queue before each test"""
        self.queue = RequestQueue(max_size=10, dedup_window=2.0)  # 2秒去重窗口
    
    def test_duplicate_request_rejected_within_window(self):
        """测试去重窗口内的重复请求被拒绝 / Test duplicate request rejected within window"""
        # 创建两个内容相同但request_id不同的请求
        # Create two requests with same content but different request_ids
        request1 = create_navigate_request(1.0, 2.0, yaw=0.5)
        request2 = create_navigate_request(1.0, 2.0, yaw=0.5)
        
        # 第一个请求应该成功 / First request should succeed
        success1, message1 = self.queue.enqueue(request1)
        assert success1 is True
        
        # 第二个请求应该被拒绝（内容重复）/ Second request should be rejected (duplicate content)
        success2, message2 = self.queue.enqueue(request2)
        assert success2 is False
        assert "duplicate" in message2.lower()
        assert request1.request_id in message2  # 应包含原始request_id
        
        # 统计信息 / Statistics
        stats = self.queue.get_statistics()
        assert stats['total_rejected'] == 1
    
    def test_duplicate_request_accepted_after_window(self):
        """测试去重窗口过期后相同请求可再次入队 / Test duplicate request accepted after window expires"""
        request1 = create_navigate_request(5.0, 6.0)
        
        # 第一次入队 / First enqueue
        success1, _ = self.queue.enqueue(request1)
        assert success1 is True
        
        # 等待去重窗口过期 / Wait for dedup window to expire
        time.sleep(2.5)
        
        # 创建相同内容的请求 / Create request with same content
        request2 = create_navigate_request(5.0, 6.0)
        success2, message2 = self.queue.enqueue(request2)
        assert success2 is True
        assert "successfully" in message2
    
    def test_different_actions_not_treated_as_duplicates(self):
        """测试不同动作类型的请求不被视为重复 / Test different action types not treated as duplicates"""
        request1 = create_navigate_request(1.0, 2.0)
        request2 = create_patrol_request("route.yaml")
        
        success1, _ = self.queue.enqueue(request1)
        success2, _ = self.queue.enqueue(request2)
        
        assert success1 is True
        assert success2 is True
        assert self.queue.size() == 2
    
    def test_same_action_different_params_not_duplicates(self):
        """测试相同动作但不同参数不被视为重复 / Test same action with different params not duplicates"""
        request1 = create_navigate_request(1.0, 2.0)
        request2 = create_navigate_request(3.0, 4.0)
        
        success1, _ = self.queue.enqueue(request1)
        success2, _ = self.queue.enqueue(request2)
        
        assert success1 is True
        assert success2 is True
        assert self.queue.size() == 2


class TestRequestQueuePriority:
    """测试优先级管理 / Test priority management"""
    
    def setup_method(self):
        """每个测试前初始化队列 / Initialize queue before each test"""
        self.queue = RequestQueue(max_size=10, dedup_window=5.0)
    
    def test_emergency_stop_has_priority(self):
        """测试emergency_stop插队 / Test emergency_stop priority"""
        # 先添加普通请求 / Add normal requests first
        nav_request1 = create_navigate_request(1.0, 2.0)
        nav_request2 = create_navigate_request(3.0, 4.0)
        self.queue.enqueue(nav_request1)
        self.queue.enqueue(nav_request2)
        
        # 添加紧急停止请求 / Add emergency stop request
        emergency_request = create_emergency_stop_request()
        self.queue.enqueue(emergency_request)
        
        # 紧急停止应该最先出队 / Emergency stop should dequeue first
        first = self.queue.dequeue()
        assert first.action == ActionType.EMERGENCY_STOP
        assert first.request_id == emergency_request.request_id
        
        # 接下来是普通请求（FIFO顺序）/ Next are normal requests (FIFO order)
        second = self.queue.dequeue()
        assert second.request_id == nav_request1.request_id
        
        third = self.queue.dequeue()
        assert third.request_id == nav_request2.request_id
    
    def test_normal_requests_follow_fifo(self):
        """测试普通请求按FIFO顺序 / Test normal requests follow FIFO order"""
        requests = []
        for i in range(5):
            request = create_navigate_request(float(i), float(i))
            requests.append(request)
            self.queue.enqueue(request)
        
        # 出队顺序应与入队顺序一致 / Dequeue order should match enqueue order
        for i in range(5):
            dequeued = self.queue.dequeue()
            assert dequeued.request_id == requests[i].request_id
    
    def test_multiple_emergency_stops(self):
        """测试多个emergency_stop按FIFO顺序 / Test multiple emergency stops follow FIFO"""
        # 添加普通请求 / Add normal request
        nav_request = create_navigate_request(1.0, 2.0)
        self.queue.enqueue(nav_request)
        
        # 添加两个紧急停止请求（使用不同params避免去重）
        # Add two emergency stop requests (with different params to avoid deduplication)
        emergency1 = create_emergency_stop_request(force_immediate=False)
        emergency2 = create_emergency_stop_request(force_immediate=True)  # 不同的params
        self.queue.enqueue(emergency1)
        self.queue.enqueue(emergency2)
        
        # 两个紧急停止应该先出队，且按FIFO顺序
        # Both emergency stops should dequeue first, in FIFO order
        first = self.queue.dequeue()
        assert first.request_id == emergency1.request_id
        
        second = self.queue.dequeue()
        assert second.request_id == emergency2.request_id
        
        third = self.queue.dequeue()
        assert third.request_id == nav_request.request_id


class TestRequestQueueThreadSafety:
    """测试线程安全 / Test thread safety"""
    
    def setup_method(self):
        """每个测试前初始化队列 / Initialize queue before each test"""
        self.queue = RequestQueue(max_size=200, dedup_window=5.0)
    
    def test_concurrent_enqueue(self):
        """测试并发入队 / Test concurrent enqueue"""
        num_threads = 10
        requests_per_thread = 10
        
        def enqueue_requests(thread_id):
            """每个线程入队请求 / Each thread enqueues requests"""
            for i in range(requests_per_thread):
                request = create_navigate_request(
                    float(thread_id * 100 + i),
                    float(thread_id * 100 + i)
                )
                self.queue.enqueue(request)
        
        # 创建并启动线程 / Create and start threads
        threads = []
        for i in range(num_threads):
            thread = threading.Thread(target=enqueue_requests, args=(i,))
            threads.append(thread)
            thread.start()
        
        # 等待所有线程完成 / Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # 验证所有请求都入队成功 / Verify all requests enqueued successfully
        assert self.queue.size() == num_threads * requests_per_thread
        assert self.queue.get_statistics()['total_enqueued'] == num_threads * requests_per_thread
    
    def test_concurrent_enqueue_and_dequeue(self):
        """测试并发入队和出队 / Test concurrent enqueue and dequeue"""
        num_enqueue_threads = 5
        num_dequeue_threads = 3
        requests_per_thread = 20
        
        dequeued_count = {'value': 0}
        dequeued_lock = threading.Lock()
        
        def enqueue_requests(thread_id):
            """入队线程 / Enqueue thread"""
            for i in range(requests_per_thread):
                # 使用thread_id确保不同线程的请求不重复
                # Use thread_id to ensure requests from different threads don't duplicate
                request = create_navigate_request(
                    float(thread_id * 100 + i),
                    float(thread_id * 100 + i)
                )
                self.queue.enqueue(request)
                time.sleep(0.01)  # 稍微延迟避免过快
        
        def dequeue_requests():
            """出队线程 / Dequeue thread"""
            while True:
                request = self.queue.dequeue(timeout=0.5)
                if request is not None:
                    with dequeued_lock:
                        dequeued_count['value'] += 1
                else:
                    break
        
        # 启动入队线程 / Start enqueue threads
        enqueue_threads = []
        for i in range(num_enqueue_threads):
            thread = threading.Thread(target=enqueue_requests, args=(i,))
            enqueue_threads.append(thread)
            thread.start()
        
        # 启动出队线程 / Start dequeue threads
        dequeue_threads = []
        for _ in range(num_dequeue_threads):
            thread = threading.Thread(target=dequeue_requests)
            dequeue_threads.append(thread)
            thread.start()
        
        # 等待所有线程完成 / Wait for all threads to complete
        for thread in enqueue_threads:
            thread.join()
        
        for thread in dequeue_threads:
            thread.join()
        
        # 验证入队和出队数量一致 / Verify enqueue and dequeue counts match
        stats = self.queue.get_statistics()
        assert stats['total_enqueued'] == num_enqueue_threads * requests_per_thread
        assert dequeued_count['value'] == num_enqueue_threads * requests_per_thread


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
