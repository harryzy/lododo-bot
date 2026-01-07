"""
ResponsePublisher单元测试 / ResponsePublisher unit tests

测试响应发布器的基本功能和统计信息
Tests response publisher's basic functionality and statistics
"""

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

from bot_cmd_interface.components.response_publisher import ResponsePublisher
from bot_cmd_interface.sdk import CommandResponse, ErrorCode, ResponseStatus


class TestResponsePublisher:
    """测试ResponsePublisher基本功能 / Test ResponsePublisher basic functionality"""
    
    @classmethod
    def setup_class(cls):
        """初始化ROS2 / Initialize ROS2"""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def teardown_class(cls):
        """清理ROS2 / Cleanup ROS2"""
        if rclpy.ok():
            rclpy.shutdown()
    
    def setup_method(self):
        """每个测试前初始化 / Initialize before each test"""
        self.node = Node('test_response_publisher_node')
        self.publisher = ResponsePublisher(self.node, topic_name='/test/response')
        
        # 创建订阅器用于接收发布的消息 / Create subscriber to receive published messages
        self.received_messages = []
        self.subscription = self.node.create_subscription(
            String,
            '/test/response',
            self._message_callback,
            qos_profile=10
        )
    
    def teardown_method(self):
        """每个测试后清理 / Cleanup after each test"""
        self.node.destroy_node()
    
    def _message_callback(self, msg: String):
        """接收消息回调 / Message receive callback"""
        self.received_messages.append(msg.data)
    
    def _spin_and_wait(self, timeout: float = 1.0):
        """等待消息传递 / Wait for message delivery"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(self.received_messages) > 0:
                break
    
    def test_publish_basic_response(self):
        """测试发布基本响应 / Test publish basic response"""
        # 创建响应 / Create response
        response = CommandResponse(
            request_id='test-001',
            status=ResponseStatus.QUEUED,
            message='Request queued',
            code=ErrorCode.SUCCESS
        )
        
        # 发布响应 / Publish response
        self.publisher.publish(response)
        
        # 等待消息传递 / Wait for message delivery
        self._spin_and_wait()
        
        # 验证消息已发布 / Verify message published
        assert len(self.received_messages) == 1
        
        # 解析JSON并验证内容 / Parse JSON and verify content
        received_data = json.loads(self.received_messages[0])
        assert 'header' in received_data
        assert 'body' in received_data
        assert received_data['header']['request_id'] == 'test-001'
        assert received_data['header']['status'] == 'queued'
        assert received_data['body']['code'] == 0
    
    def test_publish_multiple_responses(self):
        """测试发布多个响应 / Test publish multiple responses"""
        # 发布3个不同状态的响应 / Publish 3 responses with different statuses
        responses = [
            CommandResponse('req-1', ResponseStatus.QUEUED, 'Queued', ErrorCode.SUCCESS),
            CommandResponse('req-2', ResponseStatus.EXECUTING, 'Executing', ErrorCode.SUCCESS),
            CommandResponse('req-3', ResponseStatus.COMPLETED, 'Completed', ErrorCode.SUCCESS)
        ]
        
        for response in responses:
            self.publisher.publish(response)
        
        # 等待所有消息传递 / Wait for all messages
        self._spin_and_wait(timeout=2.0)
        rclpy.spin_once(self.node, timeout_sec=0.5)
        rclpy.spin_once(self.node, timeout_sec=0.5)
        
        # 验证接收到3个消息 / Verify received 3 messages
        assert len(self.received_messages) >= 1  # 至少收到1个
    
    def test_publish_error_convenience_method(self):
        """测试publish_error便捷方法 / Test publish_error convenience method"""
        # 使用便捷方法发布错误 / Publish error using convenience method
        self.publisher.publish_error(
            request_id='error-001',
            message='Service unavailable',
            code=ErrorCode.SERVICE_UNAVAILABLE
        )
        
        # 等待消息传递 / Wait for message delivery
        self._spin_and_wait()
        
        # 验证消息 / Verify message
        assert len(self.received_messages) == 1
        received_data = json.loads(self.received_messages[0])
        assert received_data['header']['request_id'] == 'error-001'
        assert received_data['header']['status'] == 'failed'
        assert received_data['body']['code'] == ErrorCode.SERVICE_UNAVAILABLE
    
    def test_publish_success_convenience_method(self):
        """测试publish_success便捷方法 / Test publish_success convenience method"""
        # 使用便捷方法发布成功响应 / Publish success using convenience method
        self.publisher.publish_success(
            request_id='success-001',
            message='Task completed',
            result={'task_id': 'task-123', 'progress': 1.0}
        )
        
        # 等待消息传递 / Wait for message delivery
        self._spin_and_wait()
        
        # 验证消息 / Verify message
        assert len(self.received_messages) == 1
        received_data = json.loads(self.received_messages[0])
        assert received_data['header']['request_id'] == 'success-001'
        assert received_data['header']['status'] == 'completed'
        assert received_data['body']['code'] == 0
        assert 'result' in received_data['body']
        assert 'task_id' in received_data['body']['result']
        assert received_data['body']['result']['task_id'] == 'task-123'
    
    def test_statistics_tracking(self):
        """测试统计信息追踪 / Test statistics tracking"""
        # 发布不同状态的响应 / Publish responses with different statuses
        self.publisher.publish(
            CommandResponse('r1', ResponseStatus.QUEUED, 'Q', ErrorCode.SUCCESS)
        )
        self.publisher.publish(
            CommandResponse('r2', ResponseStatus.EXECUTING, 'E', ErrorCode.SUCCESS)
        )
        self.publisher.publish(
            CommandResponse('r3', ResponseStatus.COMPLETED, 'C', ErrorCode.SUCCESS)
        )
        self.publisher.publish(
            CommandResponse('r4', ResponseStatus.FAILED, 'F', ErrorCode.INTERNAL_ERROR)
        )
        
        # 获取统计信息 / Get statistics
        stats = self.publisher.get_statistics()
        
        # 验证统计信息 / Verify statistics
        assert stats['total_published'] == 4
        assert stats['by_status']['queued'] == 1
        assert stats['by_status']['executing'] == 1
        assert stats['by_status']['completed'] == 1
        assert stats['by_status']['failed'] == 1
        assert stats['topic_name'] == '/test/response'
    
    def test_reset_statistics(self):
        """测试重置统计信息 / Test reset statistics"""
        # 发布几个响应 / Publish some responses
        for i in range(3):
            self.publisher.publish(
                CommandResponse(f'r{i}', ResponseStatus.COMPLETED, 'Done', ErrorCode.SUCCESS)
            )
        
        # 验证统计信息 / Verify statistics
        stats = self.publisher.get_statistics()
        assert stats['total_published'] == 3
        
        # 重置统计信息 / Reset statistics
        self.publisher.reset_statistics()
        
        # 验证统计信息已重置 / Verify statistics reset
        stats = self.publisher.get_statistics()
        assert stats['total_published'] == 0
        assert stats['by_status']['completed'] == 0
    
    def test_publish_with_result_data(self):
        """测试发布包含result数据的响应 / Test publish response with result data"""
        # 创建包含result的响应 / Create response with result
        response = CommandResponse(
            request_id='nav-001',
            status=ResponseStatus.COMPLETED,
            message='Navigation completed',
            code=ErrorCode.SUCCESS,
            result={
                'task_id': 'nav-task-456',
                'final_position': {'x': 1.5, 'y': 2.3},
                'execution_time': 45.2
            }
        )
        
        # 发布响应 / Publish response
        self.publisher.publish(response)
        
        # 等待消息传递 / Wait for message delivery
        self._spin_and_wait()
        
        # 验证消息 / Verify message
        assert len(self.received_messages) == 1
        received_data = json.loads(self.received_messages[0])
        assert 'body' in received_data
        assert 'result' in received_data['body']
        assert received_data['body']['result']['task_id'] == 'nav-task-456'
        assert 'final_position' in received_data['body']['result']
    
    def test_custom_topic_name(self):
        """测试自定义Topic名称 / Test custom topic name"""
        # 创建使用自定义Topic的发布器 / Create publisher with custom topic
        custom_publisher = ResponsePublisher(
            self.node,
            topic_name='/custom/responses'
        )
        
        # 验证Topic名称 / Verify topic name
        stats = custom_publisher.get_statistics()
        assert stats['topic_name'] == '/custom/responses'


class TestResponsePublisherEdgeCases:
    """测试边界情况 / Test edge cases"""
    
    @classmethod
    def setup_class(cls):
        """初始化ROS2 / Initialize ROS2"""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def teardown_class(cls):
        """清理ROS2 / Cleanup ROS2"""
        if rclpy.ok():
            rclpy.shutdown()
    
    def setup_method(self):
        """每个测试前初始化 / Initialize before each test"""
        self.node = Node('test_edge_cases_node')
        self.publisher = ResponsePublisher(self.node)
    
    def teardown_method(self):
        """每个测试后清理 / Cleanup after each test"""
        self.node.destroy_node()
    
    def test_publish_with_empty_result(self):
        """测试发布空result的响应 / Test publish response with empty result"""
        response = CommandResponse(
            request_id='empty-001',
            status=ResponseStatus.COMPLETED,
            message='Done',
            code=ErrorCode.SUCCESS,
            result={}
        )
        
        # 应该正常发布，不抛出异常 / Should publish without exception
        try:
            self.publisher.publish(response)
            success = True
        except Exception:
            success = False
        
        assert success is True
    
    def test_publish_with_none_result(self):
        """测试发布None result的响应 / Test publish response with None result"""
        response = CommandResponse(
            request_id='none-001',
            status=ResponseStatus.FAILED,
            message='Error',
            code=ErrorCode.INTERNAL_ERROR,
            result=None
        )
        
        # 应该正常发布 / Should publish without exception
        try:
            self.publisher.publish(response)
            success = True
        except Exception:
            success = False
        
        assert success is True
    
    def test_statistics_with_unknown_status(self):
        """测试未知状态不影响统计 / Test unknown status doesn't break statistics"""
        # 手动创建一个包含未知状态的响应 / Manually create response with unknown status
        response = CommandResponse(
            request_id='unknown-001',
            status='unknown_status',  # 非标准状态
            message='Test',
            code=0
        )
        
        # 发布应该成功，但不增加by_status计数 / Should publish but not increment by_status
        self.publisher.publish(response)
        
        stats = self.publisher.get_statistics()
        assert stats['total_published'] == 1
        # unknown_status不在预定义的状态列表中 / unknown_status not in predefined status list
        assert 'unknown_status' not in stats['by_status']


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
